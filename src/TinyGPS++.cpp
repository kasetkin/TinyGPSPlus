/*
TinyGPS++ - a small GPS library for Arduino providing universal NMEA parsing
Based on work by and "distanceBetween" and "courseTo" courtesy of Maarten Lamers.
Suggestion to add satellites, courseTo(), and cardinal() by Matt Monson.
Location precision improvements suggested by Wayne Holder.
Copyright (C) 2008-2024 Mikal Hart
All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "TinyGPS++.h"

#include <ctype.h>
#include <cstddef>
#include <cstdlib>
#include <cmath>
#include <numbers>
#include <charconv>
#include <chrono>
#include <iterator>
#include <algorithm>
#include <string_view>

#include "unicore.h"

#define _RMCterm "RMC"
#define _GGAterm "GGA"
#define _GSAterm "GSA"

static constexpr std::string_view GNSS_TALKER_SUFFIXES{"PNABL"}; // GP, GN, GA, GB, GL

/// Map the second character of an NMEA talker ID (the suffix after 'G') to the
/// GNSS constellation it identifies. Used when a GSA sentence omits the
/// NMEA 4.10 System ID field (term 18). 'N' (combined GN talker) maps to
/// Unknown because the constellation is only carried in the field.
static constexpr GnssSystemId suffixToSystemId(char suffix) noexcept
{
   switch (suffix)
   {
      case 'P': return GnssSystemId::GPS;
      case 'L': return GnssSystemId::GLONASS;
      case 'A': return GnssSystemId::Galileo;
      case 'B': return GnssSystemId::BeiDou;   // GB
      case 'Q': return GnssSystemId::QZSS;
      case 'N': return GnssSystemId::Unknown;  // GN — combined, rely on field 18
      default:  return GnssSystemId::Unknown;
   }
}

#if !defined(ARDUINO) && !defined(__AVR__)
// Alternate implementation of millis() that relies on std
unsigned long millis()
{
    static auto start_time = std::chrono::system_clock::now();

    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    return static_cast<unsigned long>(duration.count());
}
#endif

TinyGPSPlus::TinyGPSPlus()
  :  parity(0)
  ,  isChecksumTerm(false)
  ,  curSentenceType(GPS_SENTENCE_OTHER)
  ,  curTermNumber(0)
  ,  curTermOffset(0)
  ,  sentenceHasFix(false)
  ,  sentenceChecksumCharsSize(0)
  ,  parity32bit(0)
  ,  customElts(0)
  ,  customCandidates(0)
  ,  encodedCharCount(0)
  ,  sentencesWithFixCount(0)
  ,  failedChecksumCount(0)
  ,  passedChecksumCount(0)
{
  term[0] = '\0';
}

//
// public methods
//

bool TinyGPSPlus::encode(char c)
{
  ++encodedCharCount;

  switch(c)
  {
  case ',': // term terminators
    parity ^= (uint8_t)c;
    pushByte32BitCrc(c, parity32bit);
    [[fallthrough]];
  case '\r':
  case '\n':
  case '*':
    {
      bool isValidSentence = false;
      if (curTermOffset < term.size())
      {
        term[curTermOffset] = 0;
        isValidSentence = endOfTermHandler();
      }
      ++curTermNumber;
      curTermOffset = 0;
      isChecksumTerm = c == '*';
      return isValidSentence;
    }
    break;

  case '#': // sentence begin in Unicore protocol
    parity32bit = 0;
    [[fallthrough]];
  case '$': // sentence begin
    sentenceChecksumCharsSize = c == '#' ? 8 : 2;
    curTermNumber = curTermOffset = 0;
    parity = 0;
    curSentenceType = GPS_SENTENCE_OTHER;
    isChecksumTerm = false;
    sentenceHasFix = false;
    return false;

  [[likely]] default: // ordinary characters
    if (curTermOffset < term.size() - 1)
      term[curTermOffset++] = c;
    if (!isChecksumTerm)
      parity ^= c;
    if ((!isChecksumTerm) && (sentenceChecksumCharsSize == 8))
      pushByte32BitCrc(c, parity32bit);
    return false;
  }

  return false;
}

//
// internal utilities
//
int TinyGPSPlus::fromHex(char a)
{
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else
    return a - '0';
}

// static
// Parse a (potentially negative) number with up to 2 decimal digits -xxxx.yy
// Integer from_chars approach — no FPU required
int32_t TinyGPSPlus::parseDecimal(std::string_view term)
{
  bool negative = !term.empty() && term.front() == '-';
  if (negative)
    term.remove_prefix(1);
  int32_t intPart = 0;
  auto [ptr, ec] = std::from_chars(term.data(), term.data() + term.size(), intPart);
  int32_t ret = intPart * 100;
  const auto* const end = term.data() + term.size();
  if (ptr < end && *ptr == '.')
  {
    ++ptr;
    int32_t frac = 0;
    const auto* const fracEnd = ptr + std::min<ptrdiff_t>(end - ptr, 2);
    auto [fptr, fec] = std::from_chars(ptr, fracEnd, frac);
    const int digits = static_cast<int>(fptr - ptr);
    if (digits == 1)
      ret += frac * 10;
    else if (digits == 2)
      ret += frac;
  }
  return negative ? -ret : ret;
}

// static
// Parse degrees in that funny NMEA format DDMM.MMMM
void TinyGPSPlus::parseDegrees(std::string_view term, RawDegrees &deg)
{
  uint32_t leftOfDecimal = 0;
  auto [ptr, ec] = std::from_chars(term.data(), term.data() + term.size(), leftOfDecimal);
  uint16_t minutes = (uint16_t)(leftOfDecimal % 100);
  uint32_t multiplier = 10000000UL;
  uint32_t tenMillionthsOfMinutes = minutes * multiplier;
  deg.deg = (int16_t)(leftOfDecimal / 100);
  const auto* const end = term.data() + term.size();
  if (ptr < end && *ptr == '.')
  {
    ++ptr;
    uint32_t frac = 0;
    // Cap at 7 digits — the multiplier (10^7) would underflow to 0 beyond
    // that and silently drop the entire fractional contribution. Receivers
    // like the UM980 emit 8 fractional digits in the minutes field; the
    // extra digit is truncated, matching the parseDecimal pattern.
    const auto* const fracEnd = ptr + std::min<ptrdiff_t>(end - ptr, 7);
    auto [fptr, fec] = std::from_chars(ptr, fracEnd, frac);
    const int digits = static_cast<int>(fptr - ptr);
    for (int i = 0; i < digits; ++i)
      multiplier /= 10;
    tenMillionthsOfMinutes += frac * multiplier;
  }
  deg.billionths = (5 * tenMillionthsOfMinutes + 1) / 3;
  deg.negative = false;
}

#define COMBINE(sentence_type, term_number) (((unsigned)(sentence_type) << 5) | term_number)

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool TinyGPSPlus::endOfTermHandler()
{
  const std::string_view termSv{term.data(), curTermOffset};

  // If it's the checksum term, and the checksum checks out, commit
  if (isChecksumTerm)
  {
    bool correctChecksum = false;
    if (sentenceChecksumCharsSize == 8) {
      uint32_t checksum = 0;
      for (size_t i = 0; i < sentenceChecksumCharsSize; ++i)
        checksum = static_cast<std::uint32_t>(16) * checksum + fromHex(term[i]);

      // Serial.printf("32 bits checksum from data: %d, checksum from last 8 bytes: %d\n", parity32bit, checksum);
      correctChecksum = checksum == parity32bit;
    } else {
      uint8_t checksum = 16 * fromHex(term[0]) + fromHex(term[1]);
      // Serial.printf("8 bits checksum from data: %d, checksum from last 2 bytes: %d\n", parity, checksum);
      correctChecksum = checksum == parity;
    }

    if (correctChecksum)
    {
      passedChecksumCount++;
      if (sentenceHasFix)
        ++sentencesWithFixCount;

      switch(curSentenceType)
      {
      case GPS_SENTENCE_RMC:
        date.commit();
        time.commit();
        if (sentenceHasFix)
        {
           location.commit();
           speed.commit();
           course.commit();
        }
        break;
      case GPS_SENTENCE_GGA:
        time.commit();
        if (sentenceHasFix)
        {
          location.commit();
          altitude.commit();
          geoidHeight.commit();
        }
        satellitesUsedCount.commit();
        hdop.commit();
        break;
      case GPS_SENTENCE_GSA:
        satellites.commitGsa();
        break;
      }

      // Commit all custom listeners of this sentence type
      for (TinyGPSCustom *p = customCandidates; p != NULL && p->sentenceName == customCandidates->sentenceName; p = p->next)
         p->commit();
      return true;
    }

    else
    {
      ++failedChecksumCount;
    }

    return false;
  }

  // the first term determines the sentence type
  if (curTermNumber == 0)
  {
    if (term[0] == 'G' && std::ranges::contains(GNSS_TALKER_SUFFIXES, term[1]) && std::string_view(term.data() + 2) == _RMCterm)
      curSentenceType = GPS_SENTENCE_RMC;
    else if (term[0] == 'G' && std::ranges::contains(GNSS_TALKER_SUFFIXES, term[1]) && std::string_view(term.data() + 2) == _GGAterm)
      curSentenceType = GPS_SENTENCE_GGA;
    else if (term[0] == 'G' && std::ranges::contains(GNSS_TALKER_SUFFIXES, term[1]) && std::string_view(term.data() + 2) == _GSAterm)
    {
      curSentenceType = GPS_SENTENCE_GSA;
      satellites.beginGsaSentence(suffixToSystemId(term[1]));
    }
    else
      curSentenceType = GPS_SENTENCE_OTHER;

    // Any custom candidates of this sentence type?
    for (customCandidates = customElts; customCandidates != NULL && customCandidates->sentenceName < termSv; customCandidates = customCandidates->next);
    if (customCandidates != NULL && customCandidates->sentenceName > termSv)
       customCandidates = NULL;

    return false;
  }

  if (curSentenceType != GPS_SENTENCE_OTHER && term[0])
    switch(COMBINE(curSentenceType, curTermNumber))
  {
    case COMBINE(GPS_SENTENCE_RMC, 1): // Time in both sentences
    case COMBINE(GPS_SENTENCE_GGA, 1):
      time.setTime(termSv);
      break;
    case COMBINE(GPS_SENTENCE_RMC, 2): // RMC validity
      sentenceHasFix = term[0] == 'A';
      break;
    case COMBINE(GPS_SENTENCE_RMC, 3): // Latitude
    case COMBINE(GPS_SENTENCE_GGA, 2):
      location.setLatitude(termSv);
      break;
    case COMBINE(GPS_SENTENCE_RMC, 4): // N/S
    case COMBINE(GPS_SENTENCE_GGA, 3):
      location.rawNewLatData.negative = term[0] == 'S';
      break;
    case COMBINE(GPS_SENTENCE_RMC, 5): // Longitude
    case COMBINE(GPS_SENTENCE_GGA, 4):
      location.setLongitude(termSv);
      break;
    case COMBINE(GPS_SENTENCE_RMC, 6): // E/W
    case COMBINE(GPS_SENTENCE_GGA, 5):
      location.rawNewLngData.negative = term[0] == 'W';
      break;
    case COMBINE(GPS_SENTENCE_RMC, 7): // Speed (RMC)
      speed.set(termSv);
      break;
    case COMBINE(GPS_SENTENCE_RMC, 8): // Course (RMC)
      course.set(termSv);
      break;
    case COMBINE(GPS_SENTENCE_RMC, 9): // Date (RMC)
      date.setDate(termSv);
      break;
    case COMBINE(GPS_SENTENCE_GGA, 6): // Fix data (GGA)
      sentenceHasFix = term[0] > '0';
      location.newFixQuality = (TinyGPSLocation::Quality)term[0];
      break;
    case COMBINE(GPS_SENTENCE_GGA, 7): // Satellites used (GGA)
      satellitesUsedCount.set(termSv);
      break;
    case COMBINE(GPS_SENTENCE_GGA, 8): // HDOP
      hdop.set(termSv);
      break;
    case COMBINE(GPS_SENTENCE_GGA, 9): // Altitude (GGA)
      altitude.set(termSv);
      break;
    case COMBINE(GPS_SENTENCE_RMC, 12):
      location.newFixMode = (TinyGPSLocation::Mode)term[0];
      break;
    case COMBINE(GPS_SENTENCE_GGA, 11): // Height over Geoid
      geoidHeight.set(termSv);
      break;
    case COMBINE(GPS_SENTENCE_GSA, 1): // Mode (Auto/Manual)
      satellites.setMode(termSv);
      break;
    case COMBINE(GPS_SENTENCE_GSA, 2): // Fix type (1=no fix, 2=2D, 3=3D)
      satellites.setFixType(termSv);
      break;
    case COMBINE(GPS_SENTENCE_GSA, 3):  // Satellite PRNs in fix (12 slots, terms 3..14)
    case COMBINE(GPS_SENTENCE_GSA, 4):
    case COMBINE(GPS_SENTENCE_GSA, 5):
    case COMBINE(GPS_SENTENCE_GSA, 6):
    case COMBINE(GPS_SENTENCE_GSA, 7):
    case COMBINE(GPS_SENTENCE_GSA, 8):
    case COMBINE(GPS_SENTENCE_GSA, 9):
    case COMBINE(GPS_SENTENCE_GSA, 10):
    case COMBINE(GPS_SENTENCE_GSA, 11):
    case COMBINE(GPS_SENTENCE_GSA, 12):
    case COMBINE(GPS_SENTENCE_GSA, 13):
    case COMBINE(GPS_SENTENCE_GSA, 14):
      satellites.appendGsaSatellite(termSv);
      break;
    case COMBINE(GPS_SENTENCE_GSA, 15): // PDOP
      satellites.setPdop(termSv);
      break;
    case COMBINE(GPS_SENTENCE_GSA, 16): // HDOP
      satellites.setHdop(termSv);
      break;
    case COMBINE(GPS_SENTENCE_GSA, 17): // VDOP
      satellites.setVdop(termSv);
      break;
    case COMBINE(GPS_SENTENCE_GSA, 18): // System ID (NMEA 0183 4.10)
      satellites.setGsaSystemId(termSv);
      break;
  }

  // Set custom values as needed
  for (TinyGPSCustom *p = customCandidates; p != NULL && p->sentenceName == customCandidates->sentenceName && p->termNumber <= curTermNumber; p = p->next)
    if (p->termNumber == curTermNumber)
         p->set(termSv);

  return false;
}

/* static */
double TinyGPSPlus::radians(double degrees)
{
  constexpr double scale = std::numbers::pi / 180.0;
  return degrees * scale;
}

double TinyGPSPlus::degrees(double radians)
{
  constexpr double scale = 180.0 / std::numbers::pi;
  return radians * scale;
}

double TinyGPSPlus::sq(double x)
{
  return x * x;
}

/* static */
double TinyGPSPlus::distanceBetween(double lat1, double long1, double lat2, double long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6371009 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * _GPS_EARTH_MEAN_RADIUS;
}

double TinyGPSPlus::courseTo(double lat1, double long1, double lat2, double long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  double dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += 2.0 * std::numbers::pi;
  }
  return degrees(a2);
}

std::string_view TinyGPSPlus::cardinal(double course)
{
  static constexpr std::array<std::string_view, 16> directions{
    "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
    "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
  };
  return directions[(int)((course + 11.25f) / 22.5f) % directions.size()];
}

void TinyGPSLocation::commit()
{
   rawLatData = rawNewLatData;
   rawLngData = rawNewLngData;
   fixQuality = newFixQuality;
   fixMode = newFixMode;
   lastCommitTime = millis();
   valid = updated = true;
}

void TinyGPSLocation::setLatitude(std::string_view term)
{
   TinyGPSPlus::parseDegrees(term, rawNewLatData);
}

void TinyGPSLocation::setLongitude(std::string_view term)
{
   TinyGPSPlus::parseDegrees(term, rawNewLngData);
}

double TinyGPSLocation::lat()
{
   updated = false;
   double ret = rawLatData.deg + rawLatData.billionths / 1000000000.0;
   return rawLatData.negative ? -ret : ret;
}

double TinyGPSLocation::lng()
{
   updated = false;
   double ret = rawLngData.deg + rawLngData.billionths / 1000000000.0;
   return rawLngData.negative ? -ret : ret;
}

void TinyGPSDate::commit()
{
   date = newDate;
   lastCommitTime = millis();
   valid = updated = true;
}

void TinyGPSTime::commit()
{
   time = newTime;
   lastCommitTime = millis();
   valid = updated = true;
}

void TinyGPSTime::setTime(std::string_view term)
{
   newTime = (uint32_t)TinyGPSPlus::parseDecimal(term);
}

void TinyGPSDate::setDate(std::string_view term)
{
   uint32_t val = 0;
   std::from_chars(term.data(), term.data() + term.size(), val);
   newDate = val;
}

uint16_t TinyGPSDate::year()
{
   updated = false;
   uint16_t year = date % 100;
   return year + 2000;
}

uint8_t TinyGPSDate::month()
{
   updated = false;
   return (date / 100) % 100;
}

uint8_t TinyGPSDate::day()
{
   updated = false;
   return date / 10000;
}

uint8_t TinyGPSTime::hour()
{
   updated = false;
   return time / 1000000;
}

uint8_t TinyGPSTime::minute()
{
   updated = false;
   return (time / 10000) % 100;
}

uint8_t TinyGPSTime::second()
{
   updated = false;
   return (time / 100) % 100;
}

uint8_t TinyGPSTime::centisecond()
{
   updated = false;
   return time % 100;
}

void TinyGPSDecimal::commit()
{
   val = newval;
   lastCommitTime = millis();
   valid = updated = true;
}

void TinyGPSDecimal::set(std::string_view term)
{
   newval = TinyGPSPlus::parseDecimal(term);
}

void TinyGPSInteger::commit()
{
   val = newval;
   lastCommitTime = millis();
   valid = updated = true;
}

void TinyGPSInteger::set(std::string_view term)
{
   uint32_t val = 0;
   std::from_chars(term.data(), term.data() + term.size(), val);
   newval = val;
}

TinyGPSCustom::TinyGPSCustom(TinyGPSPlus &gps, std::string_view _sentenceName, int _termNumber)
{
   begin(gps, _sentenceName, _termNumber);
}

void TinyGPSCustom::begin(TinyGPSPlus &gps, std::string_view _sentenceName, int _termNumber)
{
   lastCommitTime = 0;
   updated = valid = false;
   sentenceName = _sentenceName;
   termNumber = _termNumber;
   stagingBuffer.fill('\0');
   buffer.fill('\0');

   // Insert this item into the GPS tree
   gps.insertCustom(this, _sentenceName, _termNumber);
}

void TinyGPSCustom::commit()
{
   buffer = stagingBuffer;
   lastCommitTime = millis();
   valid = updated = true;
}

void TinyGPSCustom::set(std::string_view term)
{
   const auto n = term.copy(stagingBuffer.data(), stagingBuffer.size() - 1);
   stagingBuffer[n] = '\0';
}

void TinyGPSPlus::insertCustom(TinyGPSCustom *pElt, std::string_view sentenceName, int termNumber)
{
   TinyGPSCustom **ppelt;

   for (ppelt = &this->customElts; *ppelt != NULL; ppelt = &(*ppelt)->next)
   {
      if (sentenceName < (*ppelt)->sentenceName || (sentenceName == (*ppelt)->sentenceName && termNumber < (*ppelt)->termNumber))
         break;
   }

   pElt->next = *ppelt;
   *ppelt = pElt;
}

//
// TinyGPSSatellites — native parsing of $--GSA sentences
//

void TinyGPSSatellites::beginGsaSentence(GnssSystemId derivedFromTalker)
{
   stagingCount = 0;
   stagingSystemId = derivedFromTalker;
   newMode    = GsaFixMode::Auto;
   newFixType = GsaFixType::None;
   newPdop    = 0;
   newHdop    = 0;
   newVdop    = 0;
}

void TinyGPSSatellites::setMode(std::string_view term)
{
   if (!term.empty() && (term.front() == 'A' || term.front() == 'M'))
      newMode = static_cast<GsaFixMode>(term.front());
}

void TinyGPSSatellites::setFixType(std::string_view term)
{
   uint8_t v = 0;
   if (std::from_chars(term.data(), term.data() + term.size(), v).ec == std::errc{} && v >= 1 && v <= 3)
      newFixType = static_cast<GsaFixType>(v);
}

void TinyGPSSatellites::setPdop(std::string_view term)
{
   newPdop = TinyGPSPlus::parseDecimal(term);
}

void TinyGPSSatellites::setHdop(std::string_view term)
{
   newHdop = TinyGPSPlus::parseDecimal(term);
}

void TinyGPSSatellites::setVdop(std::string_view term)
{
   newVdop = TinyGPSPlus::parseDecimal(term);
}

void TinyGPSSatellites::setGsaSystemId(std::string_view term)
{
   uint8_t v = 0;
   if (std::from_chars(term.data(), term.data() + term.size(), v).ec == std::errc{}
       && v >= static_cast<uint8_t>(GnssSystemId::GPS)
       && v <= static_cast<uint8_t>(GnssSystemId::QZSS))
   {
      stagingSystemId = static_cast<GnssSystemId>(v);
   }
   // empty / unparseable / out-of-range: leave the talker-derived value intact
}

void TinyGPSSatellites::appendGsaSatellite(std::string_view term)
{
   if (term.empty() || stagingCount >= MaxPerSystem)
      return;
   uint8_t prn = 0;
   if (std::from_chars(term.data(), term.data() + term.size(), prn).ec != std::errc{})
      return;
   if (prn == 0)
      return;
   TinyGPSSatellite& slot = stagingList[stagingCount++];
   slot = TinyGPSSatellite{};
   slot.prn = prn;
   slot.systemId = stagingSystemId;
   slot.inSolution = true;
}

void TinyGPSSatellites::commitGsa()
{
   mode    = newMode;
   fixType = newFixType;
   pdop    = newPdop;
   hdop    = newHdop;
   vdop    = newVdop;

   // Per-system snapshot: store under the constellation index. The Unknown
   // talker (GN with no System ID field) gets routed to the GPS slot as a
   // best-effort fallback so its satellites still appear in inSolution()
   // — pre-NMEA-4.10 GN sentences shouldn't be emitted by spec-compliant
   // receivers, and the UM980 does send the System ID field.
   const auto sysIdx = (stagingSystemId == GnssSystemId::Unknown)
        ? static_cast<std::size_t>(GnssSystemId::GPS) - 1
        : static_cast<std::size_t>(stagingSystemId) - 1;
   // The talker prefix tagged each satellite at append-time, but the NMEA
   // 4.10 System ID field (term 18) may have refined it afterwards. Apply
   // the final stagingSystemId to every satellite so callers see a
   // consistent constellation tag on each TinyGPSSatellite record.
   const GnssSystemId committedSystemId = (stagingSystemId == GnssSystemId::Unknown)
        ? GnssSystemId::GPS
        : stagingSystemId;
   for (std::size_t i = 0; i < stagingCount; ++i)
      stagingList[i].systemId = committedSystemId;

   if (sysIdx < NumKnownSystems)
   {
      PerSystem& slot = committedBySystem[sysIdx];
      slot.count = stagingCount;
      for (std::size_t i = 0; i < stagingCount; ++i)
         slot.sats[i] = stagingList[i];
      slot.lastCommitTime = millis();
      slot.valid = true;
   }

   rebuildFlatList();
   lastCommitTime = millis();
   valid = true;
   updated = true;
}

void TinyGPSSatellites::rebuildFlatList()
{
   flatCount = 0;
   for (const PerSystem& slot : committedBySystem)
   {
      if (!slot.valid)
         continue;
      for (std::size_t i = 0; i < slot.count && flatCount < flatList.size(); ++i)
         flatList[flatCount++] = slot.sats[i];
   }
}
