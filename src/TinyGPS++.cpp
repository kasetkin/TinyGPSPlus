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
#include <iterator>
#include <algorithm>
#include <string_view>

#include "unicore.h"

static constexpr std::string_view RMC_TERM{"RMC"};
static constexpr std::string_view GGA_TERM{"GGA"};
static constexpr std::string_view GSA_TERM{"GSA"};
static constexpr std::string_view GSV_TERM{"GSV"};

static constexpr std::string_view GNSS_TALKER_SUFFIXES{"PNABLQ"}; // GP, GN, GA, GB, GL, GQ

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
#include "esp_timer.h"
// ESP-IDF port: monotonic microseconds since boot. Always non-zero by the time
// app_main runs, so lastCommitTime = millis() never collides with the "never
// committed" sentinel of 0.
unsigned long millis()
{
    return static_cast<unsigned long>(esp_timer_get_time() / 1000);
}
#endif

TinyGPSPlus::TinyGPSPlus()
  :  parity(0)
  ,  isChecksumTerm(false)
  ,  curSentenceType(SentenceType::Other)
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
    curSentenceType = SentenceType::Other;
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

constexpr unsigned TinyGPSPlus::combine(SentenceType sentenceType, unsigned termNumber) noexcept
{
   return (static_cast<unsigned>(sentenceType) << 5) | termNumber;
}

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
      case SentenceType::RMC:
        date.commit();
        time.commit();
        if (sentenceHasFix)
        {
           location.commit();
           speed.commit();
           course.commit();
        }
        break;
      case SentenceType::GGA:
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
      case SentenceType::GSA:
        satellites.commitGsa();
        break;
      case SentenceType::GSV:
        satellites.commitGsv();
        break;
      case SentenceType::Other:
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
    if (term[0] == 'G' && std::ranges::contains(GNSS_TALKER_SUFFIXES, term[1]) && std::string_view(term.data() + 2) == RMC_TERM)
      curSentenceType = SentenceType::RMC;
    else if (term[0] == 'G' && std::ranges::contains(GNSS_TALKER_SUFFIXES, term[1]) && std::string_view(term.data() + 2) == GGA_TERM)
      curSentenceType = SentenceType::GGA;
    else if (term[0] == 'G' && std::ranges::contains(GNSS_TALKER_SUFFIXES, term[1]) && std::string_view(term.data() + 2) == GSA_TERM)
    {
      curSentenceType = SentenceType::GSA;
      satellites.beginGsaSentence(suffixToSystemId(term[1]));
    }
    else if (term[0] == 'G' && std::ranges::contains(GNSS_TALKER_SUFFIXES, term[1]) && std::string_view(term.data() + 2) == GSV_TERM)
    {
      curSentenceType = SentenceType::GSV;
      satellites.beginGsvSentence(suffixToSystemId(term[1]));
    }
    else
      curSentenceType = SentenceType::Other;

    // Any custom candidates of this sentence type?
    for (customCandidates = customElts; customCandidates != NULL && customCandidates->sentenceName < termSv; customCandidates = customCandidates->next);
    if (customCandidates != NULL && customCandidates->sentenceName > termSv)
       customCandidates = NULL;

    return false;
  }

  if (curSentenceType != SentenceType::Other && term[0])
    switch(combine(curSentenceType, curTermNumber))
  {
    case combine(SentenceType::RMC, 1): // Time in both sentences
    case combine(SentenceType::GGA, 1):
      time.setTime(termSv);
      break;
    case combine(SentenceType::RMC, 2): // RMC validity
      sentenceHasFix = term[0] == 'A';
      break;
    case combine(SentenceType::RMC, 3): // Latitude
    case combine(SentenceType::GGA, 2):
      location.setLatitude(termSv);
      break;
    case combine(SentenceType::RMC, 4): // N/S
    case combine(SentenceType::GGA, 3):
      location.staging.lat.negative = term[0] == 'S';
      break;
    case combine(SentenceType::RMC, 5): // Longitude
    case combine(SentenceType::GGA, 4):
      location.setLongitude(termSv);
      break;
    case combine(SentenceType::RMC, 6): // E/W
    case combine(SentenceType::GGA, 5):
      location.staging.lng.negative = term[0] == 'W';
      break;
    case combine(SentenceType::RMC, 7): // Speed (RMC)
      speed.set(termSv);
      break;
    case combine(SentenceType::RMC, 8): // Course (RMC)
      course.set(termSv);
      break;
    case combine(SentenceType::RMC, 9): // Date (RMC)
      date.setDate(termSv);
      break;
    case combine(SentenceType::GGA, 6): // Fix data (GGA)
      sentenceHasFix = term[0] > '0';
      location.staging.fixQuality = static_cast<TinyGPSLocation::Quality>(term[0]);
      break;
    case combine(SentenceType::GGA, 7): // Satellites used (GGA)
      satellitesUsedCount.set(termSv);
      break;
    case combine(SentenceType::GGA, 8): // HDOP
      hdop.set(termSv);
      break;
    case combine(SentenceType::GGA, 9): // Altitude (GGA)
      altitude.set(termSv);
      break;
    case combine(SentenceType::RMC, 12):
      location.staging.fixMode = static_cast<TinyGPSLocation::Mode>(term[0]);
      break;
    case combine(SentenceType::GGA, 11): // Height over Geoid
      geoidHeight.set(termSv);
      break;
    case combine(SentenceType::GSA, 1): // Mode (Auto/Manual)
      satellites.setMode(termSv);
      break;
    case combine(SentenceType::GSA, 2): // Fix type (1=no fix, 2=2D, 3=3D)
      satellites.setFixType(termSv);
      break;
    case combine(SentenceType::GSA, 3):  // Satellite PRNs in fix (12 slots, terms 3..14)
    case combine(SentenceType::GSA, 4):
    case combine(SentenceType::GSA, 5):
    case combine(SentenceType::GSA, 6):
    case combine(SentenceType::GSA, 7):
    case combine(SentenceType::GSA, 8):
    case combine(SentenceType::GSA, 9):
    case combine(SentenceType::GSA, 10):
    case combine(SentenceType::GSA, 11):
    case combine(SentenceType::GSA, 12):
    case combine(SentenceType::GSA, 13):
    case combine(SentenceType::GSA, 14):
      satellites.appendGsaSatellite(termSv);
      break;
    case combine(SentenceType::GSA, 15): // PDOP
      satellites.setPdop(termSv);
      break;
    case combine(SentenceType::GSA, 16): // HDOP
      satellites.setHdop(termSv);
      break;
    case combine(SentenceType::GSA, 17): // VDOP
      satellites.setVdop(termSv);
      break;
    case combine(SentenceType::GSA, 18): // System ID (NMEA 0183 4.10)
      satellites.setGsaSystemId(termSv);
      break;
    case combine(SentenceType::GSV, 1): // Total number of GSV sentences in this group
      satellites.setGsvTotalMessages(termSv);
      break;
    case combine(SentenceType::GSV, 2): // This sentence's number within the group
      satellites.setGsvMessageNumber(termSv);
      break;
    case combine(SentenceType::GSV, 3): // Total satellites in view
      satellites.setGsvTotalInView(termSv);
      break;
    case combine(SentenceType::GSV, 4):  // Up to four {PRN, elevation, azimuth, SNR}
    case combine(SentenceType::GSV, 5):  // quads, optionally followed by a Signal ID
    case combine(SentenceType::GSV, 6):  // (NMEA >=4.10). Empty fields are skipped by
    case combine(SentenceType::GSV, 7):  // the `term[0]` guard above; the Signal ID's
    case combine(SentenceType::GSV, 8):  // varying position is resolved in commitGsv().
    case combine(SentenceType::GSV, 9):
    case combine(SentenceType::GSV, 10):
    case combine(SentenceType::GSV, 11):
    case combine(SentenceType::GSV, 12):
    case combine(SentenceType::GSV, 13):
    case combine(SentenceType::GSV, 14):
    case combine(SentenceType::GSV, 15):
    case combine(SentenceType::GSV, 16):
    case combine(SentenceType::GSV, 17):
    case combine(SentenceType::GSV, 18):
    case combine(SentenceType::GSV, 19):
    case combine(SentenceType::GSV, 20):
      satellites.setGsvField(curTermNumber, termSv);
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
  return delta * GPS_EARTH_MEAN_RADIUS;
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

double TinyGPSLocation::Data::latDeg() const
{
   const double ret = lat.deg + lat.billionths / 1000000000.0;
   return lat.negative ? -ret : ret;
}

double TinyGPSLocation::Data::lngDeg() const
{
   const double ret = lng.deg + lng.billionths / 1000000000.0;
   return lng.negative ? -ret : ret;
}

void TinyGPSLocation::commit()
{
   committed = staging;
   lastCommitTime = millis();
}

void TinyGPSLocation::setLatitude(std::string_view term)
{
   TinyGPSPlus::parseDegrees(term, staging.lat);
}

void TinyGPSLocation::setLongitude(std::string_view term)
{
   TinyGPSPlus::parseDegrees(term, staging.lng);
}

void TinyGPSDate::commit()
{
   committed = staging;
   lastCommitTime = millis();
}

void TinyGPSDate::setDate(std::string_view term)
{
   uint32_t val = 0;
   std::from_chars(term.data(), term.data() + term.size(), val);
   staging = val;
}

void TinyGPSTime::commit()
{
   committed = staging;
   lastCommitTime = millis();
}

void TinyGPSTime::setTime(std::string_view term)
{
   staging = static_cast<uint32_t>(TinyGPSPlus::parseDecimal(term));
}

void TinyGPSDecimal::commit()
{
   committed = staging;
   lastCommitTime = millis();
}

void TinyGPSDecimal::set(std::string_view term)
{
   staging = TinyGPSPlus::parseDecimal(term);
}

void TinyGPSInteger::commit()
{
   committed = staging;
   lastCommitTime = millis();
}

void TinyGPSInteger::set(std::string_view term)
{
   uint32_t val = 0;
   std::from_chars(term.data(), term.data() + term.size(), val);
   staging = val;
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
   staging = Scalars{};
}

void TinyGPSSatellites::setMode(std::string_view term)
{
   if (!term.empty() && (term.front() == 'A' || term.front() == 'M'))
      staging.mode = static_cast<GsaFixMode>(term.front());
}

void TinyGPSSatellites::setFixType(std::string_view term)
{
   uint8_t v = 0;
   if (std::from_chars(term.data(), term.data() + term.size(), v).ec == std::errc{} && v >= 1 && v <= 3)
      staging.fixType = static_cast<GsaFixType>(v);
}

void TinyGPSSatellites::setPdop(std::string_view term)
{
   staging.pdop = TinyGPSPlus::parseDecimal(term);
}

void TinyGPSSatellites::setHdop(std::string_view term)
{
   staging.hdop = TinyGPSPlus::parseDecimal(term);
}

void TinyGPSSatellites::setVdop(std::string_view term)
{
   staging.vdop = TinyGPSPlus::parseDecimal(term);
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
   committed = staging;

   // Per-system snapshot: store under the constellation index. The Unknown
   // talker (GN with no System ID field) gets routed to the GPS slot as a
   // best-effort fallback so its satellites still appear in the union view
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

//
// TinyGPSSatellites — native parsing of $--GSV sentences (satellites in view)
//

void TinyGPSSatellites::beginGsvSentence(GnssSystemId derivedFromTalker)
{
   gsvStagingSystemId = derivedFromTalker;
   gsvFieldVal.fill(0);
   gsvFieldHas.fill(false);
   gsvMaxFieldIdx = -1;
}

void TinyGPSSatellites::setGsvTotalMessages(std::string_view term)
{
   uint8_t v = 0;
   if (std::from_chars(term.data(), term.data() + term.size(), v).ec == std::errc{})
      gsvTotalMessages = v;
}

void TinyGPSSatellites::setGsvMessageNumber(std::string_view term)
{
   uint8_t v = 0;
   if (std::from_chars(term.data(), term.data() + term.size(), v).ec == std::errc{})
      gsvMessageNumber = v;
}

void TinyGPSSatellites::setGsvTotalInView(std::string_view term)
{
   uint8_t v = 0;
   if (std::from_chars(term.data(), term.data() + term.size(), v).ec == std::errc{})
      gsvTotalInView = v;
}

// Capture one trailing GSV field (terms 4..20) by position; index = termNumber-4.
// Empty fields never reach here (skipped by the term[0] guard upstream), so any
// present entry is a real value. The Signal ID's varying position is resolved later.
void TinyGPSSatellites::setGsvField(uint8_t termNumber, std::string_view term)
{
   const unsigned idx = termNumber - 4u;
   if (idx >= gsvFieldVal.size())
      return;
   int16_t value = 0;
   if (std::from_chars(term.data(), term.data() + term.size(), value).ec == std::errc{})
   {
      gsvFieldVal[idx] = value;
      gsvFieldHas[idx] = true;
   }
   // Track extent even when the value didn't parse so the Signal ID position
   // (always the last field) is still located correctly at commit time.
   if (static_cast<int>(idx) > gsvMaxFieldIdx)
      gsvMaxFieldIdx = static_cast<int>(idx);
}

void TinyGPSSatellites::commitGsv()
{
   // Resolve the trailing-field layout. K data fields follow term 3; NMEA >=4.10
   // appends one non-null Signal ID after the satellite blocks, so a count one
   // past a multiple of four (K % 4 == 1) means the last field is that ID.
   const int K = gsvMaxFieldIdx + 1;
   const bool hasSignalId = (K % 4) == 1;
   const int satFieldCount = hasSignalId ? K - 1 : K;
   const int numSats = (satFieldCount + 3) / 4;   // round up over trailing empties
   const int sigId = (hasSignalId && gsvFieldHas[K - 1]) ? gsvFieldVal[K - 1] : -1;

   // msgNum == 1 begins a new signal-group sequence: reset the accumulator.
   if (gsvMessageNumber == 1)
   {
      gsvStagingList.fill(TinyGPSSatellite{});
      gsvStagingCount = 0;
   }

   // Fold this (checksum-passed) sentence's satellites into the sequence.
   for (int i = 0; i < numSats; ++i)
   {
      const int base = 4 * i;
      if (!gsvFieldHas[base])                     // no PRN -> empty/unused block
         continue;
      const int16_t prn = gsvFieldVal[base];
      if (prn <= 0 || prn > 255)
         continue;
      if (gsvStagingCount >= gsvStagingList.size())
         break;
      TinyGPSSatellite sat{};
      sat.prn = static_cast<uint8_t>(prn);
      sat.systemId = gsvStagingSystemId;
      if (gsvFieldHas[base + 1]) sat.elevationDeg = gsvFieldVal[base + 1];
      if (gsvFieldHas[base + 2]) sat.azimuthDeg   = gsvFieldVal[base + 2];
      if (gsvFieldHas[base + 3]) sat.cn0DbHz      = gsvFieldVal[base + 3];
      gsvStagingList[gsvStagingCount++] = sat;
   }

   // Not the last sentence of the group yet — wait for the remaining sentences.
   if (gsvMessageNumber == 0 || gsvMessageNumber < gsvTotalMessages)
      return;

   // Whole signal group received: merge it into the per-system in-view list. GN
   // (Unknown) is forbidden for GSV by spec; route it to the GPS slot as a
   // best-effort fallback, matching commitGsa().
   const std::size_t sysIdx = (gsvStagingSystemId == GnssSystemId::Unknown)
        ? static_cast<std::size_t>(GnssSystemId::GPS) - 1
        : static_cast<std::size_t>(gsvStagingSystemId) - 1;
   if (sysIdx >= NumKnownSystems)
      return;
   const GnssSystemId committedSystemId = (gsvStagingSystemId == GnssSystemId::Unknown)
        ? GnssSystemId::GPS
        : gsvStagingSystemId;

   PerSystemInView& slot = inViewBySystem[sysIdx];

   // New epoch for this constellation? Signal IDs within one epoch are emitted in
   // non-decreasing order and restart (<= last) at the next epoch. An absent
   // Signal ID (sigId < 0) means single-signal output: always replace.
   const bool newEpoch = sigId < 0 || sigId <= slot.lastSignalId;
   if (newEpoch)
   {
      slot.sats.fill(TinyGPSSatellite{});
      slot.count = 0;
   }

   // Merge the sequence, deduping per PRN and keeping the strongest C/N0
   // (an absent C/N0 counts as the weakest).
   for (std::size_t i = 0; i < gsvStagingCount; ++i)
   {
      TinyGPSSatellite sat = gsvStagingList[i];
      sat.systemId = committedSystemId;

      bool merged = false;
      for (std::size_t j = 0; j < slot.count; ++j)
      {
         if (slot.sats[j].prn != sat.prn)
            continue;
         const int existing = slot.sats[j].cn0DbHz.value_or(INT16_MIN);
         const int incoming = sat.cn0DbHz.value_or(INT16_MIN);
         if (incoming > existing)
            slot.sats[j] = sat;
         merged = true;
         break;
      }
      if (!merged && slot.count < slot.sats.size())
         slot.sats[slot.count++] = sat;
   }

   slot.valid = true;
   slot.lastSignalId = sigId;
   slot.lastCommitTime = millis();

   gsvLastCommitTime = millis();
   gsvFresh = true;
}

std::optional<TinyGPSSatellites::Data> TinyGPSSatellites::consume()
{
   if (!isGsaUpdated() && !isInViewUpdated())
      return std::nullopt;

   Data out;

   // GSA-derived scalars + in-solution list (defaults if no GSA has committed).
   if (committed)
   {
      out.mode = committed->mode;
      out.fixType = committed->fixType;
      out.pdop = committed->pdop;
      out.hdop = committed->hdop;
      out.vdop = committed->vdop;
   }
   std::copy_n(flatList.begin(), flatCount, out.inSolution.begin());
   out.inSolutionCount = flatCount;

   // GSV-derived in-view list: flatten the per-system committed lists on demand.
   out.inViewCount = 0;
   for (const PerSystemInView& slot : inViewBySystem)
   {
      if (!slot.valid)
         continue;
      for (std::size_t i = 0; i < slot.count && out.inViewCount < out.inViewArr.size(); ++i)
         out.inViewArr[out.inViewCount++] = slot.sats[i];
   }

   // Join by (systemId, prn): enrich the in-solution entries with elevation /
   // azimuth / C/N0 from GSV, and flag the in-view entries that are in solution.
   for (std::size_t i = 0; i < out.inViewCount; ++i)
   {
      TinyGPSSatellite& v = out.inViewArr[i];
      for (std::size_t k = 0; k < out.inSolutionCount; ++k)
      {
         TinyGPSSatellite& s = out.inSolution[k];
         if (s.systemId != v.systemId || s.prn != v.prn)
            continue;
         v.inSolution   = true;
         s.elevationDeg = v.elevationDeg;
         s.azimuthDeg   = v.azimuthDeg;
         s.cn0DbHz      = v.cn0DbHz;
         break;
      }
   }

   committed.reset();
   gsvFresh = false;
   return out;
}
