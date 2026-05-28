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



#pragma once

#include <inttypes.h>
// #include "Arduino.h"
#include <limits.h>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <charconv>

inline constexpr std::string_view GPS_VERSION          = "1.1.0";
inline constexpr double           GPS_MPH_PER_KNOT     = 1.15077945;
inline constexpr double           GPS_MPS_PER_KNOT     = 0.51444444;
inline constexpr double           GPS_KMPH_PER_KNOT    = 1.852;
inline constexpr double           GPS_MILES_PER_METER  = 0.00062137112;
inline constexpr double           GPS_KM_PER_METER     = 0.001;
inline constexpr double           GPS_FEET_PER_METER   = 3.2808399;
// was 15 in the original; PPPNAV messages (solution_status, position_status) need more
inline constexpr std::size_t      GPS_MAX_FIELD_SIZE   = 25;
inline constexpr int32_t          GPS_EARTH_MEAN_RADIUS = 6371009;  // old: 6372795

struct RawDegrees
{
   uint16_t deg;
   uint32_t billionths;
   bool negative;
public:
   RawDegrees() : deg(0), billionths(0), negative(false)
   {}
};

#if !defined(ARDUINO) && !defined(__AVR__)
// Alternate implementation of millis() that relies on ESP-IDF
unsigned long millis();
#endif

/// GNSS constellation identifier; values match the NMEA 0183 4.10 "System ID"
/// field used in $--GSA sentences (Section A.2.42).
enum class GnssSystemId : uint8_t
{
   Unknown = 0,
   GPS     = 1,
   GLONASS = 2,
   Galileo = 3,
   BeiDou  = 4,
   QZSS    = 5,
   NavIC   = 6,   // IRNSS; talker "GI"
};

/// GSA field 1: Mode (Auto / Manual switch between 2D and 3D).
enum class GsaFixMode : char
{
   Manual = 'M',
   Auto   = 'A',
};

/// GSA field 2: Fix type.
enum class GsaFixType : uint8_t
{
   None  = 1,
   Fix2D = 2,
   Fix3D = 3,
};

/// One satellite, identified by (systemId, prn). GSA sets inSolution; GSV sets
/// inView plus elevation/azimuth/C/N0. The std::optionals stay empty until a GSV
/// reports them, so "never observed" is a distinct type-level state.
struct TinyGPSSatellite
{
   uint8_t                prn          = 0;
   GnssSystemId           systemId     = GnssSystemId::Unknown;
   std::optional<int16_t> elevationDeg;  // from GSV
   std::optional<int16_t> azimuthDeg;    // from GSV
   std::optional<int16_t> cn0DbHz;       // from GSV (strongest across signals)
   bool                   inSolution   = false;  // set by GSA (used in the fix)
   bool                   inView       = false;  // set by GSV (visible)
};

class TinyGPSLocation
{
   friend class TinyGPSPlus;
public:
   enum class Quality : char
   {
      Invalid   = '0',
      GPS       = '1',
      DGPS      = '2',
      PPS       = '3',
      RTK       = '4',
      FloatRTK  = '5',
      Estimated = '6',
      Manual    = '7',
      Simulated = '8',
   };
   enum class Mode : char
   {
      N = 'N',
      A = 'A',
      D = 'D',
      E = 'E',
   };

   struct Data
   {
      RawDegrees lat{};
      RawDegrees lng{};
      Quality    fixQuality = Quality::Invalid;
      Mode       fixMode    = Mode::N;

      double latDeg() const;
      double lngDeg() const;
   };

   bool isUpdated() const
   {
      return committed.has_value();
   }

   uint32_t age() const
   {
      if (lastCommitTime == 0)
         return static_cast<uint32_t>(ULONG_MAX);
      return millis() - lastCommitTime;
   }

   std::optional<Data> consume()
   {
      if (!committed)
         return std::nullopt;
      Data out = std::move(*committed);
      committed.reset();
      return out;
   }

   TinyGPSLocation() = default;

private:
   uint32_t lastCommitTime = 0;
   Data staging;
   std::optional<Data> committed;

   void commit();
   void setLatitude(std::string_view term);
   void setLongitude(std::string_view term);
};

class TinyGPSDate
{
   friend class TinyGPSPlus;
public:
   struct Data
   {
      uint32_t raw = 0; // DDMMYY packed

      uint16_t year() const
      {
         return raw % 100 + 2000;
      }

      uint8_t month() const
      {
         return (raw / 100) % 100;
      }

      uint8_t day() const
      {
         return raw / 10000;
      }
   };

   bool isUpdated() const
   {
      return committed.has_value();
   }

   uint32_t age() const
   {
      if (lastCommitTime == 0)
         return static_cast<uint32_t>(ULONG_MAX);
      return millis() - lastCommitTime;
   }

   std::optional<Data> consume()
   {
      if (!committed)
         return std::nullopt;
      Data out{ *committed };
      committed.reset();
      return out;
   }

   TinyGPSDate() = default;

private:
   uint32_t staging = 0;
   std::optional<uint32_t> committed;
   uint32_t lastCommitTime = 0;

   void commit();
   void setDate(std::string_view term);
};

class TinyGPSTime
{
   friend class TinyGPSPlus;
public:
   struct Data
   {
      uint32_t raw = 0; // HHMMSSCC packed

      uint8_t hour() const
      {
         return raw / 1000000;
      }

      uint8_t minute() const
      {
         return (raw / 10000) % 100;
      }

      uint8_t second() const
      {
         return (raw / 100) % 100;
      }

      uint8_t centisecond() const
      {
         return raw % 100;
      }
   };

   bool isUpdated() const
   {
      return committed.has_value();
   }

   uint32_t age() const
   {
      if (lastCommitTime == 0)
         return static_cast<uint32_t>(ULONG_MAX);
      return millis() - lastCommitTime;
   }

   std::optional<Data> consume()
   {
      if (!committed)
         return std::nullopt;
      Data out{ *committed };
      committed.reset();
      return out;
   }

   TinyGPSTime() = default;

private:
   uint32_t staging = 0;
   std::optional<uint32_t> committed;
   uint32_t lastCommitTime = 0;

   void commit();
   void setTime(std::string_view term);
};

class TinyGPSDecimal
{
   friend class TinyGPSPlus;
public:
   bool isUpdated() const
   {
      return committed.has_value();
   }

   uint32_t age() const
   {
      if (lastCommitTime == 0)
         return static_cast<uint32_t>(ULONG_MAX);
      return millis() - lastCommitTime;
   }

   TinyGPSDecimal() = default;

protected:
   /// Take ownership of the committed int. Returns nullopt if no fresh commit.
   std::optional<int32_t> consumeRaw()
   {
      if (!committed)
         return std::nullopt;
      int32_t out = *committed;
      committed.reset();
      return out;
   }

   void set(std::string_view term);
   void commit();

private:
   int32_t staging = 0;
   std::optional<int32_t> committed;
   uint32_t lastCommitTime = 0;
};

class TinyGPSInteger
{
   friend class TinyGPSPlus;
public:
   struct Data
   {
      uint32_t raw = 0;
   };

   bool isUpdated() const
   {
      return committed.has_value();
   }

   uint32_t age() const
   {
      if (lastCommitTime == 0)
         return static_cast<uint32_t>(ULONG_MAX);
      return millis() - lastCommitTime;
   }

   std::optional<Data> consume()
   {
      if (!committed)
         return std::nullopt;
      Data out{ *committed };
      committed.reset();
      return out;
   }

   TinyGPSInteger() = default;

private:
   uint32_t staging = 0;
   std::optional<uint32_t> committed;
   uint32_t lastCommitTime = 0;

   void commit();
   void set(std::string_view term);
};

class TinyGPSSpeed : public TinyGPSDecimal
{
public:
   struct Data
   {
      int32_t raw = 0;

      double knots() const
      {
         return raw / 100.0;
      }

      double mph() const
      {
         return GPS_MPH_PER_KNOT * raw / 100.0;
      }

      double mps() const
      {
         return GPS_MPS_PER_KNOT * raw / 100.0;
      }

      double kmph() const
      {
         return GPS_KMPH_PER_KNOT * raw / 100.0;
      }
   };

   std::optional<Data> consume()
   {
      auto raw = consumeRaw();
      if (!raw)
         return std::nullopt;
      return Data{ *raw };
   }
};

class TinyGPSCourse : public TinyGPSDecimal
{
public:
   struct Data
   {
      int32_t raw = 0;

      double deg() const
      {
         return raw / 100.0;
      }
   };

   std::optional<Data> consume()
   {
      auto raw = consumeRaw();
      if (!raw)
         return std::nullopt;
      return Data{ *raw };
   }
};

class TinyGPSAltitude : public TinyGPSDecimal
{
public:
   struct Data
   {
      int32_t raw = 0;

      double meters() const
      {
         return raw / 100.0;
      }

      double miles() const
      {
         return GPS_MILES_PER_METER * raw / 100.0;
      }

      double kilometers() const
      {
         return GPS_KM_PER_METER * raw / 100.0;
      }

      double feet() const
      {
         return GPS_FEET_PER_METER * raw / 100.0;
      }
   };

   std::optional<Data> consume()
   {
      auto raw = consumeRaw();
      if (!raw)
         return std::nullopt;
      return Data{ *raw };
   }
};

class TinyGPSHDOP : public TinyGPSDecimal
{
public:
   struct Data
   {
      int32_t raw = 0;

      double hdop() const
      {
         return raw / 100.0;
      }
   };

   std::optional<Data> consume()
   {
      auto raw = consumeRaw();
      if (!raw)
         return std::nullopt;
      return Data{ *raw };
   }
};

/// Per-epoch satellite picture assembled into ONE buffer keyed by (systemId, prn):
///  - $--RMC starts each epoch; on it the buffer is cleared (beginEpoch()).
///  - $--GSA marks the satellites used in the fix (sets inSolution) and carries
///    the scalar quality fields (mode / fix type / PDOP / HDOP / VDOP). A >12-sat
///    solution split across two GSA sentences is just more upserts.
///  - $--GSV reports every satellite in view (sets inView + elevation / azimuth /
///    C/N0); a PRN repeated across signals is deduped in place, keeping the
///    strongest C/N0, and enriches the in-solution entry when the PRN is also used.
/// The UM980 epoch order is RMC -> GGA -> GSA -> GSV, so isUpdated() (GSA && GSV
/// fresh) turns true after the GSV burst with a complete, enriched snapshot.
class TinyGPSSatellites
{
   friend class TinyGPSPlus;
public:
   static constexpr std::size_t NumKnownSystems = 6;          // GPS..NavIC in GnssSystemId
   // Max distinct satellites (all systems) tracked per epoch. A single GSV group is
   // capped by NMEA at 9 sentences x 4 = 36 per constellation, but all six maxing out
   // at once (216) is physically impossible. The busiest real all-GNSS sky shows ~80
   // in view (this capture peaks at 58); 96 leaves headroom while keeping the by-value
   // snapshot small enough for the task stacks.
   static constexpr std::size_t MaxSatellitesTracked = 96;
   static constexpr std::size_t GsaPrnsPerSentence = 12;      // PRN slots in one $--GSA sentence

   struct Data
   {
      GsaFixMode mode    = GsaFixMode::Auto;
      GsaFixType fixType = GsaFixType::None;
      int32_t    pdop    = 0;
      int32_t    hdop    = 0;
      int32_t    vdop    = 0;

      /// Satellites used in the fix (GSA), enriched with GSV elevation/azimuth/C/N0.
      std::array<TinyGPSSatellite, MaxSatellitesTracked> inSolution{};
      std::size_t inSolutionCount = 0;

      /// Satellites in view (GSV), each with elevation/azimuth/C/N0 and an inSolution flag.
      std::array<TinyGPSSatellite, MaxSatellitesTracked> inViewArr{};
      std::size_t inViewCount = 0;

      /// Trimmed in-solution view; valid for this Data's lifetime.
      std::span<const TinyGPSSatellite> satellites() const
      {
         return std::span<const TinyGPSSatellite>(inSolution.data(), inSolutionCount);
      }

      /// Trimmed in-view view; valid for this Data's lifetime.
      std::span<const TinyGPSSatellite> inView() const
      {
         return std::span<const TinyGPSSatellite>(inViewArr.data(), inViewCount);
      }
   };

   /// Fresh only when BOTH GSA and GSV have committed since the last consume()
   /// (a complete, enriched per-epoch snapshot).
   bool isUpdated() const
   {
      return isGsaUpdated() && isInViewUpdated();
   }

   /// Per-source freshness, so each NMEA stream stays observable in isolation.
   bool isGsaUpdated() const   { return gsaFresh; }
   bool isInViewUpdated() const { return gsvFresh; }

   /// Age of the most-recent of the GSA / GSV commits.
   uint32_t age() const
   {
      const uint32_t newest = gsaCommitTime > gsvCommitTime ? gsaCommitTime : gsvCommitTime;
      if (newest == 0)
         return static_cast<uint32_t>(ULONG_MAX);
      return millis() - newest;
   }

   std::optional<Data> consume();

   TinyGPSSatellites() = default;

private:
   struct Scalars
   {
      GsaFixMode mode    = GsaFixMode::Auto;
      GsaFixType fixType = GsaFixType::None;
      int32_t    pdop    = 0;
      int32_t    hdop    = 0;
      int32_t    vdop    = 0;
   };

   // ── one buffer, keyed by (systemId, prn); cleared each epoch on RMC ─────────
   std::array<TinyGPSSatellite, MaxSatellitesTracked> buffer{};
   std::size_t bufferCount = 0;

   Scalars committedScalars;
   bool gsaFresh = false;
   bool gsvFresh = false;
   uint32_t gsaCommitTime = 0;
   uint32_t gsvCommitTime = 0;

   // ── $--GSA per-sentence parse state (folded into the buffer on commit) ──────
   Scalars staging;
   std::array<uint8_t, GsaPrnsPerSentence> stagingPrns{};
   std::size_t stagingPrnCount = 0;
   GnssSystemId stagingSystemId = GnssSystemId::Unknown;

   // ── $--GSV per-sentence parse state. Trailing fields (terms 4..20), index =
   //    termNumber-4: up to four {PRN, elev, azim, SNR} quads, optionally a
   //    trailing hex Signal ID whose position varies with the satellite count
   //    (located at commit via the field count; its value is never used). ───────
   std::array<int16_t, 17> gsvFieldVal{};
   std::array<bool, 17>    gsvFieldHas{};
   int gsvMaxFieldIdx = -1;
   GnssSystemId gsvStagingSystemId = GnssSystemId::Unknown;

   /// Find the buffer entry for (systemId, prn), or append a fresh one. Returns
   /// nullptr only if the buffer is full and the satellite is not already present.
   TinyGPSSatellite* upsert(GnssSystemId systemId, uint8_t prn);

   void beginEpoch();   // clear the buffer + freshness (called on RMC)

   void beginGsaSentence(GnssSystemId derivedFromTalker);
   void setMode(std::string_view term);
   void setFixType(std::string_view term);
   void setPdop(std::string_view term);
   void setHdop(std::string_view term);
   void setVdop(std::string_view term);
   void setGsaSystemId(std::string_view term);
   void appendGsaSatellite(std::string_view term);
   void commitGsa();

   void beginGsvSentence(GnssSystemId derivedFromTalker);
   void setGsvField(uint8_t termNumber, std::string_view term);
   void commitGsv();
};

class TinyGPSPlus;
class TinyGPSCustom
{
public:
   TinyGPSCustom() {};
   TinyGPSCustom(TinyGPSPlus &gps, std::string_view sentenceName, int termNumber);
   void begin(TinyGPSPlus &gps, std::string_view _sentenceName, int _termNumber);

   bool isUpdated() const  { return updated; }
   bool isValid() const    { return valid; }
   uint32_t age() const    { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
   std::string_view value() { updated = false; return buffer.data(); }

private:
   void commit();
   void set(std::string_view term);

   std::array<char, GPS_MAX_FIELD_SIZE + 1> stagingBuffer{};
   std::array<char, GPS_MAX_FIELD_SIZE + 1> buffer{};
   unsigned long lastCommitTime;
   bool valid, updated;
   std::string_view sentenceName;
   int termNumber;
   friend class TinyGPSPlus;
   TinyGPSCustom *next;
};

class TinyGPSPlus
{
public:
  TinyGPSPlus();
  bool encode(char c); // process one character received from GPS
  TinyGPSPlus &operator << (char c) {encode(c); return *this;}

  TinyGPSLocation location;
  TinyGPSDate date;
  TinyGPSTime time;
  TinyGPSSpeed speed;
  TinyGPSCourse course;
  TinyGPSAltitude altitude;
  TinyGPSInteger satellitesUsedCount;  // GGA term 7: number of satellites used in the fix
  TinyGPSSatellites satellites;        // GSA-derived satellite list + DOP/mode/fix-type
  TinyGPSHDOP hdop;
  TinyGPSAltitude geoidHeight;

  static std::string_view libraryVersion() { return GPS_VERSION; }

  static double distanceBetween(double lat1, double long1, double lat2, double long2);
  static double courseTo(double lat1, double long1, double lat2, double long2);
  static double radians(double degrees);
  static double degrees(double radians);
  static std::string_view cardinal(double course);

  static int32_t parseDecimal(std::string_view term);
  static void parseDegrees(std::string_view term, RawDegrees &deg);

  uint32_t charsProcessed()   const { return encodedCharCount; }
  uint32_t sentencesWithFix() const { return sentencesWithFixCount; }
  uint32_t failedChecksum()   const { return failedChecksumCount; }
  uint32_t passedChecksum()   const { return passedChecksumCount; }

  enum class SentenceType : uint8_t
  {
     GGA,
     RMC,
     GSA,
     GSV,
     Other,
  };
  SentenceType sentenceType()  const { return curSentenceType; }

private:
  // parsing state variables
  uint8_t parity;
  bool isChecksumTerm;
  std::array<char, GPS_MAX_FIELD_SIZE> term{};
  SentenceType curSentenceType;
  uint8_t curTermNumber;
  uint8_t curTermOffset;
  bool sentenceHasFix;

  /// my code
  uint8_t sentenceChecksumCharsSize;
  uint32_t parity32bit;

  // custom element support
  friend class TinyGPSCustom;
  TinyGPSCustom *customElts;
  TinyGPSCustom *customCandidates;
  void insertCustom(TinyGPSCustom *pElt, std::string_view sentenceName, int index);

  // statistics
  uint32_t encodedCharCount;
  uint32_t sentencesWithFixCount;
  uint32_t failedChecksumCount;
  uint32_t passedChecksumCount;

  // internal utilities
  int fromHex(char a);
  bool endOfTermHandler();

  static double sq(double x);

  /// Pack (sentence type, term number) into a single integral key so that the
  /// per-term dispatch in endOfTermHandler can be a flat switch. Evaluated at
  /// compile time for case labels and at runtime for the switch expression.
  static constexpr unsigned combine(SentenceType sentenceType, unsigned termNumber) noexcept;
};
