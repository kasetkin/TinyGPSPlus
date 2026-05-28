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

/// A single satellite as reported by GSA (and later GSV).
/// Fields populated only by GSV are kept as std::optional so "never observed"
/// is a distinct type-level state.
struct TinyGPSSatellite
{
   uint8_t                prn          = 0;
   GnssSystemId           systemId     = GnssSystemId::Unknown;
   std::optional<int16_t> elevationDeg;  // reserved for GSV
   std::optional<int16_t> azimuthDeg;    // reserved for GSV
   std::optional<int16_t> cn0DbHz;       // reserved for GSV
   bool                   inSolution   = false;  // set true by GSA
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

/// Holds the satellite picture assembled from two NMEA sources:
///  - $--GSA gives the active-solution PRNs plus the scalar quality fields
///    (mode / fix type / PDOP / HDOP / VDOP). Multi-constellation receivers (GN
///    talker) emit one GSA per constellation per epoch; they are accumulated
///    keyed by SystemID so the in-solution list is the union across systems.
///  - $--GSV gives every satellite in view with elevation / azimuth / C/N0. GSV
///    spans multiple sentences per constellation and (NMEA >=4.10) repeats a PRN
///    once per signal; entries are deduped per (system, PRN) keeping the strongest
///    C/N0. The in-view data fills TinyGPSSatellite::elevationDeg/azimuthDeg/cn0DbHz
///    and, where a PRN is also in solution, enriches the matching in-solution entry.
/// isUpdated() is fresh only when BOTH a GSA and a GSV cycle have committed.
class TinyGPSSatellites
{
   friend class TinyGPSPlus;
public:
   static constexpr std::size_t MaxGsaSatsPerSentence = 12; // PRN slots in one $--GSA sentence
   static constexpr std::size_t MaxPerSystem = 24;          // accumulated in-solution per system (up to two GSA sentences)
   static constexpr std::size_t NumKnownSystems = 6;        // GPS..NavIC in GnssSystemId
   static constexpr std::size_t MaxTotalSatellites = NumKnownSystems * MaxPerSystem;
   static constexpr std::size_t MaxInViewPerSystem = 20;    // satellite slots per GSV constellation
   static constexpr std::size_t MaxTotalInView = NumKnownSystems * MaxInViewPerSystem;

   struct Data
   {
      GsaFixMode mode    = GsaFixMode::Auto;
      GsaFixType fixType = GsaFixType::None;
      int32_t    pdop    = 0;
      int32_t    hdop    = 0;
      int32_t    vdop    = 0;

      /// Satellites in solution, owned by this snapshot (up to 5 systems × 12).
      std::array<TinyGPSSatellite, MaxTotalSatellites> inSolution{};
      std::size_t inSolutionCount = 0;

      /// Satellites in view from $--GSV (superset of the in-solution list); each
      /// carries elevation/azimuth/C/N0 and an inSolution flag. Owned by this snapshot.
      std::array<TinyGPSSatellite, MaxTotalInView> inViewArr{};
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

   /// Fresh only when BOTH a GSA and a GSV cycle have committed since the last
   /// consume(), i.e. a complete in-solution + in-view + enriched snapshot is ready.
   bool isUpdated() const
   {
      return isGsaUpdated() && isInViewUpdated();
   }

   /// Per-source freshness, so each NMEA stream stays observable in isolation.
   bool isGsaUpdated() const
   {
      return committed.has_value();
   }

   bool isInViewUpdated() const
   {
      return gsvFresh;
   }

   /// Age of the most-recent of the GSA / GSV commits.
   uint32_t age() const
   {
      const uint32_t newest = lastCommitTime > gsvLastCommitTime ? lastCommitTime : gsvLastCommitTime;
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

   struct PerSystem
   {
      std::array<TinyGPSSatellite, MaxPerSystem> sats{};
      std::size_t count = 0;
      uint32_t lastCommitTime = 0;
      bool valid = false;
   };

   uint32_t lastCommitTime = 0;

   Scalars staging;
   std::optional<Scalars> committed;

   std::array<TinyGPSSatellite, MaxGsaSatsPerSentence> stagingList{};
   std::size_t stagingCount = 0;
   GnssSystemId stagingSystemId = GnssSystemId::Unknown;

   std::array<PerSystem, NumKnownSystems> committedBySystem{};

   // System whose GSA run is currently open for continuation: consecutive
   // same-System-ID GSA sentences (a >12-sat solution split across two sentences)
   // append; any committed non-GSA sentence ends the run so the next epoch replaces.
   GnssSystemId gsaRunSystemId = GnssSystemId::Unknown;

   // ── GSV (satellites in view) ────────────────────────────────────────────
   struct PerSystemInView
   {
      std::array<TinyGPSSatellite, MaxInViewPerSystem> sats{};
      std::size_t count = 0;
      uint32_t lastCommitTime = 0;
      bool valid = false;
      int lastSignalId = -1;   // NMEA >=4.10 signal ID of the last merged group
   };

   // Trailing fields (terms 4..20) of the GSV sentence being decoded; index is
   // termNumber-4. Up to four {PRN, elevation, azimuth, SNR} quads, optionally
   // followed by a Signal ID (NMEA >=4.10). The Signal ID's position varies with
   // the satellite count, so it is located at commit time from the field count.
   std::array<int16_t, 17> gsvFieldVal{};
   std::array<bool, 17>    gsvFieldHas{};
   int gsvMaxFieldIdx = -1;

   // one signal group, accumulated across its msgNum 1..totalMessages sentences
   std::array<TinyGPSSatellite, MaxInViewPerSystem> gsvStagingList{};
   std::size_t gsvStagingCount = 0;
   GnssSystemId gsvStagingSystemId = GnssSystemId::Unknown;
   uint8_t gsvTotalMessages = 0;
   uint8_t gsvMessageNumber = 0;
   uint8_t gsvTotalInView = 0;

   std::array<PerSystemInView, NumKnownSystems> inViewBySystem{};

   bool gsvFresh = false;
   uint32_t gsvLastCommitTime = 0;

   void beginGsaSentence(GnssSystemId derivedFromTalker);
   void setMode(std::string_view term);
   void setFixType(std::string_view term);
   void setPdop(std::string_view term);
   void setHdop(std::string_view term);
   void setVdop(std::string_view term);
   void setGsaSystemId(std::string_view term);
   void appendGsaSatellite(std::string_view term);
   void commitGsa();
   void endGsaRun();

   void beginGsvSentence(GnssSystemId derivedFromTalker);
   void setGsvTotalMessages(std::string_view term);
   void setGsvMessageNumber(std::string_view term);
   void setGsvTotalInView(std::string_view term);
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
