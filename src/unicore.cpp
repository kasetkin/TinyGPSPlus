#include "unicore.h"
#include <cctype>
#include <cmath>
#include <string>
#include <charconv>
#include <algorithm>
#include <ranges>

int32_t parseDegreesLatLon(std::string_view str)
{
    // LOG_DEBUG("parse decimal degree from string [%s]", str);

    const bool isNegative = !str.empty() && str.front() == '-';
    if (isNegative)
        str.remove_prefix(1);

    // An invalid character
    if (str.empty() || !std::isdigit(static_cast<unsigned char>(str.front())))
        return PPP_BAD_LATLON;

    int32_t roundDigits = 0;
    auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), roundDigits);
    // LOG_DEBUG("left part of str is %d", roundDigits);
    if (roundDigits > 181)
        return PPP_BAD_LATLON;

    // Degree must have a decimal point
    if (ptr >= str.data() + str.size() || *ptr != '.')
        return PPP_BAD_LATLON;

    ++ptr; // skip '.'

    constexpr int32_t meshtasticLatLonMultiplier = 1000 * 1000 * 10;
    int32_t currectDigitMultiplier = meshtasticLatLonMultiplier / 10;
    int32_t accumulator = 0;
    const auto* const end = str.data() + str.size();
    while (ptr < end && std::isdigit(static_cast<unsigned char>(*ptr)) && currectDigitMultiplier > 0)
    {
        // LOG_DEBUG("digit is [%c], multiplier is [%d], accum is [%d]", *ptr, currectDigitMultiplier, accumulator);
        accumulator += (*ptr - '0') * currectDigitMultiplier;
        currectDigitMultiplier /= 10;
        ++ptr;
    }

    const int32_t result = roundDigits * meshtasticLatLonMultiplier + accumulator;
    // LOG_DEBUG("result is [%d]", result);
    return isNegative ? -result : result;
}

static std::string prepareString(std::string_view str)
{
    return str
        | std::views::transform([](unsigned char c) static {
            return static_cast<char>(std::toupper(c));
        })
        | std::ranges::to<std::string>();
}

static std::pair<std::string, std::string> splitAndPrepareString(std::string_view str)
{
    const std::string cppStr = prepareString(str);
    /// first check and skip "17;" part of "17;SOL_COMPUTED" string
    if (const auto delimiterPos = cppStr.find(';'); delimiterPos != std::string::npos) {
        auto first = cppStr.substr(0, delimiterPos);
        auto second = cppStr.substr(delimiterPos + 1);
        return {first, second};
    }

    return {"", cppStr};
}

PppSolutionStatus parseSolutionStatus(std::string_view str, uint16_t &outputDelayMs)
{
    const auto [leapSecsStr, solStatusStr] = splitAndPrepareString(str);

    uint16_t val = 0;
    std::from_chars(leapSecsStr.data(), leapSecsStr.data() + leapSecsStr.size(), val);
    outputDelayMs = val;

    if (solStatusStr == "SOL_COMPUTED")
        return PppSolutionStatus::SOL_COMPUTED;
    else if (solStatusStr == "INSUFFICIENT_OBS")
        return PppSolutionStatus::INSUFFICIENT_OBS;
    else if (solStatusStr == "NO_CONVERGENCE")
        return PppSolutionStatus::NO_CONVERGENCE;
    else if (solStatusStr == "COV_TRACE")
        return PppSolutionStatus::COV_TRACE;
    else
        return PppSolutionStatus::NO_VALUE;
}

PositionVelocityType parsePositionType(std::string_view str)
{
    const std::string cppStr = prepareString(str);
    if (cppStr == "NONE")
        return PositionVelocityType::NONE;
    else if (cppStr == "FIXEDPOS")
        return PositionVelocityType::FIXEDPOS;
    else if (cppStr == "FIXEDHEIGHT")
        return PositionVelocityType::FIXEDHEIGHT;
    else if (cppStr == "DOPPLER_VELOCITY")
        return PositionVelocityType::DOPPLER_VELOCITY;
    else if (cppStr == "SINGLE")
        return PositionVelocityType::SINGLE;
    else if (cppStr == "PSRDIFF")
        return PositionVelocityType::PSRDIFF;
    else if (cppStr == "SBAS")
        return PositionVelocityType::SBAS;
    else if (cppStr == "L1_FLOAT")
        return PositionVelocityType::L1_FLOAT;
    else if (cppStr == "IONOFREE_FLOAT")
        return PositionVelocityType::IONOFREE_FLOAT;
    else if (cppStr == "NARROW_FLOAT")
        return PositionVelocityType::NARROW_FLOAT;
    else if (cppStr == "L1_INT")
        return PositionVelocityType::L1_INT;
    else if (cppStr == "WIDE_INT")
        return PositionVelocityType::WIDE_INT;
    else if (cppStr == "NARROW_INT")
        return PositionVelocityType::NARROW_INT;
    else if (cppStr == "INS")
        return PositionVelocityType::INS;
    else if (cppStr == "INS_PSRSP")
        return PositionVelocityType::INS_PSRSP;
    else if (cppStr == "INS_PSRDIFF")
        return PositionVelocityType::INS_PSRDIFF;
    else if (cppStr == "INS_RTKFLOAT")
        return PositionVelocityType::INS_RTKFLOAT;
    else if (cppStr == "INS_RTKFIXED")
        return PositionVelocityType::INS_RTKFIXED;
    else if (cppStr == "PPP_CONVERGING")
        return PositionVelocityType::PPP_CONVERGING;
    else if (cppStr == "PPP")
        return PositionVelocityType::PPP;
    else
        return PositionVelocityType::NO_VALUE;
}

PppDatumId parseDatumId(std::string_view str)
{
    const std::string cppStr = prepareString(str);
    if (cppStr == "WGS84")
        return PppDatumId::WGS84;
    else if (cppStr == "B2B")
        return PppDatumId::B2b;
    else
        return PppDatumId::NO_VALUE;
}

int32_t parseStationId(std::string_view str)
{
    /// for some unclear reason it is not 'just' 9901 but "9901",
    /// so we need to remove quotes
    if (str.starts_with('"') && str.ends_with('"') && str.size() > 2)
        str = str.substr(1, str.size() - 2);
    int32_t result = 0;
    std::from_chars(str.data(), str.data() + str.size(), result);
    // LOG_DEBUG("parse StationId for PPP, result is [%d]", result);
    return result;
}

// constexpr flat_map not possible (heap escapes constant eval); switch on 7 ints is equally fast
PppService parsePppService(const int32_t stationId)
{
    switch (stationId)
    {
    case 9901:
        return PppService::GALILEO;
    case 9959:
        return PppService::BEIDOU;
    case 9960:
        return PppService::BEIDOU;
    case 9961:
        return PppService::BEIDOU;
    case 9934:
        return PppService::QZSS;
    case 9936:
        return PppService::QZSS;
    case 9939:
        return PppService::QZSS;
    default:
        return PppService::NO_VALUE;
    }
}

void pushByte32BitCrc(std::uint8_t newChar, std::uint32_t &checksum)
{
    checksum = aulCrcTable[(checksum ^ newChar) & 0xff] ^ (checksum >> 8);
}

uint32_t computeUtxTime(const int32_t week, const int32_t milliSecsOfWeek, const uint32_t leapSecs, uint32_t &outMillisecs)
{
    constexpr uint32_t MILLIS_IN_SEC = 1000;

    /// delta in seconds between:
    ///     - UTC epoch 1970-01-01 00:00:00
    /// and
    ///     - GPS epoch 1980-01-06 00:00:00
    constexpr uint32_t GPS_EPOCH_TO_UNIX_EPOCH = 3657 * 24 * 3600;

    outMillisecs = milliSecsOfWeek % MILLIS_IN_SEC;

    constexpr uint32_t SECONDS_IN_WEEK = 60 * 60 * 24 * 7;
    uint32_t result = static_cast<uint32_t>(week) * SECONDS_IN_WEEK
                    + (static_cast<uint32_t>(milliSecsOfWeek) / MILLIS_IN_SEC)
                    - static_cast<uint32_t>(leapSecs)
                    + GPS_EPOCH_TO_UNIX_EPOCH;

    return result;
}

// solutionStatusStr, positionTypeStr, serviceIdStr, datumIdStr — moved to unicore.h as inline constexpr
