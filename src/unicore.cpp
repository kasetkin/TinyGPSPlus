#include "unicore.h"
#include <cctype>
#include <cmath>
#include <string>
#include <algorithm>
#include <ranges>

int32_t parseDegreesLatLon(const char *str)
{
    // LOG_DEBUG("parse decimal degree from sring [%s]", str);

    const bool isNegative = (*str == '-');
    if (isNegative)
    	++str;

    // An invalid character
    if (!isdigit(*str))
        return PPP_BAD_LATLON;

    const int32_t roundDigits = static_cast<int32_t>(std::atol(str));
    // LOG_DEBUG("left part of str is %d", roundDigits);
    if (roundDigits > 181)
        return PPP_BAD_LATLON;

    while (isdigit(*str))
        ++str;

    // Degree must have a decimal point
    if (*str != '.')
        return PPP_BAD_LATLON;
        
    /// skip '.'
    ++str;

    constexpr int32_t meshtasticLatLonMultiplier = 1000 * 1000 * 10;
    int32_t currectDigitMultiplier = meshtasticLatLonMultiplier / 10;
    int32_t accumulator = 0;
    do
    {
        // LOG_DEBUG("digit is [%c], multiplier is [%d], accum is [%d]", *str, currectDigitMultiplier, accumulator);
        accumulator += (*str - '0') * currectDigitMultiplier;
        currectDigitMultiplier /= 10;
    } while (isdigit(*++str) && (currectDigitMultiplier > 0));

    // const uint64_t rightOfDecimal = static_cast<uint64_t>(atoll(str));
    // LOG_DEBUG("right part of str is %d", rightOfDecimal);
    
    // const int32_t result = leftOfDecimal * meshtasticLatLonMultiplier + rightOfDecimal;
    // LOG_DEBUG("result value is %d", result);

    const int32_t result = roundDigits * meshtasticLatLonMultiplier + accumulator;
    // LOG_DEBUG("result is [%d]", result);
    return isNegative ? -result : result;
}

std::string prepareString(const char *str)
{
    return std::string_view(str)
        | std::views::transform([](unsigned char c) static {
            return static_cast<char>(std::toupper(c));
        })
        | std::ranges::to<std::string>();
}

std::pair<std::string, std::string> splitAndPrepareString(const char *str)
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

PppSolutionStatus parseSolutionStatus(const char *str, uint16_t &outputDelayMs)
{
    const auto [leapSecsStr, solStatusStr] = splitAndPrepareString(str);

    outputDelayMs = static_cast<uint16_t>(std::atol(leapSecsStr.c_str()));

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

    return PppSolutionStatus::NO_VALUE;
}

PositionVelocityType parsePositionType(const char *str)
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

    return PositionVelocityType::NO_VALUE;
}

PppDatumId parseDatumId(const char *str)
{
    const std::string cppStr = prepareString(str);
    if (cppStr == "WGS84")
        return PppDatumId::WGS84;
    else if (cppStr == "B2B")
        return PppDatumId::B2b;
    else
        return PppDatumId::NO_VALUE;

    return PppDatumId::NO_VALUE;
}

int32_t parseStationId(const char *str)
{
    /// for some unclear reason it is not 'just' 9901 but "9901",
    /// so we need to remove quotes
    std::string cppStr(str);
    if ((cppStr[0] == '"') && (cppStr[cppStr.size() - 1] == '"') && (cppStr.size() > 2))
        cppStr = cppStr.substr(1, cppStr.size() - 2);

    const int32_t result = static_cast<int32_t>(std::atol(cppStr.c_str()));
    // LOG_DEBUG("parse StationId for PPP, input [%s], result is [%d]", str, result);
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

    return PppService::NO_VALUE;
}

// std::uint32_t calculateCRC32(std::uint8_t *szBuf, int iSize)
// {
//     std::uint32_t ulCRC = 0;
//     for (int iIndex = 0; iIndex < iSize; iIndex++)
//         ulCRC = aulCrcTable[(ulCRC ^ szBuf[iIndex]) & 0xff] ^ (ulCRC >> 8);

//     return ulCRC;
// }

void pushByte32BitCrc(std::uint8_t newChar, std::uint32_t &checksum)
{
    // std::uint32_t oldValue = checksum;
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
