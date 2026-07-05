#pragma once

#include <string>
#include <vector>

class Tracker650Parser
{
public:
    // Why a sentence produced no velocity — so callers can log the reason
    // instead of discarding silently.
    enum class Result
    {
        OK,
        NOT_DVPDX,        // not a $DVPDX sentence
        MALFORMED,        // too few fields or non-numeric field
        ZERO_CONFIDENCE,  // DVL reports confidence 0 (no bottom lock)
        BAD_DT,           // dt <= 0, cannot derive velocity
    };

    struct DvlVelocity
    {
        double time{};
        double vx{};
        double vy{};
        double vz{};
        int confidence{};
    };

    Result parseDVPDX(const std::string& line, DvlVelocity& out);

    static const char* resultName(Result r);

private:
    static std::vector<std::string> split(const std::string& line, char delimiter);
};
