#include "tracker650_ros_driver/tracker650_parser.hpp"
#include <sstream>
#include <exception>

std::vector<std::string> Tracker650Parser::split(const std::string& line, char delimiter)
{
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, delimiter))
        fields.push_back(field);
    return fields;
}

const char* Tracker650Parser::resultName(Result r)
{
    switch (r) {
        case Result::OK:              return "OK";
        case Result::NOT_DVPDX:       return "NOT_DVPDX";
        case Result::MALFORMED:       return "MALFORMED";
        case Result::ZERO_CONFIDENCE: return "ZERO_CONFIDENCE";
        case Result::BAD_DT:          return "BAD_DT";
    }
    return "UNKNOWN";
}

Tracker650Parser::Result Tracker650Parser::parseDVPDX(const std::string& line, DvlVelocity& out)
{
    auto fields = split(line, ',');

    if (fields.empty() || fields[0] != "$DVPDX")
        return Result::NOT_DVPDX;

    if (fields.size() < 14)
        return Result::MALFORMED;

    try {
        double tu  = std::stod(fields[1]);
        double dtu = std::stod(fields[2]);
        double pdx = std::stod(fields[6]);
        double pdy = std::stod(fields[7]);
        double pdz = std::stod(fields[8]);
        int confidence = std::stoi(fields[9]);

        if (confidence == 0)
            return Result::ZERO_CONFIDENCE;

        double dt_sec = dtu / 1e6;
        if (dt_sec <= 0.0)
            return Result::BAD_DT;

        out.time       = tu / 1e6;
        out.vx         = pdx / dt_sec;
        out.vy         = pdy / dt_sec;
        out.vz         = pdz / dt_sec;
        out.confidence = confidence;
        return Result::OK;
    }
    catch (const std::exception&) {
        return Result::MALFORMED;
    }
}
