#include "tracker650_ros_driver/tracker650_parser.hpp"
#include <sstream>
#include <exception>

const char* Tracker650Parser::lastStatus() const
{
    return last_status_;
}

std::vector<std::string> Tracker650Parser::split(const std::string& line, char delimiter)
{
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, delimiter))
        fields.push_back(field);
    return fields;
}

std::string Tracker650Parser::formatStdoff(const std::string& field)
{
    size_t star_pos = field.find('*');
    if (star_pos != std::string::npos)
        return field.substr(0, star_pos);
    return field;
}

bool Tracker650Parser::parseDVPDX(const std::string& line, DvlVelocity& out)
{
    auto fields = split(line, ',');

    if (fields.empty() || fields[0] != "$DVPDX")
        return false;

    if (fields.size() < 14)
        return false;

    try {
        double tu  = std::stod(fields[1]);
        double dtu = std::stod(fields[2]);
        double pdx = std::stod(fields[6]);
        double pdy = std::stod(fields[7]);
        double pdz = std::stod(fields[8]);
        int confidence = std::stoi(fields[9]);

        if (confidence == 0)
            return false;

        double dt_sec = dtu / 1e6;
        if (dt_sec <= 0.0)
            return false;

        out.time       = tu / 1e6;
        out.vx         = pdx / dt_sec;
        out.vy         = pdy / dt_sec;
        out.vz         = pdz / dt_sec;
        out.confidence = confidence;
        return true;
    }
    catch (const std::exception&) {
        return false;
    }
}
