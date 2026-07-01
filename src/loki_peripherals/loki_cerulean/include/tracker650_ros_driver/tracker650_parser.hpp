#pragma once

#include <string>
#include <vector>

class Tracker650Parser
{
public:
    struct DvlVelocity 
    {
        double time{};
        double vx{};
        double vy{};
        double vz{};
        int confidence{};
    };

    bool parseDVPDX(const std::string& line, DvlVelocity& out);
    const char* lastStatus() const;

private:
    const char* last_status_ = "BOOT";

    static std::vector<std::string> split(const std::string& line, char delimiter);
    static std::string formatStdoff(const std::string& field);
};