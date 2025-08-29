/**
 * Fleet & Telematics Services
 * Author: adzetto
 */
#ifndef FLEET_TELEMATICS_H
#define FLEET_TELEMATICS_H

#include <string>
#include <vector>
#include <chrono>
#include <unordered_map>

namespace telematics
{
    struct TripSample { std::chrono::system_clock::time_point t; double lat, lon, speed, soc; };
    struct TripLog { std::string id; std::vector<TripSample> samples; };

    class DataRecorder
    {
    public:
        void begin_trip(const std::string& id) { current_.id = id; current_.samples.clear(); }
        void add(double lat, double lon, double speed, double soc) {
            current_.samples.push_back({std::chrono::system_clock::now(), lat, lon, speed, soc});
        }
        TripLog end_trip() { auto t = current_; current_ = {}; return t; }
    private:
        TripLog current_{};
    };

    class CommandGateway
    {
    public:
        void set_command(const std::string& vin, const std::string& cmd, const std::string& payload) {
            cmds_[vin+":"+cmd] = payload;
        }
        std::string get_command(const std::string& vin, const std::string& cmd) const {
            auto it = cmds_.find(vin+":"+cmd); return it==cmds_.end()? std::string{} : it->second;
        }
    private:
        std::unordered_map<std::string,std::string> cmds_{};
    };
}

#endif // FLEET_TELEMATICS_H

