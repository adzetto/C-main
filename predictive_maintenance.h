/**
 * Predictive Maintenance & RUL
 * Author: adzetto
 */
#ifndef PREDICTIVE_MAINTENANCE_H
#define PREDICTIVE_MAINTENANCE_H

#include <string>
#include <vector>
#include <numeric>
#include <cmath>

namespace pm
{
    struct FeatureVector { std::vector<double> x; };
    struct ComponentHealth { std::string name; double score{1.0}; double rul_hours{1000.0}; };

    class SimpleModel
    {
    public:
        ComponentHealth evaluate(const std::string& name, const FeatureVector& f) const {
            double avg = 0.0; if (!f.x.empty()) avg = std::accumulate(f.x.begin(), f.x.end(), 0.0) / (double)f.x.size();
            ComponentHealth h{name, std::max(0.0, 1.0-avg), std::max(10.0, 2000.0*(1.0-avg))};
            return h;
        }
    };

    class MaintenanceAdvisor
    {
    public:
        void add_component(const std::string& name) { comps_.push_back(name); }
        std::vector<ComponentHealth> assess(const std::vector<FeatureVector>& data) const {
            std::vector<ComponentHealth> out; size_t i=0; for (const auto& c : comps_) {
                FeatureVector fx = data.empty()? FeatureVector{} : data[std::min(i, data.size()-1)];
                out.push_back(model_.evaluate(c, fx)); i++;
            } return out;
        }
    private:
        std::vector<std::string> comps_{}; SimpleModel model_{};
    };
}

#endif // PREDICTIVE_MAINTENANCE_H

