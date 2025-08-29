/**
 * Simulation Toolkit (Scenarios & Fault Injection)
 * Author: adzetto
 */
#ifndef SIMULATION_TOOLKIT_H
#define SIMULATION_TOOLKIT_H

#include <string>
#include <vector>
#include <functional>
#include <chrono>

namespace sim
{
    struct Step { double t; std::function<void()> fn; };
    struct Scenario { std::string name; std::vector<Step> steps; };

    class Simulator
    {
    public:
        void add(const Scenario& s) { scenarios_.push_back(s); }
        void run_all() {
            for (auto& s : scenarios_) run(s);
        }
        void run(const Scenario& s) {
            double now = 0.0;
            for (const auto& st : s.steps) {
                if (st.t > now) {/* sleep omitted for brevity */ now = st.t;}
                if (st.fn) st.fn();
            }
        }
    private:
        std::vector<Scenario> scenarios_{};
    };
}

#endif // SIMULATION_TOOLKIT_H

