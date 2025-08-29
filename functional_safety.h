/**
 * Functional Safety Monitor (ISO 26262 inspired)
 * Author: adzetto
 */
#ifndef FUNCTIONAL_SAFETY_H
#define FUNCTIONAL_SAFETY_H

#include <string>
#include <vector>
#include <chrono>
#include <iostream>

namespace safety
{
    enum class HazardLevel { NONE, QM, ASIL_A, ASIL_B, ASIL_C, ASIL_D };
    enum class SafetyState { NOMINAL, DEGRADED, LIMP_HOME, EMERGENCY_STOP };
    enum class FaultType { SENSOR, ACTUATOR, POWER, THERMAL, COMMUNICATION, SOFTWARE, UNKNOWN };

    struct HazardEvent {
        std::chrono::system_clock::time_point t;
        FaultType type{FaultType::UNKNOWN};
        HazardLevel level{HazardLevel::QM};
        std::string component;
        std::string descr;
    };

    struct SafetyGoal { std::string id; std::string description; HazardLevel level{HazardLevel::QM}; };
    struct Diagnostic { std::string id; bool ok{true}; std::string note; };

    class SafetyMonitor
    {
    public:
        void add_goal(const SafetyGoal& g) { goals_.push_back(g); }
        void log_event(const HazardEvent& e) { events_.push_back(e); evaluate(e); }
        void add_diagnostic(const Diagnostic& d) { diags_.push_back(d); }
        SafetyState state() const { return state_; }
        const std::vector<HazardEvent>& events() const { return events_; }
        void clear_events() { events_.clear(); }

        void evaluate(const HazardEvent& e) {
            // Simplified escalation policy
            switch (e.level) {
                case HazardLevel::ASIL_D: state_ = SafetyState::EMERGENCY_STOP; break;
                case HazardLevel::ASIL_C: state_ = SafetyState::LIMP_HOME; break;
                case HazardLevel::ASIL_B: state_ = SafetyState::DEGRADED; break;
                default: /* QM / ASIL_A */ break;
            }
        }

        void apply_mitigations() {
            // Example mitigations
            if (state_ == SafetyState::DEGRADED) {
                speed_limit_kph_ = 80; torque_limit_pct_ = 60;
            } else if (state_ == SafetyState::LIMP_HOME) {
                speed_limit_kph_ = 40; torque_limit_pct_ = 30;
            } else if (state_ == SafetyState::EMERGENCY_STOP) {
                speed_limit_kph_ = 0; torque_limit_pct_ = 0;
            } else {
                speed_limit_kph_ = 250; torque_limit_pct_ = 100;
            }
        }

        void display_status() const {
            std::cout << "\n=== Functional Safety Status ===\n";
            std::cout << "State: " << to_string(state_) << "\n";
            std::cout << "Speed limit: " << speed_limit_kph_ << " km/h, Torque limit: " << torque_limit_pct_ << "%\n";
            std::cout << "Goals: " << goals_.size() << ", Diagnostics: " << diags_.size() << ", Events: " << events_.size() << "\n";
        }

        static const char* to_string(SafetyState s) {
            switch (s) {
                case SafetyState::NOMINAL: return "NOMINAL";
                case SafetyState::DEGRADED: return "DEGRADED";
                case SafetyState::LIMP_HOME: return "LIMP_HOME";
                case SafetyState::EMERGENCY_STOP: return "EMERGENCY_STOP";
                default: return "UNKNOWN";
            }
        }

    private:
        SafetyState state_{SafetyState::NOMINAL};
        int speed_limit_kph_{250};
        int torque_limit_pct_{100};
        std::vector<SafetyGoal> goals_{};
        std::vector<Diagnostic> diags_{};
        std::vector<HazardEvent> events_{};
    };
}

#endif // FUNCTIONAL_SAFETY_H

