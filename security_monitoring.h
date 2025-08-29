/**
 * Security Monitoring & IDS
 * Author: adzetto
 */
#ifndef SECURITY_MONITORING_H
#define SECURITY_MONITORING_H

#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <iostream>

namespace secmon
{
    enum class Severity { INFO, WARN, ALERT, CRITICAL };

    struct Event {
        std::chrono::system_clock::time_point t{std::chrono::system_clock::now()};
        std::string source;
        std::string signature;
        Severity level{Severity::INFO};
        std::string payload;
    };

    class Rules
    {
    public:
        void add(const std::string& sig, Severity sev) { rules_[sig]=sev; }
        Severity evaluate(const std::string& sig) const {
            auto it = rules_.find(sig); return it==rules_.end()? Severity::INFO : it->second;
        }
    private:
        std::unordered_map<std::string,Severity> rules_{};
    };

    class EventPipeline
    {
    public:
        void submit(Event e) { events_.push_back(std::move(e)); }
        const std::vector<Event>& events() const { return events_; }
        void clear() { events_.clear(); }
    private:
        std::vector<Event> events_{};
    };

    class SecurityMonitor
    {
    public:
        void add_rule(const std::string& sig, Severity sev) { rules_.add(sig, sev); }
        void observe(const std::string& src, const std::string& sig, const std::string& payload="") {
            Event e; e.source=src; e.signature=sig; e.payload=payload; e.level = rules_.evaluate(sig);
            pipe_.submit(e);
        }
        void report() const {
            std::cout << "\n=== Security Monitor ===\n";
            for (const auto& e : pipe_.events()) {
                std::cout << e.signature << " from " << e.source << " [" << (int)e.level << "]\n";
            }
        }
        void clear() { pipe_.clear(); }
    private:
        Rules rules_{}; EventPipeline pipe_{};
    };
}

#endif // SECURITY_MONITORING_H

