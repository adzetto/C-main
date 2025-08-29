/**
 * Vehicle-to-Grid (V2G) and Grid Services Integration
 * Author: adzetto
 */
#ifndef V2G_GRID_INTEGRATION_H
#define V2G_GRID_INTEGRATION_H

#include <iostream>
#include <vector>
#include <string>
#include <functional>
#include <chrono>
#include <map>

namespace v2g
{
    enum class Service { NONE, FFR, FCR, DR, TOU_OPT, BACKUP_POWER };
    enum class Direction { CHARGE, DISCHARGE, HOLD };

    struct GridSignal { double frequency{50.0}; double price_cents{10.0}; double carbon_intensity{200.0}; };
    struct Capability { double max_charge_kw{11}; double max_discharge_kw{7.4}; double soc_min{20}; double soc_max{90}; };

    struct DispatchCommand {
        Service service{Service::NONE};
        Direction direction{Direction::HOLD};
        double target_power_kw{0.0};
        std::string rationale;
    };

    class Optimizer
    {
    public:
        DispatchCommand decide(const GridSignal& sig, const Capability& cap, double soc)
        {
            DispatchCommand cmd{};
            // Simple policy: frequency support near 49.9/50.1
            if (sig.frequency < 49.9 && soc > cap.soc_min + 5) {
                cmd.service = Service::FCR; cmd.direction = Direction::DISCHARGE; cmd.target_power_kw = std::min(cap.max_discharge_kw, 3.5);
                cmd.rationale = "Under-frequency support";
            } else if (sig.frequency > 50.1 && soc < cap.soc_max - 5) {
                cmd.service = Service::FCR; cmd.direction = Direction::CHARGE; cmd.target_power_kw = std::min(cap.max_charge_kw, 3.5);
                cmd.rationale = "Over-frequency absorption";
            } else if (sig.price_cents > 40 && soc > cap.soc_min + 10) {
                cmd.service = Service::DR; cmd.direction = Direction::DISCHARGE; cmd.target_power_kw = std::min(cap.max_discharge_kw, 2.0);
                cmd.rationale = "Peak price discharge";
            } else if (sig.price_cents < 10 && soc < cap.soc_max - 10) {
                cmd.service = Service::TOU_OPT; cmd.direction = Direction::CHARGE; cmd.target_power_kw = std::min(cap.max_charge_kw, 7.0);
                cmd.rationale = "Low price charge";
            } else {
                cmd.direction = Direction::HOLD; cmd.rationale = "No profitable action";
            }
            return cmd;
        }
    };

    class AggregatorClient
    {
    public:
        void register_vehicle(const std::string& vin, const Capability& cap) { caps_[vin] = cap; }
        void send_availability(const std::string& vin, double soc) { (void)vin; (void)soc; }
        GridSignal get_signal() const { return signal_; }
        void set_signal(const GridSignal& s) { signal_ = s; }
    private:
        std::map<std::string, Capability> caps_{};
        GridSignal signal_{};
    };

    class V2GController
    {
    public:
        V2GController(std::string vin, Capability cap) : vin_(std::move(vin)), cap_(cap) {}
        DispatchCommand step(const GridSignal& sig, double soc) { return opt_.decide(sig, cap_, soc); }
        const Capability& capability() const { return cap_; }
    private:
        std::string vin_;
        Capability cap_{};
        Optimizer opt_{};
    };
}

#endif // V2G_GRID_INTEGRATION_H

