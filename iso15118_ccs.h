/**
 * ISO15118 + CCS Communication Stack (EV-EVSE)
 * Author: adzetto
 * Notes: High-level scaffold implementing session state machine, SLAC handshake,
 *        certificate handling, and power delivery messages. Header-only stubs
 *        designed to be compile-safe and extendable.
 */
#ifndef ISO15118_CCS_H
#define ISO15118_CCS_H

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <chrono>
#include <optional>
#include <functional>

namespace iso15118
{
    enum class LinkState { DISCONNECTED, SLAC, ASSOCIATED };
    enum class SessionState { IDLE, NEGOTIATING, AUTHENTICATING, CHARGING, TERMINATING, FAULT };
    enum class AuthMode { EIM, PnC };
    enum class EnergyTransferMode { AC, DC };

    struct Certificate {
        std::string subject;
        std::string issuer;
        std::string serial;
        std::chrono::system_clock::time_point not_before;
        std::chrono::system_clock::time_point not_after;
    };

    struct EVSEOffer {
        EnergyTransferMode mode{EnergyTransferMode::DC};
        double max_voltage{1000.0};
        double max_current{500.0};
        double max_power{350.0};
        std::string tariff_id{"default"};
    };

    struct EVRequest {
        EnergyTransferMode mode{EnergyTransferMode::DC};
        double target_soc{80.0};
        double max_voltage{900.0};
        double max_current{400.0};
        AuthMode auth{AuthMode::PnC};
    };

    struct MeterInfo { double voltage{0}; double current{0}; double power{0}; double energy_kwh{0}; };

    class SLAC
    {
    public:
        bool run_association() {
            state_ = LinkState::SLAC;
            // Simulate time cost
            state_ = LinkState::ASSOCIATED;
            return true;
        }
        LinkState state() const { return state_; }
    private:
        LinkState state_{LinkState::DISCONNECTED};
    };

    class SessionManager
    {
    public:
        bool start_session(const EVRequest& req, const EVSEOffer& offer) {
            request_ = req; offer_ = offer; session_state_ = SessionState::NEGOTIATING; return true;
        }
        void set_auth_mode(AuthMode a) { request_.auth = a; }
        void attach_cert(const Certificate& c) { cert_ = c; }
        SessionState state() const { return session_state_; }
        void authenticate() {
            if (!cert_.has_value() && request_.auth == AuthMode::PnC) { session_state_ = SessionState::FAULT; return; }
            session_state_ = SessionState::AUTHENTICATING;
            // accept
            session_state_ = SessionState::CHARGING;
        }
        void terminate() { session_state_ = SessionState::TERMINATING; }
        const EVRequest& request() const { return request_; }
        const EVSEOffer& offer() const { return offer_; }
    private:
        EVRequest request_{};
        EVSEOffer offer_{};
        std::optional<Certificate> cert_{};
        SessionState session_state_{SessionState::IDLE};
    };

    class PowerDelivery
    {
    public:
        void set_limits(double v, double c, double p) { limits_v_ = v; limits_c_ = c; limits_p_ = p; }
        MeterInfo ramp_to(double target_power_kw) {
            MeterInfo m{};
            if (target_power_kw < 0) target_power_kw = 0;
            if (target_power_kw > limits_p_) target_power_kw = limits_p_;
            m.power = target_power_kw;
            // heuristic voltage/current split
            m.voltage = std::min(limits_v_, 800.0);
            m.current = std::min(limits_c_, (target_power_kw * 1000.0) / std::max(1.0, m.voltage));
            m.energy_kwh += m.power / 60.0; // minute step
            return m;
        }
    private:
        double limits_v_{1000.0};
        double limits_c_{500.0};
        double limits_p_{350.0};
    };

    class ISO15118Stack
    {
    public:
        bool connect() { return slac_.run_association(); }
        bool start(const EVRequest& req, const EVSEOffer& offer) { return session_.start_session(req, offer); }
        void provide_certificate(const Certificate& c) { session_.attach_cert(c); }
        void authenticate() { session_.authenticate(); }
        MeterInfo deliver(double kw) { return power_.ramp_to(kw); }
        SessionState state() const { return session_.state(); }
        void teardown() { session_.terminate(); }
    private:
        SLAC slac_{};
        SessionManager session_{};
        PowerDelivery power_{};
    };
}

#endif // ISO15118_CCS_H

