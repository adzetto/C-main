/**
 * DC Fast Charging Control
 * Author: adzetto
 */
#ifndef DC_FAST_CHARGING_CONTROL_H
#define DC_FAST_CHARGING_CONTROL_H

#include <algorithm>
#include <cmath>

namespace dcfc
{
    struct Limits { double v_max{1000}, c_max{500}, p_max{350}; };
    struct Setpoint { double v{0}, c{0}, p{0}; };
    struct Thermal { double coolant_temp{25.0}; double fan_pct{0}; };

    class PI
    {
    public:
        PI(double kp, double ki) : kp_(kp), ki_(ki) {}
        double step(double set, double act, double dt) {
            double e = set - act; integ_ += e*ki_*dt; return kp_*e + integ_;
        }
    private:
        double kp_, ki_, integ_{0};
    };

    class Controller
    {
    public:
        Controller() : vloop_(0.1, 0.01), cloop_(0.2, 0.02) {}
        void set_limits(const Limits& l) { lim_ = l; }
        Setpoint track_power(double target_kw, double v_meas, double c_meas, double dt) {
            target_kw = std::clamp(target_kw, 0.0, lim_.p_max);
            double v_sp = std::min(lim_.v_max, std::max(200.0, v_meas));
            double p_err = target_kw*1000.0 - v_meas*c_meas;
            double c_sp = std::clamp(c_meas + p_err/(std::max(1.0, v_meas)), 0.0, lim_.c_max);
            // PI loops
            double v_cmd = v_meas + vloop_.step(v_sp, v_meas, dt);
            double c_cmd = c_meas + cloop_.step(c_sp, c_meas, dt);
            Setpoint sp{std::clamp(v_cmd, 0.0, lim_.v_max), std::clamp(c_cmd, 0.0, lim_.c_max), target_kw};
            last_ = sp; return sp;
        }
        Thermal cool_step(double pack_temp_c) {
            Thermal t{}; if (pack_temp_c > 40) t.fan_pct = std::min(100.0, (pack_temp_c-40)*5);
            t.coolant_temp = std::max(20.0, 45.0 - t.fan_pct*0.1); return t;
        }
        Setpoint last() const { return last_; }
    private:
        Limits lim_{}; PI vloop_, cloop_; Setpoint last_{};
    };
}

#endif // DC_FAST_CHARGING_CONTROL_H

