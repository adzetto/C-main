/**
 * Localization and Sensor Fusion (EKF/UKF, Lidar, Radar)
 * Author: adzetto
 */
#ifndef LOCALIZATION_SENSORS_H
#define LOCALIZATION_SENSORS_H

#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <optional>

namespace localization
{
    struct Pose { double x{0}, y{0}, yaw{0}; double vx{0}, vy{0}, yaw_rate{0}; };
    struct Imu { double ax{0}, ay{0}, gz{0}; };
    struct Gps { double x{0}, y{0}; double cov_x{1}, cov_y{1}; bool valid{false}; };

    class EKF
    {
    public:
        EKF() { // init state and covariance
            for (auto& r: P_) for (double& c: r) c = 0; P_[0][0]=P_[1][1]=P_[2][2]=1; }
        void predict(double dt, const Imu& imu) {
            // very simple kinematic yaw-rate model
            state_.x += state_.vx * dt; state_.y += state_.vy * dt; state_.yaw += imu.gz * dt;
            // inflate covariance
            P_[0][0]+=0.1; P_[1][1]+=0.1; P_[2][2]+=0.05;
        }
        void update_gps(const Gps& gps) {
            if (!gps.valid) return; // position update
            double kx = P_[0][0]/(P_[0][0]+gps.cov_x);
            double ky = P_[1][1]/(P_[1][1]+gps.cov_y);
            state_.x += kx*(gps.x - state_.x); state_.y += ky*(gps.y - state_.y);
            P_[0][0]*=(1-kx); P_[1][1]*=(1-ky);
        }
        Pose state() const { return state_; }
    private:
        Pose state_{}; std::array<std::array<double,6>,6> P_{};
    };

    class LidarScanMatcher
    {
    public:
        Pose register_frame(const Pose& initial) const { return initial; }
    };

    class RadarTracker
    {
    public:
        void step() {}
    };
}

#endif // LOCALIZATION_SENSORS_H

