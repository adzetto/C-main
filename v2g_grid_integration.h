/**
 * @file v2g_grid_integration.h
 * @author adzetto
 * @brief Vehicle-to-Grid (V2G) Integration System
 * @version 1.0
 * @date 2025-08-31
 *
 * @copyright Copyright (c) 2025
 *
 * @details This module provides a framework for Vehicle-to-Grid (V2G) integration,
 *          enabling EVs to communicate with the power grid to both draw power (G2V)
 *          and supply power back to the grid (V2G). It simulates grid conditions and
 *          the decision-making process for bidirectional energy transfer.
 */

#ifndef V2G_GRID_INTEGRATION_H
#define V2G_GRID_INTEGRATION_H

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <random>

namespace v2g_grid_integration {

/**
 * @brief Represents the state of the power grid.
 */
struct GridStatus {
    double frequency_hz;
    double voltage_v;
    double price_per_kwh;
    bool demand_response_event_active;
};

/**
 * @brief Defines the V2G operation mode.
 */
enum class V2GMode {
    IDLE,          // Not connected or not performing any V2G operation
    CHARGING,      // Grid to Vehicle (G2V)
    DISCHARGING    // Vehicle to Grid (V2G)
};

/**
 * @brief Simulates the state of the power grid.
 */
class GridSimulator {
public:
    GridStatus getCurrentStatus() {
        // Simulate fluctuating grid conditions
        std::uniform_real_distribution<double> freq_dist(49.95, 50.05);
        std::uniform_real_distribution<double> volt_dist(225.0, 235.0);
        std::uniform_real_distribution<double> price_dist(0.10, 0.50); // Price per kWh
        
        GridStatus status;
        status.frequency_hz = freq_dist(gen);
        status.voltage_v = volt_dist(gen);
        status.price_per_kwh = price_dist(gen);
        status.demand_response_event_active = (price_dist(gen) > 0.45);

        return status;
    }

private:
    std::default_random_engine gen{static_cast<long unsigned int>(std::chrono::high_resolution_clock::now().time_since_epoch().count())};
};

/**
 * @brief Manages the V2G logic for a single EV.
 */
class V2GManager {
public:
    V2GManager(double battery_capacity_kwh, double max_charge_rate_kw, double max_discharge_rate_kw)
        : battery_capacity(battery_capacity_kwh),
          max_charge_rate(max_charge_rate_kw),
          max_discharge_rate(max_discharge_rate_kw),
          current_mode(V2GMode::IDLE) {}

    /**
     * @brief Decides the V2G operation based on grid status and EV state.
     *
     * @param grid_status The current status of the power grid.
     * @param current_soc The current state of charge of the EV battery (in percent).
     * @param user_preference_soc The minimum SoC the user wants to maintain.
     * @return The decided V2GMode.
     */
    V2GMode decide(const GridStatus& grid_status, double current_soc, double user_preference_soc) {
        std::cout << "\n[V2GManager] Analyzing grid and EV state...\n";
        std::cout << "  Grid Price: " << grid_status.price_per_kwh << " $/kWh\n";
        std::cout << "  Current SoC: " << current_soc << "%\n";
        std::cout << "  User SoC Preference: " << user_preference_soc << "%\n";

        if (grid_status.demand_response_event_active && current_soc > user_preference_soc + 10) {
            std::cout << "  Decision: High grid demand. Discharging to support grid.\n";
            current_mode = V2GMode::DISCHARGING;
        } else if (grid_status.price_per_kwh < 0.15 && current_soc < 95) {
            std::cout << "  Decision: Low grid price. Charging vehicle.\n";
            current_mode = V2GMode::CHARGING;
        } else if (current_soc < user_preference_soc) {
            std::cout << "  Decision: SoC below user preference. Charging vehicle.\n";
            current_mode = V2GMode::CHARGING;
        } else {
            std::cout << "  Decision: No action needed. Idle.\n";
            current_mode = V2GMode::IDLE;
        }
        return current_mode;
    }

    /**
     * @brief Gets the current V2G operation mode.
     */
    V2GMode getCurrentMode() const {
        return current_mode;
    }

private:
    double battery_capacity;
    double max_charge_rate;
    double max_discharge_rate;
    V2GMode current_mode;
};

} // namespace v2g_grid_integration

#endif // V2G_GRID_INTEGRATION_H