/**
 * @file ev_system_demo.cpp
 * @author adzetto
 * @brief Demo of the integrated Electric Vehicle System
 * @version 0.1
 * @date 2025-08-29
 *
 * @copyright Copyright (c) 2025
 *
 * @details This file contains a demo of the integrated Electric Vehicle System.
 * It initializes and runs all the vehicle systems in parallel threads.
 */

#include "adas_system.h"
#include "advanced_bms.h"
#include "advanced_powertrain_control.h"
#include "advanced_vehicle_dynamics.h"
#include "autonomous_driving.h"
#include "battery_management.h"
#include "can_bus_system.h"
#include "charging_station.cpp"
#include "energy_management.h"
#include "ev_controller.cpp"
#include "human_machine_interface.h"
#include "machine_learning_engine.h"
#include "motor_control_system.h"
#include "thermal_management.h"
#include "vehicle_connectivity.h"
#include "vehicle_cybersecurity.h"
#include "vehicle_diagnostics.h"

#include <iostream>
#include <thread>
#include <vector>

int main() {
    // Initialize all vehicle systems
    ADASSystem adas_system;
    AdvancedBMS advanced_bms;
    AdvancedPowertrainControl advanced_powertrain_control;
    AdvancedVehicleDynamics advanced_vehicle_dynamics;
    AutonomousDriving autonomous_driving;
    BatteryManagementSystem battery_management_system(96);
    CANBusSystem can_bus_system;
    EnergyManagement energy_management;
    HumanMachineInterface human_machine_interface;
    MachineLearningEngine machine_learning_engine;
    MotorControlSystem motor_control_system;
    ThermalManagementSystem thermal_management_system;
    VehicleConnectivity vehicle_connectivity;
    VehicleCybersecurity vehicle_cybersecurity;
    VehicleDiagnosticsSystem vehicle_diagnostics_system;

    // Create threads for each system
    std::vector<std::thread> threads;
    threads.emplace_back([&]() { adas_system.run(); });
    threads.emplace_back([&]() { advanced_bms.run(); });
    threads.emplace_back([&]() { advanced_powertrain_control.run(); });
    threads.emplace_back([&]() { advanced_vehicle_dynamics.run(); });
    threads.emplace_back([&]() { autonomous_driving.run(); });
    threads.emplace_back([&]() { battery_management_system.update(); });
    threads.emplace_back([&]() { can_bus_system.run(); });
    threads.emplace_back([&]() { energy_management.run(); });
    threads.emplace_back([&]() { human_machine_interface.run(); });
    threads.emplace_back([&]() { machine_learning_engine.run(); });
    threads.emplace_back([&]() { motor_control_system.update(0.0, 0.0); });
    threads.emplace_back([&]() { thermal_management_system.run(); });
    threads.emplace_back([&]() { vehicle_connectivity.run(); });
    threads.emplace_back([&]() { vehicle_cybersecurity.run(); });
    threads.emplace_back([&]() { vehicle_diagnostics_system.run(); });

    // Wait for all threads to complete
    for (auto& thread : threads) {
        thread.join();
    }

    return 0;
}
