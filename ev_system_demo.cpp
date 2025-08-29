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
#include "energy_management.h"
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
#include <memory>

int main() {
    // Initialize all vehicle systems
    std::unique_ptr<ADASSystem> adas_system = std::make_unique<ADASSystem>();
    std::unique_ptr<AdvancedBMS> advanced_bms = std::make_unique<AdvancedBMS>();
    std::unique_ptr<AdvancedPowertrainControl> advanced_powertrain_control = std::make_unique<AdvancedPowertrainControl>();
    std::unique_ptr<AdvancedVehicleDynamicsControl> advanced_vehicle_dynamics = std::make_unique<AdvancedVehicleDynamicsControl>();
    std::unique_ptr<AutonomousDrivingSystem> autonomous_driving = std::make_unique<AutonomousDrivingSystem>();
    std::unique_ptr<BatteryManagementSystem> battery_management_system = std::make_unique<BatteryManagementSystem>(96);
    std::unique_ptr<CANBusSystem> can_bus_system = std::make_unique<CANBusSystem>();
    std::unique_ptr<EnergyManagementSystem> energy_management = std::make_unique<EnergyManagementSystem>();
    std::unique_ptr<HumanMachineInterface> human_machine_interface = std::make_unique<HumanMachineInterface>();
    std::unique_ptr<MachineLearningEngine> machine_learning_engine = std::make_unique<MachineLearningEngine>();
    std::unique_ptr<MotorControlSystem> motor_control_system = std::make_unique<MotorControlSystem>();
    std::unique_ptr<ThermalManagementSystem> thermal_management_system = std::make_unique<ThermalManagementSystem>();
    std::unique_ptr<VehicleConnectivitySystem> vehicle_connectivity = std::make_unique<VehicleConnectivitySystem>("VIN123456789");
    std::unique_ptr<VehicleCybersecurity> vehicle_cybersecurity = std::make_unique<VehicleCybersecurity>();
    std::unique_ptr<VehicleDiagnosticsSystem> vehicle_diagnostics_system = std::make_unique<VehicleDiagnosticsSystem>();

    // Create threads for each system
    std::vector<std::thread> threads;
    threads.emplace_back([&]() { adas_system->run(); });
    threads.emplace_back([&]() { advanced_bms->displayDetailedStatus(); });
    threads.emplace_back([&]() { advanced_powertrain_control->getSystemStatus(); });
    threads.emplace_back([&]() { advanced_vehicle_dynamics->getSystemStatus(); });
    threads.emplace_back([&]() { autonomous_driving->displaySystemStatus(); });
    threads.emplace_back([&]() { battery_management_system->update(); });
    threads.emplace_back([&]() { can_bus_system->run(); });
    threads.emplace_back([&]() { energy_management->displaySystemStatus(); });
    threads.emplace_back([&]() { human_machine_interface->updateDisplay(); });
    threads.emplace_back([&]() { machine_learning_engine->getModelStatus(); });
    threads.emplace_back([&]() { motor_control_system->update(0.0, 0.0); });
    threads.emplace_back([&]() { thermal_management_system->run(); });
    threads.emplace_back([&]() { vehicle_connectivity->start(); });
    threads.emplace_back([&]() { vehicle_cybersecurity->generateSecurityReport(); });
    threads.emplace_back([&]() { vehicle_diagnostics_system->run(); });

    // Wait for all threads to complete
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    return 0;
}