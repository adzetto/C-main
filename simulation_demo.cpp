/**
 * @file simulation_demo.cpp
 * @author adzetto
 * @brief Demo application for the Simulation Toolkit
 * @version 1.0
 * @date 2025-08-31
 *
 * @copyright Copyright (c) 2025
 */

#include "simulation_toolkit.h"
#include <iostream>

using namespace simulation;

void runEVPerformanceSimulation() {
    std::cout << "\n--- Running EV Performance Simulation ---" << std::endl;

    SimulationEngine engine;

    // Create and add models
    auto battery = std::make_shared<BatteryModel>(75.0, 100.0); // 75 kWh capacity, 100% initial SoC
    auto powertrain = std::make_shared<PowertrainModel>();
    auto aux_systems = std::make_shared<AuxSystemsModel>();

    engine.addModel(battery);
    engine.addModel(powertrain);
    engine.addModel(aux_systems);

    // Set up a scenario
    ScenarioManager scenario(engine.getScheduler());
    scenario.loadDrivingCycle();

    // Run the simulation
    engine.run(100.0, 0.1); // Simulate for 100 seconds with a 0.1s time step
}

int main() {
    try {
        std::cout << "=== Simulation Toolkit Demo ===\n";
        std::cout << "Author: adzetto\n";
        std::cout << "Version: 1.0\n";
        std::cout << "Date: 2025-08-31\n";

        runEVPerformanceSimulation();

        std::cout << "\n=== Demo Completed Successfully ===\n";

    } catch (const std::exception& e) {
        std::cerr << "An error occurred during the simulation demo: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}