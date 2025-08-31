/**
 * @file simulation_demo.cpp
 * @author adzetto
 * @brief Demo application for the Simulation Toolkit and other utilities
 * @version 1.1
 * @date 2025-08-31
 *
 * @copyright Copyright (c) 2025
 */

#include "simulation_toolkit.h"
#include "fleet_telematics.h"
#include <iostream>

using namespace simulation;
using namespace fleet_telematics;

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

void runFleetTelematicsDemo() {
    std::cout << "\n--- Running Fleet Telematics Demo ---" << std::endl;

    FleetManager fleetManager;
    fleetManager.addVehicle("EV-001");
    fleetManager.addVehicle("EV-002");

    fleetManager.startAllTelematics("tcp://telematics.server.com:1234", 5);

    auto vehicle1 = fleetManager.getVehicle("EV-001");
    auto vehicle2 = fleetManager.getVehicle("EV-002");

    if (vehicle1 && vehicle2) {
        // Simulate some data updates for 20 seconds
        for (int i = 0; i < 10; ++i) {
            VehicleDataSnapshot data1, data2;
            data1.vehicleId = "EV-001";
            data1.location = {40.7128, -74.0060, 10.0, 50.5};
            data1.battery_soc = 80.0 - i * 2.0;
            data1.motor_temp_c = 65.0 + i;
            data1.system_health_score = 95.0 - i;

            data2.vehicleId = "EV-002";
            data2.location = {34.0522, -118.2437, 100.0, 80.0};
            data2.battery_soc = 75.0 - i * 1.5;
            data2.motor_temp_c = 70.0 + i * 0.5;
            data2.system_health_score = 98.0 - i * 0.5;

            vehicle1->updateVehicleData(data1);
            vehicle2->updateVehicleData(data2);

            std::cout << "\n[MainDemo] Updated vehicle data. Waiting for telematics cycle...\n";
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    fleetManager.stopAllTelematics();
}

int main(int argc, char *argv[]) {
    try {
        std::cout << "=== Simulation & Utilities Demo ===\n";
        std::cout << "Author: adzetto\n";
        std::cout << "Version: 1.1\n";
        std::cout << "Date: 2025-08-31\n";

        if (argc > 1 && std::string(argv[1]) == "telematics") {
            runFleetTelematicsDemo();
        } else {
            runEVPerformanceSimulation();
        }

        std::cout << "\n=== Demo Completed Successfully ===\n";

    } catch (const std::exception& e) {
        std::cerr << "An error occurred during the demo: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
