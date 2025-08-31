/**
 * @file simulation_demo.cpp
 * @author adzetto
 * @brief Demo application for the Simulation Toolkit and other utilities
 * @version 1.5
 * @date 2025-08-31
 *
 * @copyright Copyright (c) 2025
 */

#include "simulation_toolkit.h"
#include "fleet_telematics.h"
#include "predictive_maintenance.h"
#include "functional_safety.h"
#include "v2g_grid_integration.h"
#include "ota_secure_update.h"
#include <iostream>

using namespace simulation;
using namespace fleet_telematics;
using namespace predictive_maintenance;
using namespace functional_safety;
using namespace v2g_grid_integration;
using namespace ota_secure_update;

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
        for (int i = 0; i < 4; ++i) {
            VehicleDataSnapshot data1, data2;
            data1.vehicleId = "EV-001";
            data1.location = {40.7128, -74.0060, 10.0, 50.5};
            data1.battery_soc = 80.0 - i * 5.0;
            data1.motor_temp_c = 65.0 + i * 2.0;
            data1.system_health_score = 95.0 - i * 2.0;

            data2.vehicleId = "EV-002";
            data2.location = {34.0522, -118.2437, 100.0, 80.0};
            data2.battery_soc = 75.0 - i * 3.0;
            data2.motor_temp_c = 70.0 + i * 1.0;
            data2.system_health_score = 98.0 - i * 1.0;

            vehicle1->updateVehicleData(data1);
            vehicle2->updateVehicleData(data2);

            std::cout << "\n[MainDemo] Updated vehicle data. Waiting for telematics cycle...\n";
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }

    fleetManager.stopAllTelematics();
}

void runPredictiveMaintenanceDemo() {
    std::cout << "\n--- Running Predictive Maintenance Demo ---" << std::endl;

    MaintenanceManager maintenanceManager;
    maintenanceManager.trainAllModels();

    std::cout << "\n--- Analyzing Component Health ---" << std::endl;

    // Simulate analyzing a few components with different health states
    ComponentDataPoint battery_data_healthy = {1200, 55.0, 50.0, 5};
    ComponentDataPoint battery_data_warning = {4800, 75.0, 65.0, 25};
    ComponentDataPoint motor_data_critical = {8000, 95.0, 80.0, 100};

    auto prediction1 = maintenanceManager.analyzeComponent("BatterySystem", battery_data_healthy);
    auto prediction2 = maintenanceManager.analyzeComponent("BatterySystem", battery_data_warning);
    auto prediction3 = maintenanceManager.analyzeComponent("MotorAssembly", motor_data_critical);

    auto print_prediction = [](const RULPrediction& p) {
        if (!p.component_id.empty()) {
            std::cout << "\nPrediction for: " << p.component_id << std::endl;
            std::cout << "  Remaining Useful Life: " << p.remaining_useful_life_hours << " hours" << std::endl;
            std::cout << "  Confidence: " << p.confidence_percent << "%" << std::endl;
            std::cout << "  Recommendation: " << p.recommendation << std::endl;
        }
    };

    print_prediction(prediction1);
    print_prediction(prediction2);
    print_prediction(prediction3);
}

void runFunctionalSafetyDemo() {
    std::cout << "\n--- Running Functional Safety Demo ---" << std::endl;

    SafetyManager brakeManager("BrakingSystem", ASIL::D);
    SafetyCriticalValue<int> brake_pressure(0);

    auto apply_brakes = [&](int pressure) {
        std::cout << "Attempting to apply brakes with pressure: " << pressure << std::endl;
        if (pressure > 1000) {
            throw std::runtime_error("Pressure sensor failure!");
        }
        brake_pressure.set(pressure);
        std::cout << "Brakes applied successfully.\n";
    };

    // 1. Successful execution
    brakeManager.execute([&](){ apply_brakes(100); }, ASIL::D);
    int p_val;
    if (brake_pressure.get(p_val)) {
        std::cout << "Brake pressure verified: " << p_val << std::endl;
    }

    // 2. Execution that causes an error
    std::cout << "\n--- Simulating a fault ---" << std::endl;
    brakeManager.execute([&](){ apply_brakes(2000); }, ASIL::D);
    std::cout << "Current safety state: " << static_cast<int>(brakeManager.getSafetyState()) << std::endl;

    // 3. Attempt to execute in a non-nominal state
    std::cout << "\n--- Attempting to operate in safe state ---" << std::endl;
    brakeManager.execute([&](){ apply_brakes(50); }, ASIL::D);
}

void runV2GDemo() {
    std::cout << "\n--- Running V2G Grid Integration Demo ---" << std::endl;

    GridSimulator grid;
    V2GManager v2g_manager(75.0, 50.0, 10.0); // 75kWh battery, 50kW charge, 10kW discharge

    double current_soc = 60.0;
    double user_soc_pref = 50.0;

    for (int i = 0; i < 5; ++i) {
        GridStatus status = grid.getCurrentStatus();
        V2GMode mode = v2g_manager.decide(status, current_soc, user_soc_pref);

        switch(mode) {
            case V2GMode::CHARGING:
                current_soc += 2.0; // Simulate charging
                break;
            case V2GMode::DISCHARGING:
                current_soc -= 1.0; // Simulate discharging
                break;
            case V2GMode::IDLE:
                // No change
                break;
        }
        current_soc = std::max(0.0, std::min(100.0, current_soc));
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void runOTADemo() {
    std::cout << "\n--- Running OTA Secure Update Demo ---" << std::endl;

    OTAManager otaManager("1.0.0");
    std::cout << "Initial software version: " << otaManager.getCurrentVersion() << std::endl;

    UpdateBundle bundle;
    bundle.version = "1.1.0";
    bundle.url = "https://updates.example.com/ev-firmware/1.1.0";
    bundle.hash = "mock_hash_string_for_1.1.0";
    bundle.signature = "mock_signature_string_for_1.1.0";

    otaManager.startUpdate(bundle);

    std::cout << "Final software version: " << otaManager.getCurrentVersion() << std::endl;
}

int main(int argc, char *argv[]) {
    try {
        std::cout << "=== Simulation & Utilities Demo ===\n";
        std::cout << "Author: adzetto\n";
        std::cout << "Version: 1.5\n";
        std::cout << "Date: 2025-08-31\n";

        if (argc > 1) {
            std::string arg = argv[1];
            if (arg == "telematics") {
                runFleetTelematicsDemo();
            } else if (arg == "maintenance") {
                runPredictiveMaintenanceDemo();
            } else if (arg == "safety") {
                runFunctionalSafetyDemo();
            } else if (arg == "v2g") {
                runV2GDemo();
            } else if (arg == "ota") {
                runOTADemo();
            } else {
                runEVPerformanceSimulation();
            }
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
