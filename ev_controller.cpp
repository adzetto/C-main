#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include "ev_communication_protocols.h"
#include "iso15118_ccs.h"
#include "realtime_system_monitor.h"
#include "advanced_bms.h"
#include "fleet_telematics.h"
#include "predictive_maintenance.h"
#include "v2g_grid_integration.h"
#include "functional_safety.h"
#include "advanced_ai_systems.h"
#include "advanced_powertrain_control.h"
#include "can_bus_system.h"
#include "vehicle_diagnostics.h"
#include "vehicle_connectivity.h"
#include "security_monitoring.h"
#include "advanced_powertrain_control.h"
#include "ota_secure_update.h"
#include "energy_management.h"
#include "dc_fast_charging_control.h"

class ElectricVehicle {
private:
    std::string model;
    double batteryLevel;
    double maxSpeed;
    double currentSpeed;
    bool isCharging;
    bool isMoving;
    ev_communication::EVCommunicationController comms_controller;
    
public:
    ElectricVehicle(std::string m, double maxSpd) : 
        model(m), batteryLevel(100.0), maxSpeed(maxSpd), 
        currentSpeed(0.0), isCharging(false), isMoving(false) {
            comms_controller.initialize();
        }
    
    ~ElectricVehicle() {
    }
    
    void startEngine() {
        if (batteryLevel > 5.0) {
            std::cout << model << " engine started successfully!\n";
            isMoving = true;
        } else {
            std::cout << "Low battery! Cannot start engine.\n";
        }
    }
    
    void stopEngine() {
        currentSpeed = 0.0;
        isMoving = false;
        std::cout << model << " engine stopped.\n";
    }
    
    void accelerate(double speed) {
        if (!isMoving) {
            std::cout << "Start the engine first!\n";
            return;
        }
        
        if (speed <= maxSpeed && batteryLevel > 0) {
            currentSpeed = speed;
            batteryLevel -= speed * 0.01;
            std::cout << "Accelerating to " << speed << " km/h\n";
            std::cout << "Battery: " << batteryLevel << "%\n";
        } else if (speed > maxSpeed) {
            std::cout << "Speed limit exceeded! Max speed: " << maxSpeed << " km/h\n";
        } else {
            std::cout << "Battery empty!\n";
        }
    }
    
    void brake() {
        currentSpeed *= 0.5;
        std::cout << "Braking... Current speed: " << currentSpeed << " km/h\n";
        
        if (currentSpeed < 5.0) {
            currentSpeed = 0.0;
            std::cout << "Vehicle stopped.\n";
        }
    }
    
    void startCharging() {
        if (batteryLevel < 100.0) {
            isCharging = true;
            std::cout << "Charging started...\n";
            
            while (batteryLevel < 100.0 && isCharging) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                batteryLevel += 0.5;
                std::cout << "Battery: " << batteryLevel << "%\r" << std::flush;
            }
            
            isCharging = false;
            std::cout << "\nCharging complete!\n";
        } else {
            std::cout << "Battery already full!\n";
        }
    }
    
    void stopCharging() {
        isCharging = false;
        std::cout << "Charging stopped.\n";
    }
    
    void displayStatus() {
        std::cout << "\n=== " << model << " Status ===\n";
        std::cout << "Battery Level: " << batteryLevel << "%\n";
        std::cout << "Current Speed: " << currentSpeed << " km/h\n";
        std::cout << "Max Speed: " << maxSpeed << " km/h\n";
        std::cout << "Engine: " << (isMoving ? "ON" : "OFF") << "\n";
        std::cout << "Charging: " << (isCharging ? "YES" : "NO") << "\n";
        
        auto comm_status = comms_controller.getCommStatus();
        std::cout << "--- Communication Status ---\n";
        for(const auto& pair : comm_status) {
            std::cout << pair.first << ": " << pair.second << "\n";
        }
        std::cout << "========================\n\n";
    }
    
    double getBatteryLevel() const { return batteryLevel; }
    double getCurrentSpeed() const { return currentSpeed; }
    bool getMovingStatus() const { return isMoving; }

    void shutdownCommunications() {
        // Ensure communication threads are stopped deterministically
        comms_controller.shutdown();
    }

    void simulateChargingSession() {
        std::cout << "\n--- Simulating ISO 15118 Charging Session ---\n";
        iso15118::ISO15118Stack iso_stack;
        if (!iso_stack.connect()) {
            std::cout << "SLAC association failed!\n";
            return;
        }

        iso15118::EVRequest ev_req;
        iso15118::EVSEOffer evse_offer;
        iso_stack.start(ev_req, evse_offer);

        iso15118::Certificate cert = {"EV_CERT", "OEM_CA", "12345", std::chrono::system_clock::now(), std::chrono::system_clock::now() + std::chrono::hours(24*365)};
        iso_stack.provide_certificate(cert);
        iso_stack.authenticate();

        if (iso_stack.state() == iso15118::SessionState::CHARGING) {
            std::cout << "Authentication successful. Starting charge.\n";
            double target_kw = 50.0;
            iso15118::MeterInfo meter_info = iso_stack.deliver(target_kw);
            std::cout << "Charging at " << meter_info.power << " kW\n";
        } else {
            std::cout << "Authentication failed!\n";
        }

        iso_stack.teardown();
        std::cout << "--- End Charging Session ---\n";
    }

    void simulateAdvancedBMS() {
        std::cout << "\n--- Simulating Advanced BMS ---\n";
        AdvancedBMS bms;
        bms.setPackCurrent(isCharging ? 25.0 : -50.0);
        bms.updateAllCells();
        bms.performCellBalancing();
        bms.checkSystemFaults();
        bms.displayDetailedStatus();
        bms.displayCellDetails(0, 10);
        std::cout << "--- End Advanced BMS---\n";
    }

    void simulateFleetTelematics() {
        std::cout << "\n--- Simulating Fleet Telematics ---\n";
        fleet_telematics::FleetManager fleet_manager;
        fleet_manager.addVehicle(model);
        fleet_manager.startAllTelematics("tcp://telematics.example.com:1234", 5);

        auto vehicle_telematics = fleet_manager.getVehicle(model);
        if (vehicle_telematics) {
            fleet_telematics::VehicleDataSnapshot snapshot;
            snapshot.vehicleId = model;
            snapshot.location = {47.6, -122.3, 50.0, currentSpeed};
            snapshot.battery_soc = batteryLevel;
            snapshot.motor_temp_c = 65.0;
            snapshot.system_health_score = 95.0;
            vehicle_telematics->updateVehicleData(snapshot);
        }

        std::this_thread::sleep_for(std::chrono::seconds(6));
        fleet_manager.stopAllTelematics();
        std::cout << "--- End Fleet Telematics ---\n";
    }

    void simulatePredictiveMaintenance() {
        std::cout << "\n--- Simulating Predictive Maintenance ---\n";
        predictive_maintenance::MaintenanceManager maintenance_manager;
        maintenance_manager.trainAllModels();

        predictive_maintenance::ComponentDataPoint battery_data = {2500, 45.0, 60.0, 5};
        auto battery_prediction = maintenance_manager.analyzeComponent("BatterySystem", battery_data);
        std::cout << "Battery RUL: " << battery_prediction.remaining_useful_life_hours << " hours, Recommendation: " << battery_prediction.recommendation << "\n";

        predictive_maintenance::ComponentDataPoint motor_data = {2500, 80.0, 75.0, 2};
        auto motor_prediction = maintenance_manager.analyzeComponent("MotorAssembly", motor_data);
        std::cout << "Motor RUL: " << motor_prediction.remaining_useful_life_hours << " hours, Recommendation: " << motor_prediction.recommendation << "\n";
        std::cout << "--- End Predictive Maintenance ---\n";
    }

    void simulateV2G() {
        std::cout << "\n--- Simulating V2G Grid Integration ---\n";
        v2g_grid_integration::GridSimulator grid_sim;
        v2g_grid_integration::V2GManager v2g_manager(100.0, 50.0, 10.0);

        for (int i = 0; i < 3; ++i) {
            auto grid_status = grid_sim.getCurrentStatus();
            v2g_manager.decide(grid_status, batteryLevel, 50.0);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "--- End V2G Grid Integration---\n";
    }

    void simulateFunctionalSafety() {
        std::cout << "\n--- Simulating Functional Safety ---\n";
        functional_safety::SafetyManager safety_manager("BrakingSystem", functional_safety::ASIL::D);

        safety_manager.execute([this]() {
            std::cout << "Executing critical braking operation...\n";
            this->brake();
        }, functional_safety::ASIL::D);

        // Simulate a fault
        safety_manager.reportError("Sensor reading out of range");
        std::cout << "Safety state after error: " << static_cast<int>(safety_manager.getSafetyState()) << "\n";

        safety_manager.execute([this]() {
            std::cout << "Attempting to execute operation in degraded state...\n";
        }, functional_safety::ASIL::C);

        std::cout << "--- End Functional Safety ---\n";
    }

    void simulateAdvancedAI() {
        std::cout << "\n--- Simulating Advanced AI Systems ---\n";
        ai_systems::AISystemsController ai_controller;
        ai_controller.initialize();

        // Simulate computer vision
        ai_systems::ImageData image(1280, 720);
        auto detected_objects = ai_controller.getComputerVision()->processImage(image);
        std::cout << "Detected " << detected_objects.size() << " objects.\n";

        // Simulate NLP
        auto nlp_result = ai_controller.getNLPSystem()->processVoiceCommand("Navigate to home");
        std::cout << "NLP Intent: " << nlp_result["intent"] << ", Response: " << nlp_result["response"] << "\n";

        // Simulate decision making
        std::map<std::string, double> vehicle_state = {{"speed", currentSpeed}, {"battery", batteryLevel}};
        std::string decision = ai_controller.getDecisionEngine()->makeDecision(ai_systems::DrivingScenario::CITY_DRIVING, detected_objects, vehicle_state);
        std::cout << "AI Decision: " << decision << "\n";

        ai_controller.shutdown();
        std::cout << "--- End Advanced AI Systems ---\n";
    }

    void simulateAdvancedPowertrain() {
        std::cout << "\n--- Simulating Advanced Powertrain Control ---\n";
        AdvancedPowertrainControl powertrain_control;
        powertrain_control.enablePowertrain();
        powertrain_control.setDriveMode(AdvancedPowertrainControl::DriveMode::SPORT);
        float wheel_speeds[4] = {50.0f, 50.0f, 50.0f, 50.0f};
        powertrain_control.updateVehicleDynamics(50.0f, wheel_speeds, 0.1f, 0.05f);
        powertrain_control.requestTorque(200.0f);
        powertrain_control.getSystemStatus();
        powertrain_control.disablePowertrain();
        std::cout << "--- End Advanced Powertrain Control ---\n";
    }

        void simulateCANBus() {
                std::cout << "\n--- Simulating CAN Bus ---\n";
                CANBusSystem can;
                CANMessageManager mgr;
                CANNetworkManager net;

                // Encode a few signals
                CANMessage speedMsg = mgr.encode("VehicleSpeed", currentSpeed);
                CANMessage socMsg = mgr.encode("BatterySOC", batteryLevel);

                // Send over network (stubbed)
                net.send(speedMsg);
                net.send(socMsg);

                // Receive and decode (stubbed)
                auto rx = net.receive();
                (void)rx; // avoid unused warning in stub

                double decodedSpeed = mgr.decode(speedMsg, "VehicleSpeed");
                double decodedSOC = mgr.decode(socMsg, "BatterySOC");

                std::cout << "Decoded VehicleSpeed: " << decodedSpeed << " km/h\n";
                std::cout << "Decoded BatterySOC: " << decodedSOC << " %\n";
                std::cout << "--- End CAN Bus ---\n";
        }

            void simulateDiagnostics() {
                    std::cout << "\n--- Simulating Vehicle Diagnostics ---\n";
                    VehicleDiagnosticsSystem diag;
                    DTCManager dtc_mgr;
                    DataLogger logger;
                    RemoteDiagnostics remote;

                    // Set a few DTCs (stubbed)
                    dtc_mgr.set_dtc({"P0A1F", "Battery control module fault"});
                    dtc_mgr.set_dtc({"C0035", "Left front wheel speed sensor"});

                    // Log some data
                    logger.log_data("Speed=" + std::to_string(currentSpeed));
                    logger.log_data("SOC=" + std::to_string(batteryLevel));

                    // Remote request
                    remote.handle_request("READ_DTC");

                    // Fetch and print (stubbed empty in header)
                    auto all = dtc_mgr.get_all_dtcs();
                    std::cout << "Recorded DTC count: " << all.size() << "\n";
                    std::cout << "--- End Vehicle Diagnostics ---\n";
            }

                void simulateConnectivity() {
                        std::cout << "\n--- Simulating Vehicle Connectivity ---\n";
                        VehicleConnectivitySystem vcs(model);
                        vcs.start();
                        // Update vehicle status a few times
                        vcs.updateVehicleStatus(currentSpeed, 0.0);
                        vcs.requestTrafficData();
                        vcs.requestWeatherData();
                        vcs.checkForUpdates();
                        vcs.displaySystemStatus();
                        vcs.stop();
                        std::cout << "--- End Vehicle Connectivity ---\n";
                }

                    void simulateSecurityMonitoring() {
                            std::cout << "\n--- Simulating Security Monitoring ---\n";
                            secmon::SecurityMonitor mon;
                            mon.add_rule("CAN_FLOOD", secmon::Severity::CRITICAL);
                            mon.add_rule("UNAUTH_REMOTE", secmon::Severity::ALERT);
                            mon.add_rule("LOW_BATT_TAMPER", secmon::Severity::WARN);

                            mon.observe("CAN", "CAN_FLOOD");
                            mon.observe("Connectivity", "UNAUTH_REMOTE", "suspicious token");
                            mon.observe("Sensors", "LOW_BATT_TAMPER");

                            mon.report();
                            mon.clear();
                            std::cout << "--- End Security Monitoring ---\n";
                    }

    void simulateOTAUpdate() {
        std::cout << "\n--- Simulating OTA Secure Update ---\n";
        ota_secure_update::OTAManager ota("1.0.0");
        ota_secure_update::UpdateBundle bundle{
            "1.1.0",
            "https://updates.example.com/ev/firmware_1_1_0.bin",
            "SIG_FAKE_ABC123",
            "HASH_FAKE_ABC123"
        };
        ota.startUpdate(bundle);
        std::cout << "Current firmware version: " << ota.getCurrentVersion() << "\n";
        std::cout << "--- End OTA Secure Update ---\n";
    }

    void simulateEnergyManagement() {
        std::cout << "\n--- Simulating Energy Management ---\n";
        EnergyManagementSystem ems(82.0);
        ems.setSOC(67.5);
        ems.setDrivingMode(DrivingMode::ECO);

    // Build a simple route
    std::vector<RouteSegment> route;
    RouteSegment s1; s1.distance = 15.0; s1.elevation = 50.0; s1.speedLimit = 100.0; s1.trafficFactor = 1.0; s1.roadType = "highway"; s1.weatherFactor = 1.0; route.push_back(s1);
    RouteSegment s2; s2.distance = 5.0; s2.elevation = -20.0; s2.speedLimit = 60.0; s2.trafficFactor = 1.2; s2.roadType = "city"; s2.weatherFactor = 1.1; route.push_back(s2);
    RouteSegment s3; s3.distance = 30.0; s3.elevation = 120.0; s3.speedLimit = 110.0; s3.trafficFactor = 0.9; s3.roadType = "highway"; s3.weatherFactor = 1.0; route.push_back(s3);
        auto prediction = ems.planRoute(route);

        // Grid conditions and charging plan
        ems.updateGridConditions(0.85, 0.18, 0.35);
        ems.planChargingSession(80.0, 1.5);
        ems.executeChargingSession();

        // Simulate power usage snapshot
        ems.updateEnergyConsumption(45.0, 5.0);
        ems.displaySystemStatus();
        std::cout << "--- End Energy Management ---\n";
    }

    void simulateDCFastChargingControl() {
        std::cout << "\n--- Simulating DC Fast Charging Control ---\n";
        dcfc::Controller ctrl;
        dcfc::Limits lim{920.0, 500.0, 250.0};
        ctrl.set_limits(lim);
        double v = 400.0, c = 0.0; // measurements
        for (int i = 0; i < 5; ++i) {
            auto sp = ctrl.track_power(150.0, v, c, 0.1);
            // naive plant: move v,c slightly toward commands
            v += (sp.v - v) * 0.25;
            c += (sp.c - c) * 0.25;
            auto th = ctrl.cool_step(35.0 + i*2);
            std::cout << "Step " << i
                      << ": v_cmd=" << sp.v
                      << ", c_cmd=" << sp.c
                      << ", fan=" << th.fan_pct << "%\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::cout << "--- End DC Fast Charging Control ---\n";
    }

    void simulateMonitoring() {
        std::cout << "\n--- Simulating Real-Time Monitoring ---\n";
        system_monitor::RealTimeSystemMonitor monitor;
        monitor.initialize();
        monitor.start();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << monitor.getSystemStatus();
        monitor.stop();
        monitor.shutdown();
        std::cout << "--- End Monitoring---\n";
    }

    void simulateCommunications() {
        std::cout << "\n--- Simulating Communications ---\n";
        comms_controller.sendEmergencyAlert("ACCIDENT_AHEAD", "Multi-vehicle collision on I-5 North");
        
        if (comms_controller.getV2IComm() && comms_controller.getV2IComm()->isConnected()) {
            comms_controller.getV2IComm()->requestTrafficSignalInfo("INTERSECTION_123");
        }

        if (comms_controller.getCellularModem() && comms_controller.getCellularModem()->isConnected()) {
            std::map<std::string, double> telemetry_data = {
                {"speed", currentSpeed},
                {"battery", batteryLevel},
                {"odometer", 25100.5}
            };
            comms_controller.getCellularModem()->sendTelemetryData(telemetry_data);
        }
        std::cout << "--- End Simulation ---\n";
    }
};

int main() {
    ElectricVehicle tesla("Tesla Model 3", 250.0);
    
    std::cout << "Electric Vehicle Controller\n";
    std::cout << "==========================\n\n";
    
    tesla.displayStatus();
    
    tesla.startEngine();
    tesla.accelerate(80);
    tesla.displayStatus();
    
    tesla.simulateCommunications();
    
    tesla.accelerate(150);
    tesla.displayStatus();
    
    tesla.brake();
    tesla.brake();
    tesla.displayStatus();
    
    tesla.stopEngine();

    tesla.simulateChargingSession();

    tesla.simulateAdvancedBMS();

    tesla.simulateFleetTelematics();

    tesla.simulatePredictiveMaintenance();

    tesla.simulateV2G();

    tesla.simulateFunctionalSafety();

    tesla.simulateAdvancedAI();

    tesla.simulateAdvancedPowertrain();

        tesla.simulateCANBus();

        tesla.simulateDiagnostics();

            tesla.simulateConnectivity();

            tesla.simulateSecurityMonitoring();

    tesla.simulateOTAUpdate();
    tesla.simulateEnergyManagement();
    tesla.simulateDCFastChargingControl();
    
    if (tesla.getBatteryLevel() < 50.0) {
        tesla.startCharging();
        tesla.displayStatus();
    }

    tesla.simulateMonitoring();
    // Gracefully stop communications before exit
    tesla.shutdownCommunications();
    
    return 0;
}
