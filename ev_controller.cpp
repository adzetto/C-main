#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include "ev_communication_protocols.h"
#include "iso15118_ccs.h"

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
        comms_controller.shutdown();
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
    
    if (tesla.getBatteryLevel() < 50.0) {
        tesla.startCharging();
        tesla.displayStatus();
    }
    
    return 0;
}
