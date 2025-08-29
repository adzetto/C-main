/**
 * Comprehensive Electric Vehicle System Demo and Test Suite
 * Author: adzetto
 * Integrates: BMS, ADAS, CAN Bus, Motor Control, Thermal Management, Diagnostics
 */

#include "advanced_bms.h"
#include "adas_system.h"
#include "can_bus_system.h"
#include "motor_control_system.h"
#include "thermal_management.h"
#include "vehicle_diagnostics.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <random>
#include <iomanip>

class EVSystemIntegrator {
private:
    // Core Systems
    std::unique_ptr<AdvancedBMS> bms;
    std::unique_ptr<ADAS> adas;
    std::unique_ptr<EVCANSystem> canSystem;
    std::unique_ptr<MotorController> motorController;
    std::unique_ptr<RegenerativeBraking> regenBraking;
    std::unique_ptr<TorqueVectoring> torqueVectoring;
    std::unique_ptr<ThermalManagementSystem> thermalSystem;
    std::unique_ptr<VehicleDiagnosticsSystem> diagnostics;
    
    // Vehicle State
    double vehicleSpeed;
    double acceleratorPedal;
    double brakePedal;
    double steeringAngle;
    double batterySOC;
    double batteryVoltage;
    double batteryCurrent;
    double batteryTemperature;
    double motorTemperature;
    double ambientTemperature;
    
    // Simulation parameters
    std::mt19937 randomEngine;
    std::uniform_real_distribution<double> noiseDist;
    bool systemRunning;
    
public:
    EVSystemIntegrator() : vehicleSpeed(0), acceleratorPedal(0), brakePedal(0),
                          steeringAngle(0), batterySOC(80.0), batteryVoltage(370.0),
                          batteryCurrent(0), batteryTemperature(25.0), motorTemperature(70.0),
                          ambientTemperature(22.0), randomEngine(std::random_device{}()),
                          noiseDist(-0.1, 0.1), systemRunning(false) {
        
        initializeSystems();
    }
    
    void initializeSystems() {
        std::cout << "Initializing Electric Vehicle Systems...\n";
        std::cout << "======================================\n";
        
        // Initialize BMS
        bms = std::make_unique<AdvancedBMS>(96, 50.0);
        std::cout << "✓ Advanced Battery Management System initialized\n";
        
        // Initialize ADAS
        adas = std::make_unique<ADAS>();
        adas->setMode(ADASMode::ACTIVE);
        std::cout << "✓ ADAS System initialized\n";
        
        // Initialize CAN System
        canSystem = std::make_unique<EVCANSystem>();
        canSystem->start();
        std::cout << "✓ CAN Bus System initialized\n";
        
        // Initialize Motor Controller
        motorController = std::make_unique<MotorController>(MotorType::PMSM);
        motorController->enable();
        std::cout << "✓ Motor Controller initialized\n";
        
        // Initialize Regenerative Braking
        regenBraking = std::make_unique<RegenerativeBraking>();
        regenBraking->setMode(RegenerativeMode::MODERATE);
        std::cout << "✓ Regenerative Braking System initialized\n";
        
        // Initialize Torque Vectoring
        torqueVectoring = std::make_unique<TorqueVectoring>();
        torqueVectoring->enable();
        std::cout << "✓ Torque Vectoring System initialized\n";
        
        // Initialize Thermal Management
        thermalSystem = std::make_unique<ThermalManagementSystem>();
        thermalSystem->setMode(CoolingMode::NORMAL);
        thermalSystem->enableHVAC();
        thermalSystem->setHVACMode(HVACMode::AUTO);
        std::cout << "✓ Thermal Management System initialized\n";
        
        // Initialize Diagnostics
        diagnostics = std::make_unique<VehicleDiagnosticsSystem>();
        std::cout << "✓ Vehicle Diagnostics System initialized\n";
        
        std::cout << "======================================\n";
        std::cout << "All systems initialized successfully!\n\n";
    }
    
    void startSimulation() {
        systemRunning = true;
        std::cout << "Starting Electric Vehicle System Simulation...\n\n";
        
        int cycleCount = 0;
        
        while (systemRunning && cycleCount < 100) {
            // Simulate vehicle inputs
            simulateVehicleInputs(cycleCount);
            
            // Update all systems
            updateSystems();
            
            // Send CAN messages
            sendCANData();
            
            // Process diagnostics
            updateDiagnostics();
            
            // Display status every 10 cycles
            if (cycleCount % 10 == 0) {
                displaySystemStatus();
            }
            
            // Simulate system faults occasionally
            if (cycleCount % 25 == 0) {
                simulateRandomFault();
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            cycleCount++;
        }
        
        std::cout << "\nSimulation completed. Generating final reports...\n";
        generateFinalReport();
    }
    
    void stopSimulation() {
        systemRunning = false;
    }
    
    void runTestSuite() {
        std::cout << "\n=== COMPREHENSIVE TEST SUITE ===\n";
        std::cout << "Testing all EV system components...\n\n";
        
        // Test 1: BMS Functionality
        testBMSFunctionality();
        
        // Test 2: ADAS Systems
        testADASFunctionality();
        
        // Test 3: Motor Control
        testMotorControl();
        
        // Test 4: Thermal Management
        testThermalManagement();
        
        // Test 5: CAN Communication
        testCANCommunication();
        
        // Test 6: Diagnostics
        testDiagnostics();
        
        // Test 7: Integration Test
        testSystemIntegration();
        
        std::cout << "=== TEST SUITE COMPLETED ===\n\n";
    }
    
private:
    void simulateVehicleInputs(int cycle) {
        // Simulate driving scenario
        double time = cycle * 0.1; // 0.1 second per cycle
        
        // Simulate acceleration and braking pattern
        if (cycle < 20) {
            // Acceleration phase
            acceleratorPedal = 0.5 + 0.3 * sin(time);
            brakePedal = 0.0;
            vehicleSpeed = std::min(80.0, vehicleSpeed + 2.0);
        } else if (cycle < 40) {
            // Cruise phase
            acceleratorPedal = 0.3;
            brakePedal = 0.0;
            vehicleSpeed = 80.0 + 5.0 * sin(time * 0.5);
        } else if (cycle < 60) {
            // Braking phase
            acceleratorPedal = 0.0;
            brakePedal = 0.4 + 0.2 * sin(time);
            vehicleSpeed = std::max(20.0, vehicleSpeed - 1.5);
        } else {
            // Low speed maneuvering
            acceleratorPedal = 0.2;
            brakePedal = 0.0;
            vehicleSpeed = 20.0 + 10.0 * sin(time * 0.3);
            steeringAngle = 15.0 * sin(time * 0.8); // Steering input
        }
        
        // Add noise to measurements
        batteryVoltage = 370.0 + batterySOC * 0.5 + noiseDist(randomEngine);
        batteryCurrent = (acceleratorPedal - brakePedal) * 100.0 + noiseDist(randomEngine) * 5;
        batteryTemperature = 25.0 + batteryCurrent * 0.1 + noiseDist(randomEngine);
        motorTemperature = 70.0 + abs(batteryCurrent) * 0.2 + noiseDist(randomEngine) * 2;
        
        // Update SOC based on current
        batterySOC -= batteryCurrent * 0.001; // Simplified SOC calculation
        batterySOC = std::max(10.0, std::min(100.0, batterySOC));
    }
    
    void updateSystems() {
        // Update BMS
        bms->setPackCurrent(batteryCurrent);
        bms->updateAllCells();
        bms->performCellBalancing();
        bms->checkSystemFaults();
        
        // Update Motor Controller
        if (acceleratorPedal > 0.1) {
            double targetTorque = acceleratorPedal * 300.0; // Max 300 Nm
            motorController->setTorqueCommand(targetTorque);
        } else {
            motorController->setSpeedCommand(vehicleSpeed);
        }
        
        // Update measurements
        ThreePhaseValues current(batteryCurrent/3, batteryCurrent/3, batteryCurrent/3);
        ThreePhaseValues voltage(batteryVoltage/3, batteryVoltage/3, batteryVoltage/3);
        motorController->updateMeasurements(current, voltage, vehicleSpeed * 30, // Convert to RPM
                                          0.0, batteryVoltage, motorTemperature);
        motorController->controlLoop();
        
        // Update Regenerative Braking
        regenBraking->updateInputs(vehicleSpeed, brakePedal, acceleratorPedal, batterySOC);
        double regenTorque = regenBraking->calculateRegenerativeTorque();
        
        // Update Torque Vectoring
        torqueVectoring->updateVehicleState(steeringAngle, 0.0, 0.0, vehicleSpeed);
        torqueVectoring->calculateTorqueDistribution(motorController->getActualTorque() + regenTorque);
        
        // Update ADAS
        simulateADASObjects();
        adas->updateVehicleState(vehicleSpeed, false, false);
        adas->processFrame();
        
        // Update Thermal Management
        thermalSystem->updateEnvironmentalConditions(ambientTemperature, vehicleSpeed);
        thermalSystem->updateBatteryTemperature(batteryTemperature);
        thermalSystem->updateMotorTemperature(motorTemperature);
    }
    
    void simulateADASObjects() {
        std::vector<Object> radarObjects, lidarObjects, cameraObjects;
        
        // Simulate occasional objects for ADAS testing
        if (randomEngine() % 100 < 20) { // 20% chance of object
            Object vehicle;
            vehicle.id = 1;
            vehicle.position = Point3D(50.0 + noiseDist(randomEngine) * 10, 
                                     noiseDist(randomEngine) * 2, 0);
            vehicle.velocity = Point3D(-10.0, 0, 0);
            vehicle.type = "vehicle";
            vehicle.confidence = 0.9;
            
            radarObjects.push_back(vehicle);
            lidarObjects.push_back(vehicle);
            cameraObjects.push_back(vehicle);
        }
        
        adas->updateSensorData(radarObjects, lidarObjects, cameraObjects);
    }
    
    void sendCANData() {
        // Send BMS status
        canSystem->sendBMSStatus(batteryVoltage, batteryCurrent, batterySOC, batteryTemperature);
        
        // Send Motor status
        canSystem->sendMotorCommand(motorController->getActualTorque(), 
                                   vehicleSpeed, motorController->isEnabled());
        
        // Send ADAS data
        canSystem->sendADASData(adas->getTargetSpeed(), 
                               adas->getSteeringCorrection(), 
                               adas->getBrakingForce() > 50.0);
        
        // Process received messages
        canSystem->processReceivedMessages();
    }
    
    void updateDiagnostics() {
        // Update sensor data for diagnostics
        std::unordered_map<int, double> sensorData;
        sensorData[1] = batteryVoltage;
        sensorData[2] = batteryCurrent;
        sensorData[3] = batteryTemperature;
        sensorData[4] = batterySOC;
        sensorData[5] = motorTemperature;
        sensorData[6] = vehicleSpeed * 30; // Convert to RPM
        sensorData[7] = motorController->getActualTorque();
        sensorData[8] = vehicleSpeed;
        sensorData[9] = ambientTemperature;
        sensorData[10] = batteryVoltage;
        
        diagnostics->updateSensorData(sensorData);
        diagnostics->performRoutineDiagnostics();
    }
    
    void simulateRandomFault() {
        int faultType = randomEngine() % 100;
        
        if (faultType < 10) {
            // Battery fault
            diagnostics->raiseFault(0xB001); // Overvoltage
        } else if (faultType < 20) {
            // Motor fault
            diagnostics->raiseFault(0xM001); // Overtemperature
        } else if (faultType < 30) {
            // ADAS fault
            diagnostics->raiseFault(0xA001); // Radar sensor fault
        }
        
        // Clear some faults randomly
        if (faultType > 80) {
            diagnostics->clearFault(0xB001);
            diagnostics->clearFault(0xM001);
        }
    }
    
    void displaySystemStatus() {
        std::cout << "\n" << std::string(60, '=') << "\n";
        std::cout << "VEHICLE STATUS - Speed: " << std::fixed << std::setprecision(1) 
                  << vehicleSpeed << " km/h, SOC: " << batterySOC << "%\n";
        std::cout << std::string(60, '=') << "\n";
        
        // Display key system status
        std::cout << "Battery: " << batteryVoltage << "V, " << batteryCurrent << "A, " 
                  << batteryTemperature << "°C\n";
        std::cout << "Motor: " << motorController->getActualTorque() << "Nm, " 
                  << motorTemperature << "°C\n";
        std::cout << "Regen Power: " << regenBraking->getRegenerativePower() << "kW\n";
        std::cout << "Thermal Power: " << thermalSystem->getTotalPowerConsumption() << "kW\n";
        std::cout << "Diagnostics: " << (diagnostics->hasActiveFaults() ? "FAULTS ACTIVE" : "OK") << "\n";
        
        if (diagnostics->hasCriticalFaults()) {
            std::cout << "*** CRITICAL FAULTS DETECTED ***\n";
        }
        
        std::cout << std::string(60, '-') << "\n\n";
    }
    
    void generateFinalReport() {
        std::cout << "\n" << std::string(50, '=') << "\n";
        std::cout << "FINAL SYSTEM REPORT\n";
        std::cout << std::string(50, '=') << "\n";
        
        bms->displayDetailedStatus();
        adas->displayStatus();
        motorController->displayStatus();
        regenBraking->displayStatus();
        torqueVectoring->displayStatus();
        thermalSystem->displaySystemStatus();
        diagnostics->displaySystemStatus();
        canSystem->displayStatus();
        
        // Generate diagnostic report
        diagnostics->generateMaintenanceReport();
    }
    
    // Test Functions
    void testBMSFunctionality() {
        std::cout << "Testing BMS Functionality...\n";
        
        // Test cell balancing
        bms->enableBalancing(true);
        bms->setBalancingMethod(BalancingMethod::ACTIVE);
        
        // Simulate cell imbalance and test balancing
        for (int i = 0; i < 5; i++) {
            bms->updateAllCells();
            bms->performCellBalancing();
        }
        
        std::cout << "✓ BMS cell balancing test completed\n";
        
        // Test fault detection
        bms->setPackCurrent(500.0); // Overcurrent
        bms->checkSystemFaults();
        
        std::cout << "✓ BMS fault detection test completed\n\n";
    }
    
    void testADASFunctionality() {
        std::cout << "Testing ADAS Functionality...\n";
        
        // Test different ADAS modes
        adas->setMode(ADASMode::ACTIVE);
        adas->setMode(ADASMode::STANDBY);
        adas->setMode(ADASMode::ACTIVE);
        
        // Test sensor fusion with mock objects
        std::vector<Object> testObjects;
        Object testVehicle;
        testVehicle.id = 99;
        testVehicle.position = Point3D(30.0, 0.5, 0);
        testVehicle.type = "vehicle";
        testObjects.push_back(testVehicle);
        
        adas->updateSensorData(testObjects, testObjects, testObjects);
        adas->processFrame();
        
        std::cout << "✓ ADAS sensor fusion test completed\n";
        std::cout << "✓ ADAS control logic test completed\n\n";
    }
    
    void testMotorControl() {
        std::cout << "Testing Motor Control...\n";
        
        // Test different control modes
        motorController->setControlMode(ControlMode::TORQUE_CONTROL);
        motorController->setTorqueCommand(100.0);
        
        motorController->setControlMode(ControlMode::SPEED_CONTROL);
        motorController->setSpeedCommand(1000.0);
        
        // Test regenerative braking
        regenBraking->setMode(RegenerativeMode::AGGRESSIVE);
        double regenTorque = regenBraking->calculateRegenerativeTorque();
        
        std::cout << "✓ Motor control modes test completed\n";
        std::cout << "✓ Regenerative braking test completed\n\n";
    }
    
    void testThermalManagement() {
        std::cout << "Testing Thermal Management...\n";
        
        // Test different cooling modes
        thermalSystem->setMode(CoolingMode::PERFORMANCE);
        thermalSystem->updateBatteryTemperature(45.0);
        thermalSystem->updateMotorTemperature(85.0);
        
        // Test HVAC modes
        thermalSystem->setHVACMode(HVACMode::COOLING);
        thermalSystem->setTargetCabinTemp(20.0);
        
        std::cout << "✓ Thermal management test completed\n\n";
    }
    
    void testCANCommunication() {
        std::cout << "Testing CAN Communication...\n";
        
        // Test message transmission
        canSystem->sendBMSStatus(400.0, 50.0, 75.0, 30.0);
        canSystem->sendMotorCommand(200.0, 2000.0, true);
        canSystem->processReceivedMessages();
        
        std::cout << "✓ CAN communication test completed\n\n";
    }
    
    void testDiagnostics() {
        std::cout << "Testing Diagnostics System...\n";
        
        // Test fault detection and clearing
        diagnostics->raiseFault(0xB001);
        diagnostics->raiseFault(0xM001);
        
        // Test OBD interface
        std::string obdResponse = diagnostics->processOBDCommand("0100");
        std::cout << "OBD Response: " << obdResponse << "\n";
        
        diagnostics->clearFault(0xB001);
        
        std::cout << "✓ Diagnostics test completed\n\n";
    }
    
    void testSystemIntegration() {
        std::cout << "Testing System Integration...\n";
        
        // Test inter-system communication and coordination
        for (int i = 0; i < 10; i++) {
            simulateVehicleInputs(i);
            updateSystems();
            sendCANData();
            updateDiagnostics();
        }
        
        std::cout << "✓ System integration test completed\n\n";
    }
};

int main() {
    try {
        std::cout << "Electric Vehicle System Demo and Test Suite\n";
        std::cout << "==========================================\n";
        std::cout << "Author: adzetto\n";
        std::cout << "Features: Advanced BMS, ADAS, Motor Control, CAN Bus,\n";
        std::cout << "          Thermal Management, Vehicle Diagnostics\n\n";
        
        EVSystemIntegrator evSystem;
        
        std::cout << "Choose operation mode:\n";
        std::cout << "1. Run Test Suite\n";
        std::cout << "2. Run Real-time Simulation\n";
        std::cout << "3. Run Both\n";
        std::cout << "Enter choice (1-3): ";
        
        int choice;
        std::cin >> choice;
        
        switch (choice) {
            case 1:
                evSystem.runTestSuite();
                break;
            case 2:
                evSystem.startSimulation();
                break;
            case 3:
                evSystem.runTestSuite();
                std::cout << "\nStarting real-time simulation...\n";
                std::this_thread::sleep_for(std::chrono::seconds(2));
                evSystem.startSimulation();
                break;
            default:
                std::cout << "Invalid choice. Running test suite...\n";
                evSystem.runTestSuite();
                break;
        }
        
        std::cout << "\nDemo completed successfully!\n";
        std::cout << "Check diagnostic_report.txt for detailed analysis.\n";
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}