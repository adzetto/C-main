/**
 * Comprehensive Thermal Management System for Electric Vehicles
 * Author: adzetto
 * Features: Battery cooling, motor cooling, cabin HVAC, predictive thermal control
 */

#ifndef THERMAL_MANAGEMENT_H
#define THERMAL_MANAGEMENT_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>
#include <thread>
#include <atomic>

enum class CoolingMode {
    OFF,
    ECO,
    NORMAL,
    PERFORMANCE,
    EMERGENCY
};

enum class HVACMode {
    OFF,
    HEATING,
    COOLING,
    AUTO,
    DEFROST
};

enum class ThermalZone {
    BATTERY_PACK,
    MOTOR_INVERTER,
    CABIN,
    POWER_ELECTRONICS,
    CHARGING_SYSTEM
};

struct ThermalSensor {
    int sensorId;
    ThermalZone zone;
    double temperature;
    double maxTemperature;
    double minTemperature;
    bool isActive;
    std::chrono::steady_clock::time_point lastUpdate;
    
    ThermalSensor(int id, ThermalZone z, double max = 60.0, double min = -20.0) 
        : sensorId(id), zone(z), temperature(25.0), maxTemperature(max), 
          minTemperature(min), isActive(true) {
        lastUpdate = std::chrono::steady_clock::now();
    }
    
    void updateTemperature(double temp) {
        temperature = temp;
        lastUpdate = std::chrono::steady_clock::now();
    }
    
    bool isOverheated() const { return temperature > maxTemperature; }
    bool isUndercooled() const { return temperature < minTemperature; }
    bool isFaulty() const {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - lastUpdate);
        return duration.count() > 10; // No update for 10 seconds
    }
};

class Pump {
private:
    int pumpId;
    std::string pumpName;
    double flowRate;        // L/min
    double maxFlowRate;     // L/min
    double power;           // W
    bool enabled;
    double efficiency;
    
public:
    Pump(int id, const std::string& name, double maxFlow = 20.0) 
        : pumpId(id), pumpName(name), flowRate(0.0), maxFlowRate(maxFlow),
          power(0.0), enabled(false), efficiency(0.8) {}
    
    void setFlowRate(double rate) {
        if (!enabled) return;
        
        flowRate = std::max(0.0, std::min(maxFlowRate, rate));
        power = (flowRate / maxFlowRate) * 100.0; // Simplified power calculation
    }
    
    void enable() { 
        enabled = true; 
        std::cout << "Pump " << pumpName << " enabled\n";
    }
    
    void disable() { 
        enabled = false; 
        flowRate = 0.0;
        power = 0.0;
        std::cout << "Pump " << pumpName << " disabled\n";
    }
    
    double getFlowRate() const { return flowRate; }
    double getPower() const { return power; }
    bool isEnabled() const { return enabled; }
    std::string getName() const { return pumpName; }
};

class Fan {
private:
    int fanId;
    std::string fanName;
    double speed;           // RPM
    double maxSpeed;        // RPM
    double power;           // W
    bool enabled;
    double airflow;         // CFM
    
public:
    Fan(int id, const std::string& name, double maxRPM = 3000.0) 
        : fanId(id), fanName(name), speed(0.0), maxSpeed(maxRPM),
          power(0.0), enabled(false), airflow(0.0) {}
    
    void setSpeed(double rpm) {
        if (!enabled) return;
        
        speed = std::max(0.0, std::min(maxSpeed, rpm));
        power = (speed / maxSpeed) * 50.0; // Simplified power calculation
        airflow = (speed / maxSpeed) * 200.0; // CFM
    }
    
    void enable() { 
        enabled = true; 
        std::cout << "Fan " << fanName << " enabled\n";
    }
    
    void disable() { 
        enabled = false; 
        speed = 0.0;
        power = 0.0;
        airflow = 0.0;
        std::cout << "Fan " << fanName << " disabled\n";
    }
    
    double getSpeed() const { return speed; }
    double getPower() const { return power; }
    double getAirflow() const { return airflow; }
    bool isEnabled() const { return enabled; }
    std::string getName() const { return fanName; }
};

class Radiator {
private:
    int radiatorId;
    std::string radiatorName;
    double effectiveness;
    double heatRejection;   // kW
    double inletTemp;       // °C
    double outletTemp;      // °C
    double airflow;         // CFM
    double coolantFlow;     // L/min
    
public:
    Radiator(int id, const std::string& name, double eff = 0.85) 
        : radiatorId(id), radiatorName(name), effectiveness(eff),
          heatRejection(0.0), inletTemp(25.0), outletTemp(25.0),
          airflow(0.0), coolantFlow(0.0) {}
    
    void updateConditions(double inlet, double ambientTemp, double flow, double air) {
        inletTemp = inlet;
        coolantFlow = flow;
        airflow = air;
        
        // Simplified heat transfer calculation
        double deltaT = inletTemp - ambientTemp;
        double flowFactor = std::min(1.0, coolantFlow / 10.0);
        double airFactor = std::min(1.0, airflow / 100.0);
        
        heatRejection = effectiveness * deltaT * flowFactor * airFactor * 0.1;
        outletTemp = inletTemp - (heatRejection / (coolantFlow * 0.06)); // Simplified
        
        outletTemp = std::max(ambientTemp, outletTemp);
    }
    
    double getHeatRejection() const { return heatRejection; }
    double getOutletTemp() const { return outletTemp; }
    std::string getName() const { return radiatorName; }
};

class ThermalLoop {
private:
    int loopId;
    std::string loopName;
    std::vector<std::shared_ptr<ThermalSensor>> sensors;
    std::shared_ptr<Pump> pump;
    std::shared_ptr<Fan> fan;
    std::shared_ptr<Radiator> radiator;
    
    double coolantTemperature;
    double targetTemperature;
    double ambientTemperature;
    double thermostatSetpoint;
    bool thermostatOpen;
    
public:
    ThermalLoop(int id, const std::string& name) 
        : loopId(id), loopName(name), coolantTemperature(25.0),
          targetTemperature(40.0), ambientTemperature(25.0),
          thermostatSetpoint(85.0), thermostatOpen(false) {
        
        pump = std::make_shared<Pump>(id * 10, name + " Pump");
        fan = std::make_shared<Fan>(id * 10 + 1, name + " Fan");
        radiator = std::make_shared<Radiator>(id * 10 + 2, name + " Radiator");
    }
    
    void addSensor(std::shared_ptr<ThermalSensor> sensor) {
        sensors.push_back(sensor);
    }
    
    void setTargetTemperature(double target) {
        targetTemperature = target;
    }
    
    void updateLoop(double ambient) {
        ambientTemperature = ambient;
        
        // Calculate average temperature from sensors
        if (!sensors.empty()) {
            double totalTemp = 0.0;
            int activeSensors = 0;
            
            for (const auto& sensor : sensors) {
                if (sensor->isActive && !sensor->isFaulty()) {
                    totalTemp += sensor->temperature;
                    activeSensors++;
                }
            }
            
            if (activeSensors > 0) {
                coolantTemperature = totalTemp / activeSensors;
            }
        }
        
        // Control thermostat
        if (coolantTemperature > thermostatSetpoint) {
            thermostatOpen = true;
        } else if (coolantTemperature < thermostatSetpoint - 5.0) {
            thermostatOpen = false;
        }
        
        // Control pump
        controlPump();
        
        // Control fan
        controlFan();
        
        // Update radiator
        if (thermostatOpen) {
            radiator->updateConditions(coolantTemperature, ambientTemperature, 
                                     pump->getFlowRate(), fan->getAirflow());
        }
    }
    
    void displayStatus() const {
        std::cout << "=== " << loopName << " Status ===\n";
        std::cout << "Coolant Temperature: " << coolantTemperature << " °C\n";
        std::cout << "Target Temperature: " << targetTemperature << " °C\n";
        std::cout << "Ambient Temperature: " << ambientTemperature << " °C\n";
        std::cout << "Thermostat: " << (thermostatOpen ? "OPEN" : "CLOSED") << "\n";
        std::cout << "Pump Flow: " << pump->getFlowRate() << " L/min\n";
        std::cout << "Fan Speed: " << fan->getSpeed() << " RPM\n";
        std::cout << "Heat Rejection: " << radiator->getHeatRejection() << " kW\n";
        std::cout << "Active Sensors: " << getActiveSensorCount() << "/" << sensors.size() << "\n";
        std::cout << "============================\n\n";
    }
    
    double getCoolantTemperature() const { return coolantTemperature; }
    bool hasOverheating() const {
        for (const auto& sensor : sensors) {
            if (sensor->isOverheated()) return true;
        }
        return false;
    }
    
private:
    void controlPump() {
        if (coolantTemperature > targetTemperature + 5.0) {
            pump->enable();
            pump->setFlowRate(20.0); // Full flow
        } else if (coolantTemperature > targetTemperature) {
            pump->enable();
            double flowRate = ((coolantTemperature - targetTemperature) / 5.0) * 20.0;
            pump->setFlowRate(flowRate);
        } else {
            pump->setFlowRate(5.0); // Minimum flow to prevent stagnation
        }
    }
    
    void controlFan() {
        if (thermostatOpen && coolantTemperature > targetTemperature) {
            fan->enable();
            double tempDiff = coolantTemperature - targetTemperature;
            double fanSpeed = std::min(3000.0, tempDiff * 100.0);
            fan->setSpeed(fanSpeed);
        } else {
            fan->disable();
        }
    }
    
    int getActiveSensorCount() const {
        int count = 0;
        for (const auto& sensor : sensors) {
            if (sensor->isActive && !sensor->isFaulty()) count++;
        }
        return count;
    }
};

class HVACSystem {
private:
    HVACMode currentMode;
    double cabinTemperature;
    double targetTemperature;
    double outsideTemperature;
    double compressorPower;
    double heaterPower;
    double blowerSpeed;
    bool enabled;
    
    // Components
    bool compressorRunning;
    bool heaterRunning;
    double refrigerantPressure;
    double evaporatorTemp;
    double condenserTemp;
    
public:
    HVACSystem() : currentMode(HVACMode::OFF), cabinTemperature(25.0),
                   targetTemperature(22.0), outsideTemperature(25.0),
                   compressorPower(0.0), heaterPower(0.0), blowerSpeed(0.0),
                   enabled(false), compressorRunning(false), heaterRunning(false),
                   refrigerantPressure(0.0), evaporatorTemp(0.0), condenserTemp(0.0) {}
    
    void enable() { enabled = true; }
    void disable() { 
        enabled = false; 
        currentMode = HVACMode::OFF;
        stopAllComponents();
    }
    
    void setMode(HVACMode mode) {
        if (!enabled) return;
        currentMode = mode;
        std::cout << "HVAC mode set to: " << getModeString() << "\n";
    }
    
    void setTargetTemperature(double target) {
        targetTemperature = std::max(16.0, std::min(30.0, target));
    }
    
    void updateConditions(double cabin, double outside) {
        cabinTemperature = cabin;
        outsideTemperature = outside;
        
        if (!enabled) return;
        
        switch (currentMode) {
            case HVACMode::AUTO:
                autoModeControl();
                break;
            case HVACMode::COOLING:
                coolingControl();
                break;
            case HVACMode::HEATING:
                heatingControl();
                break;
            case HVACMode::DEFROST:
                defrostControl();
                break;
            case HVACMode::OFF:
                stopAllComponents();
                break;
        }
    }
    
    void displayStatus() const {
        std::cout << "\n=== HVAC System Status ===\n";
        std::cout << "Mode: " << getModeString() << "\n";
        std::cout << "Enabled: " << (enabled ? "YES" : "NO") << "\n";
        std::cout << "Cabin Temperature: " << cabinTemperature << " °C\n";
        std::cout << "Target Temperature: " << targetTemperature << " °C\n";
        std::cout << "Outside Temperature: " << outsideTemperature << " °C\n";
        std::cout << "Compressor: " << (compressorRunning ? "ON" : "OFF") << " (" << compressorPower << " kW)\n";
        std::cout << "Heater: " << (heaterRunning ? "ON" : "OFF") << " (" << heaterPower << " kW)\n";
        std::cout << "Blower Speed: " << blowerSpeed << " %\n";
        std::cout << "Power Consumption: " << getTotalPowerConsumption() << " kW\n";
        std::cout << "========================\n\n";
    }
    
    double getTotalPowerConsumption() const {
        return compressorPower + heaterPower + (blowerSpeed / 100.0 * 0.5);
    }
    
private:
    void autoModeControl() {
        double tempDiff = targetTemperature - cabinTemperature;
        
        if (tempDiff > 2.0) {
            // Need heating
            heatingControl();
        } else if (tempDiff < -2.0) {
            // Need cooling
            coolingControl();
        } else {
            // Maintain
            maintainTemperature();
        }
    }
    
    void coolingControl() {
        if (cabinTemperature > targetTemperature + 1.0) {
            compressorRunning = true;
            heaterRunning = false;
            
            double tempDiff = cabinTemperature - targetTemperature;
            compressorPower = std::min(3.0, tempDiff * 0.5); // Max 3kW
            blowerSpeed = std::min(100.0, tempDiff * 20.0);
            
            // Update refrigerant conditions
            evaporatorTemp = cabinTemperature - 10.0;
            condenserTemp = outsideTemperature + 15.0;
            refrigerantPressure = 15.0 + compressorPower * 2.0;
        } else {
            compressorRunning = false;
            compressorPower = 0.0;
        }
    }
    
    void heatingControl() {
        if (cabinTemperature < targetTemperature - 1.0) {
            heaterRunning = true;
            compressorRunning = false;
            
            double tempDiff = targetTemperature - cabinTemperature;
            
            // Use heat pump if outside temp > 0°C, otherwise resistive heating
            if (outsideTemperature > 0.0) {
                // Heat pump mode
                compressorRunning = true;
                compressorPower = std::min(2.5, tempDiff * 0.4);
                heaterPower = 0.0;
            } else {
                // Resistive heating
                heaterPower = std::min(5.0, tempDiff * 0.8); // Max 5kW
            }
            
            blowerSpeed = std::min(100.0, tempDiff * 15.0);
        } else {
            heaterRunning = false;
            heaterPower = 0.0;
        }
    }
    
    void defrostControl() {
        // High heat, high airflow
        heaterRunning = true;
        heaterPower = 4.0; // High power for defrost
        blowerSpeed = 80.0; // High airflow
        compressorRunning = false;
        compressorPower = 0.0;
    }
    
    void maintainTemperature() {
        // Reduce power consumption in maintain mode
        compressorPower *= 0.5;
        heaterPower *= 0.5;
        blowerSpeed = std::max(20.0, blowerSpeed * 0.3); // Minimum air circulation
    }
    
    void stopAllComponents() {
        compressorRunning = false;
        heaterRunning = false;
        compressorPower = 0.0;
        heaterPower = 0.0;
        blowerSpeed = 0.0;
    }
    
    std::string getModeString() const {
        switch (currentMode) {
            case HVACMode::OFF: return "OFF";
            case HVACMode::HEATING: return "HEATING";
            case HVACMode::COOLING: return "COOLING";
            case HVACMode::AUTO: return "AUTO";
            case HVACMode::DEFROST: return "DEFROST";
            default: return "UNKNOWN";
        }
    }
};

class ThermalManagementSystem {
private:
    std::vector<std::shared_ptr<ThermalLoop>> thermalLoops;
    std::vector<std::shared_ptr<ThermalSensor>> allSensors;
    std::unique_ptr<HVACSystem> hvacSystem;
    
    CoolingMode currentMode;
    double ambientTemperature;
    double vehicleSpeed;
    bool emergencyMode;
    
    // Power consumption tracking
    double totalThermalPower;
    
public:
    ThermalManagementSystem() : currentMode(CoolingMode::NORMAL), ambientTemperature(25.0),
                               vehicleSpeed(0.0), emergencyMode(false), totalThermalPower(0.0) {
        
        setupThermalLoops();
        setupSensors();
        hvacSystem = std::make_unique<HVACSystem>();
    }
    
    void setMode(CoolingMode mode) {
        currentMode = mode;
        updateCoolingParameters();
        std::cout << "Thermal management mode set to: " << getModeString() << "\n";
    }
    
    void updateEnvironmentalConditions(double ambient, double speed) {
        ambientTemperature = ambient;
        vehicleSpeed = speed;
        
        // Update all thermal loops
        for (auto& loop : thermalLoops) {
            loop->updateLoop(ambientTemperature);
        }
        
        // Update HVAC
        double cabinTemp = 25.0; // Would come from cabin sensor
        hvacSystem->updateConditions(cabinTemp, ambientTemperature);
        
        // Check for emergency conditions
        checkEmergencyConditions();
        
        // Calculate total power consumption
        calculateTotalPowerConsumption();
    }
    
    void updateBatteryTemperature(double temp) {
        for (auto& sensor : allSensors) {
            if (sensor->zone == ThermalZone::BATTERY_PACK) {
                sensor->updateTemperature(temp);
            }
        }
    }
    
    void updateMotorTemperature(double temp) {
        for (auto& sensor : allSensors) {
            if (sensor->zone == ThermalZone::MOTOR_INVERTER) {
                sensor->updateTemperature(temp);
            }
        }
    }
    
    void updateChargingTemperature(double temp) {
        for (auto& sensor : allSensors) {
            if (sensor->zone == ThermalZone::CHARGING_SYSTEM) {
                sensor->updateTemperature(temp);
            }
        }
    }
    
    void enableHVAC() { hvacSystem->enable(); }
    void disableHVAC() { hvacSystem->disable(); }
    void setHVACMode(HVACMode mode) { hvacSystem->setMode(mode); }
    void setTargetCabinTemp(double temp) { hvacSystem->setTargetTemperature(temp); }
    
    void displaySystemStatus() {
        std::cout << "\n=== Thermal Management System Status ===\n";
        std::cout << "Cooling Mode: " << getModeString() << "\n";
        std::cout << "Ambient Temperature: " << ambientTemperature << " °C\n";
        std::cout << "Vehicle Speed: " << vehicleSpeed << " km/h\n";
        std::cout << "Emergency Mode: " << (emergencyMode ? "ACTIVE" : "INACTIVE") << "\n";
        std::cout << "Total Thermal Power: " << totalThermalPower << " kW\n";
        std::cout << "========================================\n\n";
        
        // Display thermal loops
        for (const auto& loop : thermalLoops) {
            loop->displayStatus();
        }
        
        // Display HVAC status
        hvacSystem->displayStatus();
        
        // Display sensor summary
        displaySensorSummary();
    }
    
    bool hasOverheating() const {
        for (const auto& loop : thermalLoops) {
            if (loop->hasOverheating()) return true;
        }
        return false;
    }
    
    bool isEmergencyMode() const { return emergencyMode; }
    double getTotalPowerConsumption() const { return totalThermalPower; }
    
private:
    void setupThermalLoops() {
        // Battery cooling loop
        auto batteryLoop = std::make_shared<ThermalLoop>(1, "Battery Cooling");
        batteryLoop->setTargetTemperature(35.0);
        thermalLoops.push_back(batteryLoop);
        
        // Motor/Inverter cooling loop
        auto motorLoop = std::make_shared<ThermalLoop>(2, "Motor/Inverter Cooling");
        motorLoop->setTargetTemperature(70.0);
        thermalLoops.push_back(motorLoop);
        
        // Power electronics cooling loop
        auto powerLoop = std::make_shared<ThermalLoop>(3, "Power Electronics Cooling");
        powerLoop->setTargetTemperature(60.0);
        thermalLoops.push_back(powerLoop);
    }
    
    void setupSensors() {
        // Battery sensors
        allSensors.push_back(std::make_shared<ThermalSensor>(1, ThermalZone::BATTERY_PACK, 50.0));
        allSensors.push_back(std::make_shared<ThermalSensor>(2, ThermalZone::BATTERY_PACK, 50.0));
        allSensors.push_back(std::make_shared<ThermalSensor>(3, ThermalZone::BATTERY_PACK, 50.0));
        
        // Motor sensors
        allSensors.push_back(std::make_shared<ThermalSensor>(4, ThermalZone::MOTOR_INVERTER, 120.0));
        allSensors.push_back(std::make_shared<ThermalSensor>(5, ThermalZone::MOTOR_INVERTER, 120.0));
        
        // Power electronics sensors
        allSensors.push_back(std::make_shared<ThermalSensor>(6, ThermalZone::POWER_ELECTRONICS, 85.0));
        
        // Charging system sensor
        allSensors.push_back(std::make_shared<ThermalSensor>(7, ThermalZone::CHARGING_SYSTEM, 80.0));
        
        // Cabin sensor
        allSensors.push_back(std::make_shared<ThermalSensor>(8, ThermalZone::CABIN, 40.0, -10.0));
        
        // Assign sensors to thermal loops
        for (auto& sensor : allSensors) {
            switch (sensor->zone) {
                case ThermalZone::BATTERY_PACK:
                    thermalLoops[0]->addSensor(sensor);
                    break;
                case ThermalZone::MOTOR_INVERTER:
                    thermalLoops[1]->addSensor(sensor);
                    break;
                case ThermalZone::POWER_ELECTRONICS:
                    thermalLoops[2]->addSensor(sensor);
                    break;
                default:
                    break;
            }
        }
    }
    
    void updateCoolingParameters() {
        // Adjust target temperatures based on cooling mode
        double batteryTarget = 35.0;
        double motorTarget = 70.0;
        double powerTarget = 60.0;
        
        switch (currentMode) {
            case CoolingMode::ECO:
                batteryTarget = 40.0;
                motorTarget = 80.0;
                powerTarget = 70.0;
                break;
            case CoolingMode::NORMAL:
                // Default values
                break;
            case CoolingMode::PERFORMANCE:
                batteryTarget = 30.0;
                motorTarget = 60.0;
                powerTarget = 50.0;
                break;
            case CoolingMode::EMERGENCY:
                batteryTarget = 25.0;
                motorTarget = 50.0;
                powerTarget = 40.0;
                break;
            case CoolingMode::OFF:
                batteryTarget = 60.0;
                motorTarget = 100.0;
                powerTarget = 90.0;
                break;
        }
        
        if (thermalLoops.size() >= 3) {
            thermalLoops[0]->setTargetTemperature(batteryTarget);
            thermalLoops[1]->setTargetTemperature(motorTarget);
            thermalLoops[2]->setTargetTemperature(powerTarget);
        }
    }
    
    void checkEmergencyConditions() {
        emergencyMode = false;
        
        for (const auto& sensor : allSensors) {
            if (sensor->isOverheated()) {
                emergencyMode = true;
                std::cout << "THERMAL EMERGENCY: Sensor " << sensor->sensorId 
                          << " overheated (" << sensor->temperature << "°C)\n";
                break;
            }
        }
        
        if (emergencyMode && currentMode != CoolingMode::EMERGENCY) {
            setMode(CoolingMode::EMERGENCY);
        }
    }
    
    void calculateTotalPowerConsumption() {
        totalThermalPower = hvacSystem->getTotalPowerConsumption();
        
        // Add pump and fan power from thermal loops
        // This would be implemented by accessing pump and fan objects
        // For now, estimate based on mode
        switch (currentMode) {
            case CoolingMode::OFF:
                totalThermalPower += 0.1;
                break;
            case CoolingMode::ECO:
                totalThermalPower += 0.5;
                break;
            case CoolingMode::NORMAL:
                totalThermalPower += 1.0;
                break;
            case CoolingMode::PERFORMANCE:
                totalThermalPower += 2.0;
                break;
            case CoolingMode::EMERGENCY:
                totalThermalPower += 3.0;
                break;
        }
    }
    
    void displaySensorSummary() {
        std::cout << "=== Sensor Summary ===\n";
        for (const auto& sensor : allSensors) {
            std::cout << "Sensor " << sensor->sensorId << " (" 
                      << getZoneString(sensor->zone) << "): "
                      << sensor->temperature << "°C";
            if (sensor->isOverheated()) std::cout << " [OVERHEATED]";
            if (sensor->isFaulty()) std::cout << " [FAULT]";
            std::cout << "\n";
        }
        std::cout << "=====================\n\n";
    }
    
    std::string getModeString() const {
        switch (currentMode) {
            case CoolingMode::OFF: return "OFF";
            case CoolingMode::ECO: return "ECO";
            case CoolingMode::NORMAL: return "NORMAL";
            case CoolingMode::PERFORMANCE: return "PERFORMANCE";
            case CoolingMode::EMERGENCY: return "EMERGENCY";
            default: return "UNKNOWN";
        }
    }
    
    std::string getZoneString(ThermalZone zone) const {
        switch (zone) {
            case ThermalZone::BATTERY_PACK: return "Battery";
            case ThermalZone::MOTOR_INVERTER: return "Motor";
            case ThermalZone::CABIN: return "Cabin";
            case ThermalZone::POWER_ELECTRONICS: return "Power";
            case ThermalZone::CHARGING_SYSTEM: return "Charging";
            default: return "Unknown";
        }
    }
};

#endif // THERMAL_MANAGEMENT_H