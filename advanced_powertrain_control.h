#ifndef ADVANCED_POWERTRAIN_CONTROL_H
#define ADVANCED_POWERTRAIN_CONTROL_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <string>
#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>

class AdvancedPowertrainControl {
public:
    enum class DriveMode {
        ECO,
        COMFORT,
        SPORT,
        TRACK,
        SNOW,
        SAND,
        MUD,
        ROCK,
        CUSTOM
    };

    enum class PowertrainState {
        OFF,
        STANDBY,
        READY,
        DRIVING,
        REGENERATING,
        CHARGING,
        FAULT,
        EMERGENCY
    };

    enum class MotorType {
        FRONT_LEFT,
        FRONT_RIGHT,
        REAR_LEFT,
        REAR_RIGHT
    };

    struct MotorController {
        MotorType type;
        float targetTorque;
        float currentTorque;
        float speed;
        float temperature;
        float efficiency;
        float power;
        bool enabled;
        bool faultActive;
        std::string faultCode;
        
        // Advanced control parameters
        float fieldWeakeningPoint;
        float maxTorqueAtSpeed[100]; // Torque curve lookup table
        float thermalDeratingFactor;
        float vibrationLevel;
        std::chrono::steady_clock::time_point lastUpdate;
    };

    struct InverterController {
        int inverterId;
        float dcVoltage;
        float dcCurrent;
        float acVoltageRMS;
        float acCurrentRMS;
        float switchingFrequency;
        float temperature;
        float efficiency;
        bool enabled;
        bool faultActive;
        
        // Power semiconductor parameters
        float igbtTemperature[6]; // 6 IGBTs per inverter
        float diodeTemperature[6];
        float gateDriverVoltage[6];
        float deadTime;
        float modulationIndex;
    };

    struct TransmissionController {
        enum class GearState {
            PARK,
            REVERSE,
            NEUTRAL,
            DRIVE,
            SPORT,
            LOW
        } currentGear;
        
        float gearRatio;
        float inputSpeed;
        float outputSpeed;
        float torqueMultiplier;
        float efficiency;
        float oilTemperature;
        float oilPressure;
        bool shiftInProgress;
        bool faultActive;
        
        // Advanced features
        bool adaptiveShifting;
        float shiftQuality;
        std::vector<float> shiftMap[6]; // Shift points for each gear
        float predictiveShiftTiming;
    };

    struct DifferentialController {
        enum class DifferentialType {
            OPEN,
            LIMITED_SLIP,
            ELECTRONIC,
            TORQUE_VECTORING
        } type;
        
        float lockPercentage;
        float leftTorque;
        float rightTorque;
        float biasRatio;
        bool enabled;
        float efficiency;
        float temperature;
        
        // Torque vectoring parameters
        float yawMoment;
        float lateralAcceleration;
        float understeerCompensation;
        float oversteerCompensation;
    };

    struct PowerDistributionUnit {
        float totalPower;
        float frontPowerDistribution;
        float rearPowerDistribution;
        float leftPowerDistribution;
        float rightPowerDistribution;
        
        // Dynamic control
        bool adaptivePowerDistribution;
        float tractionControlIntegration;
        float stabilityControlIntegration;
        float performanceOptimization;
        
        // Efficiency optimization
        float powerLossMinimization;
        float thermalBalance;
    };

    struct RegenerativeBrakingController {
        float maxRegenTorque;
        float currentRegenTorque;
        float regenPower;
        float energyRecovered;
        float blendingRatio; // Friction vs regen braking
        bool coastingRegen;
        bool onepedalDriving;
        
        // Advanced features
        float predictiveRegen;
        float terrainAdaptation;
        float weatherAdaptation;
        std::vector<float> regenMap; // Regen curve based on speed
        float cooperativeRegenBraking;
    };

    struct ThermalManagement {
        struct CoolingLoop {
            std::string name;
            float flowRate;
            float inletTemperature;
            float outletTemperature;
            float pressure;
            bool pumpActive;
            float pumpSpeed;
        };
        
        std::vector<CoolingLoop> coolingLoops;
        float ambientTemperature;
        float radiatorEfficiency;
        bool thermalEmergency;
        
        // Predictive thermal control
        float thermalLoadPrediction;
        float coolingDemandForecast;
        bool preemptiveCooling;
    };

    struct PowerElectronicsController {
        float dcBusVoltage;
        float dcBusCapacitance;
        float powerFactor;
        float harmonicDistortion;
        bool prechargeComplete;
        bool insulationMonitoring;
        float insulationResistance;
        
        // Advanced power management
        float activeFiltering;
        float reactivePowerCompensation;
        float gridTieCapability;
        bool bidirectionalPower;
        float powerQuality;
    };

    struct VehicleDynamicsInterface {
        float vehicleSpeed;
        float wheelSpeeds[4];
        float steeringAngle;
        float yawRate;
        float lateralAcceleration;
        float longitudinalAcceleration;
        float verticalAcceleration;
        float rollAngle;
        float pitchAngle;
        
        // Road condition detection
        float roadFriction;
        float roadGrade;
        bool roadConditionValid;
        std::string surfaceType;
    };

public:
    AdvancedPowertrainControl() : 
        currentDriveMode(DriveMode::COMFORT),
        powertrainState(PowertrainState::OFF),
        systemEnabled(false),
        torqueVectoringEnabled(true),
        adaptivePowerDistribution(true),
        predictiveControl(true),
        thermalProtection(true),
        emergencyShutdownArmed(true),
        diagnosticsActive(true),
        controlLoopRunning(false) {
        
        initializeMotorControllers();
        initializeInverters();
        initializeTransmission();
        initializeDifferentials();
        initializePowerDistribution();
        initializeRegenBraking();
        initializeThermalManagement();
        initializePowerElectronics();
    startControlLoop();
        
        std::cout << "Advanced Powertrain Control System initialized" << std::endl;
    }

    ~AdvancedPowertrainControl() {
        // Ensure background control loop stops before destruction
        stopControlLoop();
    }

    void initializeMotorControllers() {
        // Initialize four motor controllers for AWD system
        for (int i = 0; i < 4; i++) {
            MotorController motor;
            motor.type = static_cast<MotorType>(i);
            motor.targetTorque = 0.0f;
            motor.currentTorque = 0.0f;
            motor.speed = 0.0f;
            motor.temperature = 25.0f;
            motor.efficiency = 0.95f;
            motor.power = 0.0f;
            motor.enabled = true;
            motor.faultActive = false;
            motor.fieldWeakeningPoint = 3000.0f; // RPM
            motor.thermalDeratingFactor = 1.0f;
            motor.vibrationLevel = 0.0f;
            motor.lastUpdate = std::chrono::steady_clock::now();
            
            // Initialize torque curve
            for (int j = 0; j < 100; j++) {
                float speed = j * 100.0f; // 0-10000 RPM
                if (speed < motor.fieldWeakeningPoint) {
                    motor.maxTorqueAtSpeed[j] = 400.0f; // Constant torque region
                } else {
                    // Field weakening region - power limited
                    motor.maxTorqueAtSpeed[j] = (400.0f * motor.fieldWeakeningPoint) / speed;
                }
            }
            
            motorControllers.push_back(motor);
        }
        
        std::cout << "Motor controllers initialized: " << motorControllers.size() << " motors" << std::endl;
    }

    void initializeInverters() {
        // Initialize inverters for each motor
        for (int i = 0; i < 4; i++) {
            InverterController inverter;
            inverter.inverterId = i;
            inverter.dcVoltage = 400.0f;
            inverter.dcCurrent = 0.0f;
            inverter.acVoltageRMS = 0.0f;
            inverter.acCurrentRMS = 0.0f;
            inverter.switchingFrequency = 10000.0f; // 10 kHz
            inverter.temperature = 25.0f;
            inverter.efficiency = 0.98f;
            inverter.enabled = true;
            inverter.faultActive = false;
            inverter.deadTime = 2.0f; // microseconds
            inverter.modulationIndex = 0.0f;
            
            // Initialize IGBT temperatures
            for (int j = 0; j < 6; j++) {
                inverter.igbtTemperature[j] = 25.0f;
                inverter.diodeTemperature[j] = 25.0f;
                inverter.gateDriverVoltage[j] = 15.0f;
            }
            
            inverters.push_back(inverter);
        }
        
        std::cout << "Inverters initialized: " << inverters.size() << " inverters" << std::endl;
    }

    void initializeTransmission() {
        transmission.currentGear = TransmissionController::GearState::PARK;
        transmission.gearRatio = 1.0f;
        transmission.inputSpeed = 0.0f;
        transmission.outputSpeed = 0.0f;
        transmission.torqueMultiplier = 1.0f;
        transmission.efficiency = 0.97f;
        transmission.oilTemperature = 80.0f;
        transmission.oilPressure = 5.0f; // bar
        transmission.shiftInProgress = false;
        transmission.faultActive = false;
        transmission.adaptiveShifting = true;
        transmission.shiftQuality = 0.9f;
        transmission.predictiveShiftTiming = 0.0f;
        
        // Initialize shift maps for different gears
        for (int i = 0; i < 6; i++) {
            transmission.shiftMap[i].resize(100);
            for (int j = 0; j < 100; j++) {
                // Simple shift map based on vehicle speed and throttle
                transmission.shiftMap[i][j] = 2000.0f + i * 500.0f + j * 10.0f;
            }
        }
        
        std::cout << "Transmission controller initialized" << std::endl;
    }

    void initializeDifferentials() {
        // Front differential
        DifferentialController frontDiff;
        frontDiff.type = DifferentialController::DifferentialType::TORQUE_VECTORING;
        frontDiff.lockPercentage = 0.0f;
        frontDiff.leftTorque = 0.0f;
        frontDiff.rightTorque = 0.0f;
        frontDiff.biasRatio = 0.5f;
        frontDiff.enabled = true;
        frontDiff.efficiency = 0.95f;
        frontDiff.temperature = 60.0f;
        frontDiff.yawMoment = 0.0f;
        frontDiff.lateralAcceleration = 0.0f;
        frontDiff.understeerCompensation = 0.0f;
        frontDiff.oversteerCompensation = 0.0f;
        
        // Rear differential
        DifferentialController rearDiff = frontDiff;
        
        differentials["front"] = frontDiff;
        differentials["rear"] = rearDiff;
        
        std::cout << "Differential controllers initialized: " << differentials.size() << " differentials" << std::endl;
    }

    void initializePowerDistribution() {
        powerDistribution.totalPower = 0.0f;
        powerDistribution.frontPowerDistribution = 0.5f;
        powerDistribution.rearPowerDistribution = 0.5f;
        powerDistribution.leftPowerDistribution = 0.5f;
        powerDistribution.rightPowerDistribution = 0.5f;
        powerDistribution.adaptivePowerDistribution = true;
        powerDistribution.tractionControlIntegration = 1.0f;
        powerDistribution.stabilityControlIntegration = 1.0f;
        powerDistribution.performanceOptimization = 0.8f;
        powerDistribution.powerLossMinimization = 0.9f;
        powerDistribution.thermalBalance = 0.8f;
        
        std::cout << "Power distribution unit initialized" << std::endl;
    }

    void initializeRegenBraking() {
        regenBraking.maxRegenTorque = 200.0f; // Nm per motor
        regenBraking.currentRegenTorque = 0.0f;
        regenBraking.regenPower = 0.0f;
        regenBraking.energyRecovered = 0.0f;
        regenBraking.blendingRatio = 0.7f; // 70% regen, 30% friction initially
        regenBraking.coastingRegen = true;
        regenBraking.onepedalDriving = false;
        regenBraking.predictiveRegen = 0.0f;
        regenBraking.terrainAdaptation = 1.0f;
        regenBraking.weatherAdaptation = 1.0f;
        regenBraking.cooperativeRegenBraking = 0.0f;
        
        // Initialize regen map
        regenBraking.regenMap.resize(100);
        for (int i = 0; i < 100; i++) {
            float speed = i * 2.0f; // 0-200 km/h
            if (speed < 5.0f) {
                regenBraking.regenMap[i] = 0.0f; // No regen at very low speeds
            } else {
                regenBraking.regenMap[i] = regenBraking.maxRegenTorque * (speed / 100.0f);
            }
        }
        
        std::cout << "Regenerative braking controller initialized" << std::endl;
    }

    void initializeThermalManagement() {
        thermalMgmt.ambientTemperature = 25.0f;
        thermalMgmt.radiatorEfficiency = 0.85f;
        thermalMgmt.thermalEmergency = false;
        thermalMgmt.thermalLoadPrediction = 0.0f;
        thermalMgmt.coolingDemandForecast = 0.0f;
        thermalMgmt.preemptiveCooling = true;
        
        // Initialize cooling loops
        ThermalManagement::CoolingLoop motorLoop = {
            "Motor Cooling", 10.0f, 25.0f, 30.0f, 2.0f, true, 50.0f
        };
        ThermalManagement::CoolingLoop inverterLoop = {
            "Inverter Cooling", 8.0f, 25.0f, 35.0f, 1.8f, true, 60.0f
        };
        ThermalManagement::CoolingLoop batteryLoop = {
            "Battery Cooling", 12.0f, 25.0f, 28.0f, 2.5f, true, 40.0f
        };
        
        thermalMgmt.coolingLoops.push_back(motorLoop);
        thermalMgmt.coolingLoops.push_back(inverterLoop);
        thermalMgmt.coolingLoops.push_back(batteryLoop);
        
        std::cout << "Thermal management initialized: " << thermalMgmt.coolingLoops.size() << " cooling loops" << std::endl;
    }

    void initializePowerElectronics() {
        powerElectronics.dcBusVoltage = 400.0f;
        powerElectronics.dcBusCapacitance = 2000.0f; // microfarads
        powerElectronics.powerFactor = 0.98f;
        powerElectronics.harmonicDistortion = 0.03f; // 3% THD
        powerElectronics.prechargeComplete = false;
        powerElectronics.insulationMonitoring = true;
        powerElectronics.insulationResistance = 500.0f; // kOhm
        powerElectronics.activeFiltering = 0.95f;
        powerElectronics.reactivePowerCompensation = 0.0f;
        powerElectronics.gridTieCapability = false;
        powerElectronics.bidirectionalPower = true;
        powerElectronics.powerQuality = 0.98f;
        
        std::cout << "Power electronics controller initialized" << std::endl;
    }

    void setDriveMode(DriveMode mode) {
        currentDriveMode = mode;
        
        switch (mode) {
            case DriveMode::ECO:
                configureDriveMode(0.7f, 0.3f, 0.8f, true, true);
                break;
            case DriveMode::COMFORT:
                configureDriveMode(0.8f, 0.5f, 0.7f, true, true);
                break;
            case DriveMode::SPORT:
                configureDriveMode(1.0f, 0.9f, 0.5f, true, false);
                break;
            case DriveMode::TRACK:
                configureDriveMode(1.0f, 1.0f, 0.3f, false, false);
                break;
            case DriveMode::SNOW:
                configureDriveMode(0.6f, 0.2f, 0.9f, true, true);
                break;
            case DriveMode::SAND:
                configureDriveMode(0.8f, 0.6f, 0.6f, true, true);
                break;
            case DriveMode::MUD:
                configureDriveMode(0.9f, 0.7f, 0.4f, false, true);
                break;
            case DriveMode::ROCK:
                configureDriveMode(0.5f, 0.3f, 0.8f, false, true);
                break;
        }
        
        std::cout << "Drive mode set to: " << getDriveModeString(mode) << std::endl;
    }

    void updateVehicleDynamics(float speed, const float wheelSpeeds[4], 
                               float steeringAngle, float yawRate) {
        vehicleDynamics.vehicleSpeed = speed;
        for (int i = 0; i < 4; i++) {
            vehicleDynamics.wheelSpeeds[i] = wheelSpeeds[i];
        }
        vehicleDynamics.steeringAngle = steeringAngle;
        vehicleDynamics.yawRate = yawRate;
        
        // Calculate accelerations and angles
        calculateVehicleDynamics();
        
        // Detect road conditions
        detectRoadConditions();
        
        // Update powertrain based on dynamics
        updatePowertrainControl();
    }

    void requestTorque(float totalTorqueNm) {
        if (powertrainState != PowertrainState::READY && 
            powertrainState != PowertrainState::DRIVING) {
            return;
        }
        
        // Distribute torque based on drive mode and conditions
        distributeTorque(totalTorqueNm);
        
        // Apply torque vectoring
        if (torqueVectoringEnabled) {
            applyTorqueVectoring();
        }
        
        // Update motor controllers
        updateMotorControllers();
        
        // Update regenerative braking
        updateRegenerativeBraking();
        
        powertrainState = PowertrainState::DRIVING;
    }

    void enablePowertrain() {
        if (performPreflightChecks()) {
            powertrainState = PowertrainState::STANDBY;
            systemEnabled = true;
            
            // Precharge DC bus
            prechargeSystem();
            
            // Enable motor controllers
            for (auto& motor : motorControllers) {
                motor.enabled = true;
            }
            
            // Enable inverters
            for (auto& inverter : inverters) {
                inverter.enabled = true;
            }
            
            powertrainState = PowertrainState::READY;
            std::cout << "Powertrain enabled and ready" << std::endl;
        } else {
            std::cout << "Powertrain preflight checks failed" << std::endl;
        }
    }

    void disablePowertrain() {
        powertrainState = PowertrainState::STANDBY;
        
        // Set all torques to zero
        for (auto& motor : motorControllers) {
            motor.targetTorque = 0.0f;
            motor.enabled = false;
        }
        
        // Disable inverters
        for (auto& inverter : inverters) {
            inverter.enabled = false;
        }
        
        // Discharge DC bus safely
        dischargeSystem();
        
        systemEnabled = false;
        powertrainState = PowertrainState::OFF;
        std::cout << "Powertrain disabled" << std::endl;
    }

    void emergencyShutdown() {
        std::cout << "EMERGENCY POWERTRAIN SHUTDOWN INITIATED" << std::endl;
        
        powertrainState = PowertrainState::EMERGENCY;
        
        // Immediately disable all power outputs
        for (auto& motor : motorControllers) {
            motor.targetTorque = 0.0f;
            motor.currentTorque = 0.0f;
            motor.enabled = false;
        }
        
        for (auto& inverter : inverters) {
            inverter.enabled = false;
            inverter.dcCurrent = 0.0f;
        }
        
        // Emergency discharge of DC bus
        emergencyDischarge();
        
        systemEnabled = false;
        std::cout << "Emergency shutdown completed" << std::endl;
    }

    void runDiagnostics() {
        std::cout << "\n=== Powertrain Diagnostics ===" << std::endl;
        
        // Motor diagnostics
        diagnosticsActive = true;
        runMotorDiagnostics();
        runInverterDiagnostics();
        runTransmissionDiagnostics();
        runThermalDiagnostics();
        runPowerElectronicsDiagnostics();
        runSystemIntegrityCheck();
        
        std::cout << "Diagnostics completed" << std::endl;
    }

    void getSystemStatus() {
        std::cout << "\n=== Powertrain System Status ===" << std::endl;
        std::cout << "Drive Mode: " << getDriveModeString(currentDriveMode) << std::endl;
        std::cout << "System State: " << getPowertrainStateString(powertrainState) << std::endl;
        std::cout << "System Enabled: " << (systemEnabled ? "Yes" : "No") << std::endl;
        std::cout << "Total Power: " << powerDistribution.totalPower << " kW" << std::endl;
        
        // Motor status
        std::cout << "\nMotor Status:" << std::endl;
        for (size_t i = 0; i < motorControllers.size(); i++) {
            const auto& motor = motorControllers[i];
            std::cout << "  Motor " << i << ": " 
                      << motor.currentTorque << " Nm, "
                      << motor.speed << " RPM, "
                      << motor.temperature << "°C, "
                      << (motor.faultActive ? "FAULT" : "OK") << std::endl;
        }
        
        // Power distribution
        std::cout << "\nPower Distribution:" << std::endl;
        std::cout << "  Front/Rear: " 
                  << (powerDistribution.frontPowerDistribution * 100) << "% / "
                  << (powerDistribution.rearPowerDistribution * 100) << "%" << std::endl;
        std::cout << "  Left/Right: "
                  << (powerDistribution.leftPowerDistribution * 100) << "% / "
                  << (powerDistribution.rightPowerDistribution * 100) << "%" << std::endl;
        
        // Thermal status
        std::cout << "\nThermal Status:" << std::endl;
        for (const auto& loop : thermalMgmt.coolingLoops) {
            std::cout << "  " << loop.name << ": "
                      << loop.inletTemperature << "°C -> "
                      << loop.outletTemperature << "°C" << std::endl;
        }
        
        // Regenerative braking
        std::cout << "\nRegeneration Status:" << std::endl;
        std::cout << "  Current Regen Torque: " << regenBraking.currentRegenTorque << " Nm" << std::endl;
        std::cout << "  Energy Recovered: " << regenBraking.energyRecovered << " kWh" << std::endl;
        
        std::cout << "========================" << std::endl;
    }

private:
    // System state
    DriveMode currentDriveMode;
    PowertrainState powertrainState;
    bool systemEnabled;
    bool torqueVectoringEnabled;
    bool adaptivePowerDistribution;
    bool predictiveControl;
    bool thermalProtection;
    bool emergencyShutdownArmed;
    bool diagnosticsActive;

    // Control parameters
    float powerLimitFactor = 1.0f;
    float thermalLimitFactor = 1.0f;
    float efficiencyOptimization = 0.8f;
    bool regenEnabled = true;
    bool predictiveShiftingEnabled = true;

    // System components
    std::vector<MotorController> motorControllers;
    std::vector<InverterController> inverters;
    TransmissionController transmission;
    std::unordered_map<std::string, DifferentialController> differentials;
    PowerDistributionUnit powerDistribution;
    RegenerativeBrakingController regenBraking;
    ThermalManagement thermalMgmt;
    PowerElectronicsController powerElectronics;
    VehicleDynamicsInterface vehicleDynamics;
    std::atomic<bool> controlLoopRunning;
    std::thread controlThread;

    void configureDriveMode(float powerLimit, float aggressiveness, 
                           float efficiency, bool smoothness, bool ecoMode) {
        powerLimitFactor = powerLimit;
        efficiencyOptimization = efficiency;
        
        // Configure motor response
        for (auto& motor : motorControllers) {
            motor.thermalDeratingFactor = powerLimit;
        }
        
        // Configure power distribution
        powerDistribution.performanceOptimization = aggressiveness;
        powerDistribution.powerLossMinimization = efficiency;
        
        // Configure regenerative braking
        regenBraking.blendingRatio = ecoMode ? 0.8f : 0.6f;
        regenBraking.onepedalDriving = ecoMode;
        
        std::cout << "Drive mode configured - Power: " << (powerLimit * 100) 
                  << "%, Efficiency: " << (efficiency * 100) << "%" << std::endl;
    }

    void calculateVehicleDynamics() {
        // Calculate lateral acceleration from yaw rate and speed
        vehicleDynamics.lateralAcceleration = 
            vehicleDynamics.yawRate * vehicleDynamics.vehicleSpeed / 3.6f;
        
        // Estimate longitudinal acceleration from wheel speed changes
        static float lastSpeed = 0.0f;
        static auto lastTime = std::chrono::steady_clock::now();
        
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - lastTime).count();
        
        if (dt > 0.01f) { // Update every 10ms
            vehicleDynamics.longitudinalAcceleration = 
                (vehicleDynamics.vehicleSpeed - lastSpeed) / dt;
            lastSpeed = vehicleDynamics.vehicleSpeed;
            lastTime = now;
        }
        
        // Calculate roll and pitch angles (simplified)
        vehicleDynamics.rollAngle = vehicleDynamics.lateralAcceleration * 0.1f; // rad
        vehicleDynamics.pitchAngle = vehicleDynamics.longitudinalAcceleration * 0.05f; // rad
    }

    void detectRoadConditions() {
        // Analyze wheel speed differences to detect road friction
        float wheelSpeedDiff = 0.0f;
        float avgWheelSpeed = 0.0f;
        
        for (int i = 0; i < 4; i++) {
            avgWheelSpeed += vehicleDynamics.wheelSpeeds[i];
        }
        avgWheelSpeed /= 4.0f;
        
        for (int i = 0; i < 4; i++) {
            wheelSpeedDiff += std::abs(vehicleDynamics.wheelSpeeds[i] - avgWheelSpeed);
        }
        
        // Estimate road friction based on wheel slip
        if (wheelSpeedDiff > 5.0f) { // Significant wheel speed difference
            vehicleDynamics.roadFriction = 0.3f; // Low friction (ice/snow)
            vehicleDynamics.surfaceType = "Low Friction";
        } else if (wheelSpeedDiff > 2.0f) {
            vehicleDynamics.roadFriction = 0.6f; // Medium friction (wet)
            vehicleDynamics.surfaceType = "Wet";
        } else {
            vehicleDynamics.roadFriction = 0.9f; // High friction (dry)
            vehicleDynamics.surfaceType = "Dry";
        }
        
        vehicleDynamics.roadConditionValid = true;
        
        // Estimate road grade from longitudinal acceleration and motor torque
        // (This is a simplified estimation)
        float expectedAccel = powerDistribution.totalPower / (1500.0f * vehicleDynamics.vehicleSpeed);
        vehicleDynamics.roadGrade = 
            (vehicleDynamics.longitudinalAcceleration - expectedAccel) * 5.7f; // degrees
    }

    void updatePowertrainControl() {
        // Adapt control based on vehicle dynamics and road conditions
        
        // Adjust power distribution for road conditions
        if (vehicleDynamics.roadFriction < 0.5f) {
            // Low friction - distribute power evenly, reduce torque vectoring
            powerDistribution.frontPowerDistribution = 0.5f;
            powerDistribution.rearPowerDistribution = 0.5f;
            torqueVectoringEnabled = false;
        } else {
            // Normal conditions - allow full torque vectoring
            torqueVectoringEnabled = true;
            
            // Adjust distribution based on drive mode and dynamics
            if (currentDriveMode == DriveMode::SPORT) {
                // Sport mode - rear bias for better handling
                powerDistribution.frontPowerDistribution = 0.3f;
                powerDistribution.rearPowerDistribution = 0.7f;
            } else {
                // Balanced distribution
                powerDistribution.frontPowerDistribution = 0.5f;
                powerDistribution.rearPowerDistribution = 0.5f;
            }
        }
        
        // Adjust thermal management based on power demands
        updateThermalManagement();
    }

    void distributeTorque(float totalTorque) {
        // Calculate individual motor torques based on power distribution
        float frontTorque = totalTorque * powerDistribution.frontPowerDistribution;
        float rearTorque = totalTorque * powerDistribution.rearPowerDistribution;
        
        // Distribute to left and right motors
        float frontLeftTorque = frontTorque * powerDistribution.leftPowerDistribution;
        float frontRightTorque = frontTorque * powerDistribution.rightPowerDistribution;
        float rearLeftTorque = rearTorque * powerDistribution.leftPowerDistribution;
        float rearRightTorque = rearTorque * powerDistribution.rightPowerDistribution;
        
        // Apply torque limits based on motor capabilities
        motorControllers[0].targetTorque = limitMotorTorque(0, frontLeftTorque);
        motorControllers[1].targetTorque = limitMotorTorque(1, frontRightTorque);
        motorControllers[2].targetTorque = limitMotorTorque(2, rearLeftTorque);
        motorControllers[3].targetTorque = limitMotorTorque(3, rearRightTorque);
        
        // Calculate total power
        powerDistribution.totalPower = 0.0f;
        for (const auto& motor : motorControllers) {
            powerDistribution.totalPower += motor.power;
        }
    }

    void applyTorqueVectoring() {
        // Calculate desired yaw moment based on steering input and vehicle dynamics
        float desiredYawMoment = vehicleDynamics.steeringAngle * 0.1f * 
                                vehicleDynamics.vehicleSpeed * 0.01f;
        
        // Apply understeer/oversteer compensation
        float yawError = desiredYawMoment - vehicleDynamics.yawRate;
        
        // Calculate torque adjustments
        float torqueAdjustment = yawError * 50.0f; // Proportional gain
        
        // Apply to differentials
        differentials["front"].yawMoment = torqueAdjustment;
        differentials["rear"].yawMoment = torqueAdjustment;
        
        // Adjust individual motor torques
        motorControllers[1].targetTorque += torqueAdjustment; // Front right
        motorControllers[3].targetTorque += torqueAdjustment; // Rear right
        motorControllers[0].targetTorque -= torqueAdjustment; // Front left
        motorControllers[2].targetTorque -= torqueAdjustment; // Rear left
        
        // Ensure torque limits are not exceeded
        for (int i = 0; i < 4; i++) {
            motorControllers[i].targetTorque = limitMotorTorque(i, motorControllers[i].targetTorque);
        }
    }

    float limitMotorTorque(int motorIndex, float requestedTorque) {
        const auto& motor = motorControllers[motorIndex];
        
        // Get maximum torque at current speed
        int speedIndex = std::min(99, (int)(motor.speed / 100.0f));
        float maxTorque = motor.maxTorqueAtSpeed[speedIndex];
        
        // Apply thermal derating
        maxTorque *= motor.thermalDeratingFactor;
        
        // Apply system power limit
        maxTorque *= powerLimitFactor;
        
        // Limit to available torque
        return std::clamp(requestedTorque, -maxTorque, maxTorque);
    }

    void updateMotorControllers() {
        for (auto& motor : motorControllers) {
            if (!motor.enabled) continue;
            
            // Simple first-order response to target torque
            float torqueError = motor.targetTorque - motor.currentTorque;
            motor.currentTorque += torqueError * 0.1f; // Response rate
            
            // Calculate power
            motor.power = motor.currentTorque * motor.speed * 2.0f * M_PI / (60.0f * 1000.0f); // kW
            
            // Update temperature based on power and cooling
            float heatGeneration = motor.power * (1.0f - motor.efficiency);
            float cooling = (motor.temperature - 25.0f) * 0.1f; // Simple cooling model
            motor.temperature += (heatGeneration * 10.0f - cooling) * 0.01f;
            
            // Thermal protection
            if (motor.temperature > 120.0f) {
                motor.thermalDeratingFactor = 0.5f;
                if (motor.temperature > 150.0f) {
                    motor.faultActive = true;
                    motor.faultCode = "THERMAL_FAULT";
                    motor.enabled = false;
                }
            } else {
                motor.thermalDeratingFactor = 1.0f;
            }
            
            // Update vibration based on torque ripple
            motor.vibrationLevel = std::abs(motor.currentTorque) * 0.01f + 
                                  std::sin(motor.speed * 0.1f) * 0.5f;
            
            motor.lastUpdate = std::chrono::steady_clock::now();
        }
    }

    void updateRegenerativeBraking() {
        // Calculate available regen torque based on speed
        int speedIndex = std::min(99, (int)(vehicleDynamics.vehicleSpeed / 2.0f));
        float availableRegenTorque = regenBraking.regenMap[speedIndex];
        
        // Apply terrain and weather adaptations
        availableRegenTorque *= regenBraking.terrainAdaptation;
        availableRegenTorque *= regenBraking.weatherAdaptation;
        
        // Calculate current regen demand (simplified)
        if (vehicleDynamics.longitudinalAcceleration < -0.1f) { // Deceleration
            regenBraking.currentRegenTorque = 
                std::min(availableRegenTorque, std::abs(vehicleDynamics.longitudinalAcceleration) * 100.0f);
        } else {
            regenBraking.currentRegenTorque = 0.0f;
        }
        
        // Calculate regen power
        regenBraking.regenPower = regenBraking.currentRegenTorque * 
                                 vehicleDynamics.vehicleSpeed * 4.0f / 1000.0f; // kW (4 motors)
        
        // Update energy recovered
        static auto lastTime = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - lastTime).count();
        regenBraking.energyRecovered += regenBraking.regenPower * dt / 3600.0f; // kWh
        lastTime = now;
    }

    void updateThermalManagement() {
        // Update cooling loop temperatures based on heat loads
        for (auto& loop : thermalMgmt.coolingLoops) {
            if (loop.name == "Motor Cooling") {
                // Calculate motor heat load
                float totalMotorHeat = 0.0f;
                for (const auto& motor : motorControllers) {
                    totalMotorHeat += motor.power * (1.0f - motor.efficiency);
                }
                
                loop.outletTemperature = loop.inletTemperature + 
                                       (totalMotorHeat * 2.0f) / loop.flowRate;
                
                // Adjust pump speed based on temperature
                if (loop.outletTemperature > 80.0f) {
                    loop.pumpSpeed = 100.0f;
                } else {
                    loop.pumpSpeed = 50.0f + (loop.outletTemperature - 25.0f);
                }
            }
        }
        
        // Check for thermal emergency
        for (const auto& loop : thermalMgmt.coolingLoops) {
            if (loop.outletTemperature > 100.0f) {
                thermalMgmt.thermalEmergency = true;
                std::cout << "THERMAL EMERGENCY: " << loop.name << " overheating!" << std::endl;
            }
        }
        
        // Predictive thermal control
        if (thermalMgmt.preemptiveCooling) {
            // Predict thermal load based on current power trends
            thermalMgmt.thermalLoadPrediction = powerDistribution.totalPower * 1.2f; // 20% margin
            
            // Increase cooling proactively
            for (auto& loop : thermalMgmt.coolingLoops) {
                if (thermalMgmt.thermalLoadPrediction > 50.0f) { // High load predicted
                    loop.pumpSpeed = std::min(100.0f, loop.pumpSpeed * 1.2f);
                }
            }
        }
    }

    bool performPreflightChecks() {
        std::cout << "Performing powertrain preflight checks..." << std::endl;
        
        bool allChecksPass = true;
        
        // Check motor controllers
        for (const auto& motor : motorControllers) {
            if (motor.faultActive) {
                std::cout << "Motor fault detected: " << motor.faultCode << std::endl;
                allChecksPass = false;
            }
        }
        
        // Check inverters
        for (const auto& inverter : inverters) {
            if (inverter.faultActive) {
                std::cout << "Inverter fault detected" << std::endl;
                allChecksPass = false;
            }
        }
        
        // Check power electronics
        if (powerElectronics.insulationResistance < 100.0f) {
            std::cout << "Insulation resistance too low" << std::endl;
            allChecksPass = false;
        }
        
        // Check thermal system
        if (thermalMgmt.thermalEmergency) {
            std::cout << "Thermal emergency active" << std::endl;
            allChecksPass = false;
        }
        
        return allChecksPass;
    }

    void prechargeSystem() {
        std::cout << "Precharging DC bus..." << std::endl;
        
        // Simulate precharge sequence
        for (int i = 0; i <= 100; i += 10) {
            powerElectronics.dcBusVoltage = 400.0f * i / 100.0f;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        powerElectronics.prechargeComplete = true;
        std::cout << "DC bus precharge complete: " << powerElectronics.dcBusVoltage << "V" << std::endl;
    }

    void dischargeSystem() {
        std::cout << "Discharging DC bus..." << std::endl;
        
        // Simulate controlled discharge
        while (powerElectronics.dcBusVoltage > 50.0f) {
            powerElectronics.dcBusVoltage *= 0.9f;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        powerElectronics.prechargeComplete = false;
        std::cout << "DC bus discharge complete" << std::endl;
    }

    void emergencyDischarge() {
        std::cout << "Emergency DC bus discharge..." << std::endl;
        powerElectronics.dcBusVoltage = 0.0f;
        powerElectronics.prechargeComplete = false;
        std::cout << "Emergency discharge complete" << std::endl;
    }

    std::string getDriveModeString(DriveMode mode) {
        switch (mode) {
            case DriveMode::ECO: return "ECO";
            case DriveMode::COMFORT: return "COMFORT";
            case DriveMode::SPORT: return "SPORT";
            case DriveMode::TRACK: return "TRACK";
            case DriveMode::SNOW: return "SNOW";
            case DriveMode::SAND: return "SAND";
            case DriveMode::MUD: return "MUD";
            case DriveMode::ROCK: return "ROCK";
            case DriveMode::CUSTOM: return "CUSTOM";
        }
        return "UNKNOWN";
    }

    std::string getPowertrainStateString(PowertrainState state) {
        switch (state) {
            case PowertrainState::OFF: return "OFF";
            case PowertrainState::STANDBY: return "STANDBY";
            case PowertrainState::READY: return "READY";
            case PowertrainState::DRIVING: return "DRIVING";
            case PowertrainState::REGENERATING: return "REGENERATING";
            case PowertrainState::CHARGING: return "CHARGING";
            case PowertrainState::FAULT: return "FAULT";
            case PowertrainState::EMERGENCY: return "EMERGENCY";
        }
        return "UNKNOWN";
    }

    void runMotorDiagnostics() {
        std::cout << "\nMotor Diagnostics:" << std::endl;
        for (size_t i = 0; i < motorControllers.size(); i++) {
            const auto& motor = motorControllers[i];
            std::cout << "  Motor " << i << ":" << std::endl;
            std::cout << "    Temperature: " << motor.temperature << "°C" << std::endl;
            std::cout << "    Vibration: " << motor.vibrationLevel << " g" << std::endl;
            std::cout << "    Efficiency: " << (motor.efficiency * 100) << "%" << std::endl;
            std::cout << "    Status: " << (motor.faultActive ? "FAULT" : "OK") << std::endl;
        }
    }

    void runInverterDiagnostics() {
        std::cout << "\nInverter Diagnostics:" << std::endl;
        for (size_t i = 0; i < inverters.size(); i++) {
            const auto& inv = inverters[i];
            std::cout << "  Inverter " << i << ":" << std::endl;
            std::cout << "    DC Voltage: " << inv.dcVoltage << "V" << std::endl;
            std::cout << "    Temperature: " << inv.temperature << "°C" << std::endl;
            std::cout << "    Efficiency: " << (inv.efficiency * 100) << "%" << std::endl;
            std::cout << "    THD: " << (powerElectronics.harmonicDistortion * 100) << "%" << std::endl;
        }
    }

    void runTransmissionDiagnostics() {
        std::cout << "\nTransmission Diagnostics:" << std::endl;
        std::cout << "  Current Gear: " << (int)transmission.currentGear << std::endl;
        std::cout << "  Oil Temperature: " << transmission.oilTemperature << "°C" << std::endl;
        std::cout << "  Oil Pressure: " << transmission.oilPressure << " bar" << std::endl;
        std::cout << "  Efficiency: " << (transmission.efficiency * 100) << "%" << std::endl;
    }

    void runThermalDiagnostics() {
        std::cout << "\nThermal Diagnostics:" << std::endl;
        for (const auto& loop : thermalMgmt.coolingLoops) {
            std::cout << "  " << loop.name << ":" << std::endl;
            std::cout << "    Inlet: " << loop.inletTemperature << "°C" << std::endl;
            std::cout << "    Outlet: " << loop.outletTemperature << "°C" << std::endl;
            std::cout << "    Flow Rate: " << loop.flowRate << " L/min" << std::endl;
            std::cout << "    Pump Speed: " << loop.pumpSpeed << "%" << std::endl;
        }
    }

    void runPowerElectronicsDiagnostics() {
        std::cout << "\nPower Electronics Diagnostics:" << std::endl;
        std::cout << "  DC Bus Voltage: " << powerElectronics.dcBusVoltage << "V" << std::endl;
        std::cout << "  Power Factor: " << powerElectronics.powerFactor << std::endl;
        std::cout << "  Insulation Resistance: " << powerElectronics.insulationResistance << " kΩ" << std::endl;
        std::cout << "  Power Quality: " << (powerElectronics.powerQuality * 100) << "%" << std::endl;
    }

    void runSystemIntegrityCheck() {
        std::cout << "\nSystem Integrity Check:" << std::endl;
        
        bool integrityOk = true;
        
        // Check communication between components
        for (const auto& motor : motorControllers) {
            auto timeSinceUpdate = std::chrono::steady_clock::now() - motor.lastUpdate;
            if (timeSinceUpdate > std::chrono::milliseconds(100)) {
                std::cout << "  WARNING: Motor communication timeout" << std::endl;
                integrityOk = false;
            }
        }
        
        // Check power balance
        float totalMotorPower = 0.0f;
        for (const auto& motor : motorControllers) {
            totalMotorPower += motor.power;
        }
        
        if (std::abs(totalMotorPower - powerDistribution.totalPower) > 1.0f) {
            std::cout << "  WARNING: Power balance mismatch" << std::endl;
            integrityOk = false;
        }
        
        std::cout << "  System Integrity: " << (integrityOk ? "OK" : "WARNING") << std::endl;
    }

    void startControlLoop() {
        controlLoopRunning.store(true);
        controlThread = std::thread([this]() {
            while (controlLoopRunning.load()) {
                // Update all controllers at 100Hz when enabled
                if (systemEnabled) {
                    updateMotorControllers();
                    updateRegenerativeBraking();
                    updateThermalManagement();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }

    void stopControlLoop() {
        controlLoopRunning.store(false);
        systemEnabled = false;
        powertrainState = PowertrainState::OFF;
        if (controlThread.joinable()) {
            controlThread.join();
        }
    }
};

class PowertrainTestController {
public:
    PowertrainTestController() : powertrain(std::make_unique<AdvancedPowertrainControl>()) {
        std::cout << "Powertrain Test Controller initialized" << std::endl;
    }

    void runComprehensiveTest() {
        std::cout << "\n=== Comprehensive Powertrain Test ===" << std::endl;
        
        testSystemInitialization();
        testDriveModes();
        testPowerDistribution();
        testTorqueVectoring();
        testRegenerativeBraking();
        testThermalManagement();
        testEmergencyProcedures();
        
        std::cout << "Comprehensive powertrain test completed" << std::endl;
    }

private:
    std::unique_ptr<AdvancedPowertrainControl> powertrain;

    void testSystemInitialization() {
        std::cout << "\nTesting system initialization..." << std::endl;
        
        powertrain->enablePowertrain();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        powertrain->getSystemStatus();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        std::cout << "System initialization test completed" << std::endl;
    }

    void testDriveModes() {
        std::cout << "\nTesting drive modes..." << std::endl;
        
        // Test all drive modes
        powertrain->setDriveMode(AdvancedPowertrainControl::DriveMode::ECO);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        powertrain->setDriveMode(AdvancedPowertrainControl::DriveMode::SPORT);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        powertrain->setDriveMode(AdvancedPowertrainControl::DriveMode::SNOW);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        powertrain->setDriveMode(AdvancedPowertrainControl::DriveMode::COMFORT);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        std::cout << "Drive modes test completed" << std::endl;
    }

    void testPowerDistribution() {
        std::cout << "\nTesting power distribution..." << std::endl;
        
        // Simulate various driving scenarios
        float wheelSpeeds[4] = {50.0f, 50.0f, 50.0f, 50.0f};
        
        // Straight line driving
        powertrain->updateVehicleDynamics(50.0f, wheelSpeeds, 0.0f, 0.0f);
        powertrain->requestTorque(200.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Cornering
        powertrain->updateVehicleDynamics(40.0f, wheelSpeeds, 0.2f, 0.1f);
        powertrain->requestTorque(150.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Low friction conditions
        wheelSpeeds[0] = 45.0f; wheelSpeeds[1] = 55.0f;
        wheelSpeeds[2] = 48.0f; wheelSpeeds[3] = 52.0f;
        powertrain->updateVehicleDynamics(25.0f, wheelSpeeds, 0.0f, 0.0f);
        powertrain->requestTorque(100.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        std::cout << "Power distribution test completed" << std::endl;
    }

    void testTorqueVectoring() {
        std::cout << "\nTesting torque vectoring..." << std::endl;
        
        float wheelSpeeds[4] = {50.0f, 50.0f, 50.0f, 50.0f};
        
        // Simulate aggressive cornering
        powertrain->updateVehicleDynamics(60.0f, wheelSpeeds, 0.5f, 0.3f);
        powertrain->requestTorque(300.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        powertrain->getSystemStatus();
        std::cout << "Torque vectoring test completed" << std::endl;
    }

    void testRegenerativeBraking() {
        std::cout << "\nTesting regenerative braking..." << std::endl;
        
        float wheelSpeeds[4] = {60.0f, 60.0f, 60.0f, 60.0f};
        
        // Simulate deceleration
        powertrain->updateVehicleDynamics(60.0f, wheelSpeeds, 0.0f, 0.0f);
        powertrain->requestTorque(-150.0f); // Negative torque for braking
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Gradual deceleration
        for (int i = 0; i < 10; i++) {
            float speed = 60.0f - i * 6.0f;
            for (int j = 0; j < 4; j++) {
                wheelSpeeds[j] = speed;
            }
            powertrain->updateVehicleDynamics(speed, wheelSpeeds, 0.0f, 0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::cout << "Regenerative braking test completed" << std::endl;
    }

    void testThermalManagement() {
        std::cout << "\nTesting thermal management..." << std::endl;
        
        // Simulate high power operation
        float wheelSpeeds[4] = {80.0f, 80.0f, 80.0f, 80.0f};
        powertrain->updateVehicleDynamics(80.0f, wheelSpeeds, 0.0f, 0.0f);
        
        // Apply high torque for extended period
        for (int i = 0; i < 20; i++) {
            powertrain->requestTorque(400.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        powertrain->getSystemStatus();
        std::cout << "Thermal management test completed" << std::endl;
    }

    void testEmergencyProcedures() {
        std::cout << "\nTesting emergency procedures..." << std::endl;
        
        // Test emergency shutdown
        powertrain->emergencyShutdown();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Restart system
        powertrain->enablePowertrain();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "Emergency procedures test completed" << std::endl;
    }
};

#endif // ADVANCED_POWERTRAIN_CONTROL_H
