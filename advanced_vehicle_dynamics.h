#ifndef ADVANCED_VEHICLE_DYNAMICS_H
#define ADVANCED_VEHICLE_DYNAMICS_H

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
#include <random>
#include <queue>

class AdvancedVehicleDynamicsControl {
public:
    enum class VehicleMode {
        COMFORT,
        SPORT,
        TRACK,
        ECO,
        CUSTOM,
        SNOW,
        SAND,
        MUD,
        ROCK,
        DRIFT,
        LAUNCH
    };

    enum class ESCMode {
        ON,
        SPORT,
        OFF,
        TRACK_MODE,
        DRIFT_MODE
    };

    enum class TractionControlMode {
        FULL,
        SPORT,
        OFF,
        SNOW,
        SAND_MUD
    };

    struct WheelDynamics {
        float wheelSpeed;           // rad/s
        float wheelTorque;          // Nm
        float wheelForce;           // N
        float slipRatio;            // %
        float slipAngle;            // rad
        float lateralForce;         // N
        float longitudinalForce;    // N
        float verticalLoad;         // N
        float tirePressure;         // bar
        float tireTemperature;      // °C
        float treadDepth;           // mm
        float wearLevel;            // %
        bool locked;                // Brake lock status
        bool spinning;              // Traction loss
        
        // Advanced tire model parameters
        float corneringStiffness;   // N/rad
        float peakFriction;         // coefficient
        float optimalSlipRatio;     // %
        float thermalDegradation;   // %
        float compoundType;         // 0-1 soft to hard
        float sidewallStiffness;    // N/m
    };

    struct VehicleState {
        // Position and orientation
        float x, y, z;              // Position (m)
        float roll, pitch, yaw;     // Attitude (rad)
        float rollRate, pitchRate, yawRate; // Angular velocities (rad/s)
        
        // Linear motion
        float velocityX, velocityY, velocityZ; // Body frame velocity (m/s)
        float accelerationX, accelerationY, accelerationZ; // Body frame acceleration (m/s²)
        
        // Derived parameters
        float speed;                // Total speed (m/s)
        float lateralAcceleration;  // m/s²
        float longitudinalAcceleration; // m/s²
        float verticalAcceleration; // m/s²
        float sideslipAngle;        // rad
        float courseAngle;          // rad
        float bankAngle;            // Road bank angle (rad)
        float roadGradient;         // Road gradient (rad)
        
        // Center of gravity
        float cogHeight;            // m
        float cogX, cogY;           // m from vehicle center
        float mass;                 // kg
        float momentOfInertiaYaw;   // kg⋅m²
        float momentOfInertiaRoll;  // kg⋅m²
        float momentOfInertiaPitch; // kg⋅m²
    };

    struct ElectronicStabilityControl {
        bool enabled;
        ESCMode mode;
        float interventionLevel;    // 0-1
        float yawMomentDemand;      // Nm
        float underSteerGradient;   // rad/(m/s²)
        float overSteerGradient;    // rad/(m/s²)
        float stabilityMargin;      // 0-1
        bool intervening;
        
        // Intervention thresholds
        float yawRateThreshold;     // rad/s
        float sideslipThreshold;    // rad
        float lateralAccelThreshold; // m/s²
        
        // Control gains
        float yawRateGain;
        float sideslipGain;
        float lateralAccelGain;
        
        // Intervention strategies
        struct BrakeIntervention {
            float frontLeft, frontRight;
            float rearLeft, rearRight;
            bool active;
        } brakeIntervention;
        
        struct TorqueIntervention {
            float totalReduction;      // %
            float individualReduction[4]; // % per wheel
            bool active;
        } torqueIntervention;
    };

    struct TractionControl {
        bool enabled;
        TractionControlMode mode;
        float slipThreshold;        // %
        float interventionLevel;    // 0-1
        bool intervening[4];        // Per wheel
        
        // Slip control parameters
        float targetSlipRatio;      // %
        float maxSlipRatio;         // %
        float slipRatePrediction;   // %/s
        
        // Control strategies
        float torqueReduction[4];   // % per wheel
        float brakeApplication[4];  // bar per wheel
        
        // Launch control
        struct LaunchControl {
            bool enabled;
            float targetSlipRatio;   // %
            float rpmLimit;          // RPM
            float boostPressure;     // bar (if applicable)
            float clutchSlipTarget;  // % (if applicable)
        } launchControl;
    };

    struct AntiLockBrakingSystem {
        bool enabled;
        bool active[4];             // Per wheel
        float pumpPressure;         // bar
        float modulationFrequency;  // Hz
        
        // Wheel-specific parameters
        float wheelSlipThreshold;   // %
        float wheelSlipTarget;      // %
        float brakeForceModulation[4]; // % per wheel
        
        // Advanced ABS features
        struct ElectronicBrakeforceDistribution {
            bool enabled;
            float frontRearBias;    // %
            float leftRightBias;    // %
            float dynamicBias;      // %
        } ebd;
        
        struct BrakeAssist {
            bool enabled;
            float emergencyThreshold; // m/s²
            float boostFactor;      // multiplier
            bool active;
        } brakeAssist;
        
        struct HillHoldControl {
            bool enabled;
            bool active;
            float holdPressure;     // bar
            float releaseThreshold; // % throttle
            float gradient;         // rad
        } hillHoldControl;
    };

    struct ActiveAerodynamics {
        struct FrontSpoiler {
            float angle;            // degrees
            float downforce;        // N
            float drag;             // N
            bool active;
        } frontSpoiler;
        
        struct RearWing {
            float angle;            // degrees
            float downforce;        // N
            float drag;             // N
            bool drsActive;         // Drag Reduction System
        } rearWing;
        
        struct SideVanes {
            float angle;            // degrees
            float sideForce;        // N
            bool active;
        } sideVanes;
        
        struct UnderfloorPanels {
            float groundClearance;  // mm
            float venturiEffect;    // N downforce
            bool active;
        } underfloorPanels;
        
        struct ActiveGrille {
            float openingPercentage; // %
            float dragReduction;    // N
            float coolingEffect;    // %
        } activeGrille;
        
        float totalDownforce;       // N
        float totalDrag;            // N
        float aerodynamicBalance;   // % front
        float dragCoefficient;      // Cd
        float liftCoefficient;      // Cl
    };

    struct DynamicDamping {
        enum class DamperMode {
            COMFORT,
            SPORT,
            TRACK,
            ADAPTIVE,
            CUSTOM
        };
        
        struct DamperControl {
            float compression;      // N⋅s/m
            float rebound;          // N⋅s/m
            float position;         // mm from neutral
            float velocity;         // mm/s
            float force;            // N
            bool active;
            
            // Adaptive parameters
            float stiffnessRatio;   // 0-1
            float dampingRatio;     // 0-1
            float preloadForce;     // N
        };
        
        DamperControl frontLeft, frontRight;
        DamperControl rearLeft, rearRight;
        DamperMode currentMode;
        
        // Sky-Hook control
        bool skyHookEnabled;
        float skyHookGain;
        float groundHookGain;
        
        // Frequency analysis
        float sprungMassFreq;       // Hz
        float unsprungMassFreq;     // Hz
        float transmissibility;     // ratio
        
        // Road surface adaptation
        float roughnessIndex;       // 0-10
        float surfaceFrequency;     // Hz
        bool roughRoadMode;
    };

    struct TorqueVectoring {
        bool enabled;
        float yawMomentDemand;      // Nm
        float torqueDistribution[4]; // % to each wheel
        float torqueBias;           // Front/rear bias
        float leftRightBias;        // Left/right bias
        
        // Cornering enhancement
        float understeerCompensation; // Nm
        float oversteerCompensation;  // Nm
        float corneringTorque;        // Nm
        
        // Performance modes
        struct DriftMode {
            bool enabled;
            float rearBias;         // %
            float slipAngleTarget;  // rad
            float powerOversteer;   // %
        } driftMode;
        
        struct LaunchMode {
            bool enabled;
            float maxTorqueLimit;   // Nm
            float rpmLimit;         // RPM
            float slipRatioTarget;  // %
        } launchMode;
        
        // Predictive control
        float steeringAnglePrediction; // rad
        float lateralAccelPrediction;  // m/s²
        float yawRatePrediction;       // rad/s
    };

    struct RollStabilization {
        bool enabled;
        float rollAngle;            // rad
        float rollRate;             // rad/s
        float rollAcceleration;     // rad/s²
        float rollMoment;           // Nm
        
        // Anti-roll bars
        struct AntiRollBar {
            float stiffness;        // Nm/rad
            float torque;           // Nm
            bool active;
            float electricActuation; // Nm (for active systems)
        };
        
        AntiRollBar front, rear;
        
        // Active roll control
        bool activeRollControl;
        float rollStiffnessRatio;   // Front/rear
        float rollDamping;          // Nm⋅s/rad
        float rollTargetAngle;      // rad
        
        // Load transfer control
        float lateralLoadTransfer;  // N
        float rollCenter[2];        // Height front/rear (m)
        float rollAxis;             // rad
    };

public:
    AdvancedVehicleDynamicsControl() :
        currentMode(VehicleMode::COMFORT),
        systemsEnabled(true),
        diagnosticsActive(true),
        dataLoggingActive(true),
        realTimeMonitoring(true),
        performanceOptimization(true),
        safetyOverride(true),
        weatherAdaptation(true) {
        
        initializeVehicleParameters();
        initializeControlSystems();
        initializeWheelDynamics();
        initializeSensorSystems();
        initializeActuatorSystems();
        startControlLoop();
        
        std::cout << "Advanced Vehicle Dynamics Control System initialized" << std::endl;
    }

    void initializeVehicleParameters() {
        // Vehicle mass and inertia properties
        vehicleState.mass = 2200.0f;  // kg (typical EV)
        vehicleState.cogHeight = 0.5f; // m
        vehicleState.cogX = 0.0f;     // m (centered)
        vehicleState.cogY = 0.0f;     // m (centered)
        vehicleState.momentOfInertiaYaw = 3500.0f;   // kg⋅m²
        vehicleState.momentOfInertiaRoll = 800.0f;   // kg⋅m²
        vehicleState.momentOfInertiaPitch = 3200.0f; // kg⋅m²
        
        // Track and wheelbase
        trackWidth = 1.6f;      // m
        wheelbase = 2.8f;       // m
        
        // Aerodynamic parameters
        aerodynamics.dragCoefficient = 0.25f;  // Cd (excellent for EV)
        aerodynamics.liftCoefficient = 0.05f;  // Cl
        frontalArea = 2.3f;     // m²
        
        // Tire parameters
        tireRadius = 0.35f;     // m
        tireWidth = 0.235f;     // m
        
        std::cout << "Vehicle parameters initialized" << std::endl;
    }

    void initializeControlSystems() {
        // Electronic Stability Control
        esc.enabled = true;
        esc.mode = ESCMode::ON;
        esc.interventionLevel = 0.8f;
        esc.yawRateThreshold = 0.5f;        // rad/s
        esc.sideslipThreshold = 0.15f;      // rad (about 8.6 degrees)
        esc.lateralAccelThreshold = 8.0f;   // m/s²
        esc.yawRateGain = 2000.0f;          // Nm/(rad/s)
        esc.sideslipGain = 15000.0f;        // Nm/rad
        esc.lateralAccelGain = 500.0f;      // Nm/(m/s²)
        esc.intervening = false;
        esc.brakeIntervention.active = false;
        esc.torqueIntervention.active = false;
        
        // Traction Control
        tc.enabled = true;
        tc.mode = TractionControlMode::FULL;
        tc.slipThreshold = 8.0f;            // %
        tc.targetSlipRatio = 5.0f;          // %
        tc.maxSlipRatio = 15.0f;            // %
        tc.interventionLevel = 0.9f;
        
        for (int i = 0; i < 4; i++) {
            tc.intervening[i] = false;
            tc.torqueReduction[i] = 0.0f;
            tc.brakeApplication[i] = 0.0f;
        }
        
        // Launch Control
        tc.launchControl.enabled = false;
        tc.launchControl.targetSlipRatio = 8.0f;    // %
        tc.launchControl.rpmLimit = 6000.0f;        // RPM
        
        // Anti-lock Braking System
        abs.enabled = true;
        abs.pumpPressure = 180.0f;          // bar
        abs.modulationFrequency = 15.0f;    // Hz
        abs.wheelSlipThreshold = 12.0f;     // %
        abs.wheelSlipTarget = 8.0f;         // %
        
        for (int i = 0; i < 4; i++) {
            abs.active[i] = false;
            abs.brakeForceModulation[i] = 1.0f;
        }
        
        // Electronic Brakeforce Distribution
        abs.ebd.enabled = true;
        abs.ebd.frontRearBias = 70.0f;      // % to front
        abs.ebd.leftRightBias = 50.0f;      // % balanced
        abs.ebd.dynamicBias = 0.0f;
        
        // Brake Assist
        abs.brakeAssist.enabled = true;
        abs.brakeAssist.emergencyThreshold = 8.0f;  // m/s²
        abs.brakeAssist.boostFactor = 1.5f;
        abs.brakeAssist.active = false;
        
        // Hill Hold Control
        abs.hillHoldControl.enabled = true;
        abs.hillHoldControl.active = false;
        abs.hillHoldControl.holdPressure = 0.0f;
        abs.hillHoldControl.releaseThreshold = 15.0f; // % throttle
        
        // Torque Vectoring
        tv.enabled = true;
        tv.yawMomentDemand = 0.0f;
        tv.torqueBias = 50.0f;              // % front/rear
        tv.leftRightBias = 50.0f;           // % left/right
        
        for (int i = 0; i < 4; i++) {
            tv.torqueDistribution[i] = 25.0f; // % equally distributed
        }
        
        // Drift Mode
        tv.driftMode.enabled = false;
        tv.driftMode.rearBias = 80.0f;      // %
        tv.driftMode.slipAngleTarget = 0.3f; // rad (about 17 degrees)
        tv.driftMode.powerOversteer = 70.0f; // %
        
        std::cout << "Control systems initialized" << std::endl;
    }

    void initializeWheelDynamics() {
        for (int i = 0; i < 4; i++) {
            wheels[i].wheelSpeed = 0.0f;
            wheels[i].wheelTorque = 0.0f;
            wheels[i].wheelForce = 0.0f;
            wheels[i].slipRatio = 0.0f;
            wheels[i].slipAngle = 0.0f;
            wheels[i].lateralForce = 0.0f;
            wheels[i].longitudinalForce = 0.0f;
            wheels[i].verticalLoad = vehicleState.mass * 9.81f / 4.0f; // N
            wheels[i].tirePressure = 2.5f;      // bar
            wheels[i].tireTemperature = 25.0f;  // °C
            wheels[i].treadDepth = 8.0f;        // mm
            wheels[i].wearLevel = 0.0f;         // %
            wheels[i].locked = false;
            wheels[i].spinning = false;
            
            // Tire model parameters
            wheels[i].corneringStiffness = 80000.0f;    // N/rad
            wheels[i].peakFriction = 1.2f;              // coefficient
            wheels[i].optimalSlipRatio = 12.0f;         // %
            wheels[i].thermalDegradation = 0.0f;        // %
            wheels[i].compoundType = 0.5f;              // medium compound
            wheels[i].sidewallStiffness = 200000.0f;    // N/m
        }
        
        std::cout << "Wheel dynamics initialized: 4 wheels" << std::endl;
    }

    void initializeSensorSystems() {
        // IMU (Inertial Measurement Unit)
        imuData.accelerationX = 0.0f;
        imuData.accelerationY = 0.0f;
        imuData.accelerationZ = -9.81f;  // Gravity
        imuData.gyroX = 0.0f;            // Roll rate
        imuData.gyroY = 0.0f;            // Pitch rate
        imuData.gyroZ = 0.0f;            // Yaw rate
        imuData.temperature = 25.0f;
        imuData.calibrated = true;
        
        // Wheel speed sensors
        for (int i = 0; i < 4; i++) {
            wheelSpeedSensors[i].frequency = 0.0f;      // Hz
            wheelSpeedSensors[i].resolution = 48;        // pulses per revolution
            wheelSpeedSensors[i].active = true;
            wheelSpeedSensors[i].faultDetected = false;
        }
        
        // Steering angle sensor
        steeringAngleSensor.angle = 0.0f;               // rad
        steeringAngleSensor.rate = 0.0f;                // rad/s
        steeringAngleSensor.torque = 0.0f;              // Nm
        steeringAngleSensor.calibrated = true;
        steeringAngleSensor.active = true;
        
        // Pressure sensors
        for (int i = 0; i < 4; i++) {
            brakePressureSensors[i].pressure = 0.0f;    // bar
            brakePressureSensors[i].temperature = 25.0f; // °C
            brakePressureSensors[i].active = true;
        }
        
        std::cout << "Sensor systems initialized" << std::endl;
    }

    void initializeActuatorSystems() {
        // Dynamic damping system
        damping.currentMode = DynamicDamping::DamperMode::ADAPTIVE;
        damping.skyHookEnabled = true;
        damping.skyHookGain = 1500.0f;      // N⋅s/m
        damping.groundHookGain = 800.0f;    // N⋅s/m
        damping.sprungMassFreq = 1.2f;      // Hz
        damping.unsprungMassFreq = 12.0f;   // Hz
        damping.transmissibility = 0.3f;
        damping.roughnessIndex = 3.0f;
        damping.roughRoadMode = false;
        
        // Initialize dampers
        initializeDamper(damping.frontLeft, 2500.0f, 800.0f);
        initializeDamper(damping.frontRight, 2500.0f, 800.0f);
        initializeDamper(damping.rearLeft, 2200.0f, 750.0f);
        initializeDamper(damping.rearRight, 2200.0f, 750.0f);
        
        // Active aerodynamics
        aerodynamics.frontSpoiler.angle = 0.0f;         // degrees
        aerodynamics.frontSpoiler.active = true;
        aerodynamics.rearWing.angle = 5.0f;             // degrees
        aerodynamics.rearWing.drsActive = false;
        aerodynamics.rearWing.drsActive = true;
        aerodynamics.sideVanes.angle = 0.0f;
        aerodynamics.sideVanes.active = false;
        aerodynamics.underfloorPanels.groundClearance = 120.0f; // mm
        aerodynamics.underfloorPanels.active = true;
        aerodynamics.activeGrille.openingPercentage = 50.0f;    // %
        
        // Roll stabilization
        rollStab.enabled = true;
        rollStab.activeRollControl = true;
        rollStab.rollStiffnessRatio = 60.0f;    // % front
        rollStab.rollDamping = 3000.0f;         // Nm⋅s/rad
        rollStab.rollTargetAngle = 0.0f;        // rad
        
        // Anti-roll bars
        rollStab.front.stiffness = 25000.0f;    // Nm/rad
        rollStab.front.active = true;
        rollStab.rear.stiffness = 22000.0f;     // Nm/rad
        rollStab.rear.active = true;
        
        rollStab.rollCenter[0] = 0.15f;         // m (front)
        rollStab.rollCenter[1] = 0.18f;         // m (rear)
        
        std::cout << "Actuator systems initialized" << std::endl;
    }

    void setVehicleMode(VehicleMode mode) {
        currentMode = mode;
        
        switch (mode) {
            case VehicleMode::COMFORT:
                configureComfortMode();
                break;
            case VehicleMode::SPORT:
                configureSportMode();
                break;
            case VehicleMode::TRACK:
                configureTrackMode();
                break;
            case VehicleMode::ECO:
                configureEcoMode();
                break;
            case VehicleMode::SNOW:
                configureSnowMode();
                break;
            case VehicleMode::SAND:
                configureSandMode();
                break;
            case VehicleMode::MUD:
                configureMudMode();
                break;
            case VehicleMode::ROCK:
                configureRockMode();
                break;
            case VehicleMode::DRIFT:
                configureDriftMode();
                break;
            case VehicleMode::LAUNCH:
                configureLaunchMode();
                break;
            case VehicleMode::CUSTOM:
                configureCustomMode();
                break;
        }
        
        std::cout << "Vehicle mode set to: " << getVehicleModeString(mode) << std::endl;
    }

    void updateVehicleState(float deltaTime) {
        // Update IMU data
        updateIMUData();
        
        // Update wheel dynamics
        updateWheelDynamics(deltaTime);
        
        // Calculate vehicle motion
        calculateVehicleMotion(deltaTime);
        
        // Update tire forces
        calculateTireForces();
        
        // Update aerodynamic forces
        calculateAerodynamicForces();
        
        // Run control systems
        runStabilityControl(deltaTime);
        runTractionControl(deltaTime);
        runAntiLockBraking(deltaTime);
        runTorqueVectoring(deltaTime);
        
        // Update actuators
        updateDynamicDamping(deltaTime);
        updateActiveAerodynamics();
        updateRollStabilization(deltaTime);
        
        // Safety monitoring
        performSafetyChecks();
    }

    void processDriverInputs(float steeringAngle, float throttlePosition, float brakePosition) {
        // Store driver inputs
        driverInputs.steeringAngle = steeringAngle;
        driverInputs.throttlePosition = throttlePosition;
        driverInputs.brakePosition = brakePosition;
        
        // Apply steering angle
        steeringAngleSensor.angle = steeringAngle;
        steeringAngleSensor.rate = (steeringAngle - lastSteeringAngle) * 100.0f; // Assuming 100Hz update
        lastSteeringAngle = steeringAngle;
        
        // Calculate target torque from throttle
        float maxTorque = 400.0f; // Nm per wheel
        targetTorque = throttlePosition * maxTorque * 4.0f; // Total torque
        
        // Calculate brake demand
        float maxBrakePressure = 150.0f; // bar
        targetBrakePressure = brakePosition * maxBrakePressure;
        
        // Apply driver inputs through control systems
        distributeTargetTorque();
        applyBrakeSystem();
    }

    void enableLaunchControl(bool enable) {
        tc.launchControl.enabled = enable;
        
        if (enable) {
            // Configure for optimal launch
            tc.launchControl.targetSlipRatio = 8.0f;    // %
            tc.launchControl.rpmLimit = 6500.0f;        // RPM
            tv.launchMode.enabled = true;
            tv.launchMode.maxTorqueLimit = 1600.0f;     // Nm total
            tv.launchMode.slipRatioTarget = 8.0f;       // %
            
            std::cout << "Launch control enabled" << std::endl;
        } else {
            tv.launchMode.enabled = false;
            std::cout << "Launch control disabled" << std::endl;
        }
    }

    void enableDriftMode(bool enable) {
        tv.driftMode.enabled = enable;
        
        if (enable) {
            // Configure for controlled drifting
            esc.mode = ESCMode::DRIFT_MODE;
            tv.driftMode.rearBias = 75.0f;              // %
            tv.driftMode.slipAngleTarget = 0.35f;       // rad (20 degrees)
            tv.driftMode.powerOversteer = 80.0f;        // %
            
            // Adjust ESC thresholds for drift
            esc.sideslipThreshold = 0.5f;               // rad (allow more slip)
            esc.yawRateThreshold = 1.0f;                // rad/s (allow more yaw)
            esc.interventionLevel = 0.3f;               // Minimal intervention
            
            std::cout << "Drift mode enabled - ESC adjusted for controlled oversteer" << std::endl;
        } else {
            // Restore normal ESC operation
            esc.mode = ESCMode::ON;
            esc.sideslipThreshold = 0.15f;              // rad
            esc.yawRateThreshold = 0.5f;                // rad/s
            esc.interventionLevel = 0.8f;
            
            std::cout << "Drift mode disabled - Normal stability control restored" << std::endl;
        }
    }

    void setESCMode(ESCMode mode) {
        esc.mode = mode;
        
        switch (mode) {
            case ESCMode::ON:
                esc.enabled = true;
                esc.interventionLevel = 0.8f;
                esc.sideslipThreshold = 0.15f;          // rad
                esc.yawRateThreshold = 0.5f;            // rad/s
                break;
                
            case ESCMode::SPORT:
                esc.enabled = true;
                esc.interventionLevel = 0.6f;
                esc.sideslipThreshold = 0.25f;          // rad (more slip allowed)
                esc.yawRateThreshold = 0.8f;            // rad/s
                break;
                
            case ESCMode::OFF:
                esc.enabled = false;
                esc.interventionLevel = 0.0f;
                break;
                
            case ESCMode::TRACK_MODE:
                esc.enabled = true;
                esc.interventionLevel = 0.4f;
                esc.sideslipThreshold = 0.35f;          // rad
                esc.yawRateThreshold = 1.2f;            // rad/s
                break;
                
            case ESCMode::DRIFT_MODE:
                esc.enabled = true;
                esc.interventionLevel = 0.2f;
                esc.sideslipThreshold = 0.5f;           // rad
                esc.yawRateThreshold = 1.5f;            // rad/s
                break;
        }
        
        std::cout << "ESC mode set to: " << getESCModeString(mode) << std::endl;
    }

    void performDynamicsCalibration() {
        std::cout << "\n=== Vehicle Dynamics Calibration ===" << std::endl;
        
        // Calibrate IMU
        std::cout << "Calibrating IMU..." << std::endl;
        calibrateIMU();
        
        // Calibrate wheel speed sensors
        std::cout << "Calibrating wheel speed sensors..." << std::endl;
        calibrateWheelSpeedSensors();
        
        // Calibrate steering angle sensor
        std::cout << "Calibrating steering angle sensor..." << std::endl;
        calibrateSteeringAngleSensor();
        
        // Learn vehicle parameters
        std::cout << "Learning vehicle dynamics parameters..." << std::endl;
        learnVehicleDynamics();
        
        // Calibrate tire parameters
        std::cout << "Calibrating tire parameters..." << std::endl;
        calibrateTireParameters();
        
        std::cout << "Vehicle dynamics calibration completed" << std::endl;
    }

    void runDiagnostics() {
        std::cout << "\n=== Vehicle Dynamics Diagnostics ===" << std::endl;
        
        diagnosticsSensorSystems();
        diagnosticsControlSystems();
        diagnosticsActuatorSystems();
        diagnosticsVehicleHealth();
        diagnosticsPerformanceMetrics();
        
        std::cout << "Vehicle dynamics diagnostics completed" << std::endl;
    }

    void getSystemStatus() {
        std::cout << "\n=== Vehicle Dynamics System Status ===" << std::endl;
        std::cout << "Current Mode: " << getVehicleModeString(currentMode) << std::endl;
        std::cout << "Systems Enabled: " << (systemsEnabled ? "Yes" : "No") << std::endl;
        std::cout << "Speed: " << (vehicleState.speed * 3.6f) << " km/h" << std::endl;
        std::cout << "Lateral Acceleration: " << vehicleState.lateralAcceleration << " m/s²" << std::endl;
        std::cout << "Yaw Rate: " << (vehicleState.yawRate * 180.0f / M_PI) << " deg/s" << std::endl;
        
        std::cout << "\nControl Systems Status:" << std::endl;
        std::cout << "  ESC: " << getESCModeString(esc.mode) << " - " 
                  << (esc.intervening ? "ACTIVE" : "STANDBY") << std::endl;
        std::cout << "  Traction Control: " << getTCModeString(tc.mode) << " - "
                  << (isAnyWheelIntervening() ? "ACTIVE" : "STANDBY") << std::endl;
        std::cout << "  ABS: " << (abs.enabled ? "ENABLED" : "DISABLED") << " - "
                  << (isABSActive() ? "ACTIVE" : "STANDBY") << std::endl;
        std::cout << "  Torque Vectoring: " << (tv.enabled ? "ENABLED" : "DISABLED") << std::endl;
        
        std::cout << "\nWheel Status:" << std::endl;
        for (int i = 0; i < 4; i++) {
            std::cout << "  Wheel " << i << ": " 
                      << (wheels[i].wheelSpeed * tireRadius * 3.6f) << " km/h, "
                      << wheels[i].slipRatio << "% slip, "
                      << wheels[i].tireTemperature << "°C" << std::endl;
        }
        
        std::cout << "\nAerodynamics:" << std::endl;
        std::cout << "  Total Downforce: " << aerodynamics.totalDownforce << " N" << std::endl;
        std::cout << "  Total Drag: " << aerodynamics.totalDrag << " N" << std::endl;
        std::cout << "  DRS Active: " << (aerodynamics.rearWing.drsActive ? "Yes" : "No") << std::endl;
        
        std::cout << "====================================" << std::endl;
    }

private:
    // Core system state
    VehicleMode currentMode;
    VehicleState vehicleState;
    bool systemsEnabled;
    bool diagnosticsActive;
    bool dataLoggingActive;
    bool realTimeMonitoring;
    bool performanceOptimization;
    bool safetyOverride;
    bool weatherAdaptation;

    // Vehicle geometry
    float trackWidth;           // m
    float wheelbase;           // m
    float frontalArea;         // m²
    float tireRadius;          // m
    float tireWidth;           // m

    // Control systems
    ElectronicStabilityControl esc;
    TractionControl tc;
    AntiLockBrakingSystem abs;
    TorqueVectoring tv;
    DynamicDamping damping;
    ActiveAerodynamics aerodynamics;
    RollStabilization rollStab;

    // Wheel dynamics
    WheelDynamics wheels[4];

    // Sensor data structures
    struct IMUData {
        float accelerationX, accelerationY, accelerationZ;  // m/s²
        float gyroX, gyroY, gyroZ;                          // rad/s
        float temperature;                                   // °C
        bool calibrated;
    } imuData;

    struct WheelSpeedSensor {
        float frequency;        // Hz
        int resolution;         // pulses per revolution
        bool active;
        bool faultDetected;
    } wheelSpeedSensors[4];

    struct SteeringAngleSensor {
        float angle;           // rad
        float rate;            // rad/s
        float torque;          // Nm
        bool calibrated;
        bool active;
    } steeringAngleSensor;

    struct BrakePressureSensor {
        float pressure;        // bar
        float temperature;     // °C
        bool active;
    } brakePressureSensors[4];

    // Driver inputs
    struct DriverInputs {
        float steeringAngle;    // rad
        float throttlePosition; // 0-1
        float brakePosition;    // 0-1
    } driverInputs;

    float lastSteeringAngle = 0.0f;
    float targetTorque = 0.0f;
    float targetBrakePressure = 0.0f;

    void initializeDamper(DynamicDamping::DamperControl& damper, 
                         float compression, float rebound) {
        damper.compression = compression;   // N⋅s/m
        damper.rebound = rebound;          // N⋅s/m
        damper.position = 0.0f;            // mm
        damper.velocity = 0.0f;            // mm/s
        damper.force = 0.0f;               // N
        damper.active = true;
        damper.stiffnessRatio = 0.5f;      // Medium
        damper.dampingRatio = 0.6f;        // Moderate damping
        damper.preloadForce = 500.0f;      // N
    }

    void configureComfortMode() {
        // Soft suspension settings
        damping.currentMode = DynamicDamping::DamperMode::COMFORT;
        adjustDamperStiffness(0.3f); // Soft
        
        // Conservative stability control
        esc.interventionLevel = 0.9f;
        tc.interventionLevel = 0.9f;
        tv.understeerCompensation = 50.0f;   // Nm
        tv.oversteerCompensation = 100.0f;   // Nm
        
        // Gentle aerodynamics
        aerodynamics.rearWing.angle = 3.0f;  // degrees
        aerodynamics.frontSpoiler.angle = 0.0f;
    }

    void configureSportMode() {
        // Firmer suspension
        damping.currentMode = DynamicDamping::DamperMode::SPORT;
        adjustDamperStiffness(0.7f); // Firm
        
        // Responsive stability control
        esc.interventionLevel = 0.6f;
        esc.sideslipThreshold = 0.2f;        // rad
        tc.interventionLevel = 0.7f;
        tv.understeerCompensation = 80.0f;   // Nm
        tv.oversteerCompensation = 60.0f;    // Nm
        
        // Aggressive aerodynamics
        aerodynamics.rearWing.angle = 8.0f;  // degrees
        aerodynamics.frontSpoiler.angle = 2.0f;
    }

    void configureTrackMode() {
        // Track-focused suspension
        damping.currentMode = DynamicDamping::DamperMode::TRACK;
        adjustDamperStiffness(0.9f); // Very firm
        
        // Performance-oriented control
        esc.mode = ESCMode::TRACK_MODE;
        esc.interventionLevel = 0.4f;
        tc.interventionLevel = 0.5f;
        tv.understeerCompensation = 120.0f;  // Nm
        tv.oversteerCompensation = 40.0f;    // Nm
        
        // Maximum downforce
        aerodynamics.rearWing.angle = 12.0f; // degrees
        aerodynamics.frontSpoiler.angle = 5.0f;
    }

    void configureEcoMode() {
        // Efficient settings
        damping.currentMode = DynamicDamping::DamperMode::COMFORT;
        adjustDamperStiffness(0.4f); // Soft for efficiency
        
        // Conservative control for efficiency
        esc.interventionLevel = 0.8f;
        tc.interventionLevel = 0.9f;
        
        // Low-drag aerodynamics
        aerodynamics.rearWing.angle = 0.0f;  // degrees (minimal)
        aerodynamics.frontSpoiler.angle = 0.0f;
        aerodynamics.activeGrille.openingPercentage = 20.0f; // Mostly closed
    }

    void configureSnowMode() {
        // Traction-focused settings
        tc.mode = TractionControlMode::SNOW;
        tc.slipThreshold = 3.0f;             // % (very sensitive)
        tc.targetSlipRatio = 2.0f;           // % (minimal slip)
        
        // Gentle intervention
        esc.interventionLevel = 0.9f;
        esc.yawRateThreshold = 0.3f;         // rad/s (sensitive)
        
        // Equal torque distribution
        tv.torqueBias = 50.0f;               // % (equal front/rear)
        tv.leftRightBias = 50.0f;            // % (equal left/right)
    }

    void configureSandMode() {
        tc.mode = TractionControlMode::SAND_MUD;
        tc.slipThreshold = 15.0f;            // % (allow more slip)
        tc.targetSlipRatio = 10.0f;          // % (controlled slip)
        
        // Higher ground clearance simulation
        aerodynamics.underfloorPanels.groundClearance = 150.0f; // mm
        
        // Rear bias for sand
        tv.torqueBias = 40.0f;               // % (rear bias)
    }

    void configureMudMode() {
        tc.mode = TractionControlMode::SAND_MUD;
        tc.slipThreshold = 20.0f;            // % (allow significant slip)
        tc.targetSlipRatio = 15.0f;          // % (momentum maintenance)
        
        // Aggressive torque vectoring for mud
        tv.understeerCompensation = 150.0f;  // Nm
        tv.torqueBias = 35.0f;               // % (strong rear bias)
    }

    void configureRockMode() {
        // Precise control for rock crawling
        tc.slipThreshold = 5.0f;             // % (precise traction)
        tc.targetSlipRatio = 3.0f;           // % (minimal slip)
        
        // Maximum traction distribution flexibility
        tv.enabled = true;
        for (int i = 0; i < 4; i++) {
            tv.torqueDistribution[i] = 25.0f; // % (start equal)
        }
        
        // Soft suspension for articulation
        adjustDamperStiffness(0.2f);         // Very soft
    }

    void configureDriftMode() {
        // Enable drift-specific settings
        tv.driftMode.enabled = true;
        tv.driftMode.rearBias = 80.0f;       // %
        tv.driftMode.slipAngleTarget = 0.35f; // rad
        tv.driftMode.powerOversteer = 75.0f; // %
        
        // Adjust stability control for drift
        esc.mode = ESCMode::DRIFT_MODE;
        esc.sideslipThreshold = 0.6f;        // rad (allow large slip angles)
        esc.interventionLevel = 0.2f;        // Minimal intervention
    }

    void configureLaunchMode() {
        // Optimal launch configuration
        tc.launchControl.enabled = true;
        tv.launchMode.enabled = true;
        tv.launchMode.maxTorqueLimit = 1800.0f; // Nm total
        tv.launchMode.slipRatioTarget = 8.0f; // % (optimal for launch)
        
        // Firm suspension for launch
        adjustDamperStiffness(0.8f);
        
        // Aerodynamics for launch
        aerodynamics.rearWing.angle = 5.0f;  // degrees (balance)
    }

    void configureCustomMode() {
        // User-defined settings (placeholder)
        std::cout << "Custom mode configured with user preferences" << std::endl;
    }

    void adjustDamperStiffness(float stiffnessRatio) {
        float baseCompression = 2500.0f;     // N⋅s/m
        float baseRebound = 800.0f;          // N⋅s/m
        
        damping.frontLeft.compression = baseCompression * (0.5f + stiffnessRatio);
        damping.frontLeft.rebound = baseRebound * (0.5f + stiffnessRatio);
        damping.frontLeft.stiffnessRatio = stiffnessRatio;
        
        damping.frontRight.compression = baseCompression * (0.5f + stiffnessRatio);
        damping.frontRight.rebound = baseRebound * (0.5f + stiffnessRatio);
        damping.frontRight.stiffnessRatio = stiffnessRatio;
        
        damping.rearLeft.compression = baseCompression * 0.9f * (0.5f + stiffnessRatio);
        damping.rearLeft.rebound = baseRebound * 0.9f * (0.5f + stiffnessRatio);
        damping.rearLeft.stiffnessRatio = stiffnessRatio;
        
        damping.rearRight.compression = baseCompression * 0.9f * (0.5f + stiffnessRatio);
        damping.rearRight.rebound = baseRebound * 0.9f * (0.5f + stiffnessRatio);
        damping.rearRight.stiffnessRatio = stiffnessRatio;
    }

    void updateIMUData() {
        // Simulate IMU readings based on vehicle state
        imuData.accelerationX = vehicleState.longitudinalAcceleration;
        imuData.accelerationY = vehicleState.lateralAcceleration;
        imuData.accelerationZ = vehicleState.verticalAcceleration - 9.81f;
        imuData.gyroX = vehicleState.rollRate;
        imuData.gyroY = vehicleState.pitchRate;
        imuData.gyroZ = vehicleState.yawRate;
        
        // Add realistic noise
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::normal_distribution<float> accelNoise(0.0f, 0.02f);  // m/s²
        static std::normal_distribution<float> gyroNoise(0.0f, 0.001f);   // rad/s
        
        imuData.accelerationX += accelNoise(gen);
        imuData.accelerationY += accelNoise(gen);
        imuData.accelerationZ += accelNoise(gen);
        imuData.gyroX += gyroNoise(gen);
        imuData.gyroY += gyroNoise(gen);
        imuData.gyroZ += gyroNoise(gen);
    }

    void updateWheelDynamics(float deltaTime) {
        for (int i = 0; i < 4; i++) {
            // Update wheel speed based on vehicle motion and slip
            float vehicleSpeedAtWheel = vehicleState.speed;
            
            // Account for yaw rate effect on individual wheels
            if (i == 0 || i == 2) { // Left wheels
                vehicleSpeedAtWheel -= vehicleState.yawRate * trackWidth / 2.0f;
            } else { // Right wheels
                vehicleSpeedAtWheel += vehicleState.yawRate * trackWidth / 2.0f;
            }
            
            // Calculate expected wheel speed
            float expectedWheelSpeed = vehicleSpeedAtWheel / tireRadius; // rad/s
            
            // Update actual wheel speed with slip consideration
            wheels[i].wheelSpeed = expectedWheelSpeed * (1.0f + wheels[i].slipRatio / 100.0f);
            
            // Update vertical load based on load transfer
            updateVerticalLoad(i);
            
            // Update tire temperature based on slip and load
            updateTireTemperature(i, deltaTime);
            
            // Update wear level
            updateTireWear(i, deltaTime);
            
            // Check for wheel lock/spin conditions
            checkWheelLockSpin(i);
        }
    }

    void updateVerticalLoad(int wheelIndex) {
        float staticLoad = vehicleState.mass * 9.81f / 4.0f; // N
        
        // Longitudinal load transfer
        float longitudinalTransfer = vehicleState.longitudinalAcceleration * 
                                   vehicleState.mass * vehicleState.cogHeight / wheelbase;
        
        // Lateral load transfer
        float lateralTransfer = vehicleState.lateralAcceleration * 
                              vehicleState.mass * vehicleState.cogHeight / trackWidth;
        
        // Apply transfers based on wheel position
        float loadTransfer = staticLoad;
        
        if (wheelIndex == 0) { // Front left
            loadTransfer += longitudinalTransfer + lateralTransfer;
        } else if (wheelIndex == 1) { // Front right
            loadTransfer += longitudinalTransfer - lateralTransfer;
        } else if (wheelIndex == 2) { // Rear left
            loadTransfer -= longitudinalTransfer + lateralTransfer;
        } else { // Rear right
            loadTransfer -= longitudinalTransfer - lateralTransfer;
        }
        
        wheels[wheelIndex].verticalLoad = std::max(0.0f, loadTransfer);
    }

    void updateTireTemperature(int wheelIndex, float deltaTime) {
        WheelDynamics& wheel = wheels[wheelIndex];
        
        // Heat generation from slip
        float slipHeat = std::abs(wheel.slipRatio) * wheel.wheelForce * 0.01f; // Simplified
        
        // Heat generation from load
        float loadHeat = wheel.verticalLoad * 0.00001f; // W
        
        // Cooling from airflow
        float airspeed = std::max(1.0f, vehicleState.speed); // m/s
        float cooling = (wheel.tireTemperature - 25.0f) * airspeed * 0.1f; // W
        
        // Temperature change
        float tempChange = (slipHeat + loadHeat - cooling) * deltaTime * 0.1f; // K/s
        
        wheel.tireTemperature += tempChange;
        wheel.tireTemperature = std::clamp(wheel.tireTemperature, 5.0f, 120.0f); // °C
        
        // Thermal degradation
        if (wheel.tireTemperature > 80.0f) {
            wheel.thermalDegradation += (wheel.tireTemperature - 80.0f) * deltaTime * 0.001f;
            wheel.thermalDegradation = std::clamp(wheel.thermalDegradation, 0.0f, 0.3f); // Max 30% degradation
        }
    }

    void updateTireWear(int wheelIndex, float deltaTime) {
        WheelDynamics& wheel = wheels[wheelIndex];
        
        // Wear based on slip and load
        float wearRate = std::abs(wheel.slipRatio) * wheel.verticalLoad * 0.000001f; // mm/s
        
        // Increased wear at high temperature
        if (wheel.tireTemperature > 60.0f) {
            wearRate *= (1.0f + (wheel.tireTemperature - 60.0f) * 0.02f);
        }
        
        float wearAmount = wearRate * deltaTime;
        wheel.treadDepth -= wearAmount;
        wheel.treadDepth = std::max(1.5f, wheel.treadDepth); // Minimum legal depth
        
        // Update wear level percentage
        wheel.wearLevel = (8.0f - wheel.treadDepth) / 6.5f * 100.0f; // % worn
        wheel.wearLevel = std::clamp(wheel.wearLevel, 0.0f, 100.0f);
    }

    void checkWheelLockSpin(int wheelIndex) {
        WheelDynamics& wheel = wheels[wheelIndex];
        
        // Check for wheel lock (braking)
        if (targetBrakePressure > 0.1f && std::abs(wheel.slipRatio) > abs.wheelSlipThreshold) {
            wheel.locked = true;
        } else {
            wheel.locked = false;
        }
        
        // Check for wheel spin (acceleration)
        if (targetTorque > 0.1f && wheel.slipRatio > tc.slipThreshold) {
            wheel.spinning = true;
        } else {
            wheel.spinning = false;
        }
    }

    void calculateVehicleMotion(float deltaTime) {
        // Sum forces and torques from all wheels
        float totalLongitudinalForce = 0.0f;
        float totalLateralForce = 0.0f;
        float totalYawMoment = 0.0f;
        
        for (int i = 0; i < 4; i++) {
            totalLongitudinalForce += wheels[i].longitudinalForce;
            totalLateralForce += wheels[i].lateralForce;
            
            // Calculate yaw moment contribution
            float momentArm;
            if (i < 2) { // Front wheels
                momentArm = wheelbase / 2.0f;
            } else { // Rear wheels
                momentArm = -wheelbase / 2.0f;
            }
            
            if (i % 2 == 0) { // Left wheels
                totalYawMoment += wheels[i].lateralForce * momentArm - 
                                wheels[i].longitudinalForce * trackWidth / 2.0f;
            } else { // Right wheels
                totalYawMoment += wheels[i].lateralForce * momentArm + 
                                wheels[i].longitudinalForce * trackWidth / 2.0f;
            }
        }
        
        // Add aerodynamic forces
        totalLongitudinalForce -= aerodynamics.totalDrag;
        // Downforce affects tire normal forces (handled in updateVerticalLoad)
        
        // Calculate accelerations
        vehicleState.longitudinalAcceleration = totalLongitudinalForce / vehicleState.mass;
        vehicleState.lateralAcceleration = totalLateralForce / vehicleState.mass;
        float yawAcceleration = totalYawMoment / vehicleState.momentOfInertiaYaw;
        
        // Update velocities
        vehicleState.velocityX += vehicleState.longitudinalAcceleration * deltaTime;
        vehicleState.velocityY += vehicleState.lateralAcceleration * deltaTime;
        vehicleState.yawRate += yawAcceleration * deltaTime;
        
        // Update derived quantities
        vehicleState.speed = std::sqrt(vehicleState.velocityX * vehicleState.velocityX + 
                                     vehicleState.velocityY * vehicleState.velocityY);
        
        if (vehicleState.speed > 0.1f) {
            vehicleState.sideslipAngle = std::atan2(vehicleState.velocityY, vehicleState.velocityX);
        } else {
            vehicleState.sideslipAngle = 0.0f;
        }
        
        // Update position and orientation
        vehicleState.x += vehicleState.velocityX * std::cos(vehicleState.yaw) - 
                         vehicleState.velocityY * std::sin(vehicleState.yaw) * deltaTime;
        vehicleState.y += vehicleState.velocityX * std::sin(vehicleState.yaw) + 
                         vehicleState.velocityY * std::cos(vehicleState.yaw) * deltaTime;
        vehicleState.yaw += vehicleState.yawRate * deltaTime;
    }

    void calculateTireForces() {
        for (int i = 0; i < 4; i++) {
            calculateTireForce(i);
        }
    }

    void calculateTireForce(int wheelIndex) {
        WheelDynamics& wheel = wheels[wheelIndex];
        
        // Magic Formula tire model (simplified)
        float normalizedSlip = wheel.slipRatio / wheel.optimalSlipRatio;
        float frictionCoeff = wheel.peakFriction * (1.0f - wheel.thermalDegradation);
        
        // Longitudinal force
        if (std::abs(normalizedSlip) < 1.0f) {
            wheel.longitudinalForce = frictionCoeff * wheel.verticalLoad * 
                                    normalizedSlip * (2.0f - std::abs(normalizedSlip));
        } else {
            wheel.longitudinalForce = frictionCoeff * wheel.verticalLoad * 
                                    (normalizedSlip > 0 ? 1.0f : -1.0f);
        }
        
        // Lateral force (simplified cornering force)
        float corneringForce = wheel.corneringStiffness * wheel.slipAngle;
        float maxLateralForce = frictionCoeff * wheel.verticalLoad * 
                              std::sqrt(1.0f - std::pow(wheel.longitudinalForce / (frictionCoeff * wheel.verticalLoad), 2.0f));
        
        wheel.lateralForce = std::clamp(corneringForce, -maxLateralForce, maxLateralForce);
        
        // Total force magnitude
        wheel.wheelForce = std::sqrt(wheel.longitudinalForce * wheel.longitudinalForce + 
                                   wheel.lateralForce * wheel.lateralForce);
    }

    void calculateAerodynamicForces() {
        float airDensity = 1.225f; // kg/m³ at sea level
        float dynamicPressure = 0.5f * airDensity * vehicleState.speed * vehicleState.speed;
        
        // Drag force
        aerodynamics.totalDrag = aerodynamics.dragCoefficient * frontalArea * dynamicPressure;
        
        // Downforce
        aerodynamics.totalDownforce = aerodynamics.liftCoefficient * frontalArea * dynamicPressure;
        
        // Component contributions
        aerodynamics.frontSpoiler.downforce = aerodynamics.frontSpoiler.angle * 20.0f * dynamicPressure;
        aerodynamics.frontSpoiler.drag = aerodynamics.frontSpoiler.angle * 5.0f * dynamicPressure;
        
        aerodynamics.rearWing.downforce = aerodynamics.rearWing.angle * 15.0f * dynamicPressure;
        aerodynamics.rearWing.drag = aerodynamics.rearWing.angle * 8.0f * dynamicPressure;
        
        if (aerodynamics.rearWing.drsActive) {
            aerodynamics.rearWing.downforce *= 0.3f;
            aerodynamics.rearWing.drag *= 0.5f;
        }
        
        aerodynamics.underfloorPanels.venturiEffect = 
            (150.0f - aerodynamics.underfloorPanels.groundClearance) * 2.0f * dynamicPressure;
        
        // Total downforce and drag
        aerodynamics.totalDownforce += aerodynamics.frontSpoiler.downforce + 
                                     aerodynamics.rearWing.downforce + 
                                     aerodynamics.underfloorPanels.venturiEffect;
        
        aerodynamics.totalDrag += aerodynamics.frontSpoiler.drag + aerodynamics.rearWing.drag;
        
        // Aerodynamic balance (% front)
        if (aerodynamics.totalDownforce > 0.0f) {
            aerodynamics.aerodynamicBalance = 
                (aerodynamics.frontSpoiler.downforce / aerodynamics.totalDownforce) * 100.0f;
        }
    }

    void runStabilityControl(float deltaTime) {
        if (!esc.enabled) return;
        
        // Calculate target yaw rate based on steering input
        float steerAngle = steeringAngleSensor.angle;
        float understeerGradient = 0.002f; // rad/(m/s²)
        float targetYawRate = steerAngle / (1.0f + understeerGradient * vehicleState.lateralAcceleration) * 
                             vehicleState.speed / wheelbase;
        
        // Calculate yaw rate error
        float yawRateError = targetYawRate - vehicleState.yawRate;
        
        // Calculate sideslip angle error (target is typically zero)
        float sideslipError = 0.0f - vehicleState.sideslipAngle;
        
        // Check if intervention is needed
        bool needsIntervention = 
            std::abs(yawRateError) > esc.yawRateThreshold ||
            std::abs(vehicleState.sideslipAngle) > esc.sideslipThreshold ||
            std::abs(vehicleState.lateralAcceleration) > esc.lateralAccelThreshold;
        
        if (needsIntervention && esc.interventionLevel > 0.1f) {
            esc.intervening = true;
            
            // Calculate required yaw moment
            esc.yawMomentDemand = yawRateError * esc.yawRateGain * esc.interventionLevel +
                                sideslipError * esc.sideslipGain * esc.interventionLevel;
            
            // Apply brake intervention
            applyESCBrakeIntervention();
            
            // Apply torque intervention
            applyESCTorqueIntervention();
            
        } else {
            esc.intervening = false;
            esc.yawMomentDemand = 0.0f;
            esc.brakeIntervention.active = false;
            esc.torqueIntervention.active = false;
        }
    }

    void runTractionControl(float deltaTime) {
        if (!tc.enabled) return;
        
        for (int i = 0; i < 4; i++) {
            float wheelSlip = wheels[i].slipRatio;
            
            // Check if wheel needs intervention
            if (std::abs(wheelSlip) > tc.slipThreshold) {
                tc.intervening[i] = true;
                
                // Calculate torque reduction
                float slipExcess = std::abs(wheelSlip) - tc.targetSlipRatio;
                tc.torqueReduction[i] = std::clamp(slipExcess * 5.0f, 0.0f, 80.0f); // % reduction
                
                // Apply brake if torque reduction is not enough
                if (tc.torqueReduction[i] > 50.0f) {
                    tc.brakeApplication[i] = (tc.torqueReduction[i] - 50.0f) * 0.5f; // bar
                } else {
                    tc.brakeApplication[i] = 0.0f;
                }
                
            } else {
                tc.intervening[i] = false;
                tc.torqueReduction[i] = 0.0f;
                tc.brakeApplication[i] = 0.0f;
            }
        }
    }

    void runAntiLockBraking(float deltaTime) {
        if (!abs.enabled || targetBrakePressure < 0.1f) return;
        
        for (int i = 0; i < 4; i++) {
            float wheelSlip = std::abs(wheels[i].slipRatio);
            
            // Check if wheel is locking
            if (wheelSlip > abs.wheelSlipThreshold) {
                abs.active[i] = true;
                
                // Modulate brake force to maintain target slip
                float slipError = wheelSlip - abs.wheelSlipTarget;
                float modulation = 1.0f - slipError * 0.05f; // Reduce pressure
                abs.brakeForceModulation[i] = std::clamp(modulation, 0.2f, 1.0f);
                
            } else if (wheelSlip < abs.wheelSlipTarget * 0.5f) {
                // Wheel is not slipping enough, can increase pressure
                abs.active[i] = false;
                abs.brakeForceModulation[i] = 1.0f;
            }
        }
        
        // Brake Assist detection
        if (targetBrakePressure > 0.5f && 
            vehicleState.longitudinalAcceleration < -abs.brakeAssist.emergencyThreshold) {
            abs.brakeAssist.active = true;
        } else {
            abs.brakeAssist.active = false;
        }
    }

    void runTorqueVectoring(float deltaTime) {
        if (!tv.enabled) return;
        
        // Calculate desired yaw moment from steering input and vehicle state
        float steerAngle = steeringAngleSensor.angle;
        float desiredYawMoment = 0.0f;
        
        // Understeer compensation
        if (vehicleState.lateralAcceleration > 4.0f) { // Cornering situation
            float understeerIndicator = steerAngle - vehicleState.yawRate * wheelbase / vehicleState.speed;
            if (understeerIndicator > 0.05f) { // Understeering
                desiredYawMoment = tv.understeerCompensation * understeerIndicator;
            }
        }
        
        // Oversteer compensation
        if (std::abs(vehicleState.sideslipAngle) > 0.1f) { // Oversteering
            desiredYawMoment -= tv.oversteerCompensation * vehicleState.sideslipAngle;
        }
        
        // Add ESC yaw moment demand
        desiredYawMoment += esc.yawMomentDemand;
        
        tv.yawMomentDemand = desiredYawMoment;
        
        // Distribute torque to create yaw moment
        distributeTorqueForYawMoment(desiredYawMoment);
    }

    void updateDynamicDamping(float deltaTime) {
        // Update damper forces based on current mode and conditions
        updateDamperForce(damping.frontLeft, deltaTime);
        updateDamperForce(damping.frontRight, deltaTime);
        updateDamperForce(damping.rearLeft, deltaTime);
        updateDamperForce(damping.rearRight, deltaTime);
        
        // Sky-Hook control
        if (damping.skyHookEnabled) {
            applySkyHookControl();
        }
        
        // Road surface adaptation
        adaptToRoadSurface();
    }

    void updateDamperForce(DynamicDamping::DamperControl& damper, float deltaTime) {
        // Simulate damper velocity (simplified)
        float sprungMassVelocity = vehicleState.verticalAcceleration * deltaTime;
        damper.velocity = sprungMassVelocity * 100.0f; // mm/s
        
        // Calculate damper force
        if (damper.velocity > 0.0f) { // Compression
            damper.force = damper.compression * damper.velocity + damper.preloadForce;
        } else { // Rebound
            damper.force = damper.rebound * std::abs(damper.velocity) - damper.preloadForce;
        }
        
        // Apply stiffness ratio
        damper.force *= damper.stiffnessRatio;
    }

    void applySkyHookControl() {
        // Sky-Hook damping for improved ride comfort
        float bodyVelocity = vehicleState.velocityZ; // Vertical velocity
        
        // Calculate sky-hook forces
        float skyHookForceFront = -damping.skyHookGain * bodyVelocity;
        float skyHookForceRear = -damping.skyHookGain * bodyVelocity;
        
        // Apply to dampers
        damping.frontLeft.force += skyHookForceFront;
        damping.frontRight.force += skyHookForceFront;
        damping.rearLeft.force += skyHookForceRear;
        damping.rearRight.force += skyHookForceRear;
    }

    void adaptToRoadSurface() {
        // Estimate road roughness from wheel accelerations
        float roughness = 0.0f;
        for (int i = 0; i < 4; i++) {
            roughness += std::abs(wheels[i].verticalLoad - vehicleState.mass * 9.81f / 4.0f);
        }
        roughness /= (vehicleState.mass * 9.81f); // Normalize
        
        damping.roughnessIndex = roughness * 10.0f; // Scale to 0-10
        
        // Activate rough road mode if needed
        if (damping.roughnessIndex > 6.0f) {
            damping.roughRoadMode = true;
            // Soften damping for rough roads
            adjustDamperStiffness(std::max(0.3f, damping.frontLeft.stiffnessRatio - 0.2f));
        } else {
            damping.roughRoadMode = false;
        }
    }

    void updateActiveAerodynamics() {
        // Speed-based aerodynamic adjustments
        if (vehicleState.speed > 100.0f / 3.6f) { // Above 100 km/h
            // Increase downforce
            aerodynamics.rearWing.angle = std::min(12.0f, aerodynamics.rearWing.angle + 1.0f);
            aerodynamics.frontSpoiler.angle = std::min(5.0f, aerodynamics.frontSpoiler.angle + 0.5f);
        } else if (vehicleState.speed < 50.0f / 3.6f) { // Below 50 km/h
            // Reduce drag
            aerodynamics.rearWing.angle = std::max(0.0f, aerodynamics.rearWing.angle - 1.0f);
            aerodynamics.frontSpoiler.angle = std::max(0.0f, aerodynamics.frontSpoiler.angle - 0.5f);
        }
        
        // DRS (Drag Reduction System) logic
        if (currentMode == VehicleMode::TRACK && vehicleState.speed > 150.0f / 3.6f &&
            std::abs(steeringAngleSensor.angle) < 0.05f) { // Straight line, high speed
            aerodynamics.rearWing.drsActive = true;
        } else {
            aerodynamics.rearWing.drsActive = false;
        }
        
        // Active grille control
        float engineTemp = 80.0f; // Simplified - would come from thermal system
        if (engineTemp > 90.0f) {
            aerodynamics.activeGrille.openingPercentage = 100.0f;
        } else if (engineTemp < 70.0f && vehicleState.speed > 80.0f / 3.6f) {
            aerodynamics.activeGrille.openingPercentage = 20.0f; // Close for efficiency
        } else {
            aerodynamics.activeGrille.openingPercentage = 50.0f;
        }
    }

    void updateRollStabilization(float deltaTime) {
        // Calculate roll angle from lateral acceleration
        rollStab.rollAngle = vehicleState.lateralAcceleration * vehicleState.cogHeight / 
                           (trackWidth * 9.81f); // Simplified roll model
        
        // Calculate roll rate
        static float lastRollAngle = 0.0f;
        rollStab.rollRate = (rollStab.rollAngle - lastRollAngle) / deltaTime;
        lastRollAngle = rollStab.rollAngle;
        
        // Calculate required anti-roll moment
        float rollError = rollStab.rollTargetAngle - rollStab.rollAngle;
        float rollMoment = rollError * rollStab.front.stiffness + 
                         rollStab.rollRate * rollStab.rollDamping;
        
        rollStab.rollMoment = rollMoment;
        
        // Distribute moment between front and rear anti-roll bars
        float frontMoment = rollMoment * (rollStab.rollStiffnessRatio / 100.0f);
        float rearMoment = rollMoment * (1.0f - rollStab.rollStiffnessRatio / 100.0f);
        
        rollStab.front.torque = frontMoment;
        rollStab.rear.torque = rearMoment;
        
        // Calculate lateral load transfer
        rollStab.lateralLoadTransfer = std::abs(vehicleState.lateralAcceleration) * 
                                     vehicleState.mass * vehicleState.cogHeight / trackWidth;
    }

    void applyESCBrakeIntervention() {
        esc.brakeIntervention.active = true;
        
        float requiredYawMoment = esc.yawMomentDemand;
        
        // Determine which wheel to brake based on yaw moment direction
        if (requiredYawMoment > 0.0f) { // Need left yaw moment
            // Brake right wheels more
            esc.brakeIntervention.frontRight = std::clamp(requiredYawMoment / 1000.0f, 0.0f, 50.0f);
            esc.brakeIntervention.rearRight = std::clamp(requiredYawMoment / 1500.0f, 0.0f, 50.0f);
            esc.brakeIntervention.frontLeft = 0.0f;
            esc.brakeIntervention.rearLeft = 0.0f;
        } else { // Need right yaw moment
            // Brake left wheels more
            esc.brakeIntervention.frontLeft = std::clamp(-requiredYawMoment / 1000.0f, 0.0f, 50.0f);
            esc.brakeIntervention.rearLeft = std::clamp(-requiredYawMoment / 1500.0f, 0.0f, 50.0f);
            esc.brakeIntervention.frontRight = 0.0f;
            esc.brakeIntervention.rearRight = 0.0f;
        }
    }

    void applyESCTorqueIntervention() {
        esc.torqueIntervention.active = true;
        
        // Calculate total torque reduction needed
        esc.torqueIntervention.totalReduction = std::clamp(std::abs(esc.yawMomentDemand) / 100.0f, 0.0f, 30.0f);
        
        // Distribute reduction based on required yaw moment
        for (int i = 0; i < 4; i++) {
            esc.torqueIntervention.individualReduction[i] = esc.torqueIntervention.totalReduction / 4.0f;
        }
        
        // Adjust individual wheel reductions for yaw moment
        if (esc.yawMomentDemand > 0.0f) {
            // Reduce torque more on right side
            esc.torqueIntervention.individualReduction[1] *= 1.5f; // Front right
            esc.torqueIntervention.individualReduction[3] *= 1.5f; // Rear right
        } else {
            // Reduce torque more on left side
            esc.torqueIntervention.individualReduction[0] *= 1.5f; // Front left
            esc.torqueIntervention.individualReduction[2] *= 1.5f; // Rear left
        }
    }

    void distributeTorqueForYawMoment(float requiredYawMoment) {
        // Base torque distribution
        float baseTorque = targetTorque / 4.0f; // Equal distribution
        
        // Calculate torque adjustment for yaw moment
        float torqueAdjustment = requiredYawMoment / (trackWidth * 2.0f); // Simplified
        
        // Apply to individual wheels
        if (requiredYawMoment > 0.0f) { // Need left yaw moment
            tv.torqueDistribution[1] += torqueAdjustment / baseTorque * 100.0f; // Front right +
            tv.torqueDistribution[3] += torqueAdjustment / baseTorque * 100.0f; // Rear right +
            tv.torqueDistribution[0] -= torqueAdjustment / baseTorque * 100.0f; // Front left -
            tv.torqueDistribution[2] -= torqueAdjustment / baseTorque * 100.0f; // Rear left -
        } else { // Need right yaw moment
            tv.torqueDistribution[0] += (-torqueAdjustment) / baseTorque * 100.0f; // Front left +
            tv.torqueDistribution[2] += (-torqueAdjustment) / baseTorque * 100.0f; // Rear left +
            tv.torqueDistribution[1] -= (-torqueAdjustment) / baseTorque * 100.0f; // Front right -
            tv.torqueDistribution[3] -= (-torqueAdjustment) / baseTorque * 100.0f; // Rear right -
        }
        
        // Normalize and clamp torque distribution
        float totalDistribution = 0.0f;
        for (int i = 0; i < 4; i++) {
            tv.torqueDistribution[i] = std::clamp(tv.torqueDistribution[i], 0.0f, 50.0f);
            totalDistribution += tv.torqueDistribution[i];
        }
        
        // Ensure total equals 100%
        if (totalDistribution > 0.0f) {
            for (int i = 0; i < 4; i++) {
                tv.torqueDistribution[i] = tv.torqueDistribution[i] / totalDistribution * 100.0f;
            }
        }
    }

    void distributeTargetTorque() {
        // Apply torque vectoring distribution
        for (int i = 0; i < 4; i++) {
            float baseTorque = targetTorque * tv.torqueDistribution[i] / 100.0f;
            
            // Apply traction control reduction
            baseTorque *= (1.0f - tc.torqueReduction[i] / 100.0f);
            
            // Apply ESC torque intervention
            if (esc.torqueIntervention.active) {
                baseTorque *= (1.0f - esc.torqueIntervention.individualReduction[i] / 100.0f);
            }
            
            wheels[i].wheelTorque = baseTorque;
        }
    }

    void applyBrakeSystem() {
        for (int i = 0; i < 4; i++) {
            float brakePressure = targetBrakePressure;
            
            // Apply ABS modulation
            if (abs.active[i]) {
                brakePressure *= abs.brakeForceModulation[i];
            }
            
            // Apply ESC brake intervention
            if (esc.brakeIntervention.active) {
                float escBrake = 0.0f;
                switch (i) {
                    case 0: escBrake = esc.brakeIntervention.frontLeft; break;
                    case 1: escBrake = esc.brakeIntervention.frontRight; break;
                    case 2: escBrake = esc.brakeIntervention.rearLeft; break;
                    case 3: escBrake = esc.brakeIntervention.rearRight; break;
                }
                brakePressure += escBrake;
            }
            
            // Apply traction control braking
            brakePressure += tc.brakeApplication[i];
            
            // Apply brake assist
            if (abs.brakeAssist.active) {
                brakePressure *= abs.brakeAssist.boostFactor;
            }
            
            brakePressureSensors[i].pressure = std::clamp(brakePressure, 0.0f, 200.0f); // bar
        }
    }

    void performSafetyChecks() {
        // Check for system faults
        bool anyFault = false;
        
        // Check wheel speed sensors
        for (int i = 0; i < 4; i++) {
            if (wheelSpeedSensors[i].faultDetected) {
                anyFault = true;
                std::cout << "WARNING: Wheel speed sensor " << i << " fault detected" << std::endl;
            }
        }
        
        // Check IMU calibration
        if (!imuData.calibrated) {
            anyFault = true;
            std::cout << "WARNING: IMU not calibrated" << std::endl;
        }
        
        // Check for excessive tire temperature
        for (int i = 0; i < 4; i++) {
            if (wheels[i].tireTemperature > 100.0f) {
                anyFault = true;
                std::cout << "WARNING: Tire " << i << " overheating: " 
                          << wheels[i].tireTemperature << "°C" << std::endl;
            }
        }
        
        // Check for excessive tire wear
        for (int i = 0; i < 4; i++) {
            if (wheels[i].wearLevel > 80.0f) {
                anyFault = true;
                std::cout << "WARNING: Tire " << i << " critically worn: " 
                          << wheels[i].wearLevel << "%" << std::endl;
            }
        }
        
        // Safety override if critical faults detected
        if (anyFault && safetyOverride) {
            // Reduce performance in case of faults
            if (esc.enabled) esc.interventionLevel = std::min(1.0f, esc.interventionLevel + 0.2f);
            if (tc.enabled) tc.interventionLevel = std::min(1.0f, tc.interventionLevel + 0.2f);
        }
    }

    // Calibration functions
    void calibrateIMU() {
        // Simulate IMU calibration
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        imuData.calibrated = true;
        std::cout << "IMU calibration completed" << std::endl;
    }

    void calibrateWheelSpeedSensors() {
        for (int i = 0; i < 4; i++) {
            wheelSpeedSensors[i].faultDetected = false;
            wheelSpeedSensors[i].active = true;
        }
        std::cout << "Wheel speed sensors calibrated" << std::endl;
    }

    void calibrateSteeringAngleSensor() {
        steeringAngleSensor.calibrated = true;
        steeringAngleSensor.angle = 0.0f;
        std::cout << "Steering angle sensor calibrated" << std::endl;
    }

    void learnVehicleDynamics() {
        // Simulate learning vehicle parameters
        std::cout << "Learning vehicle mass distribution..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        std::cout << "Learning tire characteristics..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        std::cout << "Learning suspension parameters..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        std::cout << "Vehicle dynamics learning completed" << std::endl;
    }

    void calibrateTireParameters() {
        for (int i = 0; i < 4; i++) {
            // Reset tire parameters to optimal values
            wheels[i].corneringStiffness = 80000.0f;    // N/rad
            wheels[i].peakFriction = 1.2f;              // coefficient
            wheels[i].optimalSlipRatio = 12.0f;         // %
            wheels[i].thermalDegradation = 0.0f;        // %
        }
        std::cout << "Tire parameters calibrated" << std::endl;
    }

    // Diagnostic functions
    void diagnosticsSensorSystems() {
        std::cout << "\nSensor Systems Diagnostics:" << std::endl;
        
        std::cout << "  IMU Status: " << (imuData.calibrated ? "OK" : "FAULT") << std::endl;
        std::cout << "  IMU Temperature: " << imuData.temperature << "°C" << std::endl;
        
        for (int i = 0; i < 4; i++) {
            std::cout << "  Wheel Speed Sensor " << i << ": " 
                      << (wheelSpeedSensors[i].active && !wheelSpeedSensors[i].faultDetected ? "OK" : "FAULT") 
                      << std::endl;
        }
        
        std::cout << "  Steering Angle Sensor: " << (steeringAngleSensor.calibrated ? "OK" : "FAULT") << std::endl;
        
        for (int i = 0; i < 4; i++) {
            std::cout << "  Brake Pressure Sensor " << i << ": " 
                      << (brakePressureSensors[i].active ? "OK" : "FAULT") << std::endl;
        }
    }

    void diagnosticsControlSystems() {
        std::cout << "\nControl Systems Diagnostics:" << std::endl;
        
        std::cout << "  ESC System: " << (esc.enabled ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "    Mode: " << getESCModeString(esc.mode) << std::endl;
        std::cout << "    Intervention Level: " << (esc.interventionLevel * 100) << "%" << std::endl;
        std::cout << "    Currently Active: " << (esc.intervening ? "YES" : "NO") << std::endl;
        
        std::cout << "  Traction Control: " << (tc.enabled ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "    Mode: " << getTCModeString(tc.mode) << std::endl;
        std::cout << "    Slip Threshold: " << tc.slipThreshold << "%" << std::endl;
        
        std::cout << "  ABS System: " << (abs.enabled ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "    Pump Pressure: " << abs.pumpPressure << " bar" << std::endl;
        std::cout << "    Modulation Frequency: " << abs.modulationFrequency << " Hz" << std::endl;
        
        std::cout << "  Torque Vectoring: " << (tv.enabled ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "    Drift Mode: " << (tv.driftMode.enabled ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "    Launch Mode: " << (tv.launchMode.enabled ? "ENABLED" : "DISABLED") << std::endl;
    }

    void diagnosticsActuatorSystems() {
        std::cout << "\nActuator Systems Diagnostics:" << std::endl;
        
        std::cout << "  Dynamic Damping: " << getDamperModeString(damping.currentMode) << std::endl;
        std::cout << "    Sky-Hook Control: " << (damping.skyHookEnabled ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "    Rough Road Mode: " << (damping.roughRoadMode ? "ACTIVE" : "INACTIVE") << std::endl;
        
        std::cout << "  Active Aerodynamics:" << std::endl;
        std::cout << "    Front Spoiler: " << aerodynamics.frontSpoiler.angle << "° " 
                  << (aerodynamics.frontSpoiler.active ? "ACTIVE" : "INACTIVE") << std::endl;
        std::cout << "    Rear Wing: " << aerodynamics.rearWing.angle << "° " 
                  << (aerodynamics.rearWing.drsActive ? "ACTIVE" : "INACTIVE") << std::endl;
        std::cout << "    DRS: " << (aerodynamics.rearWing.drsActive ? "ACTIVE" : "INACTIVE") << std::endl;
        
        std::cout << "  Roll Stabilization: " << (rollStab.enabled ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "    Active Roll Control: " << (rollStab.activeRollControl ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "    Current Roll Angle: " << (rollStab.rollAngle * 180.0f / M_PI) << "°" << std::endl;
    }

    void diagnosticsVehicleHealth() {
        std::cout << "\nVehicle Health Diagnostics:" << std::endl;
        
        for (int i = 0; i < 4; i++) {
            std::cout << "  Tire " << i << ":" << std::endl;
            std::cout << "    Temperature: " << wheels[i].tireTemperature << "°C" << std::endl;
            std::cout << "    Pressure: " << wheels[i].tirePressure << " bar" << std::endl;
            std::cout << "    Tread Depth: " << wheels[i].treadDepth << " mm" << std::endl;
            std::cout << "    Wear Level: " << wheels[i].wearLevel << "%" << std::endl;
            std::cout << "    Thermal Degradation: " << (wheels[i].thermalDegradation * 100) << "%" << std::endl;
        }
        
        std::cout << "  Overall System Health: " << calculateSystemHealth() << "%" << std::endl;
    }

    void diagnosticsPerformanceMetrics() {
        std::cout << "\nPerformance Metrics:" << std::endl;
        std::cout << "  Current Speed: " << (vehicleState.speed * 3.6f) << " km/h" << std::endl;
        std::cout << "  Lateral Acceleration: " << vehicleState.lateralAcceleration << " m/s²" << std::endl;
        std::cout << "  Longitudinal Acceleration: " << vehicleState.longitudinalAcceleration << " m/s²" << std::endl;
        std::cout << "  Yaw Rate: " << (vehicleState.yawRate * 180.0f / M_PI) << " deg/s" << std::endl;
        std::cout << "  Sideslip Angle: " << (vehicleState.sideslipAngle * 180.0f / M_PI) << " deg" << std::endl;
        
        std::cout << "  Total Downforce: " << aerodynamics.totalDownforce << " N" << std::endl;
        std::cout << "  Total Drag: " << aerodynamics.totalDrag << " N" << std::endl;
        std::cout << "  Aerodynamic Balance: " << aerodynamics.aerodynamicBalance << "% front" << std::endl;
    }

    float calculateSystemHealth() {
        float health = 100.0f;
        
        // Reduce health based on tire wear
        for (int i = 0; i < 4; i++) {
            health -= wheels[i].wearLevel * 0.1f; // 0.1% per % wear
            health -= wheels[i].thermalDegradation * 50.0f; // 50% per unit thermal degradation
        }
        
        // Reduce health based on sensor faults
        for (int i = 0; i < 4; i++) {
            if (wheelSpeedSensors[i].faultDetected) health -= 5.0f;
            if (!brakePressureSensors[i].active) health -= 5.0f;
        }
        
        if (!imuData.calibrated) health -= 10.0f;
        if (!steeringAngleSensor.calibrated) health -= 8.0f;
        
        return std::clamp(health, 0.0f, 100.0f);
    }

    // Helper functions
    bool isAnyWheelIntervening() {
        for (int i = 0; i < 4; i++) {
            if (tc.intervening[i]) return true;
        }
        return false;
    }

    bool isABSActive() {
        for (int i = 0; i < 4; i++) {
            if (abs.active[i]) return true;
        }
        return false;
    }

    std::string getVehicleModeString(VehicleMode mode) {
        switch (mode) {
            case VehicleMode::COMFORT: return "COMFORT";
            case VehicleMode::SPORT: return "SPORT";
            case VehicleMode::TRACK: return "TRACK";
            case VehicleMode::ECO: return "ECO";
            case VehicleMode::SNOW: return "SNOW";
            case VehicleMode::SAND: return "SAND";
            case VehicleMode::MUD: return "MUD";
            case VehicleMode::ROCK: return "ROCK";
            case VehicleMode::DRIFT: return "DRIFT";
            case VehicleMode::LAUNCH: return "LAUNCH";
            case VehicleMode::CUSTOM: return "CUSTOM";
        }
        return "UNKNOWN";
    }

    std::string getESCModeString(ESCMode mode) {
        switch (mode) {
            case ESCMode::ON: return "ON";
            case ESCMode::SPORT: return "SPORT";
            case ESCMode::OFF: return "OFF";
            case ESCMode::TRACK_MODE: return "TRACK";
            case ESCMode::DRIFT_MODE: return "DRIFT";
        }
        return "UNKNOWN";
    }

    std::string getTCModeString(TractionControlMode mode) {
        switch (mode) {
            case TractionControlMode::FULL: return "FULL";
            case TractionControlMode::SPORT: return "SPORT";
            case TractionControlMode::OFF: return "OFF";
            case TractionControlMode::SNOW: return "SNOW";
            case TractionControlMode::SAND_MUD: return "SAND_MUD";
        }
        return "UNKNOWN";
    }

    std::string getDamperModeString(DynamicDamping::DamperMode mode) {
        switch (mode) {
            case DynamicDamping::DamperMode::COMFORT: return "COMFORT";
            case DynamicDamping::DamperMode::SPORT: return "SPORT";
            case DynamicDamping::DamperMode::TRACK: return "TRACK";
            case DynamicDamping::DamperMode::ADAPTIVE: return "ADAPTIVE";
            case DynamicDamping::DamperMode::CUSTOM: return "CUSTOM";
        }
        return "UNKNOWN";
    }

    void startControlLoop() {
        std::thread controlThread([this]() {
            while (systemsEnabled) {
                updateVehicleState(0.01f); // 100Hz update rate
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
        controlThread.detach();
    }
};

class VehicleDynamicsTestController {
public:
    VehicleDynamicsTestController() : dynamics(std::make_unique<AdvancedVehicleDynamicsControl>()) {
        std::cout << "Vehicle Dynamics Test Controller initialized" << std::endl;
    }

    void runComprehensiveDynamicsTest() {
        std::cout << "\n=== Comprehensive Vehicle Dynamics Test ===" << std::endl;
        
        testSystemInitialization();
        testVehicleModes();
        testStabilityControl();
        testTractionControl();
        testAntiLockBraking();
        testTorqueVectoring();
        testActiveAerodynamics();
        testDynamicDamping();
        testSpecialModes();
        testSafetyFeatures();
        
        std::cout << "Comprehensive vehicle dynamics test completed" << std::endl;
    }

private:
    std::unique_ptr<AdvancedVehicleDynamicsControl> dynamics;

    void testSystemInitialization() {
        std::cout << "\nTesting system initialization..." << std::endl;
        
        dynamics->performDynamicsCalibration();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        dynamics->getSystemStatus();
        std::cout << "System initialization test completed" << std::endl;
    }

    void testVehicleModes() {
        std::cout << "\nTesting vehicle modes..." << std::endl;
        
        // Test different vehicle modes
        dynamics->setVehicleMode(AdvancedVehicleDynamicsControl::VehicleMode::COMFORT);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        dynamics->setVehicleMode(AdvancedVehicleDynamicsControl::VehicleMode::SPORT);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        dynamics->setVehicleMode(AdvancedVehicleDynamicsControl::VehicleMode::TRACK);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        dynamics->setVehicleMode(AdvancedVehicleDynamicsControl::VehicleMode::ECO);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        std::cout << "Vehicle modes test completed" << std::endl;
    }

    void testStabilityControl() {
        std::cout << "\nTesting electronic stability control..." << std::endl;
        
        // Test different ESC modes
        dynamics->setESCMode(AdvancedVehicleDynamicsControl::ESCMode::ON);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        dynamics->setESCMode(AdvancedVehicleDynamicsControl::ESCMode::SPORT);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        dynamics->setESCMode(AdvancedVehicleDynamicsControl::ESCMode::TRACK_MODE);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Simulate cornering scenario
        dynamics->processDriverInputs(0.3f, 0.6f, 0.0f); // 30° steering, 60% throttle, no brake
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "Electronic stability control test completed" << std::endl;
    }

    void testTractionControl() {
        std::cout << "\nTesting traction control..." << std::endl;
        
        // Simulate high acceleration scenario
        dynamics->processDriverInputs(0.0f, 1.0f, 0.0f); // No steering, full throttle, no brake
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Simulate low traction conditions
        dynamics->setVehicleMode(AdvancedVehicleDynamicsControl::VehicleMode::SNOW);
        dynamics->processDriverInputs(0.0f, 0.8f, 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        std::cout << "Traction control test completed" << std::endl;
    }

    void testAntiLockBraking() {
        std::cout << "\nTesting anti-lock braking system..." << std::endl;
        
        // Simulate emergency braking
        dynamics->processDriverInputs(0.0f, 0.0f, 1.0f); // No steering, no throttle, full brake
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Test brake assist
        dynamics->processDriverInputs(0.0f, 0.0f, 0.9f); // Hard braking
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        std::cout << "Anti-lock braking system test completed" << std::endl;
    }

    void testTorqueVectoring() {
        std::cout << "\nTesting torque vectoring..." << std::endl;
        
        // Test aggressive cornering with torque vectoring
        dynamics->setVehicleMode(AdvancedVehicleDynamicsControl::VehicleMode::SPORT);
        dynamics->processDriverInputs(0.5f, 0.7f, 0.0f); // Sharp turn with acceleration
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        
        // Test understeer correction
        dynamics->processDriverInputs(0.6f, 0.8f, 0.0f); // More aggressive cornering
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        std::cout << "Torque vectoring test completed" << std::endl;
    }

    void testActiveAerodynamics() {
        std::cout << "\nTesting active aerodynamics..." << std::endl;
        
        // Test high-speed aerodynamics
        dynamics->setVehicleMode(AdvancedVehicleDynamicsControl::VehicleMode::TRACK);
        
        // Simulate high-speed straight line
        for (int i = 0; i < 10; i++) {
            float throttle = 0.8f;
            dynamics->processDriverInputs(0.0f, throttle, 0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::cout << "Active aerodynamics test completed" << std::endl;
    }

    void testDynamicDamping() {
        std::cout << "\nTesting dynamic damping system..." << std::endl;
        
        // Test comfort mode damping
        dynamics->setVehicleMode(AdvancedVehicleDynamicsControl::VehicleMode::COMFORT);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Test sport mode damping
        dynamics->setVehicleMode(AdvancedVehicleDynamicsControl::VehicleMode::SPORT);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Test track mode damping
        dynamics->setVehicleMode(AdvancedVehicleDynamicsControl::VehicleMode::TRACK);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        std::cout << "Dynamic damping system test completed" << std::endl;
    }

    void testSpecialModes() {
        std::cout << "\nTesting special driving modes..." << std::endl;
        
        // Test launch control
        dynamics->enableLaunchControl(true);
        dynamics->setVehicleMode(AdvancedVehicleDynamicsControl::VehicleMode::LAUNCH);
        dynamics->processDriverInputs(0.0f, 1.0f, 0.0f); // Launch scenario
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        dynamics->enableLaunchControl(false);
        
        // Test drift mode
        dynamics->enableDriftMode(true);
        dynamics->setVehicleMode(AdvancedVehicleDynamicsControl::VehicleMode::DRIFT);
        dynamics->processDriverInputs(0.4f, 0.9f, 0.0f); // Drift scenario
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        dynamics->enableDriftMode(false);
        
        // Test off-road modes
        dynamics->setVehicleMode(AdvancedVehicleDynamicsControl::VehicleMode::MUD);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        dynamics->setVehicleMode(AdvancedVehicleDynamicsControl::VehicleMode::SAND);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        dynamics->setVehicleMode(AdvancedVehicleDynamicsControl::VehicleMode::ROCK);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        std::cout << "Special driving modes test completed" << std::endl;
    }

    void testSafetyFeatures() {
        std::cout << "\nTesting safety features..." << std::endl;
        
        // Test system diagnostics
        dynamics->runDiagnostics();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Get final system status
        dynamics->getSystemStatus();
        
        std::cout << "Safety features test completed" << std::endl;
    }
};

#endif // ADVANCED_VEHICLE_DYNAMICS_H
