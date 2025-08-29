/**
 * Advanced Motor Control and Regenerative Braking System for Electric Vehicles
 * Author: adzetto
 * Features: PMSM/BLDC control, regenerative braking, torque vectoring, efficiency optimization
 */

#ifndef MOTOR_CONTROL_SYSTEM_H
#define MOTOR_CONTROL_SYSTEM_H

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>
#include <atomic>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

enum class MotorType {
    PMSM,  // Permanent Magnet Synchronous Motor
    BLDC,  // Brushless DC Motor
    ACIM   // AC Induction Motor
};

enum class ControlMode {
    TORQUE_CONTROL,
    SPEED_CONTROL,
    POSITION_CONTROL,
    FIELD_ORIENTED_CONTROL
};

enum class RegenerativeMode {
    DISABLED,
    LIGHT,
    MODERATE,
    AGGRESSIVE,
    MAXIMUM
};

struct MotorParameters {
    double ratedPower;      // kW
    double ratedTorque;     // Nm
    double ratedSpeed;      // RPM
    double maxSpeed;        // RPM
    double resistance;      // Ohms
    double inductance;      // H
    double backEMF;         // V/rpm
    double efficiency;      // %
    int polePairs;
    
    MotorParameters() : ratedPower(100.0), ratedTorque(300.0), ratedSpeed(3000.0),
                       maxSpeed(8000.0), resistance(0.1), inductance(0.001),
                       backEMF(0.1), efficiency(95.0), polePairs(4) {}
};

struct ThreePhaseValues {
    double a, b, c;
    
    ThreePhaseValues(double a = 0, double b = 0, double c = 0) : a(a), b(b), c(c) {}
    
    ThreePhaseValues operator+(const ThreePhaseValues& other) const {
        return ThreePhaseValues(a + other.a, b + other.b, c + other.c);
    }
    
    ThreePhaseValues operator*(double scalar) const {
        return ThreePhaseValues(a * scalar, b * scalar, c * scalar);
    }
};

struct DQValues {
    double d, q;
    
    DQValues(double d = 0, double q = 0) : d(d), q(q) {}
};

class PIController {
private:
    double kp, ki, kd;
    double integral;
    double previousError;
    double outputMin, outputMax;
    std::chrono::steady_clock::time_point lastUpdate;
    
public:
    PIController(double kp = 1.0, double ki = 0.1, double kd = 0.0) 
        : kp(kp), ki(ki), kd(kd), integral(0.0), previousError(0.0),
          outputMin(-1000.0), outputMax(1000.0) {
        lastUpdate = std::chrono::steady_clock::now();
    }
    
    void setLimits(double min, double max) {
        outputMin = min;
        outputMax = max;
    }
    
    void setGains(double p, double i, double d) {
        kp = p;
        ki = i;
        kd = d;
    }
    
    double calculate(double setpoint, double actual) {
        auto now = std::chrono::steady_clock::now();
        auto dt = std::chrono::duration<double>(now - lastUpdate).count();
        lastUpdate = now;
        
        double error = setpoint - actual;
        
        // Proportional term
        double proportional = kp * error;
        
        // Integral term
        integral += error * dt;
        double integralTerm = ki * integral;
        
        // Derivative term
        double derivative = (error - previousError) / dt;
        double derivativeTerm = kd * derivative;
        
        previousError = error;
        
        double output = proportional + integralTerm + derivativeTerm;
        
        // Apply limits
        output = std::max(outputMin, std::min(outputMax, output));
        
        return output;
    }
    
    void reset() {
        integral = 0.0;
        previousError = 0.0;
    }
};

class SpaceVectorModulation {
private:
    double dcVoltage;
    
public:
    SpaceVectorModulation(double vdc = 400.0) : dcVoltage(vdc) {}
    
    ThreePhaseValues calculateDutyCycles(double alpha, double beta) {
        // Sector determination
        double angle = atan2(beta, alpha);
        if (angle < 0) angle += 2 * M_PI;
        
        int sector = static_cast<int>(angle / (M_PI / 3));
        
        // Time calculations
        double magnitude = sqrt(alpha * alpha + beta * beta);
        double modIndex = magnitude / (dcVoltage / sqrt(3));
        
        // Limit modulation index
        modIndex = std::min(modIndex, 1.0);
        
        // Calculate duty cycles based on sector
        ThreePhaseValues duties;
        double sectorAngle = angle - sector * (M_PI / 3);
        
        double t1 = modIndex * sin(M_PI / 3 - sectorAngle);
        double t2 = modIndex * sin(sectorAngle);
        double t0 = 1.0 - t1 - t2;
        
        switch (sector) {
            case 0:
                duties.a = t0/2 + t1 + t2;
                duties.b = t0/2 + t2;
                duties.c = t0/2;
                break;
            case 1:
                duties.a = t0/2 + t1;
                duties.b = t0/2 + t1 + t2;
                duties.c = t0/2;
                break;
            case 2:
                duties.a = t0/2;
                duties.b = t0/2 + t1 + t2;
                duties.c = t0/2 + t2;
                break;
            case 3:
                duties.a = t0/2;
                duties.b = t0/2 + t1;
                duties.c = t0/2 + t1 + t2;
                break;
            case 4:
                duties.a = t0/2 + t2;
                duties.b = t0/2;
                duties.c = t0/2 + t1 + t2;
                break;
            case 5:
                duties.a = t0/2 + t1 + t2;
                duties.b = t0/2;
                duties.c = t0/2 + t1;
                break;
        }
        
        return duties;
    }
    
    void setDCVoltage(double voltage) { dcVoltage = voltage; }
};

class MotorController {
private:
    MotorType motorType;
    MotorParameters params;
    ControlMode controlMode;
    
    // Controllers
    PIController torqueController;
    PIController speedController;
    PIController idController;
    PIController iqController;
    
    // SVM
    SpaceVectorModulation svm;
    
    // State variables
    double targetTorque;
    double actualTorque;
    double targetSpeed;
    double actualSpeed;
    double electricalAngle;
    double mechanicalAngle;
    
    // Three-phase measurements
    ThreePhaseValues phaseCurrent;
    ThreePhaseValues phaseVoltage;
    
    // DQ frame values
    DQValues currentDQ;
    DQValues voltageDQ;
    
    // Control outputs
    ThreePhaseValues dutyCycles;
    
    double dcBusVoltage;
    double temperature;
    bool enabled;
    
    // Efficiency tracking
    double inputPower;
    double outputPower;
    double efficiency;
    
public:
    MotorController(MotorType type = MotorType::PMSM) 
        : motorType(type), controlMode(ControlMode::FIELD_ORIENTED_CONTROL),
          torqueController(5.0, 0.1, 0.01), speedController(0.5, 0.05, 0.0),
          idController(10.0, 0.5, 0.0), iqController(10.0, 0.5, 0.0),
          svm(400.0), targetTorque(0.0), actualTorque(0.0),
          targetSpeed(0.0), actualSpeed(0.0), electricalAngle(0.0),
          mechanicalAngle(0.0), dcBusVoltage(400.0), temperature(25.0),
          enabled(false), inputPower(0.0), outputPower(0.0), efficiency(0.0) {
        
        // Set controller limits
        torqueController.setLimits(-params.ratedTorque * 1.5, params.ratedTorque * 1.5);
        speedController.setLimits(-params.ratedTorque, params.ratedTorque);
        idController.setLimits(-dcBusVoltage/2, dcBusVoltage/2);
        iqController.setLimits(-dcBusVoltage/2, dcBusVoltage/2);
    }
    
    void setControlMode(ControlMode mode) { 
        controlMode = mode; 
        resetControllers();
    }
    
    void enable() { 
        enabled = true; 
        resetControllers();
        std::cout << "Motor controller enabled\n";
    }
    
    void disable() { 
        enabled = false; 
        targetTorque = 0.0;
        targetSpeed = 0.0;
        std::cout << "Motor controller disabled\n";
    }
    
    void setTorqueCommand(double torque) {
        if (!enabled) return;
        
        targetTorque = std::max(-params.ratedTorque * 1.5, 
                               std::min(params.ratedTorque * 1.5, torque));
        
        if (controlMode == ControlMode::SPEED_CONTROL) {
            controlMode = ControlMode::TORQUE_CONTROL;
        }
    }
    
    void setSpeedCommand(double speed) {
        if (!enabled) return;
        
        targetSpeed = std::max(-params.maxSpeed, 
                              std::min(params.maxSpeed, speed));
        controlMode = ControlMode::SPEED_CONTROL;
    }
    
    void updateMeasurements(const ThreePhaseValues& current, const ThreePhaseValues& voltage,
                           double speed, double angle, double busVoltage, double temp) {
        phaseCurrent = current;
        phaseVoltage = voltage;
        actualSpeed = speed;
        mechanicalAngle = angle;
        electricalAngle = angle * params.polePairs;
        dcBusVoltage = busVoltage;
        temperature = temp;
        
        // Clarke and Park transformations
        performClarkeTransform();
        performParkTransform();
        
        // Calculate actual torque
        actualTorque = 1.5 * params.polePairs * params.backEMF * currentDQ.q;
        
        // Calculate power and efficiency
        inputPower = dcBusVoltage * (phaseCurrent.a + phaseCurrent.b + phaseCurrent.c) / 3.0;
        outputPower = actualTorque * actualSpeed * 2 * M_PI / 60.0 / 1000.0; // kW
        
        if (inputPower > 0.1) {
            efficiency = (outputPower / inputPower) * 100.0;
        } else {
            efficiency = 0.0;
        }
    }
    
    void controlLoop() {
        if (!enabled) {
            dutyCycles = ThreePhaseValues(0, 0, 0);
            return;
        }
        
        switch (controlMode) {
            case ControlMode::TORQUE_CONTROL:
                performTorqueControl();
                break;
            case ControlMode::SPEED_CONTROL:
                performSpeedControl();
                break;
            case ControlMode::FIELD_ORIENTED_CONTROL:
                performFieldOrientedControl();
                break;
            default:
                break;
        }
        
        // Apply thermal derating if necessary
        applyThermalDerating();
        
        // Generate PWM signals
        generatePWMSignals();
    }
    
    // Getters
    double getActualTorque() const { return actualTorque; }
    double getActualSpeed() const { return actualSpeed; }
    double getEfficiency() const { return efficiency; }
    double getInputPower() const { return inputPower; }
    double getOutputPower() const { return outputPower; }
    double getTemperature() const { return temperature; }
    ThreePhaseValues getDutyCycles() const { return dutyCycles; }
    bool isEnabled() const { return enabled; }
    
    void displayStatus() const {
        std::cout << "\n=== Motor Controller Status ===\n";
        std::cout << "Enabled: " << (enabled ? "YES" : "NO") << "\n";
        std::cout << "Control Mode: " << getControlModeString() << "\n";
        std::cout << "Target Torque: " << targetTorque << " Nm\n";
        std::cout << "Actual Torque: " << actualTorque << " Nm\n";
        std::cout << "Target Speed: " << targetSpeed << " RPM\n";
        std::cout << "Actual Speed: " << actualSpeed << " RPM\n";
        std::cout << "Input Power: " << inputPower << " kW\n";
        std::cout << "Output Power: " << outputPower << " kW\n";
        std::cout << "Efficiency: " << efficiency << " %\n";
        std::cout << "Temperature: " << temperature << " °C\n";
        std::cout << "DC Bus Voltage: " << dcBusVoltage << " V\n";
        std::cout << "==============================\n\n";
    }
    
private:
    void performClarkeTransform() {
        // Clarke transformation (3-phase to alpha-beta)
        double alpha = phaseCurrent.a;
        double beta = (phaseCurrent.a + 2 * phaseCurrent.b) / sqrt(3.0);
        
        // Store in temporary variables for Park transform
        currentDQ.d = alpha;  // Temporary storage
        currentDQ.q = beta;   // Temporary storage
    }
    
    void performParkTransform() {
        // Park transformation (alpha-beta to d-q)
        double alpha = currentDQ.d;  // Retrieved from Clarke transform
        double beta = currentDQ.q;   // Retrieved from Clarke transform
        
        double cosTheta = cos(electricalAngle);
        double sinTheta = sin(electricalAngle);
        
        currentDQ.d = alpha * cosTheta + beta * sinTheta;
        currentDQ.q = -alpha * sinTheta + beta * cosTheta;
    }
    
    void performTorqueControl() {
        // Direct torque control using q-axis current
        double targetIq = targetTorque / (1.5 * params.polePairs * params.backEMF);
        double targetId = 0.0; // For PMSM, typically zero for maximum efficiency
        
        voltageDQ.d = idController.calculate(targetId, currentDQ.d);
        voltageDQ.q = iqController.calculate(targetIq, currentDQ.q);
    }
    
    void performSpeedControl() {
        // Speed control with torque as intermediate variable
        double torqueCommand = speedController.calculate(targetSpeed, actualSpeed);
        targetTorque = torqueCommand;
        performTorqueControl();
    }
    
    void performFieldOrientedControl() {
        // Advanced field-oriented control with field weakening
        double targetId = 0.0;
        
        // Field weakening for high speeds
        if (abs(actualSpeed) > params.ratedSpeed * 0.8) {
            double fieldWeakeningFactor = abs(actualSpeed) / params.maxSpeed;
            targetId = -fieldWeakeningFactor * 50.0; // Negative Id for field weakening
        }
        
        double targetIq = targetTorque / (1.5 * params.polePairs * params.backEMF);
        
        // Feed-forward compensation
        double feedForwardD = -actualSpeed * params.inductance * currentDQ.q * 2 * M_PI / 60.0;
        double feedForwardQ = actualSpeed * (params.inductance * currentDQ.d + params.backEMF) * 2 * M_PI / 60.0;
        
        voltageDQ.d = idController.calculate(targetId, currentDQ.d) + feedForwardD;
        voltageDQ.q = iqController.calculate(targetIq, currentDQ.q) + feedForwardQ;
    }
    
    void generatePWMSignals() {
        // Inverse Park transformation (d-q to alpha-beta)
        double cosTheta = cos(electricalAngle);
        double sinTheta = sin(electricalAngle);
        
        double vAlpha = voltageDQ.d * cosTheta - voltageDQ.q * sinTheta;
        double vBeta = voltageDQ.d * sinTheta + voltageDQ.q * cosTheta;
        
        // Space Vector Modulation
        dutyCycles = svm.calculateDutyCycles(vAlpha, vBeta);
        
        // Apply limits
        dutyCycles.a = std::max(0.0, std::min(1.0, dutyCycles.a));
        dutyCycles.b = std::max(0.0, std::min(1.0, dutyCycles.b));
        dutyCycles.c = std::max(0.0, std::min(1.0, dutyCycles.c));
    }
    
    void applyThermalDerating() {
        if (temperature > 80.0) {
            double deratingFactor = std::max(0.5, (100.0 - temperature) / 20.0);
            targetTorque *= deratingFactor;
            
            if (temperature > 100.0) {
                disable();
                std::cout << "Motor disabled due to overtemperature!\n";
            }
        }
    }
    
    void resetControllers() {
        torqueController.reset();
        speedController.reset();
        idController.reset();
        iqController.reset();
    }
    
    std::string getControlModeString() const {
        switch (controlMode) {
            case ControlMode::TORQUE_CONTROL: return "TORQUE";
            case ControlMode::SPEED_CONTROL: return "SPEED";
            case ControlMode::POSITION_CONTROL: return "POSITION";
            case ControlMode::FIELD_ORIENTED_CONTROL: return "FOC";
            default: return "UNKNOWN";
        }
    }
};

class RegenerativeBraking {
private:
    RegenerativeMode mode;
    double maxRegenerativePower;
    double currentRegenerativePower;
    double batterySOC;
    double batteryMaxCurrent;
    double vehicleSpeed;
    double brakePedal;
    double acceleratorPedal;
    
    // Regen parameters
    double coastingRegen;
    double brakingRegen;
    double maxRecoveryEfficiency;
    
    bool enabled;
    
public:
    RegenerativeBraking() : mode(RegenerativeMode::MODERATE), maxRegenerativePower(50.0),
                           currentRegenerativePower(0.0), batterySOC(50.0), 
                           batteryMaxCurrent(100.0), vehicleSpeed(0.0),
                           brakePedal(0.0), acceleratorPedal(0.0),
                           coastingRegen(0.2), brakingRegen(0.8),
                           maxRecoveryEfficiency(0.85), enabled(true) {}
    
    void setMode(RegenerativeMode newMode) {
        mode = newMode;
        updateRegenParameters();
        std::cout << "Regenerative braking mode set to: " << getModeString() << "\n";
    }
    
    void updateInputs(double speed, double brake, double accel, double soc) {
        vehicleSpeed = speed;
        brakePedal = std::max(0.0, std::min(1.0, brake));
        acceleratorPedal = std::max(0.0, std::min(1.0, accel));
        batterySOC = soc;
    }
    
    double calculateRegenerativeTorque() {
        if (!enabled || vehicleSpeed < 5.0) { // No regen below 5 km/h
            currentRegenerativePower = 0.0;
            return 0.0;
        }
        
        // Check battery conditions
        if (batterySOC > 95.0) {
            currentRegenerativePower = 0.0;
            return 0.0; // Battery too full
        }
        
        double regenTorque = 0.0;
        
        // Coasting regeneration (when accelerator is released)
        if (acceleratorPedal < 0.1 && brakePedal < 0.1) {
            regenTorque = calculateCoastingRegen();
        }
        
        // Braking regeneration
        if (brakePedal > 0.1) {
            regenTorque += calculateBrakingRegen();
        }
        
        // Apply speed-based limiting
        regenTorque = applySpeedLimiting(regenTorque);
        
        // Apply battery limiting
        regenTorque = applyBatteryLimiting(regenTorque);
        
        // Calculate power
        currentRegenerativePower = abs(regenTorque) * vehicleSpeed * 2 * M_PI / 60.0 / 1000.0;
        
        return -regenTorque; // Negative for regenerative torque
    }
    
    double getRegenerativePower() const { return currentRegenerativePower; }
    double getEnergyRecovered() const { return currentRegenerativePower * maxRecoveryEfficiency; }
    
    void displayStatus() const {
        std::cout << "\n=== Regenerative Braking Status ===\n";
        std::cout << "Mode: " << getModeString() << "\n";
        std::cout << "Enabled: " << (enabled ? "YES" : "NO") << "\n";
        std::cout << "Vehicle Speed: " << vehicleSpeed << " km/h\n";
        std::cout << "Brake Pedal: " << brakePedal * 100 << " %\n";
        std::cout << "Accelerator: " << acceleratorPedal * 100 << " %\n";
        std::cout << "Battery SOC: " << batterySOC << " %\n";
        std::cout << "Regen Power: " << currentRegenerativePower << " kW\n";
        std::cout << "Energy Recovery: " << getEnergyRecovered() << " kW\n";
        std::cout << "Recovery Efficiency: " << maxRecoveryEfficiency * 100 << " %\n";
        std::cout << "=================================\n\n";
    }
    
    void enable() { enabled = true; }
    void disable() { enabled = false; }
    bool isEnabled() const { return enabled; }
    
private:
    void updateRegenParameters() {
        switch (mode) {
            case RegenerativeMode::DISABLED:
                coastingRegen = 0.0;
                brakingRegen = 0.0;
                break;
            case RegenerativeMode::LIGHT:
                coastingRegen = 0.1;
                brakingRegen = 0.3;
                break;
            case RegenerativeMode::MODERATE:
                coastingRegen = 0.2;
                brakingRegen = 0.6;
                break;
            case RegenerativeMode::AGGRESSIVE:
                coastingRegen = 0.4;
                brakingRegen = 0.8;
                break;
            case RegenerativeMode::MAXIMUM:
                coastingRegen = 0.6;
                brakingRegen = 1.0;
                break;
        }
    }
    
    double calculateCoastingRegen() {
        // Speed-dependent coasting regeneration
        double speedFactor = std::min(1.0, vehicleSpeed / 60.0); // Full regen at 60 km/h
        return coastingRegen * maxRegenerativePower * speedFactor / (vehicleSpeed / 9.55); // Convert to torque
    }
    
    double calculateBrakingRegen() {
        // Brake pedal proportional regeneration
        double brakeFactor = brakePedal;
        return brakingRegen * maxRegenerativePower * brakeFactor / (vehicleSpeed / 9.55);
    }
    
    double applySpeedLimiting(double torque) {
        // Reduce regeneration at very low speeds for comfort
        if (vehicleSpeed < 10.0) {
            double speedFactor = vehicleSpeed / 10.0;
            torque *= speedFactor;
        }
        
        return torque;
    }
    
    double applyBatteryLimiting(double torque) {
        // Reduce regeneration as battery SOC increases
        if (batterySOC > 80.0) {
            double socFactor = (95.0 - batterySOC) / 15.0;
            torque *= std::max(0.1, socFactor);
        }
        
        // Limit based on battery charging capability
        double maxChargePower = batteryMaxCurrent * 400.0 / 1000.0; // Assuming 400V system
        double currentPower = abs(torque) * vehicleSpeed * 2 * M_PI / 60.0 / 1000.0;
        
        if (currentPower > maxChargePower) {
            torque *= maxChargePower / currentPower;
        }
        
        return torque;
    }
    
    std::string getModeString() const {
        switch (mode) {
            case RegenerativeMode::DISABLED: return "DISABLED";
            case RegenerativeMode::LIGHT: return "LIGHT";
            case RegenerativeMode::MODERATE: return "MODERATE";
            case RegenerativeMode::AGGRESSIVE: return "AGGRESSIVE";
            case RegenerativeMode::MAXIMUM: return "MAXIMUM";
            default: return "UNKNOWN";
        }
    }
};

class TorqueVectoring {
private:
    bool enabled;
    double frontLeftTorque;
    double frontRightTorque;
    double rearLeftTorque;
    double rearRightTorque;
    
    double steeringAngle;
    double yawRate;
    double lateralAcceleration;
    double vehicleSpeed;
    
    // Vehicle parameters
    double wheelbase;
    double trackWidth;
    double vehicleMass;
    
public:
    TorqueVectoring() : enabled(false), frontLeftTorque(0), frontRightTorque(0),
                       rearLeftTorque(0), rearRightTorque(0), steeringAngle(0),
                       yawRate(0), lateralAcceleration(0), vehicleSpeed(0),
                       wheelbase(2.8), trackWidth(1.6), vehicleMass(1800) {}
    
    void enable() { 
        enabled = true; 
        std::cout << "Torque vectoring enabled\n";
    }
    
    void disable() { 
        enabled = false;
        resetTorques();
        std::cout << "Torque vectoring disabled\n";
    }
    
    void updateVehicleState(double steering, double yaw, double lateral, double speed) {
        steeringAngle = steering;
        yawRate = yaw;
        lateralAcceleration = lateral;
        vehicleSpeed = speed;
    }
    
    void calculateTorqueDistribution(double totalTorque) {
        if (!enabled || vehicleSpeed < 10.0) {
            // Equal distribution when disabled or at low speeds
            distributeEqually(totalTorque);
            return;
        }
        
        // Calculate desired yaw moment based on steering input and vehicle dynamics
        double desiredYawMoment = calculateDesiredYawMoment();
        
        // Calculate torque differences needed for yaw control
        double lateralTorqueDiff = desiredYawMoment / trackWidth;
        
        // Base torque distribution (50% front, 50% rear for AWD)
        double frontTorque = totalTorque * 0.5;
        double rearTorque = totalTorque * 0.5;
        
        // Apply yaw moment correction
        if (steeringAngle > 0) { // Right turn
            frontRightTorque = frontTorque * 0.5 - lateralTorqueDiff * 0.3;
            frontLeftTorque = frontTorque * 0.5 + lateralTorqueDiff * 0.3;
            rearRightTorque = rearTorque * 0.5 - lateralTorqueDiff * 0.2;
            rearLeftTorque = rearTorque * 0.5 + lateralTorqueDiff * 0.2;
        } else if (steeringAngle < 0) { // Left turn
            frontRightTorque = frontTorque * 0.5 + lateralTorqueDiff * 0.3;
            frontLeftTorque = frontTorque * 0.5 - lateralTorqueDiff * 0.3;
            rearRightTorque = rearTorque * 0.5 + lateralTorqueDiff * 0.2;
            rearLeftTorque = rearTorque * 0.5 - lateralTorqueDiff * 0.2;
        } else {
            distributeEqually(totalTorque);
        }
        
        // Apply limits
        applyTorqueLimits();
    }
    
    void displayStatus() const {
        std::cout << "\n=== Torque Vectoring Status ===\n";
        std::cout << "Enabled: " << (enabled ? "YES" : "NO") << "\n";
        std::cout << "Vehicle Speed: " << vehicleSpeed << " km/h\n";
        std::cout << "Steering Angle: " << steeringAngle << " deg\n";
        std::cout << "Yaw Rate: " << yawRate << " deg/s\n";
        std::cout << "Lateral Accel: " << lateralAcceleration << " m/s²\n";
        std::cout << "Front Left: " << frontLeftTorque << " Nm\n";
        std::cout << "Front Right: " << frontRightTorque << " Nm\n";
        std::cout << "Rear Left: " << rearLeftTorque << " Nm\n";
        std::cout << "Rear Right: " << rearRightTorque << " Nm\n";
        std::cout << "==============================\n\n";
    }
    
    // Getters
    double getFrontLeftTorque() const { return frontLeftTorque; }
    double getFrontRightTorque() const { return frontRightTorque; }
    double getRearLeftTorque() const { return rearLeftTorque; }
    double getRearRightTorque() const { return rearRightTorque; }
    bool isEnabled() const { return enabled; }
    
private:
    void distributeEqually(double totalTorque) {
        double quarterTorque = totalTorque * 0.25;
        frontLeftTorque = quarterTorque;
        frontRightTorque = quarterTorque;
        rearLeftTorque = quarterTorque;
        rearRightTorque = quarterTorque;
    }
    
    double calculateDesiredYawMoment() {
        // Simplified yaw moment calculation based on steering input and vehicle speed
        double steeringFactor = sin(steeringAngle * M_PI / 180.0);
        double speedFactor = std::min(1.0, vehicleSpeed / 50.0);
        
        return steeringFactor * speedFactor * 500.0; // Maximum 500 Nm yaw moment
    }
    
    void applyTorqueLimits() {
        double maxTorquePerWheel = 400.0; // Nm
        
        frontLeftTorque = std::max(-maxTorquePerWheel, std::min(maxTorquePerWheel, frontLeftTorque));
        frontRightTorque = std::max(-maxTorquePerWheel, std::min(maxTorquePerWheel, frontRightTorque));
        rearLeftTorque = std::max(-maxTorquePerWheel, std::min(maxTorquePerWheel, rearLeftTorque));
        rearRightTorque = std::max(-maxTorquePerWheel, std::min(maxTorquePerWheel, rearRightTorque));
    }
    
    void resetTorques() {
        frontLeftTorque = 0;
        frontRightTorque = 0;
        rearLeftTorque = 0;
        rearRightTorque = 0;
    }
};

#endif // MOTOR_CONTROL_SYSTEM_H