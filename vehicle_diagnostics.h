/**
 * Comprehensive Vehicle Diagnostics and Fault Detection System
 * Author: adzetto
 * Features: OBD-II compatibility, predictive maintenance, fault isolation, logging
 */

#ifndef VEHICLE_DIAGNOSTICS_H
#define VEHICLE_DIAGNOSTICS_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <string>
#include <chrono>
#include <memory>
#include <queue>
#include <fstream>
#include <algorithm>
#include <bitset>

enum class FaultSeverity {
    INFO,
    WARNING,
    ERROR,
    CRITICAL,
    EMERGENCY
};

enum class SystemType {
    BATTERY_MANAGEMENT,
    MOTOR_CONTROL,
    CHARGING_SYSTEM,
    THERMAL_MANAGEMENT,
    ADAS,
    COMMUNICATION,
    POWER_ELECTRONICS,
    HVAC,
    LIGHTING,
    SAFETY_SYSTEMS
};

enum class FaultType {
    SENSOR_FAULT,
    ACTUATOR_FAULT,
    COMMUNICATION_FAULT,
    PARAMETER_OUT_OF_RANGE,
    SYSTEM_TIMEOUT,
    CHECKSUM_ERROR,
    CALIBRATION_ERROR,
    HARDWARE_FAULT,
    SOFTWARE_FAULT,
    PERFORMANCE_DEGRADATION
};

struct DiagnosticTroubleCode {
    uint16_t code;
    std::string description;
    SystemType system;
    FaultType faultType;
    FaultSeverity severity;
    std::chrono::steady_clock::time_point timestamp;
    bool isActive;
    bool isPending;
    bool isConfirmed;
    uint8_t occurenceCount;
    
    DiagnosticTroubleCode(uint16_t c, const std::string& desc, SystemType sys, 
                         FaultType type, FaultSeverity sev) 
        : code(c), description(desc), system(sys), faultType(type), severity(sev),
          isActive(false), isPending(false), isConfirmed(false), occurenceCount(0) {
        timestamp = std::chrono::steady_clock::now();
    }
};

struct SensorReading {
    int sensorId;
    std::string sensorName;
    double value;
    double minValue;
    double maxValue;
    double nominalValue;
    bool isValid;
    std::chrono::steady_clock::time_point timestamp;
    
    SensorReading(int id, const std::string& name, double min, double max, double nominal)
        : sensorId(id), sensorName(name), value(0.0), minValue(min), maxValue(max),
          nominalValue(nominal), isValid(true) {
        timestamp = std::chrono::steady_clock::now();
    }
    
    void updateValue(double newValue) {
        value = newValue;
        timestamp = std::chrono::steady_clock::now();
        isValid = (value >= minValue && value <= maxValue);
    }
    
    bool isOutOfRange() const { return !isValid; }
    double getDeviationPercent() const {
        return abs(value - nominalValue) / nominalValue * 100.0;
    }
};

class FaultDetector {
private:
    std::vector<std::unique_ptr<SensorReading>> sensors;
    std::unordered_map<uint16_t, DiagnosticTroubleCode> dtcCodes;
    std::queue<uint16_t> activeFaults;
    std::queue<uint16_t> pendingFaults;
    
    // Statistical analysis for predictive maintenance
    std::unordered_map<int, std::vector<double>> sensorHistory;
    std::unordered_map<int, double> sensorTrends;
    
    uint32_t totalFaults;
    uint32_t criticalFaults;
    
public:
    FaultDetector() : totalFaults(0), criticalFaults(0) {
        initializeDTCCodes();
        initializeSensors();
    }
    
    void addSensor(int id, const std::string& name, double min, double max, double nominal) {
        sensors.push_back(std::make_unique<SensorReading>(id, name, min, max, nominal));
        sensorHistory[id] = std::vector<double>();
        sensorHistory[id].reserve(1000); // Store last 1000 readings
    }
    
    void updateSensorReading(int sensorId, double value) {
        for (auto& sensor : sensors) {
            if (sensor->sensorId == sensorId) {
                sensor->updateValue(value);
                
                // Store in history for trend analysis
                sensorHistory[sensorId].push_back(value);
                if (sensorHistory[sensorId].size() > 1000) {
                    sensorHistory[sensorId].erase(sensorHistory[sensorId].begin());
                }
                
                // Check for faults
                checkSensorFaults(*sensor);
                
                // Update trend analysis
                updateTrendAnalysis(sensorId);
                break;
            }
        }
    }
    
    void raiseFault(uint16_t dtcCode) {
        if (dtcCodes.find(dtcCode) != dtcCodes.end()) {
            auto& dtc = dtcCodes[dtcCode];
            
            if (!dtc.isActive) {
                dtc.isPending = true;
                pendingFaults.push(dtcCode);
                
                // Confirm fault after multiple occurrences
                dtc.occurenceCount++;
                if (dtc.occurenceCount >= 3) {
                    confirmFault(dtcCode);
                }
            }
            
            dtc.timestamp = std::chrono::steady_clock::now();
            totalFaults++;
            
            if (dtc.severity >= FaultSeverity::CRITICAL) {
                criticalFaults++;
            }
            
            std::cout << "Fault raised: " << std::hex << dtcCode << std::dec 
                      << " - " << dtc.description << "\n";
        }
    }
    
    void clearFault(uint16_t dtcCode) {
        if (dtcCodes.find(dtcCode) != dtcCodes.end()) {
            auto& dtc = dtcCodes[dtcCode];
            dtc.isActive = false;
            dtc.isPending = false;
            dtc.occurenceCount = 0;
            
            std::cout << "Fault cleared: " << std::hex << dtcCode << std::dec 
                      << " - " << dtc.description << "\n";
        }
    }
    
    std::vector<DiagnosticTroubleCode> getActiveFaults() const {
        std::vector<DiagnosticTroubleCode> active;
        for (const auto& pair : dtcCodes) {
            if (pair.second.isActive) {
                active.push_back(pair.second);
            }
        }
        return active;
    }
    
    std::vector<DiagnosticTroubleCode> getPendingFaults() const {
        std::vector<DiagnosticTroubleCode> pending;
        for (const auto& pair : dtcCodes) {
            if (pair.second.isPending && !pair.second.isActive) {
                pending.push_back(pair.second);
            }
        }
        return pending;
    }
    
    void performDiagnosticScan() {
        std::cout << "\n=== Diagnostic Scan Results ===\n";
        
        // Check all sensors
        for (const auto& sensor : sensors) {
            if (sensor->isOutOfRange()) {
                std::cout << "Sensor " << sensor->sensorName 
                          << " out of range: " << sensor->value << "\n";
            }
        }
        
        // Display active faults
        auto activeFaults = getActiveFaults();
        std::cout << "Active DTCs: " << activeFaults.size() << "\n";
        for (const auto& fault : activeFaults) {
            displayDTCInfo(fault);
        }
        
        // Display pending faults
        auto pendingFaults = getPendingFaults();
        std::cout << "Pending DTCs: " << pendingFaults.size() << "\n";
        for (const auto& fault : pendingFaults) {
            displayDTCInfo(fault);
        }
        
        std::cout << "==============================\n\n";
    }
    
    void generateDiagnosticReport() {
        std::ofstream report("diagnostic_report.txt");
        if (!report.is_open()) return;
        
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        
        report << "Vehicle Diagnostic Report\n";
        report << "Generated: " << std::ctime(&time_t);
        report << "========================\n\n";
        
        // System health summary
        report << "System Health Summary:\n";
        report << "Total Faults: " << totalFaults << "\n";
        report << "Critical Faults: " << criticalFaults << "\n";
        report << "Active DTCs: " << getActiveFaults().size() << "\n";
        report << "Pending DTCs: " << getPendingFaults().size() << "\n\n";
        
        // Sensor status
        report << "Sensor Status:\n";
        for (const auto& sensor : sensors) {
            report << sensor->sensorName << ": " << sensor->value;
            if (sensor->isOutOfRange()) {
                report << " [OUT OF RANGE]";
            }
            if (sensorTrends[sensor->sensorId] > 5.0) {
                report << " [DEGRADING]";
            }
            report << "\n";
        }
        report << "\n";
        
        // Active faults details
        report << "Active Fault Details:\n";
        for (const auto& fault : getActiveFaults()) {
            report << "DTC: " << std::hex << fault.code << std::dec << "\n";
            report << "Description: " << fault.description << "\n";
            report << "System: " << getSystemTypeString(fault.system) << "\n";
            report << "Severity: " << getSeverityString(fault.severity) << "\n";
            report << "Occurrences: " << static_cast<int>(fault.occurenceCount) << "\n\n";
        }
        
        report.close();
        std::cout << "Diagnostic report generated: diagnostic_report.txt\n";
    }
    
    void displaySystemStatus() {
        std::cout << "\n=== Vehicle Diagnostics Status ===\n";
        std::cout << "Total Faults Detected: " << totalFaults << "\n";
        std::cout << "Critical Faults: " << criticalFaults << "\n";
        std::cout << "Active DTCs: " << getActiveFaults().size() << "\n";
        std::cout << "Pending DTCs: " << getPendingFaults().size() << "\n";
        
        // System-wise status
        std::unordered_map<SystemType, int> systemFaults;
        for (const auto& pair : dtcCodes) {
            if (pair.second.isActive) {
                systemFaults[pair.second.system]++;
            }
        }
        
        std::cout << "\nSystem Health:\n";
        for (const auto& pair : systemFaults) {
            std::cout << getSystemTypeString(pair.first) << ": " 
                      << pair.second << " faults\n";
        }
        
        // Sensor health
        std::cout << "\nSensor Health:\n";
        int healthySensors = 0;
        int faultySensors = 0;
        for (const auto& sensor : sensors) {
            if (sensor->isValid) {
                healthySensors++;
            } else {
                faultySensors++;
            }
        }
        std::cout << "Healthy Sensors: " << healthySensors << "\n";
        std::cout << "Faulty Sensors: " << faultySensors << "\n";
        std::cout << "=================================\n\n";
    }
    
    // Predictive maintenance functions
    void performPredictiveMaintenance() {
        std::cout << "\n=== Predictive Maintenance Analysis ===\n";
        
        for (const auto& sensor : sensors) {
            int sensorId = sensor->sensorId;
            double trend = sensorTrends[sensorId];
            double deviation = sensor->getDeviationPercent();
            
            if (trend > 10.0 || deviation > 25.0) {
                std::cout << "WARNING: " << sensor->sensorName 
                          << " showing degradation trend (" << trend << "%)\n";
                
                // Estimate time to failure
                double timeToFailure = estimateTimeToFailure(sensorId);
                if (timeToFailure < 1000.0) { // Less than 1000 hours
                    std::cout << "  Estimated time to failure: " 
                              << timeToFailure << " hours\n";
                }
            }
        }
        
        std::cout << "=====================================\n\n";
    }
    
private:
    void initializeDTCCodes() {
        // Battery Management System DTCs
        dtcCodes[0xB001] = DiagnosticTroubleCode(0xB001, "Battery cell overvoltage", 
            SystemType::BATTERY_MANAGEMENT, FaultType::PARAMETER_OUT_OF_RANGE, FaultSeverity::CRITICAL);
        dtcCodes[0xB002] = DiagnosticTroubleCode(0xB002, "Battery cell undervoltage", 
            SystemType::BATTERY_MANAGEMENT, FaultType::PARAMETER_OUT_OF_RANGE, FaultSeverity::CRITICAL);
        dtcCodes[0xB003] = DiagnosticTroubleCode(0xB003, "Battery overtemperature", 
            SystemType::BATTERY_MANAGEMENT, FaultType::PARAMETER_OUT_OF_RANGE, FaultSeverity::EMERGENCY);
        dtcCodes[0xB004] = DiagnosticTroubleCode(0xB004, "Cell balancing fault", 
            SystemType::BATTERY_MANAGEMENT, FaultType::SYSTEM_TIMEOUT, FaultSeverity::WARNING);
        
        // Motor Control DTCs
        dtcCodes[0xM001] = DiagnosticTroubleCode(0xM001, "Motor overtemperature", 
            SystemType::MOTOR_CONTROL, FaultType::PARAMETER_OUT_OF_RANGE, FaultSeverity::CRITICAL);
        dtcCodes[0xM002] = DiagnosticTroubleCode(0xM002, "Motor overcurrent", 
            SystemType::MOTOR_CONTROL, FaultType::PARAMETER_OUT_OF_RANGE, FaultSeverity::CRITICAL);
        dtcCodes[0xM003] = DiagnosticTroubleCode(0xM003, "Motor position sensor fault", 
            SystemType::MOTOR_CONTROL, FaultType::SENSOR_FAULT, FaultSeverity::ERROR);
        
        // ADAS DTCs
        dtcCodes[0xA001] = DiagnosticTroubleCode(0xA001, "Radar sensor malfunction", 
            SystemType::ADAS, FaultType::SENSOR_FAULT, FaultSeverity::WARNING);
        dtcCodes[0xA002] = DiagnosticTroubleCode(0xA002, "Camera system fault", 
            SystemType::ADAS, FaultType::SENSOR_FAULT, FaultSeverity::WARNING);
        dtcCodes[0xA003] = DiagnosticTroubleCode(0xA003, "LIDAR communication error", 
            SystemType::ADAS, FaultType::COMMUNICATION_FAULT, FaultSeverity::ERROR);
        
        // Thermal Management DTCs
        dtcCodes[0xT001] = DiagnosticTroubleCode(0xT001, "Cooling pump failure", 
            SystemType::THERMAL_MANAGEMENT, FaultType::ACTUATOR_FAULT, FaultSeverity::ERROR);
        dtcCodes[0xT002] = DiagnosticTroubleCode(0xT002, "Radiator fan malfunction", 
            SystemType::THERMAL_MANAGEMENT, FaultType::ACTUATOR_FAULT, FaultSeverity::WARNING);
        
        // Communication DTCs
        dtcCodes[0xC001] = DiagnosticTroubleCode(0xC001, "CAN bus error", 
            SystemType::COMMUNICATION, FaultType::COMMUNICATION_FAULT, FaultSeverity::ERROR);
        dtcCodes[0xC002] = DiagnosticTroubleCode(0xC002, "Node communication timeout", 
            SystemType::COMMUNICATION, FaultType::SYSTEM_TIMEOUT, FaultSeverity::WARNING);
    }
    
    void initializeSensors() {
        // Battery sensors
        addSensor(1, "Battery Voltage", 300.0, 420.0, 370.0);
        addSensor(2, "Battery Current", -200.0, 200.0, 0.0);
        addSensor(3, "Battery Temperature", -10.0, 60.0, 25.0);
        addSensor(4, "Battery SOC", 0.0, 100.0, 50.0);
        
        // Motor sensors
        addSensor(5, "Motor Temperature", -40.0, 150.0, 70.0);
        addSensor(6, "Motor Speed", 0.0, 8000.0, 2000.0);
        addSensor(7, "Motor Torque", -500.0, 500.0, 0.0);
        
        // Vehicle sensors
        addSensor(8, "Vehicle Speed", 0.0, 200.0, 50.0);
        addSensor(9, "Ambient Temperature", -40.0, 60.0, 20.0);
        addSensor(10, "DC Bus Voltage", 300.0, 450.0, 400.0);
    }
    
    void checkSensorFaults(const SensorReading& sensor) {
        if (sensor.isOutOfRange()) {
            // Map sensor ID to appropriate DTC
            switch (sensor.sensorId) {
                case 1: // Battery Voltage
                    if (sensor.value > sensor.maxValue) {
                        raiseFault(0xB001); // Overvoltage
                    } else {
                        raiseFault(0xB002); // Undervoltage
                    }
                    break;
                case 3: // Battery Temperature
                    if (sensor.value > sensor.maxValue) {
                        raiseFault(0xB003); // Overtemperature
                    }
                    break;
                case 5: // Motor Temperature
                    if (sensor.value > sensor.maxValue) {
                        raiseFault(0xM001); // Motor overtemperature
                    }
                    break;
                default:
                    // Generic sensor fault
                    break;
            }
        }
    }
    
    void confirmFault(uint16_t dtcCode) {
        if (dtcCodes.find(dtcCode) != dtcCodes.end()) {
            auto& dtc = dtcCodes[dtcCode];
            dtc.isActive = true;
            dtc.isConfirmed = true;
            dtc.isPending = false;
            activeFaults.push(dtcCode);
            
            std::cout << "Fault confirmed: " << std::hex << dtcCode << std::dec 
                      << " - " << dtc.description << "\n";
        }
    }
    
    void updateTrendAnalysis(int sensorId) {
        const auto& history = sensorHistory[sensorId];
        if (history.size() < 10) return; // Need minimum data points
        
        // Simple linear regression to find trend
        size_t n = std::min(history.size(), size_t(100)); // Use last 100 points
        double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
        
        for (size_t i = history.size() - n; i < history.size(); i++) {
            double x = i;
            double y = history[i];
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumXX += x * x;
        }
        
        double slope = (n * sumXY - sumX * sumY) / (n * sumXX - sumX * sumX);
        sensorTrends[sensorId] = slope * 100.0; // Convert to percentage change per reading
    }
    
    double estimateTimeToFailure(int sensorId) {
        const auto& sensor = std::find_if(sensors.begin(), sensors.end(),
            [sensorId](const std::unique_ptr<SensorReading>& s) {
                return s->sensorId == sensorId;
            });
        
        if (sensor == sensors.end()) return -1.0;
        
        double currentValue = (*sensor)->value;
        double failureThreshold = (*sensor)->maxValue * 0.95; // 95% of max value
        double trend = sensorTrends[sensorId];
        
        if (trend <= 0) return -1.0; // No degradation trend
        
        // Simple linear extrapolation
        double timeToFailure = (failureThreshold - currentValue) / (trend / 100.0);
        return std::max(0.0, timeToFailure); // Time in readings (convert to hours based on sampling rate)
    }
    
    void displayDTCInfo(const DiagnosticTroubleCode& dtc) {
        std::cout << "  DTC: " << std::hex << dtc.code << std::dec 
                  << " - " << dtc.description << "\n";
        std::cout << "    System: " << getSystemTypeString(dtc.system) << "\n";
        std::cout << "    Severity: " << getSeverityString(dtc.severity) << "\n";
        std::cout << "    Count: " << static_cast<int>(dtc.occurenceCount) << "\n";
    }
    
    std::string getSystemTypeString(SystemType system) const {
        switch (system) {
            case SystemType::BATTERY_MANAGEMENT: return "Battery Management";
            case SystemType::MOTOR_CONTROL: return "Motor Control";
            case SystemType::CHARGING_SYSTEM: return "Charging System";
            case SystemType::THERMAL_MANAGEMENT: return "Thermal Management";
            case SystemType::ADAS: return "ADAS";
            case SystemType::COMMUNICATION: return "Communication";
            case SystemType::POWER_ELECTRONICS: return "Power Electronics";
            case SystemType::HVAC: return "HVAC";
            case SystemType::LIGHTING: return "Lighting";
            case SystemType::SAFETY_SYSTEMS: return "Safety Systems";
            default: return "Unknown";
        }
    }
    
    std::string getSeverityString(FaultSeverity severity) const {
        switch (severity) {
            case FaultSeverity::INFO: return "INFO";
            case FaultSeverity::WARNING: return "WARNING";
            case FaultSeverity::ERROR: return "ERROR";
            case FaultSeverity::CRITICAL: return "CRITICAL";
            case FaultSeverity::EMERGENCY: return "EMERGENCY";
            default: return "UNKNOWN";
        }
    }
};

class OBDInterface {
private:
    std::shared_ptr<FaultDetector> faultDetector;
    std::unordered_map<uint8_t, std::string> pidDescriptions;
    
public:
    OBDInterface(std::shared_ptr<FaultDetector> detector) : faultDetector(detector) {
        initializePIDs();
    }
    
    std::string processOBDRequest(const std::string& request) {
        if (request.length() < 4) return "INVALID REQUEST";
        
        uint8_t mode = std::stoul(request.substr(0, 2), nullptr, 16);
        uint8_t pid = std::stoul(request.substr(2, 2), nullptr, 16);
        
        switch (mode) {
            case 0x01: // Request current powertrain diagnostic data
                return getCurrentData(pid);
            case 0x02: // Request freeze frame data
                return getFreezeFrameData(pid);
            case 0x03: // Request stored diagnostic trouble codes
                return getStoredDTCs();
            case 0x04: // Clear diagnostic trouble codes
                return clearDTCs();
            case 0x07: // Request pending diagnostic trouble codes
                return getPendingDTCs();
            default:
                return "MODE NOT SUPPORTED";
        }
    }
    
private:
    void initializePIDs() {
        pidDescriptions[0x00] = "PIDs supported [01-20]";
        pidDescriptions[0x01] = "Monitor status since DTCs cleared";
        pidDescriptions[0x02] = "Freeze DTC";
        pidDescriptions[0x03] = "Fuel system status";
        pidDescriptions[0x04] = "Calculated engine load";
        pidDescriptions[0x05] = "Engine coolant temperature";
        pidDescriptions[0x0C] = "Engine speed";
        pidDescriptions[0x0D] = "Vehicle speed";
        pidDescriptions[0x0F] = "Intake air temperature";
        // Add more PIDs as needed for EV-specific parameters
    }
    
    std::string getCurrentData(uint8_t pid) {
        // Return current diagnostic data based on PID
        switch (pid) {
            case 0x05: // Engine coolant temperature (repurposed for battery temp)
                return "41 05 " + intToHex(static_cast<uint8_t>(25 + 40)); // Battery temp + 40
            case 0x0D: // Vehicle speed
                return "41 0D " + intToHex(static_cast<uint8_t>(50)); // 50 km/h
            default:
                return "NO DATA";
        }
    }
    
    std::string getFreezeFrameData(uint8_t pid) {
        return "NO DATA"; // Simplified implementation
    }
    
    std::string getStoredDTCs() {
        auto activeFaults = faultDetector->getActiveFaults();
        if (activeFaults.empty()) {
            return "NO DTCS";
        }
        
        std::string response = "43 ";
        for (const auto& fault : activeFaults) {
            response += intToHex(static_cast<uint8_t>(fault.code >> 8));
            response += intToHex(static_cast<uint8_t>(fault.code & 0xFF));
            response += " ";
        }
        
        return response;
    }
    
    std::string clearDTCs() {
        // Clear all DTCs (simplified)
        return "44"; // Positive response
    }
    
    std::string getPendingDTCs() {
        auto pendingFaults = faultDetector->getPendingFaults();
        if (pendingFaults.empty()) {
            return "NO DTCS";
        }
        
        std::string response = "47 ";
        for (const auto& fault : pendingFaults) {
            response += intToHex(static_cast<uint8_t>(fault.code >> 8));
            response += intToHex(static_cast<uint8_t>(fault.code & 0xFF));
            response += " ";
        }
        
        return response;
    }
    
    std::string intToHex(uint8_t value) {
        std::stringstream ss;
        ss << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << static_cast<int>(value);
        return ss.str();
    }
};

class VehicleDiagnosticsSystem {
private:
    std::shared_ptr<FaultDetector> faultDetector;
    std::unique_ptr<OBDInterface> obdInterface;
    
    bool diagnosticsEnabled;
    std::chrono::steady_clock::time_point lastScan;
    
public:
    VehicleDiagnosticsSystem() : diagnosticsEnabled(true) {
        faultDetector = std::make_shared<FaultDetector>();
        obdInterface = std::make_unique<OBDInterface>(faultDetector);
        lastScan = std::chrono::steady_clock::now();
        
        std::cout << "Vehicle Diagnostics System initialized\n";
    }
    
    void enable() { diagnosticsEnabled = true; }
    void disable() { diagnosticsEnabled = false; }
    
    void updateSensorData(const std::unordered_map<int, double>& sensorData) {
        if (!diagnosticsEnabled) return;
        
        for (const auto& data : sensorData) {
            faultDetector->updateSensorReading(data.first, data.second);
        }
    }
    
    void performRoutineDiagnostics() {
        if (!diagnosticsEnabled) return;
        
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - lastScan);
        
        if (duration.count() >= 10) { // Perform scan every 10 seconds
            faultDetector->performDiagnosticScan();
            lastScan = now;
        }
    }
    
    void generateMaintenanceReport() {
        faultDetector->generateDiagnosticReport();
        faultDetector->performPredictiveMaintenance();
    }
    
    void displaySystemStatus() {
        faultDetector->displaySystemStatus();
    }
    
    std::string processOBDCommand(const std::string& command) {
        return obdInterface->processOBDRequest(command);
    }
    
    void raiseFault(uint16_t dtcCode) {
        faultDetector->raiseFault(dtcCode);
    }
    
    void clearFault(uint16_t dtcCode) {
        faultDetector->clearFault(dtcCode);
    }
    
    bool hasActiveFaults() const {
        return !faultDetector->getActiveFaults().empty();
    }
    
    bool hasCriticalFaults() const {
        auto activeFaults = faultDetector->getActiveFaults();
        for (const auto& fault : activeFaults) {
            if (fault.severity >= FaultSeverity::CRITICAL) {
                return true;
            }
        }
        return false;
    }
};

#endif // VEHICLE_DIAGNOSTICS_H