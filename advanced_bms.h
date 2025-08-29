/**
 * Advanced Battery Management System (BMS) for Electric Vehicles
 * Author: adzetto
 * Features: Cell balancing, thermal management, fault detection, SOC/SOH estimation
 */

#ifndef ADVANCED_BMS_H
#define ADVANCED_BMS_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>
#include <memory>

enum class CellState {
    HEALTHY,
    DEGRADED,
    CRITICAL,
    FAULT
};

enum class BalancingMethod {
    PASSIVE,
    ACTIVE,
    HYBRID
};

enum class BMSFault {
    NONE,
    OVERVOLTAGE,
    UNDERVOLTAGE,
    OVERTEMP,
    UNDERTEMP,
    OVERCURRENT,
    CELL_IMBALANCE,
    COMMUNICATION_ERROR
};

struct CellData {
    double voltage;
    double current;
    double temperature;
    double capacity;
    double resistance;
    double soc;  // State of Charge
    double soh;  // State of Health
    CellState state;
    uint32_t cycleCount;
    std::chrono::steady_clock::time_point lastUpdate;
    
    CellData() : voltage(3.7), current(0.0), temperature(25.0), 
                 capacity(100.0), resistance(0.01), soc(100.0), 
                 soh(100.0), state(CellState::HEALTHY), cycleCount(0),
                 lastUpdate(std::chrono::steady_clock::now()) {}
};

class AdvancedBatteryCell {
private:
    int cellId;
    CellData data;
    std::vector<double> voltageHistory;
    std::vector<double> temperatureHistory;
    bool balancingActive;
    double balancingCurrent;
    
public:
    AdvancedBatteryCell(int id) : cellId(id), balancingActive(false), balancingCurrent(0.0) {
        voltageHistory.reserve(1000);
        temperatureHistory.reserve(1000);
    }
    
    void updateVoltage(double voltage) {
        data.voltage = voltage;
        voltageHistory.push_back(voltage);
        if (voltageHistory.size() > 1000) {
            voltageHistory.erase(voltageHistory.begin());
        }
        updateSOC();
        checkFaults();
        data.lastUpdate = std::chrono::steady_clock::now();
    }
    
    void updateTemperature(double temp) {
        data.temperature = temp;
        temperatureHistory.push_back(temp);
        if (temperatureHistory.size() > 1000) {
            temperatureHistory.erase(temperatureHistory.begin());
        }
        updateSOH();
        checkFaults();
    }
    
    void updateCurrent(double current) {
        data.current = current;
        if (abs(current) > 0.1) {
            updateCycleCount();
        }
    }
    
    double calculateSOC() const {
        // Voltage-based SOC estimation with temperature compensation
        double tempCompensation = 1.0 - (data.temperature - 25.0) * 0.001;
        double normalizedVoltage = (data.voltage * tempCompensation - 3.0) / (4.2 - 3.0);
        return std::max(0.0, std::min(100.0, normalizedVoltage * 100.0));
    }
    
    double calculateSOH() const {
        // SOH based on capacity degradation and cycle count
        double cycleDegradation = 1.0 - (data.cycleCount * 0.00001);
        double tempDegradation = calculateTemperatureDegradation();
        return std::max(50.0, std::min(100.0, cycleDegradation * tempDegradation * 100.0));
    }
    
    void startBalancing(BalancingMethod method, double targetVoltage) {
        balancingActive = true;
        double voltageDiff = data.voltage - targetVoltage;
        
        switch (method) {
            case BalancingMethod::PASSIVE:
                balancingCurrent = std::min(0.1, abs(voltageDiff) * 0.5);
                break;
            case BalancingMethod::ACTIVE:
                balancingCurrent = std::min(0.3, abs(voltageDiff) * 1.0);
                break;
            case BalancingMethod::HYBRID:
                balancingCurrent = std::min(0.2, abs(voltageDiff) * 0.8);
                break;
        }
        
        std::cout << "Cell " << cellId << " balancing started: " 
                  << balancingCurrent << "A\n";
    }
    
    void stopBalancing() {
        balancingActive = false;
        balancingCurrent = 0.0;
    }
    
    BMSFault checkFaults() const {
        if (data.voltage > 4.3) return BMSFault::OVERVOLTAGE;
        if (data.voltage < 2.5) return BMSFault::UNDERVOLTAGE;
        if (data.temperature > 60.0) return BMSFault::OVERTEMP;
        if (data.temperature < -20.0) return BMSFault::UNDERTEMP;
        if (abs(data.current) > 100.0) return BMSFault::OVERCURRENT;
        return BMSFault::NONE;
    }
    
    // Getters
    int getId() const { return cellId; }
    const CellData& getData() const { return data; }
    bool isBalancing() const { return balancingActive; }
    double getBalancingCurrent() const { return balancingCurrent; }
    
private:
    void updateSOC() {
        data.soc = calculateSOC();
    }
    
    void updateSOH() {
        data.soh = calculateSOH();
    }
    
    void updateCycleCount() {
        static double lastSOC = data.soc;
        if (abs(data.soc - lastSOC) > 80.0) {
            data.cycleCount++;
            lastSOC = data.soc;
        }
    }
    
    double calculateTemperatureDegradation() const {
        double avgTemp = 0.0;
        if (!temperatureHistory.empty()) {
            for (double temp : temperatureHistory) {
                avgTemp += temp;
            }
            avgTemp /= temperatureHistory.size();
        } else {
            avgTemp = data.temperature;
        }
        
        // Optimal temperature range: 15-35°C
        if (avgTemp >= 15.0 && avgTemp <= 35.0) {
            return 1.0;
        } else if (avgTemp > 35.0) {
            return std::max(0.7, 1.0 - (avgTemp - 35.0) * 0.01);
        } else {
            return std::max(0.8, 1.0 - (35.0 - avgTemp) * 0.005);
        }
    }
};

class AdvancedBMS {
private:
    std::vector<std::unique_ptr<AdvancedBatteryCell>> cells;
    BalancingMethod balancingMethod;
    bool balancingEnabled;
    double packVoltage;
    double packCurrent;
    double packTemperature;
    double packSOC;
    double packSOH;
    double totalCapacity;
    std::vector<BMSFault> activeFaults;
    bool emergencyShutdown;
    
public:
    AdvancedBMS(int numCells = 96, double cellCapacity = 50.0) 
        : balancingMethod(BalancingMethod::HYBRID), balancingEnabled(true),
          packVoltage(0.0), packCurrent(0.0), packTemperature(0.0),
          packSOC(0.0), packSOH(0.0), totalCapacity(numCells * cellCapacity),
          emergencyShutdown(false) {
        
        for (int i = 0; i < numCells; i++) {
            cells.push_back(std::make_unique<AdvancedBatteryCell>(i + 1));
        }
        
        std::cout << "Advanced BMS initialized with " << numCells 
                  << " cells, total capacity: " << totalCapacity << " Ah\n";
    }
    
    void updateAllCells() {
        // Simulate cell data updates
        packVoltage = 0.0;
        packTemperature = 0.0;
        double totalSOC = 0.0;
        double totalSOH = 0.0;
        
        for (auto& cell : cells) {
            // Simulate slight variations in cell parameters
            double baseVoltage = 3.7 + (rand() % 100 - 50) * 0.001;
            double baseTemp = 25.0 + (rand() % 40 - 20) * 0.1;
            double baseCurrent = packCurrent / cells.size();
            
            cell->updateVoltage(baseVoltage);
            cell->updateTemperature(baseTemp);
            cell->updateCurrent(baseCurrent);
            
            packVoltage += cell->getData().voltage;
            packTemperature += cell->getData().temperature;
            totalSOC += cell->getData().soc;
            totalSOH += cell->getData().soh;
        }
        
        packTemperature /= cells.size();
        packSOC = totalSOC / cells.size();
        packSOH = totalSOH / cells.size();
    }
    
    void performCellBalancing() {
        if (!balancingEnabled) return;
        
        // Find voltage extremes
        double minVoltage = 5.0, maxVoltage = 0.0;
        for (const auto& cell : cells) {
            double voltage = cell->getData().voltage;
            minVoltage = std::min(minVoltage, voltage);
            maxVoltage = std::max(maxVoltage, voltage);
        }
        
        double voltageImbalance = maxVoltage - minVoltage;
        
        if (voltageImbalance > 0.05) { // 50mV threshold
            std::cout << "Cell balancing required. Imbalance: " 
                      << voltageImbalance * 1000 << " mV\n";
            
            double targetVoltage = (minVoltage + maxVoltage) / 2.0;
            
            for (auto& cell : cells) {
                double cellVoltage = cell->getData().voltage;
                if (abs(cellVoltage - targetVoltage) > 0.01) {
                    cell->startBalancing(balancingMethod, targetVoltage);
                }
            }
            
            // Simulate balancing time
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            for (auto& cell : cells) {
                cell->stopBalancing();
            }
        }
    }
    
    void checkSystemFaults() {
        activeFaults.clear();
        
        for (const auto& cell : cells) {
            BMSFault fault = cell->checkFaults();
            if (fault != BMSFault::NONE) {
                activeFaults.push_back(fault);
            }
        }
        
        // Check pack-level faults
        if (packVoltage > 420.0) activeFaults.push_back(BMSFault::OVERVOLTAGE);
        if (packVoltage < 240.0) activeFaults.push_back(BMSFault::UNDERVOLTAGE);
        if (packTemperature > 55.0) activeFaults.push_back(BMSFault::OVERTEMP);
        if (abs(packCurrent) > 400.0) activeFaults.push_back(BMSFault::OVERCURRENT);
        
        // Emergency shutdown conditions
        if (!activeFaults.empty()) {
            for (BMSFault fault : activeFaults) {
                if (fault == BMSFault::OVERVOLTAGE || fault == BMSFault::OVERTEMP ||
                    fault == BMSFault::OVERCURRENT) {
                    emergencyShutdown = true;
                    std::cout << "EMERGENCY SHUTDOWN TRIGGERED!\n";
                    break;
                }
            }
        }
    }
    
    void displayDetailedStatus() {
        std::cout << "\n=== Advanced BMS Status ===\n";
        std::cout << "Pack Voltage: " << packVoltage << " V\n";
        std::cout << "Pack Current: " << packCurrent << " A\n";
        std::cout << "Pack Temperature: " << packTemperature << " °C\n";
        std::cout << "Pack SOC: " << packSOC << " %\n";
        std::cout << "Pack SOH: " << packSOH << " %\n";
        std::cout << "Total Capacity: " << totalCapacity << " Ah\n";
        std::cout << "Balancing: " << (balancingEnabled ? "ENABLED" : "DISABLED") << "\n";
        
        if (!activeFaults.empty()) {
            std::cout << "Active Faults: ";
            for (BMSFault fault : activeFaults) {
                std::cout << getFaultString(fault) << " ";
            }
            std::cout << "\n";
        }
        
        std::cout << "Emergency Shutdown: " << (emergencyShutdown ? "YES" : "NO") << "\n";
        std::cout << "========================\n\n";
    }
    
    void displayCellDetails(int startCell = 0, int endCell = -1) {
        if (endCell == -1 || endCell >= static_cast<int>(cells.size())) {
            endCell = cells.size() - 1;
        }
        
        std::cout << "\n=== Cell Details ===\n";
        std::cout << "Cell | Voltage | Temp | SOC | SOH | Cycles | State\n";
        std::cout << "-----|---------|------|-----|-----|--------|-------\n";
        
        for (int i = startCell; i <= endCell && i < static_cast<int>(cells.size()); i++) {
            const CellData& data = cells[i]->getData();
            std::cout << std::setw(4) << (i + 1) << " | "
                      << std::setw(7) << std::fixed << std::setprecision(3) << data.voltage << " | "
                      << std::setw(4) << std::fixed << std::setprecision(1) << data.temperature << " | "
                      << std::setw(3) << std::fixed << std::setprecision(0) << data.soc << " | "
                      << std::setw(3) << std::fixed << std::setprecision(0) << data.soh << " | "
                      << std::setw(6) << data.cycleCount << " | "
                      << getCellStateString(data.state) << "\n";
        }
        std::cout << "===================\n\n";
    }
    
    // Setters
    void setPackCurrent(double current) { packCurrent = current; }
    void enableBalancing(bool enable) { balancingEnabled = enable; }
    void setBalancingMethod(BalancingMethod method) { balancingMethod = method; }
    
    // Getters
    double getPackVoltage() const { return packVoltage; }
    double getPackSOC() const { return packSOC; }
    double getPackSOH() const { return packSOH; }
    bool isEmergencyShutdown() const { return emergencyShutdown; }
    const std::vector<BMSFault>& getActiveFaults() const { return activeFaults; }
    
private:
    std::string getFaultString(BMSFault fault) const {
        switch (fault) {
            case BMSFault::NONE: return "NONE";
            case BMSFault::OVERVOLTAGE: return "OVERVOLTAGE";
            case BMSFault::UNDERVOLTAGE: return "UNDERVOLTAGE";
            case BMSFault::OVERTEMP: return "OVERTEMP";
            case BMSFault::UNDERTEMP: return "UNDERTEMP";
            case BMSFault::OVERCURRENT: return "OVERCURRENT";
            case BMSFault::CELL_IMBALANCE: return "CELL_IMBALANCE";
            case BMSFault::COMMUNICATION_ERROR: return "COMM_ERROR";
            default: return "UNKNOWN";
        }
    }
    
    std::string getCellStateString(CellState state) const {
        switch (state) {
            case CellState::HEALTHY: return "HEALTHY";
            case CellState::DEGRADED: return "DEGRADED";
            case CellState::CRITICAL: return "CRITICAL";
            case CellState::FAULT: return "FAULT";
            default: return "UNKNOWN";
        }
    }
};

#endif // ADVANCED_BMS_H