#ifndef BATTERY_MANAGEMENT_H
#define BATTERY_MANAGEMENT_H

#include <iostream>
#include <vector>
#include <algorithm>

class BatteryCell {
private:
    double voltage;
    double capacity;
    double temperature;
    bool isHealthy;
    
public:
    BatteryCell(double v = 3.7, double c = 100.0) : 
        voltage(v), capacity(c), temperature(25.0), isHealthy(true) {}
    
    void updateVoltage(double v) { voltage = v; }
    void updateTemperature(double t) { 
        temperature = t;
        if (t > 60.0 || t < -20.0) {
            isHealthy = false;
            std::cout << "Warning: Battery cell temperature critical!\n";
        }
    }
    
    double getVoltage() const { return voltage; }
    double getCapacity() const { return capacity; }
    double getTemperature() const { return temperature; }
    bool getHealthStatus() const { return isHealthy; }
    
    void degradeBattery(double factor = 0.99) {
        capacity *= factor;
        if (capacity < 80.0) {
            std::cout << "Battery degradation detected. Capacity: " << capacity << "%\n";
        }
    }
};

class BatteryManagementSystem {
private:
    std::vector<BatteryCell> cells;
    double totalCapacity;
    double currentCharge;
    bool balancingActive;
    
public:
    BatteryManagementSystem(int numCells = 8) : 
        totalCapacity(0.0), currentCharge(100.0), balancingActive(false) {
        
        for (int i = 0; i < numCells; i++) {
            cells.push_back(BatteryCell());
            totalCapacity += 100.0;
        }
    }
    
    void monitorCells() {
        std::cout << "\n=== Battery Cell Monitoring ===\n";
        for (size_t i = 0; i < cells.size(); i++) {
            std::cout << "Cell " << i + 1 << ": ";
            std::cout << "Voltage: " << cells[i].getVoltage() << "V, ";
            std::cout << "Temp: " << cells[i].getTemperature() << "°C, ";
            std::cout << "Health: " << (cells[i].getHealthStatus() ? "OK" : "CRITICAL") << "\n";
        }
        std::cout << "==============================\n\n";
    }
    
    void balanceCells() {
        balancingActive = true;
        std::cout << "Cell balancing started...\n";
        
        double avgVoltage = 0.0;
        for (const auto& cell : cells) {
            avgVoltage += cell.getVoltage();
        }
        avgVoltage /= cells.size();
        
        for (auto& cell : cells) {
            if (abs(cell.getVoltage() - avgVoltage) > 0.1) {
                std::cout << "Balancing cell with voltage: " << cell.getVoltage() << "V\n";
                cell.updateVoltage(avgVoltage);
            }
        }
        
        balancingActive = false;
        std::cout << "Cell balancing completed.\n";
    }
    
    double calculateRemainingRange(double efficiency = 5.0) {
        return (currentCharge / 100.0) * totalCapacity * efficiency;
    }
    
    void simulateUsage(double powerDraw) {
        if (currentCharge > 0) {
            currentCharge -= powerDraw;
            if (currentCharge < 0) currentCharge = 0;
            
            for (auto& cell : cells) {
                cell.degradeBattery(0.9999);
            }
        }
    }
    
    void displayBatteryInfo() {
        std::cout << "\n=== Battery Management System ===\n";
        std::cout << "Total Capacity: " << totalCapacity << " kWh\n";
        std::cout << "Current Charge: " << currentCharge << "%\n";
        std::cout << "Estimated Range: " << calculateRemainingRange() << " km\n";
        std::cout << "Cell Balancing: " << (balancingActive ? "ACTIVE" : "INACTIVE") << "\n";
        std::cout << "Number of Cells: " << cells.size() << "\n";
        std::cout << "================================\n\n";
    }
    
    bool isLowBattery() const { return currentCharge < 20.0; }
    bool isCriticalBattery() const { return currentCharge < 5.0; }
};

#endif