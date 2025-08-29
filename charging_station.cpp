#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <iomanip>

enum class ChargingType {
    SLOW = 3,      // 3kW
    FAST = 22,     // 22kW  
    RAPID = 50,    // 50kW
    ULTRA_RAPID = 150  // 150kW
};

class ChargingStation {
private:
    std::string stationId;
    ChargingType chargingType;
    bool isOccupied;
    bool isOperational;
    double powerOutput;
    double totalEnergyDelivered;
    
public:
    ChargingStation(std::string id, ChargingType type) : 
        stationId(id), chargingType(type), isOccupied(false), 
        isOperational(true), totalEnergyDelivered(0.0) {
        
        powerOutput = static_cast<double>(type);
    }
    
    bool connect() {
        if (!isOperational) {
            std::cout << "Station " << stationId << " is out of order!\n";
            return false;
        }
        
        if (isOccupied) {
            std::cout << "Station " << stationId << " is currently occupied!\n";
            return false;
        }
        
        isOccupied = true;
        std::cout << "Connected to charging station " << stationId << "\n";
        std::cout << "Charging power: " << powerOutput << " kW\n";
        return true;
    }
    
    void disconnect() {
        isOccupied = false;
        std::cout << "Disconnected from station " << stationId << "\n";
    }
    
    double calculateChargingTime(double batteryCapacity, double currentLevel, double targetLevel) {
        if (targetLevel <= currentLevel) {
            std::cout << "Target level must be higher than current level!\n";
            return 0.0;
        }
        
        double energyNeeded = batteryCapacity * (targetLevel - currentLevel) / 100.0;
        double chargingTime = energyNeeded / powerOutput;
        
        return chargingTime;
    }
    
    void startCharging(double batteryCapacity, double& currentLevel, double targetLevel) {
        if (!isOccupied) {
            std::cout << "Please connect to the charging station first!\n";
            return;
        }
        
        double chargingTime = calculateChargingTime(batteryCapacity, currentLevel, targetLevel);
        std::cout << std::fixed << std::setprecision(1);
        std::cout << "Estimated charging time: " << chargingTime << " hours\n";
        std::cout << "Starting charging process...\n\n";
        
        double energyToAdd = batteryCapacity * (targetLevel - currentLevel) / 100.0;
        double energyAdded = 0.0;
        
        auto startTime = std::chrono::steady_clock::now();
        
        while (currentLevel < targetLevel && isOccupied) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            
            auto currentTime = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime);
            
            double hoursElapsed = elapsed.count() / 1000.0 / 3600.0 * 1000; // Accelerated for demo
            energyAdded = std::min(powerOutput * hoursElapsed, energyToAdd);
            currentLevel = std::min(targetLevel, currentLevel + (energyAdded / batteryCapacity) * 100.0);
            
            std::cout << "\rCharging: " << std::setprecision(1) << currentLevel 
                      << "% | Power: " << powerOutput << " kW | Energy added: " 
                      << std::setprecision(2) << energyAdded << " kWh" << std::flush;
            
            if (currentLevel >= 99.9) break;
        }
        
        totalEnergyDelivered += energyAdded;
        std::cout << "\nCharging completed!\n";
        std::cout << "Final battery level: " << std::setprecision(1) << currentLevel << "%\n";
    }
    
    void displayStationInfo() {
        std::cout << "\n=== Charging Station Info ===\n";
        std::cout << "Station ID: " << stationId << "\n";
        std::cout << "Type: ";
        
        switch (chargingType) {
            case ChargingType::SLOW:
                std::cout << "Slow Charging (3kW)\n";
                break;
            case ChargingType::FAST:
                std::cout << "Fast Charging (22kW)\n";
                break;
            case ChargingType::RAPID:
                std::cout << "Rapid Charging (50kW)\n";
                break;
            case ChargingType::ULTRA_RAPID:
                std::cout << "Ultra Rapid Charging (150kW)\n";
                break;
        }
        
        std::cout << "Status: " << (isOperational ? "Operational" : "Out of Order") << "\n";
        std::cout << "Occupied: " << (isOccupied ? "Yes" : "No") << "\n";
        std::cout << "Total Energy Delivered: " << std::setprecision(2) 
                  << totalEnergyDelivered << " kWh\n";
        std::cout << "============================\n\n";
    }
    
    bool getOccupiedStatus() const { return isOccupied; }
    std::string getStationId() const { return stationId; }
};

class ChargingNetwork {
private:
    std::vector<ChargingStation> stations;
    
public:
    void addStation(const ChargingStation& station) {
        stations.push_back(station);
    }
    
    ChargingStation* findAvailableStation(ChargingType preferredType = ChargingType::FAST) {
        for (auto& station : stations) {
            if (!station.getOccupiedStatus()) {
                return &station;
            }
        }
        return nullptr;
    }
    
    void displayNetworkStatus() {
        std::cout << "\n=== Charging Network Status ===\n";
        std::cout << "Total Stations: " << stations.size() << "\n";
        
        int availableStations = 0;
        for (const auto& station : stations) {
            if (!station.getOccupiedStatus()) {
                availableStations++;
            }
        }
        
        std::cout << "Available Stations: " << availableStations << "\n";
        std::cout << "Occupied Stations: " << stations.size() - availableStations << "\n";
        std::cout << "==============================\n\n";
        
        for (auto& station : stations) {
            station.displayStationInfo();
        }
    }
};

int main() {
    std::cout << "EV Charging Station Simulator\n";
    std::cout << "=============================\n\n";
    
    ChargingNetwork network;
    network.addStation(ChargingStation("ST001", ChargingType::SLOW));
    network.addStation(ChargingStation("ST002", ChargingType::FAST));
    network.addStation(ChargingStation("ST003", ChargingType::RAPID));
    network.addStation(ChargingStation("ST004", ChargingType::ULTRA_RAPID));
    
    network.displayNetworkStatus();
    
    ChargingStation* availableStation = network.findAvailableStation();
    
    if (availableStation) {
        std::cout << "Found available station: " << availableStation->getStationId() << "\n";
        
        if (availableStation->connect()) {
            double batteryLevel = 25.0; // 25% battery
            double batteryCapacity = 75.0; // 75 kWh battery
            double targetLevel = 80.0; // Charge to 80%
            
            availableStation->startCharging(batteryCapacity, batteryLevel, targetLevel);
            availableStation->disconnect();
        }
    }
    
    network.displayNetworkStatus();
    
    return 0;
}