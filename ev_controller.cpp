#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

class ElectricVehicle {
private:
    std::string model;
    double batteryLevel;
    double maxSpeed;
    double currentSpeed;
    bool isCharging;
    bool isMoving;
    
public:
    ElectricVehicle(std::string m, double maxSpd) : 
        model(m), batteryLevel(100.0), maxSpeed(maxSpd), 
        currentSpeed(0.0), isCharging(false), isMoving(false) {}
    
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
        std::cout << "========================\n\n";
    }
    
    double getBatteryLevel() const { return batteryLevel; }
    double getCurrentSpeed() const { return currentSpeed; }
    bool getMovingStatus() const { return isMoving; }
};

int main() {
    ElectricVehicle tesla("Tesla Model 3", 250.0);
    
    std::cout << "Electric Vehicle Controller\n";
    std::cout << "==========================\n\n";
    
    tesla.displayStatus();
    
    tesla.startEngine();
    tesla.accelerate(80);
    tesla.displayStatus();
    
    tesla.accelerate(150);
    tesla.displayStatus();
    
    tesla.brake();
    tesla.brake();
    tesla.displayStatus();
    
    tesla.stopEngine();
    
    if (tesla.getBatteryLevel() < 50.0) {
        tesla.startCharging();
        tesla.displayStatus();
    }
    
    return 0;
}