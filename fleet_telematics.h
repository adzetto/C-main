/**
 * @file fleet_telematics.h
 * @author adzetto
 * @brief Fleet Telematics System for EV Fleet Management
 * @version 1.0
 * @date 2025-08-31
 *
 * @copyright Copyright (c) 2025
 *
 * @details This module provides a comprehensive system for managing a fleet of electric vehicles.
 *          It includes capabilities for vehicle tracking, health monitoring, remote diagnostics,
 *          and communication with a central fleet management server.
 */

#ifndef FLEET_TELEMATICS_H
#define FLEET_TELEMATICS_H

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <map>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

namespace fleet_telematics {

/**
 * @brief Represents the GPS location of a vehicle.
 */
struct GPSLocation {
    double latitude;
    double longitude;
    double altitude;
    double speed_kph;
};

/**
 * @brief Represents a snapshot of vehicle data.
 */
struct VehicleDataSnapshot {
    std::string vehicleId;
    std::chrono::system_clock::time_point timestamp;
    GPSLocation location;
    double battery_soc;
    double motor_temp_c;
    double system_health_score; // 0-100
    std::vector<std::string> active_dtcs; // Diagnostic Trouble Codes
};

/**
 * @brief Interface for a communication client that sends data to a remote server.
 */
class ITelematicsClient {
public:
    virtual ~ITelematicsClient() = default;
    virtual bool connect(const std::string& server_address) = 0;
    virtual bool sendData(const VehicleDataSnapshot& data) = 0;
    virtual void disconnect() = 0;
};

/**
 * @brief A mock implementation of the telematics client for demo purposes.
 */
class MockTelematicsClient : public ITelematicsClient {
public:
    bool connect(const std::string& server_address) override {
        std::cout << "[TelematicsClient] Connecting to " << server_address << "...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        connected = true;
        std::cout << "[TelematicsClient] Connection successful.\n";
        return true;
    }

    bool sendData(const VehicleDataSnapshot& data) override {
        if (!connected) {
            std::cout << "[TelematicsClient] Error: Not connected.\n";
            return false;
        }
        std::cout << "[TelematicsClient] Sending data for vehicle " << data.vehicleId << ":\n";
        std::cout << "  Timestamp: " << data.timestamp.time_since_epoch().count() << "\n";
        std::cout << "  Location: " << data.location.latitude << ", " << data.location.longitude << "\n";
        std::cout << "  SoC: " << data.battery_soc << "%\n";
        std::cout << "  Motor Temp: " << data.motor_temp_c << " C\n";
        std::cout << "  Health Score: " << data.system_health_score << "/100\n";
        // Simulate network latency
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return true;
    }

    void disconnect() override {
        if (connected) {
            std::cout << "[TelematicsClient] Disconnecting...\n";
            connected = false;
        }
    }

private:
    bool connected = false;
};

/**
 * @brief Manages the telematics for a single vehicle.
 */
class VehicleTelematicsUnit {
public:
    VehicleTelematicsUnit(const std::string& vehicleId, std::unique_ptr<ITelematicsClient> client)
        : vehicleId(vehicleId), comm_client(std::move(client)), reporting_active(false) {}

    ~VehicleTelematicsUnit() {
        stop();
    }

    /**
     * @brief Starts the telematics unit and begins reporting data.
     * @param server_address The address of the fleet management server.
     * @param reporting_interval_s The interval in seconds at which to send data.
     */
    void start(const std::string& server_address, int reporting_interval_s) {
        if (reporting_active) {
            std::cout << "[VTM] Already running.\n";
            return;
        }

        if (!comm_client->connect(server_address)) {
            std::cout << "[VTM] Failed to connect to server. Aborting start.\n";
            return;
        }

        reporting_active = true;
        reporting_thread = std::thread(&VehicleTelematicsUnit::reporting_loop, this, reporting_interval_s);
        std::cout << "[VTM] Vehicle Telematics Unit for " << vehicleId << " started.\n";
    }

    /**
     * @brief Stops the telematics unit.
     */
    void stop() {
        if (reporting_active) {
            reporting_active = false;
            if (reporting_thread.joinable()) {
                reporting_thread.join();
            }
            comm_client->disconnect();
            std::cout << "[VTM] Vehicle Telematics Unit for " << vehicleId << " stopped.\n";
        }
    }

    /**
     * @brief Updates the vehicle's current data.
     * @param data The latest data snapshot.
     */
    void updateVehicleData(const VehicleDataSnapshot& data) {
        std::lock_guard<std::mutex> lock(data_mutex);
        latest_data = data;
    }

private:
    void reporting_loop(int interval_s) {
        while (reporting_active) {
            std::this_thread::sleep_for(std::chrono::seconds(interval_s));
            if (reporting_active) {
                VehicleDataSnapshot data_to_send;
                {
                    std::lock_guard<std::mutex> lock(data_mutex);
                    data_to_send = latest_data;
                    data_to_send.timestamp = std::chrono::system_clock::now();
                }
                comm_client->sendData(data_to_send);
            }
        }
    }

    std::string vehicleId;
    std::unique_ptr<ITelematicsClient> comm_client;
    std::thread reporting_thread;
    std::atomic<bool> reporting_active;
    std::mutex data_mutex;
    VehicleDataSnapshot latest_data;
};

/**
 * @brief Manages a fleet of vehicles and their telematics units.
 */
class FleetManager {
public:
    /**
     * @brief Adds a new vehicle to the fleet.
     * @param vehicleId The unique identifier for the vehicle.
     */
    void addVehicle(const std::string& vehicleId) {
        if (fleet.find(vehicleId) != fleet.end()) {
            std::cout << "[FleetManager] Vehicle " << vehicleId << " already exists.\n";
            return;
        }
        auto client = std::make_unique<MockTelematicsClient>();
        auto vtm = std::make_shared<VehicleTelematicsUnit>(vehicleId, std::move(client));
        fleet[vehicleId] = vtm;
        std::cout << "[FleetManager] Vehicle " << vehicleId << " added to the fleet.\n";
    }

    /**
     * @brief Starts telematics for all vehicles in the fleet.
     * @param server_address The fleet management server address.
     * @param reporting_interval_s The data reporting interval.
     */
    void startAllTelematics(const std::string& server_address, int reporting_interval_s) {
        std::cout << "[FleetManager] Starting telematics for all vehicles...\n";
        for (auto const& [id, vtm] : fleet) {
            vtm->start(server_address, reporting_interval_s);
        }
    }

    /**
     * @brief Stops all telematics units.
     */
    void stopAllTelematics() {
        std::cout << "[FleetManager] Stopping all telematics...\n";
        for (auto const& [id, vtm] : fleet) {
            vtm->stop();
        }
    }

    /**
     * @brief Gets a pointer to a specific vehicle's telematics unit.
     * @param vehicleId The ID of the vehicle.
     * @return A shared pointer to the VehicleTelematicsUnit, or nullptr if not found.
     */
    std::shared_ptr<VehicleTelematicsUnit> getVehicle(const std::string& vehicleId) {
        auto it = fleet.find(vehicleId);
        if (it != fleet.end()) {
            return it->second;
        }
        return nullptr;
    }

private:
    std::map<std::string, std::shared_ptr<VehicleTelematicsUnit>> fleet;
};

} // namespace fleet_telematics

#endif // FLEET_TELEMATICS_H