/**
 * @file ev_communication_protocols.h
 * @author adzetto
 * @brief Advanced EV Communication Protocols and Network Stack
 * @version 1.0
 * @date 2025-09-01
 *
 * @copyright Copyright (c) 2025
 *
 * @details This module provides comprehensive communication protocols for electric vehicles,
 *          including V2V (Vehicle-to-Vehicle), V2I (Vehicle-to-Infrastructure), 
 *          V2G (Vehicle-to-Grid), cellular, WiFi, and satellite communications.
 */

#ifndef EV_COMMUNICATION_PROTOCOLS_H
#define EV_COMMUNICATION_PROTOCOLS_H

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <unordered_map>
#include <queue>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <random>
#include <numeric>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <condition_variable>

namespace ev_communication {

// Forward declarations
class V2VCommunication;
class V2ICommunication;
class V2GCommunication;
class CellularModem;
class WiFiController;
class SatelliteComm;
class MessageRouter;
class SecurityManager;

/**
 * @brief Communication protocol types
 */
enum class ProtocolType {
    DSRC_802_11P, LTE_V2X, C_V2X_5G, IEEE_802_11AC, BLUETOOTH_5_0,
    ZIGBEE, LORA_WAN, SATELLITE_L_BAND, CAN_BUS, LIN_BUS, FLEXRAY
};

/**
 * @brief Message priority levels
 */
enum class MessagePriority {
    CRITICAL = 0,     // Safety-critical messages
    HIGH = 1,         // Performance-critical messages  
    MEDIUM = 2,       // Normal operational messages
    LOW = 3,          // Background/diagnostic messages
    BEST_EFFORT = 4   // Non-critical messages
};

/**
 * @brief Communication security levels
 */
enum class SecurityLevel {
    NONE, BASIC_ENCRYPTION, ADVANCED_ENCRYPTION, 
    DIGITAL_SIGNATURE, CERTIFICATE_BASED, QUANTUM_SAFE
};

/**
 * @brief Network interface types
 */
enum class InterfaceType {
    ETHERNET, WIFI, CELLULAR_4G, CELLULAR_5G, DSRC, BLUETOOTH,
    SATELLITE, CAN, LIN, FLEXRAY, USB, SERIAL
};

/**
 * @brief Vehicle communication states
 */
enum class CommState {
    OFFLINE, CONNECTING, CONNECTED, AUTHENTICATING, 
    AUTHENTICATED, TRANSMITTING, RECEIVING, ERROR, TIMEOUT
};

/**
 * @brief Geographic position for location-based services
 */
struct GeoPosition {
    double latitude;
    double longitude;
    double altitude;
    double speed;
    double heading;
    std::chrono::system_clock::time_point timestamp;
    
    GeoPosition() : latitude(0.0), longitude(0.0), altitude(0.0), 
                    speed(0.0), heading(0.0) {
        timestamp = std::chrono::system_clock::now();
    }
    
    double distanceTo(const GeoPosition& other) const {
        const double R = 6371000; // Earth radius in meters
        double lat1_rad = latitude * M_PI / 180.0;
        double lat2_rad = other.latitude * M_PI / 180.0;
        double dlat_rad = (other.latitude - latitude) * M_PI / 180.0;
        double dlon_rad = (other.longitude - longitude) * M_PI / 180.0;
        
        double a = sin(dlat_rad/2) * sin(dlat_rad/2) +
                   cos(lat1_rad) * cos(lat2_rad) *
                   sin(dlon_rad/2) * sin(dlon_rad/2);
        double c = 2 * atan2(sqrt(a), sqrt(1-a));
        
        return R * c;
    }
};

/**
 * @brief Communication message structure
 */
struct CommMessage {
    std::string message_id;
    std::string sender_id;
    std::string receiver_id;
    ProtocolType protocol;
    MessagePriority priority;
    SecurityLevel security;
    std::vector<uint8_t> payload;
    std::map<std::string, std::string> headers;
    std::chrono::system_clock::time_point timestamp;
    std::chrono::milliseconds timeout;
    bool encrypted;
    std::string checksum;
    
    CommMessage() : protocol(ProtocolType::CAN_BUS), priority(MessagePriority::MEDIUM),
                    security(SecurityLevel::BASIC_ENCRYPTION), timeout(std::chrono::seconds(5)),
                    encrypted(false) {
        timestamp = std::chrono::system_clock::now();
        message_id = generateMessageId();
    }
    
    size_t getSize() const {
        return payload.size() + headers.size() * 64; // Approximate header size
    }
    
    bool isExpired() const {
        auto now = std::chrono::system_clock::now();
        return (now - timestamp) > timeout;
    }
    
    std::string toString() const {
        std::stringstream ss;
        ss << "Message ID: " << message_id << "\n"
           << "From: " << sender_id << " To: " << receiver_id << "\n"
           << "Protocol: " << static_cast<int>(protocol) << "\n"
           << "Priority: " << static_cast<int>(priority) << "\n"
           << "Size: " << payload.size() << " bytes\n"
           << "Encrypted: " << (encrypted ? "Yes" : "No") << "\n";
        return ss.str();
    }
    
private:
    std::string generateMessageId() const {
        auto now = std::chrono::high_resolution_clock::now();
        auto time_since_epoch = now.time_since_epoch();
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(time_since_epoch);
        
        std::stringstream ss;
        ss << "MSG" << std::hex << nanoseconds.count();
        return ss.str();
    }
};

/**
 * @brief Network interface base class
 */
class NetworkInterface {
public:
    NetworkInterface(InterfaceType type, const std::string& name) 
        : interface_type(type), interface_name(name), state(CommState::OFFLINE),
          bytes_sent(0), bytes_received(0), error_count(0) {
        creation_time = std::chrono::system_clock::now();
    }
    
    virtual ~NetworkInterface() = default;
    
    virtual bool initialize() = 0;
    virtual bool connect() = 0;
    virtual bool disconnect() = 0;
    virtual bool sendMessage(const CommMessage& message) = 0;
    virtual bool receiveMessage(CommMessage& message) = 0;
    virtual bool isConnected() const = 0;
    
    // Common interface methods
    InterfaceType getType() const { return interface_type; }
    const std::string& getName() const { return interface_name; }
    CommState getState() const { return state; }
    
    std::map<std::string, double> getStatistics() const {
        auto uptime = std::chrono::duration<double>(
            std::chrono::system_clock::now() - creation_time).count();
        
        return {
            {"bytes_sent", static_cast<double>(bytes_sent)},
            {"bytes_received", static_cast<double>(bytes_received)},
            {"error_count", static_cast<double>(error_count)},
            {"uptime_seconds", uptime},
            {"state", static_cast<double>(state)}
        };
    }
    
protected:
    InterfaceType interface_type;
    std::string interface_name;
    CommState state;
    std::atomic<uint64_t> bytes_sent;
    std::atomic<uint64_t> bytes_received;
    std::atomic<uint32_t> error_count;
    std::chrono::system_clock::time_point creation_time;
    mutable std::mutex interface_mutex;
};

/**
 * @brief Vehicle-to-Vehicle (V2V) Communication
 */
class V2VCommunication : public NetworkInterface {
public:
    V2VCommunication() : NetworkInterface(InterfaceType::DSRC, "V2V-DSRC"),
                         broadcast_interval(std::chrono::milliseconds(100)),
                         enable_periodic_broadcast_(false) {
        initializeV2V();
    }
    
    bool initialize() override {
        std::lock_guard<std::mutex> lock(interface_mutex);
        state = CommState::CONNECTING;
        
        // Simulate DSRC initialization
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        if (initializeDSRC()) {
            state = CommState::CONNECTED;
            if (enable_periodic_broadcast_) {
                startPeriodicBroadcast();
            }
            std::cout << "[V2V] DSRC interface initialized successfully\n";
            return true;
        }
        
        state = CommState::ERROR;
        error_count++;
        return false;
    }
    
    bool connect() override {
        return isConnected() || initialize();
    }
    
    bool disconnect() override {
        std::lock_guard<std::mutex> lock(interface_mutex);
        stopPeriodicBroadcast();
        state = CommState::OFFLINE;
        std::cout << "[V2V] DSRC interface disconnected\n";
        return true;
    }
    
    bool sendMessage(const CommMessage& message) override {
        if (!isConnected()) return false;
        
        std::lock_guard<std::mutex> lock(interface_mutex);
        
        // Add V2V specific headers
        CommMessage v2v_message = message;
        v2v_message.headers["V2V_TYPE"] = getV2VMessageType(message.payload);
        v2v_message.headers["POSITION"] = positionToString(current_position);
        v2v_message.headers["BROADCAST_RANGE"] = std::to_string(broadcast_range);
        
        // Simulate transmission
        if (simulateTransmission(v2v_message)) {
            bytes_sent += v2v_message.getSize();
            message_queue.push(v2v_message);
            // Prevent unbounded growth
            const size_t kMaxQueue = 512;
            while (message_queue.size() > kMaxQueue) {
                message_queue.pop();
            }
            
            std::cout << "[V2V] Sent message: " << message.message_id << "\n";
            return true;
        }
        
        error_count++;
        return false;
    }
    
    bool receiveMessage(CommMessage& message) override {
        std::lock_guard<std::mutex> lock(interface_mutex);
        
        if (!received_messages.empty()) {
            message = received_messages.front();
            received_messages.pop();
            bytes_received += message.getSize();
            return true;
        }
        
        return false;
    }
    
    bool isConnected() const override {
        return state == CommState::CONNECTED;
    }
    
    /**
     * @brief Broadcast basic safety message
     */
    void broadcastSafetyMessage(const GeoPosition& position, double speed, double heading) {
        if (!isConnected()) return;
        
        CommMessage bsm;
        bsm.sender_id = vehicle_id;
        bsm.receiver_id = "BROADCAST";
        bsm.priority = MessagePriority::CRITICAL;
        bsm.protocol = ProtocolType::DSRC_802_11P;
        
        // Create BSM payload
        std::stringstream payload_stream;
        payload_stream << position.latitude << "," << position.longitude << ","
                      << position.altitude << "," << speed << "," << heading;
        std::string payload_str = payload_stream.str();
        bsm.payload.assign(payload_str.begin(), payload_str.end());
        
        bsm.headers["MSG_TYPE"] = "BSM";
        bsm.headers["VEHICLE_ID"] = vehicle_id;
        bsm.timeout = std::chrono::milliseconds(500);
        
        sendMessage(bsm);
    }
    
    /**
     * @brief Send emergency warning message
     */
    void broadcastEmergencyWarning(const std::string& warning_type, 
                                  const std::string& description) {
        CommMessage ewm;
        ewm.sender_id = vehicle_id;
        ewm.receiver_id = "BROADCAST";
        ewm.priority = MessagePriority::CRITICAL;
        ewm.protocol = ProtocolType::DSRC_802_11P;
        
        std::string payload_str = warning_type + "|" + description;
        ewm.payload.assign(payload_str.begin(), payload_str.end());
        
        ewm.headers["MSG_TYPE"] = "EWM";
        ewm.headers["WARNING_TYPE"] = warning_type;
        ewm.timeout = std::chrono::seconds(10);
        
        // Broadcast multiple times for critical messages
        for (int i = 0; i < 3; ++i) {
            sendMessage(ewm);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        std::cout << "[V2V] Emergency warning broadcast: " << warning_type << "\n";
    }
    
    /**
     * @brief Get nearby vehicles
     */
    std::vector<std::string> getNearbyVehicles(double max_distance = 1000.0) const {
        std::lock_guard<std::mutex> lock(interface_mutex);
        std::vector<std::string> nearby;
        
        auto now = std::chrono::system_clock::now();
        for (const auto& vehicle : nearby_vehicles) {
            auto age = std::chrono::duration_cast<std::chrono::seconds>(
                now - vehicle.second.last_seen).count();
            
            if (age < 5 && vehicle.second.distance <= max_distance) {
                nearby.push_back(vehicle.first);
            }
        }
        
        return nearby;
    }
    
    /**
     * @brief Update vehicle position
     */
    void updatePosition(const GeoPosition& position) {
        std::lock_guard<std::mutex> lock(interface_mutex);
        current_position = position;
    }
    
    /**
     * @brief Set vehicle ID
     */
    void setVehicleId(const std::string& id) {
        vehicle_id = id;
    }
    
private:
    struct NearbyVehicle {
        GeoPosition position;
        double distance;
        std::chrono::system_clock::time_point last_seen;
        std::string vehicle_type;
    };
    
    std::string vehicle_id;
    GeoPosition current_position;
    double broadcast_range;
    std::chrono::milliseconds broadcast_interval;
    std::queue<CommMessage> message_queue;
    std::queue<CommMessage> received_messages;
    std::map<std::string, NearbyVehicle> nearby_vehicles;
    std::thread broadcast_thread;
    std::atomic<bool> broadcasting;
    bool enable_periodic_broadcast_;
    
    void initializeV2V() {
        vehicle_id = "EV_" + std::to_string(std::hash<std::string>{}(interface_name));
        broadcast_range = 1000.0; // 1km range
        broadcasting = false;
    }
    
    bool initializeDSRC() {
        // Simulate DSRC radio initialization
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);
        
        return dis(gen) > 0.1; // 90% success rate
    }
    
    void startPeriodicBroadcast() {
        broadcasting = true;
        broadcast_thread = std::thread(&V2VCommunication::broadcastLoop, this);
    }
    
    void stopPeriodicBroadcast() {
        broadcasting = false;
        if (broadcast_thread.joinable()) {
            broadcast_thread.join();
        }
    }
    
    void broadcastLoop() {
        while (broadcasting) {
            if (isConnected()) {
                broadcastSafetyMessage(current_position, current_position.speed, 
                                     current_position.heading);
            }
            std::this_thread::sleep_for(broadcast_interval);
        }
    }
    
    bool simulateTransmission(const CommMessage& message) {
        // Simulate transmission success/failure
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);
        
        double success_rate = 0.95;
        if (message.priority == MessagePriority::CRITICAL) {
            success_rate = 0.99; // Higher reliability for critical messages
        }
        
        return dis(gen) < success_rate;
    }
    
    std::string getV2VMessageType(const std::vector<uint8_t>& payload) {
        // Analyze payload to determine message type
        if (payload.size() > 0) {
            uint8_t first_byte = payload[0];
            switch (first_byte % 4) {
                case 0: return "BSM";     // Basic Safety Message
                case 1: return "EWM";     // Emergency Warning Message
                case 2: return "TIM";     // Traveler Information Message
                case 3: return "RSA";     // Road Side Alert
                default: return "UNKNOWN";
            }
        }
        return "EMPTY";
    }
    
    std::string positionToString(const GeoPosition& pos) const {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(6) 
           << pos.latitude << "," << pos.longitude << "," << pos.altitude;
        return ss.str();
    }
};

/**
 * @brief Vehicle-to-Infrastructure (V2I) Communication
 */
class V2ICommunication : public NetworkInterface {
public:
    V2ICommunication() : NetworkInterface(InterfaceType::DSRC, "V2I-DSRC"),
                         rsu_scan_interval(std::chrono::seconds(5)) {
        initializeV2I();
    }
    
    bool initialize() override {
        std::lock_guard<std::mutex> lock(interface_mutex);
        state = CommState::CONNECTING;
        
        // Initialize infrastructure communication
        if (initializeInfrastructure()) {
            state = CommState::CONNECTED;
            startRSUScan();
            std::cout << "[V2I] Infrastructure interface initialized\n";
            return true;
        }
        
        state = CommState::ERROR;
        error_count++;
        return false;
    }
    
    bool connect() override {
        return isConnected() || initialize();
    }
    
    bool disconnect() override {
        std::lock_guard<std::mutex> lock(interface_mutex);
        stopRSUScan();
        connected_rsus.clear();
        state = CommState::OFFLINE;
        std::cout << "[V2I] Infrastructure interface disconnected\n";
        return true;
    }
    
    bool sendMessage(const CommMessage& message) override {
        if (!isConnected()) return false;
        
        std::lock_guard<std::mutex> lock(interface_mutex);
        
        // Route message to appropriate RSU
        std::string target_rsu = selectBestRSU(message);
        if (target_rsu.empty()) {
            error_count++;
            return false;
        }
        
        CommMessage v2i_message = message;
        v2i_message.headers["V2I_RSU"] = target_rsu;
        v2i_message.headers["INFRA_TYPE"] = "RSU";
        
        if (simulateInfraTransmission(v2i_message, target_rsu)) {
            bytes_sent += v2i_message.getSize();
            std::cout << "[V2I] Message sent to RSU: " << target_rsu << "\n";
            return true;
        }
        
        error_count++;
        return false;
    }
    
    bool receiveMessage(CommMessage& message) override {
        std::lock_guard<std::mutex> lock(interface_mutex);
        
        if (!infrastructure_messages.empty()) {
            message = infrastructure_messages.front();
            infrastructure_messages.pop();
            bytes_received += message.getSize();
            return true;
        }
        
        return false;
    }
    
    bool isConnected() const override {
        return state == CommState::CONNECTED && !connected_rsus.empty();
    }
    
    /**
     * @brief Request traffic signal information
     */
    bool requestTrafficSignalInfo(const std::string& intersection_id) {
        if (!isConnected()) return false;
        
        CommMessage signal_request;
        signal_request.sender_id = vehicle_id;
        signal_request.receiver_id = intersection_id;
        signal_request.priority = MessagePriority::HIGH;
        signal_request.protocol = ProtocolType::DSRC_802_11P;
        
        std::string payload_str = "SIGNAL_REQUEST|" + intersection_id;
        signal_request.payload.assign(payload_str.begin(), payload_str.end());
        
        signal_request.headers["MSG_TYPE"] = "SIGNAL_REQUEST";
        signal_request.headers["INTERSECTION"] = intersection_id;
        
        return sendMessage(signal_request);
    }
    
    /**
     * @brief Request optimal speed advisory
     */
    bool requestSpeedAdvisory(double current_speed, const GeoPosition& position) {
        if (!isConnected()) return false;
        
        CommMessage speed_request;
        speed_request.sender_id = vehicle_id;
        speed_request.receiver_id = "TRAFFIC_MANAGEMENT";
        speed_request.priority = MessagePriority::MEDIUM;
        
        std::stringstream payload_stream;
        payload_stream << "SPEED_ADVISORY|" << current_speed << "|"
                      << position.latitude << "," << position.longitude;
        std::string payload_str = payload_stream.str();
        speed_request.payload.assign(payload_str.begin(), payload_str.end());
        
        speed_request.headers["MSG_TYPE"] = "SPEED_ADVISORY_REQUEST";
        
        return sendMessage(speed_request);
    }
    
    /**
     * @brief Get connected RSU information
     */
    std::vector<std::map<std::string, std::string>> getConnectedRSUs() const {
        std::lock_guard<std::mutex> lock(interface_mutex);
        std::vector<std::map<std::string, std::string>> rsu_info;
        
        for (const auto& rsu : connected_rsus) {
            std::map<std::string, std::string> info;
            info["id"] = rsu.first;
            info["signal_strength"] = std::to_string(rsu.second.signal_strength);
            info["distance"] = std::to_string(rsu.second.distance);
            info["services"] = rsu.second.services;
            rsu_info.push_back(info);
        }
        
        return rsu_info;
    }
    
    /**
     * @brief Set vehicle ID
     */
    void setVehicleId(const std::string& id) {
        vehicle_id = id;
    }
    
private:
    struct RSUInfo {
        std::string id;
        GeoPosition position;
        double signal_strength;
        double distance;
        std::string services;
        std::chrono::system_clock::time_point last_contact;
    };
    
    std::string vehicle_id;
    std::map<std::string, RSUInfo> connected_rsus;
    std::queue<CommMessage> infrastructure_messages;
    std::chrono::seconds rsu_scan_interval;
    std::thread rsu_scan_thread;
    std::atomic<bool> scanning;
    
    void initializeV2I() {
        vehicle_id = "EV_V2I_" + std::to_string(std::hash<std::string>{}(interface_name));
        scanning = false;
    }
    
    bool initializeInfrastructure() {
        // Simulate infrastructure initialization
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);
        
        return dis(gen) > 0.05; // 95% success rate
    }
    
    void startRSUScan() {
        scanning = true;
        rsu_scan_thread = std::thread(&V2ICommunication::rsuScanLoop, this);
    }
    
    void stopRSUScan() {
        scanning = false;
        if (rsu_scan_thread.joinable()) {
            rsu_scan_thread.join();
        }
    }
    
    void rsuScanLoop() {
        while (scanning.load()) {
            scanForRSUs();
            cleanupOldRSUs();
            std::this_thread::sleep_for(rsu_scan_interval);
        }
    }
    
    void scanForRSUs() {
        // Simulate RSU discovery
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> rsu_count_dist(1, 3);
        std::uniform_real_distribution<> signal_dist(0.3, 1.0);
        std::uniform_real_distribution<> distance_dist(50.0, 500.0);
        
        int new_rsus = rsu_count_dist(gen);
        
        for (int i = 0; i < new_rsus; ++i) {
            std::string rsu_id = "RSU_" + std::to_string(gen() % 1000);
            
            if (connected_rsus.find(rsu_id) == connected_rsus.end()) {
                RSUInfo rsu;
                rsu.id = rsu_id;
                rsu.signal_strength = signal_dist(gen);
                rsu.distance = distance_dist(gen);
                rsu.services = "TRAFFIC_SIGNALS,SPEED_ADVISORY,WEATHER";
                rsu.last_contact = std::chrono::system_clock::now();
                
                connected_rsus[rsu_id] = rsu;
                std::cout << "[V2I] Connected to RSU: " << rsu_id << "\n";
            }
        }
    }
    
    void cleanupOldRSUs() {
        auto now = std::chrono::system_clock::now();
        auto it = connected_rsus.begin();
        
        while (it != connected_rsus.end()) {
            auto age = std::chrono::duration_cast<std::chrono::seconds>(
                now - it->second.last_contact).count();
            
            if (age > 30) { // Remove RSUs not contacted in 30 seconds
                std::cout << "[V2I] Lost connection to RSU: " << it->first << "\n";
                it = connected_rsus.erase(it);
            } else {
                ++it;
            }
        }
    }
    
    std::string selectBestRSU(const CommMessage& message) {
        if (connected_rsus.empty()) return "";
        
        // Select RSU with best signal strength
        std::string best_rsu;
        double best_signal = 0.0;
        
        for (const auto& rsu : connected_rsus) {
            if (rsu.second.signal_strength > best_signal) {
                best_signal = rsu.second.signal_strength;
                best_rsu = rsu.first;
            }
        }
        
        return best_rsu;
    }
    
    bool simulateInfraTransmission(const CommMessage& message, const std::string& rsu_id) {
        auto rsu_it = connected_rsus.find(rsu_id);
        if (rsu_it == connected_rsus.end()) return false;
        
        // Success rate based on signal strength
        double success_rate = rsu_it->second.signal_strength * 0.9 + 0.1;
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);
        
        return dis(gen) < success_rate;
    }
};

/**
 * @brief Cellular Modem Communication
 */
class CellularModem : public NetworkInterface {
public:
    CellularModem(const std::string& network_type = "5G") 
        : NetworkInterface(InterfaceType::CELLULAR_5G, "Cellular-" + network_type),
          network_type(network_type), signal_strength(0), data_plan_remaining(10240) {
        initializeCellular();
    }
    
    bool initialize() override {
        std::lock_guard<std::mutex> lock(interface_mutex);
        state = CommState::CONNECTING;
        
        std::cout << "[Cellular] Initializing " << network_type << " modem...\n";
        
        if (registerToNetwork()) {
            state = CommState::CONNECTED;
            startSignalMonitoring();
            std::cout << "[Cellular] Connected to " << network_type << " network\n";
            return true;
        }
        
        state = CommState::ERROR;
        error_count++;
        return false;
    }
    
    bool connect() override {
        if (isConnected()) return true;
        
        std::lock_guard<std::mutex> lock(interface_mutex);
        state = CommState::CONNECTING;
        
        if (authenticateUser()) {
            state = CommState::AUTHENTICATED;
            return true;
        }
        
        state = CommState::ERROR;
        error_count++;
        return false;
    }
    
    bool disconnect() override {
        std::lock_guard<std::mutex> lock(interface_mutex);
        stopSignalMonitoring();
        state = CommState::OFFLINE;
        signal_strength = 0;
        std::cout << "[Cellular] Disconnected from network\n";
        return true;
    }
    
    bool sendMessage(const CommMessage& message) override {
        if (!isConnected()) return false;
        
        if (data_plan_remaining < static_cast<int>(message.getSize())) {
            std::cout << "[Cellular] Insufficient data plan remaining\n";
            error_count++;
            return false;
        }
        
        std::lock_guard<std::mutex> lock(interface_mutex);
        
        CommMessage cellular_message = message;
        cellular_message.headers["NETWORK_TYPE"] = network_type;
        cellular_message.headers["SIGNAL_STRENGTH"] = std::to_string(signal_strength);
        cellular_message.headers["CELL_ID"] = current_cell_id;
        
        if (simulateCellularTransmission(cellular_message)) {
            bytes_sent += cellular_message.getSize();
            data_plan_remaining -= static_cast<int>(cellular_message.getSize());
            
            std::cout << "[Cellular] Message sent via " << network_type << "\n";
            return true;
        }
        
        error_count++;
        return false;
    }
    
    bool receiveMessage(CommMessage& message) override {
        std::lock_guard<std::mutex> lock(interface_mutex);
        
        if (!received_cellular_messages.empty()) {
            message = received_cellular_messages.front();
            received_cellular_messages.pop();
            bytes_received += message.getSize();
            return true;
        }
        
        return false;
    }
    
    bool isConnected() const override {
        return (state == CommState::CONNECTED || state == CommState::AUTHENTICATED) 
               && signal_strength > 20;
    }
    
    /**
     * @brief Send cloud telemetry data
     */
    bool sendTelemetryData(const std::map<std::string, double>& telemetry) {
        if (!isConnected()) return false;
        
        CommMessage telemetry_msg;
        telemetry_msg.sender_id = vehicle_id;
        telemetry_msg.receiver_id = "CLOUD_BACKEND";
        telemetry_msg.priority = MessagePriority::LOW;
        telemetry_msg.protocol = (network_type == "5G") ? ProtocolType::C_V2X_5G : ProtocolType::LTE_V2X;
        
        // Serialize telemetry data
        std::stringstream payload_stream;
        for (const auto& data : telemetry) {
            payload_stream << data.first << ":" << data.second << ";";
        }
        std::string payload_str = payload_stream.str();
        telemetry_msg.payload.assign(payload_str.begin(), payload_str.end());
        
        telemetry_msg.headers["MSG_TYPE"] = "TELEMETRY";
        telemetry_msg.headers["TIMESTAMP"] = std::to_string(
            std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        
        return sendMessage(telemetry_msg);
    }
    
    /**
     * @brief Download over-the-air update
     */
    bool downloadOTAUpdate(const std::string& update_id, size_t update_size) {
        if (!isConnected()) return false;
        
        if (data_plan_remaining < static_cast<int>(update_size)) {
            std::cout << "[Cellular] Insufficient data for OTA update\n";
            return false;
        }
        
        std::cout << "[Cellular] Starting OTA download: " << update_id << "\n";
        
        // Simulate download process
        size_t downloaded = 0;
        size_t chunk_size = 1024 * 1024; // 1MB chunks
        
        while (downloaded < update_size && isConnected()) {
            size_t current_chunk = std::min(chunk_size, update_size - downloaded);
            
            // Simulate download time
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            downloaded += current_chunk;
            data_plan_remaining -= static_cast<int>(current_chunk);
            
            double progress = (static_cast<double>(downloaded) / update_size) * 100.0;
            if (static_cast<int>(progress) % 10 == 0) {
                std::cout << "[Cellular] Download progress: " << std::fixed 
                         << std::setprecision(1) << progress << "%\n";
            }
        }
        
        if (downloaded == update_size) {
            std::cout << "[Cellular] OTA update downloaded successfully\n";
            return true;
        }
        
        return false;
    }
    
    /**
     * @brief Get network information
     */
    std::map<std::string, std::string> getNetworkInfo() const {
        std::lock_guard<std::mutex> lock(interface_mutex);
        return {
            {"network_type", network_type},
            {"signal_strength", std::to_string(signal_strength)},
            {"cell_id", current_cell_id},
            {"data_remaining_mb", std::to_string(data_plan_remaining / 1024)},
            {"roaming", is_roaming ? "true" : "false"}
        };
    }
    
    /**
     * @brief Set vehicle ID
     */
    void setVehicleId(const std::string& id) {
        vehicle_id = id;
    }
    
private:
    std::string vehicle_id;
    std::string network_type;
    std::string current_cell_id;
    int signal_strength; // 0-100
    int data_plan_remaining; // in MB
    bool is_roaming;
    std::queue<CommMessage> received_cellular_messages;
    std::thread signal_monitor_thread;
    std::atomic<bool> monitoring;
    
    void initializeCellular() {
        vehicle_id = "EV_CELL_" + std::to_string(std::hash<std::string>{}(interface_name));
        current_cell_id = "CELL_" + std::to_string(rand() % 1000);
        is_roaming = false;
        monitoring = false;
    }
    
    bool registerToNetwork() {
        // Simulate network registration
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> signal_dist(30, 95);
        std::uniform_real_distribution<> success_dist(0.0, 1.0);
        
        signal_strength = signal_dist(gen);
        
        // Registration success rate based on signal strength
        double success_rate = signal_strength / 100.0;
        return success_dist(gen) < success_rate;
    }
    
    bool authenticateUser() {
        // Simulate user authentication
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);
        
        return dis(gen) > 0.02; // 98% authentication success rate
    }
    
    void startSignalMonitoring() {
        monitoring = true;
        signal_monitor_thread = std::thread(&CellularModem::signalMonitorLoop, this);
    }
    
    void stopSignalMonitoring() {
        monitoring = false;
        if (signal_monitor_thread.joinable()) {
            signal_monitor_thread.join();
        }
    }
    
    void signalMonitorLoop() {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<> signal_noise(0, 5);
        
        while (monitoring) {
            // Simulate signal strength fluctuations
            int new_strength = signal_strength + static_cast<int>(signal_noise(gen));
            signal_strength = std::max(0, std::min(100, new_strength));
            
            // Occasionally change cell
            std::uniform_real_distribution<> cell_change_chance(0.0, 1.0);
            if (cell_change_chance(gen) < 0.01) { // 1% chance per monitoring cycle
                current_cell_id = "CELL_" + std::to_string(gen() % 1000);
                std::cout << "[Cellular] Handover to cell: " << current_cell_id << "\n";
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    
    bool simulateCellularTransmission(const CommMessage& message) {
        // Success rate based on signal strength and network type
        double base_success_rate = 0.95;
        
        if (network_type == "5G") {
            base_success_rate = 0.98;
        } else if (network_type == "4G") {
            base_success_rate = 0.95;
        } else {
            base_success_rate = 0.90; // 3G or older
        }
        
        double signal_factor = signal_strength / 100.0;
        double final_success_rate = base_success_rate * signal_factor;
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);
        
        return dis(gen) < final_success_rate;
    }
};

/**
 * @brief Main Communication Controller
 */
class EVCommunicationController {
public:
    EVCommunicationController() : shut_down_(false) {
        v2v_comm = std::make_unique<V2VCommunication>();
        v2i_comm = std::make_unique<V2ICommunication>();
        cellular_modem = std::make_unique<CellularModem>("5G");
        
        message_router = std::make_unique<MessageRouter>();
        initializeController();
    }
    
    ~EVCommunicationController() {
        shutdown();
    }
    
    /**
     * @brief Initialize all communication systems
     */
    bool initialize() {
        std::cout << "[CommController] Initializing EV communication systems...\n";
        
        bool v2v_ok = v2v_comm->initialize();
        bool v2i_ok = v2i_comm->initialize();  
        bool cellular_ok = cellular_modem->initialize();
        
        if (v2v_ok || v2i_ok || cellular_ok) {
            std::cout << "[CommController] Communication systems initialized\n";
            return true;
        }
        
        std::cout << "[CommController] Failed to initialize communication systems\n";
        return false;
    }
    
    /**
     * @brief Shutdown all communication systems
     */
    void shutdown() {
        bool expected = false;
        if (!shut_down_.compare_exchange_strong(expected, true)) {
            return; // already shut down
        }
        std::cout << "[CommController] Shutting down communication systems...\n";
        
        if (v2v_comm) v2v_comm->disconnect();
        if (v2i_comm) v2i_comm->disconnect();
        if (cellular_modem) cellular_modem->disconnect();
        
        std::cout << "[CommController] Communication systems shut down\n";
    }
    
    /**
     * @brief Send emergency alert to all available channels
     */
    bool sendEmergencyAlert(const std::string& alert_type, const std::string& description) {
        bool sent = false;
        
        if (v2v_comm && v2v_comm->isConnected()) {
            v2v_comm->broadcastEmergencyWarning(alert_type, description);
            sent = true;
        }
        
        if (cellular_modem && cellular_modem->isConnected()) {
            CommMessage emergency_msg;
            emergency_msg.sender_id = vehicle_id;
            emergency_msg.receiver_id = "EMERGENCY_SERVICES";
            emergency_msg.priority = MessagePriority::CRITICAL;
            
            std::string payload_str = alert_type + "|" + description;
            emergency_msg.payload.assign(payload_str.begin(), payload_str.end());
            emergency_msg.headers["MSG_TYPE"] = "EMERGENCY_ALERT";
            
            sent |= cellular_modem->sendMessage(emergency_msg);
        }
        
        return sent;
    }
    
    /**
     * @brief Get overall communication status
     */
    std::map<std::string, std::string> getCommStatus() const {
        std::map<std::string, std::string> status;
        
        status["v2v_connected"] = (v2v_comm && v2v_comm->isConnected()) ? "true" : "false";
        status["v2i_connected"] = (v2i_comm && v2i_comm->isConnected()) ? "true" : "false";
        status["cellular_connected"] = (cellular_modem && cellular_modem->isConnected()) ? "true" : "false";
        
        int connected_interfaces = 0;
        if (status["v2v_connected"] == "true") connected_interfaces++;
        if (status["v2i_connected"] == "true") connected_interfaces++;
        if (status["cellular_connected"] == "true") connected_interfaces++;
        
        status["total_interfaces"] = std::to_string(connected_interfaces);
        status["overall_status"] = connected_interfaces > 0 ? "OPERATIONAL" : "OFFLINE";
        
        return status;
    }
    
    // Getters for individual communication systems
    V2VCommunication* getV2VComm() const { return v2v_comm.get(); }
    V2ICommunication* getV2IComm() const { return v2i_comm.get(); }
    CellularModem* getCellularModem() const { return cellular_modem.get(); }
    
private:
    class MessageRouter {
    public:
        void routeMessage(const CommMessage& message) {
            // Simple message routing logic
            std::cout << "[Router] Routing message: " << message.message_id << "\n";
        }
    };
    
    std::unique_ptr<V2VCommunication> v2v_comm;
    std::unique_ptr<V2ICommunication> v2i_comm;
    std::unique_ptr<CellularModem> cellular_modem;
    std::unique_ptr<MessageRouter> message_router;
    std::string vehicle_id;
    std::atomic<bool> shut_down_;
    
    void initializeController() {
        vehicle_id = "EV_MAIN_" + std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id()));
        
        if (v2v_comm) v2v_comm->setVehicleId(vehicle_id);
        if (v2i_comm) v2i_comm->setVehicleId(vehicle_id);  
        if (cellular_modem) cellular_modem->setVehicleId(vehicle_id);
        
        std::cout << "[CommController] Controller initialized with vehicle ID: " << vehicle_id << "\n";
    }
};

} // namespace ev_communication

#endif // EV_COMMUNICATION_PROTOCOLS_H