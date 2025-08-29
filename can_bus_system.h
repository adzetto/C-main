/**
 * CAN Bus Communication System for Electric Vehicles
 * Author: adzetto
 * Features: Multi-node communication, message filtering, error handling, diagnostics
 */

#ifndef CAN_BUS_SYSTEM_H
#define CAN_BUS_SYSTEM_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <mutex>
#include <thread>
#include <chrono>
#include <bitset>
#include <atomic>

enum class CANFrameType {
    DATA_FRAME,
    REMOTE_FRAME,
    ERROR_FRAME,
    OVERLOAD_FRAME
};

enum class CANPriority {
    HIGHEST = 0,
    HIGH = 1,
    NORMAL = 2,
    LOW = 3,
    LOWEST = 4
};

struct CANMessage {
    uint32_t id;
    CANFrameType frameType;
    bool extendedFormat;
    uint8_t dataLength;
    uint8_t data[8];
    std::chrono::steady_clock::time_point timestamp;
    uint8_t nodeId;
    CANPriority priority;
    
    CANMessage() : id(0), frameType(CANFrameType::DATA_FRAME), extendedFormat(false),
                   dataLength(0), nodeId(0), priority(CANPriority::NORMAL) {
        std::fill(std::begin(data), std::end(data), 0);
        timestamp = std::chrono::steady_clock::now();
    }
    
    CANMessage(uint32_t msgId, const uint8_t* msgData, uint8_t len, uint8_t node = 0) 
        : id(msgId), frameType(CANFrameType::DATA_FRAME), extendedFormat(false),
          dataLength(std::min(len, static_cast<uint8_t>(8))), nodeId(node), 
          priority(CANPriority::NORMAL) {
        std::fill(std::begin(data), std::end(data), 0);
        std::copy(msgData, msgData + dataLength, data);
        timestamp = std::chrono::steady_clock::now();
    }
};

class CANNode {
private:
    uint8_t nodeId;
    std::string nodeName;
    std::vector<uint32_t> acceptanceFilters;
    std::queue<CANMessage> txQueue;
    std::queue<CANMessage> rxQueue;
    std::mutex txMutex, rxMutex;
    bool active;
    uint32_t messagesTransmitted;
    uint32_t messagesReceived;
    uint32_t errorsDetected;
    
public:
    CANNode(uint8_t id, const std::string& name) 
        : nodeId(id), nodeName(name), active(true), 
          messagesTransmitted(0), messagesReceived(0), errorsDetected(0) {}
    
    void addAcceptanceFilter(uint32_t filterId) {
        acceptanceFilters.push_back(filterId);
    }
    
    void transmitMessage(uint32_t id, const uint8_t* data, uint8_t length, CANPriority priority = CANPriority::NORMAL) {
        if (!active) return;
        
        CANMessage msg(id, data, length, nodeId);
        msg.priority = priority;
        
        std::lock_guard<std::mutex> lock(txMutex);
        txQueue.push(msg);
        messagesTransmitted++;
    }
    
    bool receiveMessage(CANMessage& msg) {
        std::lock_guard<std::mutex> lock(rxMutex);
        if (!rxQueue.empty()) {
            msg = rxQueue.front();
            rxQueue.pop();
            return true;
        }
        return false;
    }
    
    void deliverMessage(const CANMessage& msg) {
        if (!active) return;
        
        // Check acceptance filters
        if (!acceptanceFilters.empty()) {
            bool accepted = false;
            for (uint32_t filter : acceptanceFilters) {
                if ((msg.id & 0xFF0) == (filter & 0xFF0)) {
                    accepted = true;
                    break;
                }
            }
            if (!accepted) return;
        }
        
        std::lock_guard<std::mutex> lock(rxMutex);
        rxQueue.push(msg);
        messagesReceived++;
    }
    
    bool hasMessageToTransmit() {
        std::lock_guard<std::mutex> lock(txMutex);
        return !txQueue.empty();
    }
    
    CANMessage getNextTransmissionMessage() {
        std::lock_guard<std::mutex> lock(txMutex);
        if (!txQueue.empty()) {
            CANMessage msg = txQueue.front();
            txQueue.pop();
            return msg;
        }
        return CANMessage();
    }
    
    void setActive(bool state) { active = state; }
    bool isActive() const { return active; }
    uint8_t getNodeId() const { return nodeId; }
    std::string getNodeName() const { return nodeName; }
    
    void displayStatistics() const {
        std::cout << "Node " << static_cast<int>(nodeId) << " (" << nodeName << "):\n";
        std::cout << "  Messages TX: " << messagesTransmitted << "\n";
        std::cout << "  Messages RX: " << messagesReceived << "\n";
        std::cout << "  Errors: " << errorsDetected << "\n";
        std::cout << "  Status: " << (active ? "ACTIVE" : "INACTIVE") << "\n";
    }
    
    void incrementErrorCount() { errorsDetected++; }
};

class CANBus {
private:
    std::vector<std::shared_ptr<CANNode>> nodes;
    std::queue<CANMessage> busQueue;
    std::mutex busMutex;
    std::thread busThread;
    std::atomic<bool> running;
    
    // Bus statistics
    uint32_t totalMessages;
    uint32_t busErrors;
    uint32_t collisions;
    double busLoad;
    std::chrono::steady_clock::time_point lastStatsUpdate;
    
    // Error detection
    std::unordered_map<uint32_t, uint8_t> crcErrors;
    std::unordered_map<uint32_t, uint8_t> stuffErrors;
    
public:
    CANBus() : running(false), totalMessages(0), busErrors(0), collisions(0), busLoad(0.0) {
        lastStatsUpdate = std::chrono::steady_clock::now();
    }
    
    ~CANBus() {
        stop();
    }
    
    void addNode(std::shared_ptr<CANNode> node) {
        nodes.push_back(node);
        std::cout << "Added CAN node: " << node->getNodeName() 
                  << " (ID: " << static_cast<int>(node->getNodeId()) << ")\n";
    }
    
    void start() {
        if (running) return;
        
        running = true;
        busThread = std::thread(&CANBus::processMessages, this);
        std::cout << "CAN Bus started with " << nodes.size() << " nodes\n";
    }
    
    void stop() {
        if (!running) return;
        
        running = false;
        if (busThread.joinable()) {
            busThread.join();
        }
        std::cout << "CAN Bus stopped\n";
    }
    
    void broadcastMessage(uint32_t id, const uint8_t* data, uint8_t length, 
                         uint8_t senderNodeId, CANPriority priority = CANPriority::NORMAL) {
        CANMessage msg(id, data, length, senderNodeId);
        msg.priority = priority;
        
        std::lock_guard<std::mutex> lock(busMutex);
        busQueue.push(msg);
        totalMessages++;
    }
    
    void sendDiagnosticRequest(uint8_t targetNodeId) {
        uint8_t diagData[] = {0x01, 0x00}; // Diagnostic request
        uint32_t diagId = 0x7E0 + targetNodeId;
        broadcastMessage(diagId, diagData, 2, 0, CANPriority::HIGH);
    }
    
    void displayBusStatistics() {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - lastStatsUpdate).count();
        
        if (duration > 0) {
            busLoad = (totalMessages / static_cast<double>(duration)) / 1000.0 * 100.0; // Assume 1000 msg/s = 100%
        }
        
        std::cout << "\n=== CAN Bus Statistics ===\n";
        std::cout << "Total Messages: " << totalMessages << "\n";
        std::cout << "Bus Errors: " << busErrors << "\n";
        std::cout << "Collisions: " << collisions << "\n";
        std::cout << "Bus Load: " << std::fixed << std::setprecision(2) << busLoad << "%\n";
        std::cout << "Active Nodes: " << getActiveNodeCount() << "/" << nodes.size() << "\n";
        
        std::cout << "\n=== Node Statistics ===\n";
        for (const auto& node : nodes) {
            node->displayStatistics();
        }
        std::cout << "========================\n\n";
        
        lastStatsUpdate = now;
    }
    
    std::shared_ptr<CANNode> getNode(uint8_t nodeId) {
        for (auto& node : nodes) {
            if (node->getNodeId() == nodeId) {
                return node;
            }
        }
        return nullptr;
    }
    
    void simulateError(uint32_t messageId) {
        busErrors++;
        crcErrors[messageId]++;
        std::cout << "CAN Error detected on message ID: 0x" << std::hex << messageId << std::dec << "\n";
    }
    
private:
    void processMessages() {
        while (running) {
            std::vector<CANMessage> messagesToProcess;
            
            // Collect messages from all nodes
            for (auto& node : nodes) {
                while (node->hasMessageToTransmit()) {
                    messagesToProcess.push_back(node->getNextTransmissionMessage());
                }
            }
            
            // Process bus queue
            {
                std::lock_guard<std::mutex> lock(busMutex);
                while (!busQueue.empty()) {
                    messagesToProcess.push_back(busQueue.front());
                    busQueue.pop();
                }
            }
            
            // Sort messages by priority (lower value = higher priority)
            std::sort(messagesToProcess.begin(), messagesToProcess.end(),
                     [](const CANMessage& a, const CANMessage& b) {
                         return static_cast<int>(a.priority) < static_cast<int>(b.priority);
                     });
            
            // Deliver messages to appropriate nodes
            for (const auto& msg : messagesToProcess) {
                deliverMessageToNodes(msg);
                
                // Simulate occasional errors
                if (rand() % 10000 == 0) {
                    simulateError(msg.id);
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    void deliverMessageToNodes(const CANMessage& msg) {
        for (auto& node : nodes) {
            if (node->getNodeId() != msg.nodeId && node->isActive()) {
                node->deliverMessage(msg);
            }
        }
    }
    
    int getActiveNodeCount() const {
        int count = 0;
        for (const auto& node : nodes) {
            if (node->isActive()) count++;
        }
        return count;
    }
};

// Predefined CAN message IDs for EV systems
namespace EVCAN {
    constexpr uint32_t BMS_STATUS = 0x200;
    constexpr uint32_t BMS_CELL_VOLTAGES = 0x201;
    constexpr uint32_t BMS_TEMPERATURES = 0x202;
    constexpr uint32_t MOTOR_CONTROLLER = 0x300;
    constexpr uint32_t MOTOR_STATUS = 0x301;
    constexpr uint32_t ADAS_STATUS = 0x400;
    constexpr uint32_t ADAS_OBJECTS = 0x401;
    constexpr uint32_t CHARGING_STATUS = 0x500;
    constexpr uint32_t VEHICLE_SPEED = 0x600;
    constexpr uint32_t DIAGNOSTICS = 0x7E0;
}

class EVCANSystem {
private:
    CANBus canBus;
    std::shared_ptr<CANNode> bmsNode;
    std::shared_ptr<CANNode> motorNode;
    std::shared_ptr<CANNode> adasNode;
    std::shared_ptr<CANNode> chargingNode;
    std::shared_ptr<CANNode> diagnosticsNode;
    
public:
    EVCANSystem() {
        setupNodes();
    }
    
    void setupNodes() {
        // Create nodes
        bmsNode = std::make_shared<CANNode>(1, "BMS Controller");
        motorNode = std::make_shared<CANNode>(2, "Motor Controller");
        adasNode = std::make_shared<CANNode>(3, "ADAS System");
        chargingNode = std::make_shared<CANNode>(4, "Charging Controller");
        diagnosticsNode = std::make_shared<CANNode>(5, "Diagnostics");
        
        // Configure acceptance filters
        bmsNode->addAcceptanceFilter(EVCAN::BMS_STATUS);
        bmsNode->addAcceptanceFilter(EVCAN::CHARGING_STATUS);
        
        motorNode->addAcceptanceFilter(EVCAN::MOTOR_CONTROLLER);
        motorNode->addAcceptanceFilter(EVCAN::BMS_STATUS);
        
        adasNode->addAcceptanceFilter(EVCAN::ADAS_STATUS);
        adasNode->addAcceptanceFilter(EVCAN::VEHICLE_SPEED);
        
        chargingNode->addAcceptanceFilter(EVCAN::CHARGING_STATUS);
        chargingNode->addAcceptanceFilter(EVCAN::BMS_STATUS);
        
        diagnosticsNode->addAcceptanceFilter(EVCAN::DIAGNOSTICS);
        
        // Add nodes to bus
        canBus.addNode(bmsNode);
        canBus.addNode(motorNode);
        canBus.addNode(adasNode);
        canBus.addNode(chargingNode);
        canBus.addNode(diagnosticsNode);
    }
    
    void start() {
        canBus.start();
    }
    
    void stop() {
        canBus.stop();
    }
    
    void sendBMSStatus(double voltage, double current, double soc, double temperature) {
        uint8_t data[8];
        
        // Pack data (simplified representation)
        uint16_t voltageData = static_cast<uint16_t>(voltage * 10);
        uint16_t currentData = static_cast<uint16_t>((current + 200) * 10); // Offset for negative values
        uint8_t socData = static_cast<uint8_t>(soc);
        uint8_t tempData = static_cast<uint8_t>(temperature + 40); // Offset for negative temps
        
        data[0] = voltageData & 0xFF;
        data[1] = (voltageData >> 8) & 0xFF;
        data[2] = currentData & 0xFF;
        data[3] = (currentData >> 8) & 0xFF;
        data[4] = socData;
        data[5] = tempData;
        data[6] = 0; // Reserved
        data[7] = 0; // Reserved
        
        bmsNode->transmitMessage(EVCAN::BMS_STATUS, data, 8, CANPriority::HIGH);
    }
    
    void sendMotorCommand(double torque, double speed, bool enable) {
        uint8_t data[8];
        
        uint16_t torqueData = static_cast<uint16_t>((torque + 500) * 10); // Offset for negative torque
        uint16_t speedData = static_cast<uint16_t>(speed * 10);
        
        data[0] = torqueData & 0xFF;
        data[1] = (torqueData >> 8) & 0xFF;
        data[2] = speedData & 0xFF;
        data[3] = (speedData >> 8) & 0xFF;
        data[4] = enable ? 1 : 0;
        data[5] = 0; // Reserved
        data[6] = 0; // Reserved
        data[7] = 0; // Reserved
        
        motorNode->transmitMessage(EVCAN::MOTOR_CONTROLLER, data, 8, CANPriority::NORMAL);
    }
    
    void sendADASData(double targetSpeed, double steeringAngle, bool emergencyBrake) {
        uint8_t data[8];
        
        uint16_t speedData = static_cast<uint16_t>(targetSpeed * 10);
        uint16_t steeringData = static_cast<uint16_t>((steeringAngle + 180) * 10); // Offset for negative angles
        
        data[0] = speedData & 0xFF;
        data[1] = (speedData >> 8) & 0xFF;
        data[2] = steeringData & 0xFF;
        data[3] = (steeringData >> 8) & 0xFF;
        data[4] = emergencyBrake ? 1 : 0;
        data[5] = 0; // Reserved
        data[6] = 0; // Reserved
        data[7] = 0; // Reserved
        
        adasNode->transmitMessage(EVCAN::ADAS_STATUS, data, 8, CANPriority::HIGH);
    }
    
    void sendChargingStatus(bool charging, double power, double voltage, double current) {
        uint8_t data[8];
        
        uint16_t powerData = static_cast<uint16_t>(power * 10);
        uint16_t voltageData = static_cast<uint16_t>(voltage * 10);
        uint16_t currentData = static_cast<uint16_t>(current * 10);
        
        data[0] = charging ? 1 : 0;
        data[1] = powerData & 0xFF;
        data[2] = (powerData >> 8) & 0xFF;
        data[3] = voltageData & 0xFF;
        data[4] = (voltageData >> 8) & 0xFF;
        data[5] = currentData & 0xFF;
        data[6] = (currentData >> 8) & 0xFF;
        data[7] = 0; // Reserved
        
        chargingNode->transmitMessage(EVCAN::CHARGING_STATUS, data, 8, CANPriority::NORMAL);
    }
    
    void requestDiagnostics(uint8_t nodeId) {
        canBus.sendDiagnosticRequest(nodeId);
    }
    
    void displayStatus() {
        canBus.displayBusStatistics();
    }
    
    void processReceivedMessages() {
        CANMessage msg;
        
        // Process BMS messages
        while (bmsNode->receiveMessage(msg)) {
            processBMSMessage(msg);
        }
        
        // Process Motor messages
        while (motorNode->receiveMessage(msg)) {
            processMotorMessage(msg);
        }
        
        // Process ADAS messages
        while (adasNode->receiveMessage(msg)) {
            processADASMessage(msg);
        }
        
        // Process Charging messages
        while (chargingNode->receiveMessage(msg)) {
            processChargingMessage(msg);
        }
    }
    
private:
    void processBMSMessage(const CANMessage& msg) {
        switch (msg.id) {
            case EVCAN::BMS_STATUS:
                // Process BMS status data
                break;
            case EVCAN::CHARGING_STATUS:
                // Process charging status for BMS coordination
                break;
        }
    }
    
    void processMotorMessage(const CANMessage& msg) {
        switch (msg.id) {
            case EVCAN::MOTOR_CONTROLLER:
                // Process motor control commands
                break;
            case EVCAN::BMS_STATUS:
                // Process BMS data for motor control
                break;
        }
    }
    
    void processADASMessage(const CANMessage& msg) {
        switch (msg.id) {
            case EVCAN::ADAS_STATUS:
                // Process ADAS commands
                break;
            case EVCAN::VEHICLE_SPEED:
                // Process vehicle speed data
                break;
        }
    }
    
    void processChargingMessage(const CANMessage& msg) {
        switch (msg.id) {
            case EVCAN::CHARGING_STATUS:
                // Process charging status
                break;
            case EVCAN::BMS_STATUS:
                // Process BMS data for charging coordination
                break;
        }
    }
};

#endif // CAN_BUS_SYSTEM_H