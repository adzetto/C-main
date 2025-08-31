/**
 * @file advanced_configuration_management.h
 * @author adzetto
 * @brief Advanced Configuration Management System for Electric Vehicle Applications
 * @version 1.0
 * @date 2025-08-31
 * 
 * @copyright Copyright (c) 2025
 * 
 * @details This module provides comprehensive configuration management including
 * hierarchical configurations, runtime parameter updates, validation,
 * persistence, and distributed configuration synchronization.
 */

#ifndef ADVANCED_CONFIGURATION_MANAGEMENT_H
#define ADVANCED_CONFIGURATION_MANAGEMENT_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <map>
#include <queue>
#include <deque>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <functional>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <random>
#include <tuple>
#include <condition_variable>
#include <future>
#include <set>
#include <regex>
#include <variant>
#include <any>
#include <typeinfo>
#include <type_traits>

namespace configuration_management {

// Configuration types and enums
enum class ConfigValueType {
    BOOLEAN,
    INTEGER,
    DOUBLE,
    STRING,
    ARRAY,
    OBJECT,
    ENUM,
    BINARY,
    TIMESTAMP,
    DURATION,
    COMPLEX
};

enum class ConfigAccessLevel {
    PUBLIC,        // Anyone can read/write
    PROTECTED,     // Requires authentication to write
    PRIVATE,       // Only system can access
    READ_ONLY,     // Read access only
    WRITE_ONLY,    // Write access only (rare)
    ADMIN_ONLY     // Administrator access only
};

enum class ConfigValidationType {
    NONE,
    RANGE,         // Min/max values
    REGEX,         // Regular expression
    ENUM,          // One of predefined values
    CUSTOM,        // Custom validation function
    DEPENDENCY,    // Depends on other config values
    CALLBACK       // Validation callback
};

enum class ConfigPersistence {
    MEMORY_ONLY,   // Not persisted
    FILE_BASED,    // Stored in configuration files
    DATABASE,      // Stored in database
    REGISTRY,      // System registry (Windows)
    ENVIRONMENT,   // Environment variables
    REMOTE         // Remote configuration service
};

enum class ConfigEventType {
    VALUE_CHANGED,
    VALUE_ADDED,
    VALUE_REMOVED,
    VALIDATION_FAILED,
    ACCESS_DENIED,
    BACKUP_CREATED,
    RESTORE_COMPLETED,
    SYNC_STARTED,
    SYNC_COMPLETED
};

// Configuration value wrapper
class ConfigValue {
private:
    std::any value;
    ConfigValueType type;
    std::string stringRepresentation;
    std::chrono::steady_clock::time_point lastModified;
    std::chrono::steady_clock::time_point created;
    size_t version;
    
public:
    ConfigValue() : type(ConfigValueType::STRING), 
                   lastModified(std::chrono::steady_clock::now()),
                   created(std::chrono::steady_clock::now()), 
                   version(0) {}
    
    template<typename T>
    ConfigValue(const T& val) : value(val), 
                               lastModified(std::chrono::steady_clock::now()),
                               created(std::chrono::steady_clock::now()),
                               version(1) {
        if constexpr (std::is_same_v<T, bool>) {
            type = ConfigValueType::BOOLEAN;
        } else if constexpr (std::is_integral_v<T>) {
            type = ConfigValueType::INTEGER;
        } else if constexpr (std::is_floating_point_v<T>) {
            type = ConfigValueType::DOUBLE;
        } else if constexpr (std::is_same_v<T, std::string>) {
            type = ConfigValueType::STRING;
        } else {
            type = ConfigValueType::COMPLEX;
        }
        updateStringRepresentation();
    }
    
    template<typename T>
    T get() const {
        try {
            return std::any_cast<T>(value);
        } catch (const std::bad_any_cast& e) {
            throw std::runtime_error("Invalid type cast for config value");
        }
    }
    
    template<typename T>
    void set(const T& val) {
        value = val;
        lastModified = std::chrono::steady_clock::now();
        version++;
        updateStringRepresentation();
    }
    
    bool isType(ConfigValueType t) const { return type == t; }
    ConfigValueType getType() const { return type; }
    std::string toString() const { return stringRepresentation; }
    std::chrono::steady_clock::time_point getLastModified() const { return lastModified; }
    std::chrono::steady_clock::time_point getCreated() const { return created; }
    size_t getVersion() const { return version; }
    
    bool isEmpty() const { return !value.has_value(); }
    
private:
    void updateStringRepresentation() {
        switch (type) {
            case ConfigValueType::BOOLEAN:
                stringRepresentation = get<bool>() ? "true" : "false";
                break;
            case ConfigValueType::INTEGER:
                stringRepresentation = std::to_string(get<int>());
                break;
            case ConfigValueType::DOUBLE:
                stringRepresentation = std::to_string(get<double>());
                break;
            case ConfigValueType::STRING:
                stringRepresentation = get<std::string>();
                break;
            default:
                stringRepresentation = "[complex_value]";
                break;
        }
    }
};

struct ConfigMetadata {
    std::string description;
    ConfigAccessLevel accessLevel;
    ConfigValidationType validationType;
    ConfigPersistence persistence;
    std::string category;
    std::string owner;
    bool isRequired;
    bool isReadOnly;
    bool isEncrypted;
    std::any defaultValue;
    std::any validationConstraint;
    std::function<bool(const ConfigValue&)> validator;
    std::vector<std::string> dependencies;
    std::unordered_map<std::string, std::string> tags;
    
    ConfigMetadata() : accessLevel(ConfigAccessLevel::PUBLIC),
                      validationType(ConfigValidationType::NONE),
                      persistence(ConfigPersistence::MEMORY_ONLY),
                      isRequired(false), isReadOnly(false), isEncrypted(false) {}
};

struct ConfigEvent {
    ConfigEventType eventType;
    std::string configPath;
    ConfigValue oldValue;
    ConfigValue newValue;
    std::chrono::steady_clock::time_point timestamp;
    std::string userId;
    std::string description;
    
    ConfigEvent() : eventType(ConfigEventType::VALUE_CHANGED),
                   timestamp(std::chrono::steady_clock::now()) {}
};

// Configuration Node for hierarchical structure
class ConfigurationNode {
private:
    std::unordered_map<std::string, ConfigValue> values;
    std::unordered_map<std::string, ConfigMetadata> metadata;
    std::unordered_map<std::string, std::unique_ptr<ConfigurationNode>> children;
    ConfigurationNode* parent;
    std::string nodeName;
    mutable std::recursive_mutex nodeMutex;
    
public:
    ConfigurationNode(const std::string& name = "", ConfigurationNode* parentNode = nullptr) :
        parent(parentNode), nodeName(name) {}
    
    ~ConfigurationNode() = default;
    
    // Value operations
    template<typename T>
    void setValue(const std::string& key, const T& value, const ConfigMetadata& meta = ConfigMetadata()) {
        std::lock_guard<std::recursive_mutex> lock(nodeMutex);
        values[key] = ConfigValue(value);
        metadata[key] = meta;
    }
    
    template<typename T>
    T getValue(const std::string& key, const T& defaultValue = T{}) const {
        std::lock_guard<std::recursive_mutex> lock(nodeMutex);
        auto it = values.find(key);
        if (it != values.end()) {
            try {
                return it->second.get<T>();
            } catch (...) {
                return defaultValue;
            }
        }
        return defaultValue;
    }
    
    bool hasValue(const std::string& key) const {
        std::lock_guard<std::recursive_mutex> lock(nodeMutex);
        return values.find(key) != values.end();
    }
    
    void removeValue(const std::string& key) {
        std::lock_guard<std::recursive_mutex> lock(nodeMutex);
        values.erase(key);
        metadata.erase(key);
    }
    
    std::vector<std::string> getValueKeys() const {
        std::lock_guard<std::recursive_mutex> lock(nodeMutex);
        std::vector<std::string> keys;
        for (const auto& pair : values) {
            keys.push_back(pair.first);
        }
        return keys;
    }
    
    // Child node operations
    ConfigurationNode* createChildNode(const std::string& name) {
        std::lock_guard<std::recursive_mutex> lock(nodeMutex);
        children[name] = std::make_unique<ConfigurationNode>(name, this);
        return children[name].get();
    }
    
    ConfigurationNode* getChildNode(const std::string& name) const {
        std::lock_guard<std::recursive_mutex> lock(nodeMutex);
        auto it = children.find(name);
        return (it != children.end()) ? it->second.get() : nullptr;
    }
    
    void removeChildNode(const std::string& name) {
        std::lock_guard<std::recursive_mutex> lock(nodeMutex);
        children.erase(name);
    }
    
    std::vector<std::string> getChildNames() const {
        std::lock_guard<std::recursive_mutex> lock(nodeMutex);
        std::vector<std::string> names;
        for (const auto& pair : children) {
            names.push_back(pair.first);
        }
        return names;
    }
    
    // Metadata operations
    void setMetadata(const std::string& key, const ConfigMetadata& meta) {
        std::lock_guard<std::recursive_mutex> lock(nodeMutex);
        metadata[key] = meta;
    }
    
    ConfigMetadata getMetadata(const std::string& key) const {
        std::lock_guard<std::recursive_mutex> lock(nodeMutex);
        auto it = metadata.find(key);
        return (it != metadata.end()) ? it->second : ConfigMetadata();
    }
    
    // Path operations
    std::string getFullPath() const {
        if (!parent) return nodeName;
        std::string parentPath = parent->getFullPath();
        return parentPath.empty() ? nodeName : parentPath + "." + nodeName;
    }
    
    ConfigurationNode* findNodeByPath(const std::string& path) {
        if (path.empty()) return this;
        
        size_t dotPos = path.find('.');
        std::string childName = (dotPos != std::string::npos) ? path.substr(0, dotPos) : path;
        std::string remainingPath = (dotPos != std::string::npos) ? path.substr(dotPos + 1) : "";
        
        ConfigurationNode* child = getChildNode(childName);
        if (!child) return nullptr;
        
        return remainingPath.empty() ? child : child->findNodeByPath(remainingPath);
    }
    
    // Utility methods
    std::string getName() const { return nodeName; }
    ConfigurationNode* getParent() const { return parent; }
    
    size_t getValueCount() const {
        std::lock_guard<std::recursive_mutex> lock(nodeMutex);
        return values.size();
    }
    
    size_t getChildCount() const {
        std::lock_guard<std::recursive_mutex> lock(nodeMutex);
        return children.size();
    }
};

// Configuration Validator
class ConfigurationValidator {
private:
    std::unordered_map<std::string, std::function<bool(const ConfigValue&)>> customValidators;
    mutable std::mutex validatorMutex;
    
public:
    ConfigurationValidator() = default;
    ~ConfigurationValidator() = default;
    
    // Built-in validation methods
    bool validateRange(const ConfigValue& value, double min, double max) const {
        if (value.getType() == ConfigValueType::INTEGER) {
            int intVal = value.get<int>();
            return intVal >= min && intVal <= max;
        } else if (value.getType() == ConfigValueType::DOUBLE) {
            double doubleVal = value.get<double>();
            return doubleVal >= min && doubleVal <= max;
        }
        return false;
    }
    
    bool validateRegex(const ConfigValue& value, const std::string& pattern) const {
        if (value.getType() != ConfigValueType::STRING) return false;
        
        try {
            std::regex regex(pattern);
            return std::regex_match(value.get<std::string>(), regex);
        } catch (...) {
            return false;
        }
    }
    
    bool validateEnum(const ConfigValue& value, const std::vector<std::string>& allowedValues) const {
        std::string strValue = value.toString();
        return std::find(allowedValues.begin(), allowedValues.end(), strValue) != allowedValues.end();
    }
    
    bool validateDependency(const ConfigValue& value, const std::string& dependencyPath, 
                          const ConfigValue& dependencyValue, 
                          std::function<bool(const ConfigValue&, const ConfigValue&)> condition) const {
        return condition(value, dependencyValue);
    }
    
    // Custom validator registration
    void registerValidator(const std::string& name, std::function<bool(const ConfigValue&)> validator) {
        std::lock_guard<std::mutex> lock(validatorMutex);
        customValidators[name] = validator;
    }
    
    void unregisterValidator(const std::string& name) {
        std::lock_guard<std::mutex> lock(validatorMutex);
        customValidators.erase(name);
    }
    
    bool validateWithCustom(const ConfigValue& value, const std::string& validatorName) const {
        std::lock_guard<std::mutex> lock(validatorMutex);
        auto it = customValidators.find(validatorName);
        if (it != customValidators.end()) {
            return it->second(value);
        }
        return false;
    }
    
    // Main validation method
    bool validate(const ConfigValue& value, const ConfigMetadata& metadata) const {
        switch (metadata.validationType) {
            case ConfigValidationType::NONE:
                return true;
                
            case ConfigValidationType::RANGE: {
                if (metadata.validationConstraint.has_value()) {
                    try {
                        auto range = std::any_cast<std::pair<double, double>>(metadata.validationConstraint);
                        return validateRange(value, range.first, range.second);
                    } catch (...) {
                        return false;
                    }
                }
                return false;
            }
            
            case ConfigValidationType::REGEX: {
                if (metadata.validationConstraint.has_value()) {
                    try {
                        auto pattern = std::any_cast<std::string>(metadata.validationConstraint);
                        return validateRegex(value, pattern);
                    } catch (...) {
                        return false;
                    }
                }
                return false;
            }
            
            case ConfigValidationType::ENUM: {
                if (metadata.validationConstraint.has_value()) {
                    try {
                        auto enumValues = std::any_cast<std::vector<std::string>>(metadata.validationConstraint);
                        return validateEnum(value, enumValues);
                    } catch (...) {
                        return false;
                    }
                }
                return false;
            }
            
            case ConfigValidationType::CALLBACK: {
                if (metadata.validator) {
                    return metadata.validator(value);
                }
                return false;
            }
            
            default:
                return true;
        }
    }
};

// Configuration Serializer
class ConfigurationSerializer {
private:
    std::unordered_map<std::string, std::function<std::string(const ConfigurationNode*)>> serializers;
    std::unordered_map<std::string, std::function<void(ConfigurationNode*, const std::string&)>> deserializers;
    
    std::string escapeJsonString(const std::string& input) const {
        std::string output;
        for (char c : input) {
            switch (c) {
                case '"': output += "\\\""; break;
                case '\\': output += "\\\\"; break;
                case '\n': output += "\\n"; break;
                case '\r': output += "\\r"; break;
                case '\t': output += "\\t"; break;
                default: output += c; break;
            }
        }
        return output;
    }
    
public:
    ConfigurationSerializer() {
        // Register built-in serializers
        registerSerializer("json", [this](const ConfigurationNode* node) { return serializeToJSON(node); });
        registerSerializer("xml", [this](const ConfigurationNode* node) { return serializeToXML(node); });
        registerSerializer("ini", [this](const ConfigurationNode* node) { return serializeToINI(node); });
        registerSerializer("yaml", [this](const ConfigurationNode* node) { return serializeToYAML(node); });
        
        // Register built-in deserializers
        registerDeserializer("json", [this](ConfigurationNode* node, const std::string& data) { deserializeFromJSON(node, data); });
        registerDeserializer("xml", [this](ConfigurationNode* node, const std::string& data) { deserializeFromXML(node, data); });
        registerDeserializer("ini", [this](ConfigurationNode* node, const std::string& data) { deserializeFromINI(node, data); });
        registerDeserializer("yaml", [this](ConfigurationNode* node, const std::string& data) { deserializeFromYAML(node, data); });
    }
    
    void registerSerializer(const std::string& format, std::function<std::string(const ConfigurationNode*)> serializer) {
        serializers[format] = serializer;
    }
    
    void registerDeserializer(const std::string& format, std::function<void(ConfigurationNode*, const std::string&)> deserializer) {
        deserializers[format] = deserializer;
    }
    
    std::string serialize(const ConfigurationNode* node, const std::string& format) const {
        auto it = serializers.find(format);
        if (it != serializers.end()) {
            return it->second(node);
        }
        throw std::runtime_error("Unknown serialization format: " + format);
    }
    
    void deserialize(ConfigurationNode* node, const std::string& data, const std::string& format) {
        auto it = deserializers.find(format);
        if (it != deserializers.end()) {
            it->second(node, data);
            return;
        }
        throw std::runtime_error("Unknown deserialization format: " + format);
    }
    
private:
    std::string serializeToJSON(const ConfigurationNode* node) const {
        std::ostringstream json;
        json << "{\n";
        
        auto valueKeys = node->getValueKeys();
        bool firstValue = true;
        
        for (const auto& key : valueKeys) {
            if (!firstValue) json << ",\n";
            firstValue = false;
            
            auto value = node->getValue<std::string>(key);  // Simplified - get as string
            json << "  \"" << escapeJsonString(key) << "\": \"" << escapeJsonString(value) << "\"";
        }
        
        auto childNames = node->getChildNames();
        for (const auto& childName : childNames) {
            if (!firstValue) json << ",\n";
            firstValue = false;
            
            auto childNode = node->getChildNode(childName);
            json << "  \"" << escapeJsonString(childName) << "\": " << serializeToJSON(childNode);
        }
        
        json << "\n}";
        return json.str();
    }
    
    std::string serializeToXML(const ConfigurationNode* node) const {
        std::ostringstream xml;
        xml << "<configuration>\n";
        
        auto valueKeys = node->getValueKeys();
        for (const auto& key : valueKeys) {
            auto value = node->getValue<std::string>(key);
            xml << "  <" << key << ">" << value << "</" << key << ">\n";
        }
        
        auto childNames = node->getChildNames();
        for (const auto& childName : childNames) {
            auto childNode = node->getChildNode(childName);
            xml << "  <section name=\"" << childName << "\">\n";
            // Recursively serialize child (simplified)
            xml << "  </section>\n";
        }
        
        xml << "</configuration>\n";
        return xml.str();
    }
    
    std::string serializeToINI(const ConfigurationNode* node) const {
        std::ostringstream ini;
        
        // Write values in default section
        auto valueKeys = node->getValueKeys();
        for (const auto& key : valueKeys) {
            auto value = node->getValue<std::string>(key);
            ini << key << "=" << value << "\n";
        }
        
        // Write child sections
        auto childNames = node->getChildNames();
        for (const auto& childName : childNames) {
            ini << "\n[" << childName << "]\n";
            auto childNode = node->getChildNode(childName);
            auto childKeys = childNode->getValueKeys();
            for (const auto& key : childKeys) {
                auto value = childNode->getValue<std::string>(key);
                ini << key << "=" << value << "\n";
            }
        }
        
        return ini.str();
    }
    
    std::string serializeToYAML(const ConfigurationNode* node) const {
        // Simplified YAML serialization
        std::ostringstream yaml;
        
        auto valueKeys = node->getValueKeys();
        for (const auto& key : valueKeys) {
            auto value = node->getValue<std::string>(key);
            yaml << key << ": " << value << "\n";
        }
        
        auto childNames = node->getChildNames();
        for (const auto& childName : childNames) {
            yaml << childName << ":\n";
            auto childNode = node->getChildNode(childName);
            auto childKeys = childNode->getValueKeys();
            for (const auto& key : childKeys) {
                auto value = childNode->getValue<std::string>(key);
                yaml << "  " << key << ": " << value << "\n";
            }
        }
        
        return yaml.str();
    }
    
    // Simplified deserializers (full implementations would require proper parsing)
    void deserializeFromJSON(ConfigurationNode* node, const std::string& data) {
        // Simplified JSON parsing - would need proper JSON parser
        // For now, just a basic key-value parsing
        std::istringstream stream(data);
        std::string line;
        while (std::getline(stream, line)) {
            size_t colonPos = line.find(':');
            if (colonPos != std::string::npos) {
                std::string key = line.substr(0, colonPos);
                std::string value = line.substr(colonPos + 1);
                // Remove quotes and whitespace
                key.erase(std::remove(key.begin(), key.end(), '"'), key.end());
                key.erase(std::remove(key.begin(), key.end(), ' '), key.end());
                value.erase(std::remove(value.begin(), value.end(), '"'), value.end());
                value.erase(std::remove(value.begin(), value.end(), ' '), value.end());
                value.erase(std::remove(value.begin(), value.end(), ','), value.end());
                
                if (!key.empty() && !value.empty()) {
                    node->setValue(key, value);
                }
            }
        }
    }
    
    void deserializeFromXML(ConfigurationNode* node, const std::string& data) {
        // Simplified XML parsing
        // Implementation would require proper XML parser
    }
    
    void deserializeFromINI(ConfigurationNode* node, const std::string& data) {
        std::istringstream stream(data);
        std::string line;
        ConfigurationNode* currentNode = node;
        
        while (std::getline(stream, line)) {
            // Trim whitespace
            line.erase(0, line.find_first_not_of(" \t"));
            line.erase(line.find_last_not_of(" \t") + 1);
            
            if (line.empty() || line[0] == '#' || line[0] == ';') {
                continue; // Skip empty lines and comments
            }
            
            if (line[0] == '[' && line.back() == ']') {
                // Section header
                std::string sectionName = line.substr(1, line.length() - 2);
                currentNode = node->createChildNode(sectionName);
            } else {
                // Key-value pair
                size_t equalPos = line.find('=');
                if (equalPos != std::string::npos) {
                    std::string key = line.substr(0, equalPos);
                    std::string value = line.substr(equalPos + 1);
                    
                    // Trim key and value
                    key.erase(0, key.find_first_not_of(" \t"));
                    key.erase(key.find_last_not_of(" \t") + 1);
                    value.erase(0, value.find_first_not_of(" \t"));
                    value.erase(value.find_last_not_of(" \t") + 1);
                    
                    currentNode->setValue(key, value);
                }
            }
        }
    }
    
    void deserializeFromYAML(ConfigurationNode* node, const std::string& data) {
        // Simplified YAML parsing
        // Implementation would require proper YAML parser
    }
};

// Configuration Event Manager
class ConfigurationEventManager {
private:
    std::vector<ConfigEvent> eventHistory;
    std::unordered_map<ConfigEventType, std::vector<std::function<void(const ConfigEvent&)>>> eventCallbacks;
    mutable std::mutex eventMutex;
    size_t maxEventHistory;
    
public:
    ConfigurationEventManager(size_t maxHistory = 10000) : maxEventHistory(maxHistory) {}
    
    void publishEvent(const ConfigEvent& event) {
        {
            std::lock_guard<std::mutex> lock(eventMutex);
            eventHistory.push_back(event);
            
            // Maintain history size limit
            if (eventHistory.size() > maxEventHistory) {
                eventHistory.erase(eventHistory.begin());
            }
        }
        
        // Trigger callbacks
        auto callbackIt = eventCallbacks.find(event.eventType);
        if (callbackIt != eventCallbacks.end()) {
            for (const auto& callback : callbackIt->second) {
                try {
                    callback(event);
                } catch (...) {
                    // Log callback error but continue
                }
            }
        }
    }
    
    void registerEventCallback(ConfigEventType eventType, std::function<void(const ConfigEvent&)> callback) {
        std::lock_guard<std::mutex> lock(eventMutex);
        eventCallbacks[eventType].push_back(callback);
    }
    
    void unregisterEventCallbacks(ConfigEventType eventType) {
        std::lock_guard<std::mutex> lock(eventMutex);
        eventCallbacks[eventType].clear();
    }
    
    std::vector<ConfigEvent> getEventHistory(ConfigEventType eventType = ConfigEventType::VALUE_CHANGED) const {
        std::lock_guard<std::mutex> lock(eventMutex);
        std::vector<ConfigEvent> filteredEvents;
        
        for (const auto& event : eventHistory) {
            if (event.eventType == eventType) {
                filteredEvents.push_back(event);
            }
        }
        
        return filteredEvents;
    }
    
    void clearEventHistory() {
        std::lock_guard<std::mutex> lock(eventMutex);
        eventHistory.clear();
    }
};

// Main Advanced Configuration Management System
class AdvancedConfigurationManagementSystem {
private:
    std::unique_ptr<ConfigurationNode> rootNode;
    std::unique_ptr<ConfigurationValidator> validator;
    std::unique_ptr<ConfigurationSerializer> serializer;
    std::unique_ptr<ConfigurationEventManager> eventManager;
    
    mutable std::recursive_mutex systemMutex;
    std::atomic<bool> isSystemRunning{false};
    std::thread configurationThread;
    
    struct ConfigurationSystemConfig {
        bool enableValidation;
        bool enableEventTracking;
        bool enableAutoSave;
        bool enableEncryption;
        bool enableBackups;
        std::string defaultPersistenceFormat;
        std::chrono::milliseconds autoSaveInterval;
        std::chrono::milliseconds backupInterval;
        size_t maxBackupVersions;
        std::string configurationDirectory;
        
        ConfigurationSystemConfig() : enableValidation(true), enableEventTracking(true),
                                    enableAutoSave(true), enableEncryption(false),
                                    enableBackups(true), defaultPersistenceFormat("json"),
                                    autoSaveInterval(std::chrono::milliseconds(30000)),
                                    backupInterval(std::chrono::milliseconds(300000)),
                                    maxBackupVersions(10), configurationDirectory("./config/") {}
    } systemConfig;
    
    void configurationWorker();
    void performAutoSave();
    void performAutoBackup();
    void maintainBackupVersions();
    
    std::string getCurrentUserId() const { return "system"; } // Simplified user identification
    
public:
    AdvancedConfigurationManagementSystem();
    ~AdvancedConfigurationManagementSystem();
    
    // System control
    void start();
    void stop();
    bool isRunning() const { return isSystemRunning.load(); }
    
    // Configuration management
    template<typename T>
    void setConfigValue(const std::string& path, const T& value, const ConfigMetadata& metadata = ConfigMetadata()) {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);
        
        // Parse path to find target node
        size_t lastDotPos = path.find_last_of('.');
        std::string nodePath = (lastDotPos != std::string::npos) ? path.substr(0, lastDotPos) : "";
        std::string key = (lastDotPos != std::string::npos) ? path.substr(lastDotPos + 1) : path;
        
        // Find or create target node
        ConfigurationNode* targetNode = rootNode.get();
        if (!nodePath.empty()) {
            targetNode = findOrCreateNodeByPath(nodePath);
        }
        
        // Get old value for event
        ConfigValue oldValue;
        if (targetNode->hasValue(key)) {
            oldValue = ConfigValue(targetNode->getValue<T>(key));
        }
        
        ConfigValue newValue(value);
        
        // Validate if enabled
        if (systemConfig.enableValidation && validator) {
            if (!validator->validate(newValue, metadata)) {
                if (systemConfig.enableEventTracking) {
                    ConfigEvent event;
                    event.eventType = ConfigEventType::VALIDATION_FAILED;
                    event.configPath = path;
                    event.newValue = newValue;
                    event.userId = getCurrentUserId();
                    event.description = "Validation failed for config value";
                    eventManager->publishEvent(event);
                }
                throw std::runtime_error("Configuration validation failed for path: " + path);
            }
        }
        
        // Set the value
        targetNode->setValue(key, value, metadata);
        
        // Publish event if enabled
        if (systemConfig.enableEventTracking) {
            ConfigEvent event;
            event.eventType = targetNode->hasValue(key) ? ConfigEventType::VALUE_CHANGED : ConfigEventType::VALUE_ADDED;
            event.configPath = path;
            event.oldValue = oldValue;
            event.newValue = newValue;
            event.userId = getCurrentUserId();
            event.description = "Configuration value updated";
            eventManager->publishEvent(event);
        }
    }
    
    template<typename T>
    T getConfigValue(const std::string& path, const T& defaultValue = T{}) const {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);
        
        size_t lastDotPos = path.find_last_of('.');
        std::string nodePath = (lastDotPos != std::string::npos) ? path.substr(0, lastDotPos) : "";
        std::string key = (lastDotPos != std::string::npos) ? path.substr(lastDotPos + 1) : path;
        
        ConfigurationNode* targetNode = rootNode.get();
        if (!nodePath.empty()) {
            targetNode = rootNode->findNodeByPath(nodePath);
            if (!targetNode) return defaultValue;
        }
        
        return targetNode->getValue<T>(key, defaultValue);
    }
    
    bool hasConfigValue(const std::string& path) const {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);
        
        size_t lastDotPos = path.find_last_of('.');
        std::string nodePath = (lastDotPos != std::string::npos) ? path.substr(0, lastDotPos) : "";
        std::string key = (lastDotPos != std::string::npos) ? path.substr(lastDotPos + 1) : path;
        
        ConfigurationNode* targetNode = rootNode.get();
        if (!nodePath.empty()) {
            targetNode = rootNode->findNodeByPath(nodePath);
            if (!targetNode) return false;
        }
        
        return targetNode->hasValue(key);
    }
    
    void removeConfigValue(const std::string& path) {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);
        
        size_t lastDotPos = path.find_last_of('.');
        std::string nodePath = (lastDotPos != std::string::npos) ? path.substr(0, lastDotPos) : "";
        std::string key = (lastDotPos != std::string::npos) ? path.substr(lastDotPos + 1) : path;
        
        ConfigurationNode* targetNode = rootNode.get();
        if (!nodePath.empty()) {
            targetNode = rootNode->findNodeByPath(nodePath);
            if (!targetNode) return;
        }
        
        if (targetNode->hasValue(key)) {
            ConfigValue oldValue(targetNode->getValue<std::string>(key));
            targetNode->removeValue(key);
            
            if (systemConfig.enableEventTracking) {
                ConfigEvent event;
                event.eventType = ConfigEventType::VALUE_REMOVED;
                event.configPath = path;
                event.oldValue = oldValue;
                event.userId = getCurrentUserId();
                event.description = "Configuration value removed";
                eventManager->publishEvent(event);
            }
        }
    }
    
    // Section management
    void createConfigSection(const std::string& path) {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);
        findOrCreateNodeByPath(path);
    }
    
    void removeConfigSection(const std::string& path) {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);
        
        size_t lastDotPos = path.find_last_of('.');
        std::string parentPath = (lastDotPos != std::string::npos) ? path.substr(0, lastDotPos) : "";
        std::string sectionName = (lastDotPos != std::string::npos) ? path.substr(lastDotPos + 1) : path;
        
        ConfigurationNode* parentNode = rootNode.get();
        if (!parentPath.empty()) {
            parentNode = rootNode->findNodeByPath(parentPath);
            if (!parentNode) return;
        }
        
        parentNode->removeChildNode(sectionName);
    }
    
    std::vector<std::string> getConfigSections(const std::string& path = "") const {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);
        
        ConfigurationNode* targetNode = rootNode.get();
        if (!path.empty()) {
            targetNode = rootNode->findNodeByPath(path);
            if (!targetNode) return {};
        }
        
        return targetNode->getChildNames();
    }
    
    std::vector<std::string> getConfigKeys(const std::string& path = "") const {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);
        
        ConfigurationNode* targetNode = rootNode.get();
        if (!path.empty()) {
            targetNode = rootNode->findNodeByPath(path);
            if (!targetNode) return {};
        }
        
        return targetNode->getValueKeys();
    }
    
    // Persistence operations
    void saveConfiguration(const std::string& filePath = "", const std::string& format = "") {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);
        
        std::string actualPath = filePath.empty() ? 
            (systemConfig.configurationDirectory + "config." + systemConfig.defaultPersistenceFormat) : 
            filePath;
        
        std::string actualFormat = format.empty() ? systemConfig.defaultPersistenceFormat : format;
        
        try {
            std::string serializedData = serializer->serialize(rootNode.get(), actualFormat);
            
            std::ofstream file(actualPath);
            if (!file.is_open()) {
                throw std::runtime_error("Cannot open file for writing: " + actualPath);
            }
            
            file << serializedData;
            file.close();
            
            if (systemConfig.enableEventTracking) {
                ConfigEvent event;
                event.eventType = ConfigEventType::BACKUP_CREATED;
                event.configPath = actualPath;
                event.userId = getCurrentUserId();
                event.description = "Configuration saved to file";
                eventManager->publishEvent(event);
            }
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to save configuration: " + std::string(e.what()));
        }
    }
    
    void loadConfiguration(const std::string& filePath, const std::string& format = "") {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);
        
        std::ifstream file(filePath);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open configuration file: " + filePath);
        }
        
        std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        file.close();
        
        std::string actualFormat = format.empty() ? systemConfig.defaultPersistenceFormat : format;
        
        try {
            // Create a new root node for loading
            auto newRootNode = std::make_unique<ConfigurationNode>("root");
            serializer->deserialize(newRootNode.get(), data, actualFormat);
            
            // Replace current root node
            rootNode = std::move(newRootNode);
            
            if (systemConfig.enableEventTracking) {
                ConfigEvent event;
                event.eventType = ConfigEventType::RESTORE_COMPLETED;
                event.configPath = filePath;
                event.userId = getCurrentUserId();
                event.description = "Configuration loaded from file";
                eventManager->publishEvent(event);
            }
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to load configuration: " + std::string(e.what()));
        }
    }
    
    // Event management
    void registerEventCallback(ConfigEventType eventType, std::function<void(const ConfigEvent&)> callback) {
        eventManager->registerEventCallback(eventType, callback);
    }
    
    void unregisterEventCallbacks(ConfigEventType eventType) {
        eventManager->unregisterEventCallbacks(eventType);
    }
    
    std::vector<ConfigEvent> getEventHistory(ConfigEventType eventType = ConfigEventType::VALUE_CHANGED) const {
        return eventManager->getEventHistory(eventType);
    }
    
    // Validation
    void registerValidator(const std::string& name, std::function<bool(const ConfigValue&)> validatorFunc) {
        validator->registerValidator(name, validatorFunc);
    }
    
    void unregisterValidator(const std::string& name) {
        validator->unregisterValidator(name);
    }
    
    // System configuration
    void enableValidation(bool enable) { systemConfig.enableValidation = enable; }
    void enableEventTracking(bool enable) { systemConfig.enableEventTracking = enable; }
    void enableAutoSave(bool enable) { systemConfig.enableAutoSave = enable; }
    void enableEncryption(bool enable) { systemConfig.enableEncryption = enable; }
    void enableBackups(bool enable) { systemConfig.enableBackups = enable; }
    void setDefaultFormat(const std::string& format) { systemConfig.defaultPersistenceFormat = format; }
    void setAutoSaveInterval(std::chrono::milliseconds interval) { systemConfig.autoSaveInterval = interval; }
    void setBackupInterval(std::chrono::milliseconds interval) { systemConfig.backupInterval = interval; }
    void setConfigurationDirectory(const std::string& directory) { systemConfig.configurationDirectory = directory; }
    
    // Backup and restore
    void createBackup(const std::string& backupName = "") {
        std::string actualBackupName = backupName.empty() ? 
            "backup_" + std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::system_clock::now().time_since_epoch()).count()) : backupName;
        
        std::string backupPath = systemConfig.configurationDirectory + actualBackupName + "." + systemConfig.defaultPersistenceFormat;
        saveConfiguration(backupPath);
    }
    
    void restoreFromBackup(const std::string& backupPath) {
        loadConfiguration(backupPath);
    }
    
    std::vector<std::string> getAvailableBackups() const {
        std::vector<std::string> backups;
        // In a full implementation, this would scan the backup directory
        // For now, return empty vector
        return backups;
    }
    
    // Import/Export
    std::string exportConfiguration(const std::string& format = "json") const {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);
        return serializer->serialize(rootNode.get(), format);
    }
    
    void importConfiguration(const std::string& data, const std::string& format = "json", bool merge = false) {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);
        
        if (merge) {
            // In a full implementation, this would merge the configurations
            serializer->deserialize(rootNode.get(), data, format);
        } else {
            auto newRootNode = std::make_unique<ConfigurationNode>("root");
            serializer->deserialize(newRootNode.get(), data, format);
            rootNode = std::move(newRootNode);
        }
    }
    
    // System status and diagnostics
    std::string getSystemStatus() const {
        std::ostringstream status;
        status << "=== Advanced Configuration Management System Status ===\n";
        status << "System Running: " << (isSystemRunning.load() ? "Yes" : "No") << "\n";
        status << "Validation Enabled: " << (systemConfig.enableValidation ? "Yes" : "No") << "\n";
        status << "Event Tracking Enabled: " << (systemConfig.enableEventTracking ? "Yes" : "No") << "\n";
        status << "Auto-save Enabled: " << (systemConfig.enableAutoSave ? "Yes" : "No") << "\n";
        status << "Encryption Enabled: " << (systemConfig.enableEncryption ? "Yes" : "No") << "\n";
        status << "Backups Enabled: " << (systemConfig.enableBackups ? "Yes" : "No") << "\n";
        status << "Default Format: " << systemConfig.defaultPersistenceFormat << "\n";
        status << "Auto-save Interval: " << systemConfig.autoSaveInterval.count() << " ms\n";
        status << "Backup Interval: " << systemConfig.backupInterval.count() << " ms\n";
        status << "Configuration Directory: " << systemConfig.configurationDirectory << "\n";
        
        // Configuration statistics
        status << "Total Configuration Nodes: " << countTotalNodes(rootNode.get()) << "\n";
        status << "Total Configuration Values: " << countTotalValues(rootNode.get()) << "\n";
        
        return status.str();
    }
    
    // Utility methods
    void clearAllConfiguration() {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);
        rootNode = std::make_unique<ConfigurationNode>("root");
        eventManager->clearEventHistory();
    }
    
    size_t getConfigurationSize() const {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);
        return countTotalValues(rootNode.get());
    }
    
private:
    ConfigurationNode* findOrCreateNodeByPath(const std::string& path) {
        if (path.empty()) return rootNode.get();
        
        std::vector<std::string> pathComponents;
        std::istringstream pathStream(path);
        std::string component;
        
        while (std::getline(pathStream, component, '.')) {
            pathComponents.push_back(component);
        }
        
        ConfigurationNode* currentNode = rootNode.get();
        for (const auto& component : pathComponents) {
            ConfigurationNode* childNode = currentNode->getChildNode(component);
            if (!childNode) {
                childNode = currentNode->createChildNode(component);
            }
            currentNode = childNode;
        }
        
        return currentNode;
    }
    
    size_t countTotalNodes(const ConfigurationNode* node) const {
        size_t count = 1; // Count this node
        auto childNames = node->getChildNames();
        for (const auto& childName : childNames) {
            auto childNode = node->getChildNode(childName);
            if (childNode) {
                count += countTotalNodes(childNode);
            }
        }
        return count;
    }
    
    size_t countTotalValues(const ConfigurationNode* node) const {
        size_t count = node->getValueCount();
        auto childNames = node->getChildNames();
        for (const auto& childName : childNames) {
            auto childNode = node->getChildNode(childName);
            if (childNode) {
                count += countTotalValues(childNode);
            }
        }
        return count;
    }
};

} // namespace configuration_management

// Implementation of critical inline methods

inline configuration_management::AdvancedConfigurationManagementSystem::AdvancedConfigurationManagementSystem()
    : rootNode(std::make_unique<ConfigurationNode>("root"))
    , validator(std::make_unique<ConfigurationValidator>())
    , serializer(std::make_unique<ConfigurationSerializer>())
    , eventManager(std::make_unique<ConfigurationEventManager>()) {
}

inline configuration_management::AdvancedConfigurationManagementSystem::~AdvancedConfigurationManagementSystem() {
    stop();
}

inline void configuration_management::AdvancedConfigurationManagementSystem::start() {
    if (isSystemRunning.load()) return;
    
    isSystemRunning.store(true);
    
    configurationThread = std::thread(&AdvancedConfigurationManagementSystem::configurationWorker, this);
    
    std::cout << "[AdvancedConfigurationManagement] System started successfully\n";
}

inline void configuration_management::AdvancedConfigurationManagementSystem::stop() {
    if (!isSystemRunning.load()) return;
    
    isSystemRunning.store(false);
    
    // Perform final auto-save if enabled
    if (systemConfig.enableAutoSave) {
        try {
            saveConfiguration();
        } catch (...) {
            // Ignore save errors during shutdown
        }
    }
    
    if (configurationThread.joinable()) {
        configurationThread.join();
    }
    
    std::cout << "[AdvancedConfigurationManagement] System stopped successfully\n";
}

inline void configuration_management::AdvancedConfigurationManagementSystem::configurationWorker() {
    auto lastAutoSave = std::chrono::steady_clock::now();
    auto lastBackup = std::chrono::steady_clock::now();
    
    while (isSystemRunning.load()) {
        auto now = std::chrono::steady_clock::now();
        
        // Auto-save if enabled and interval has passed
        if (systemConfig.enableAutoSave && 
            (now - lastAutoSave) >= systemConfig.autoSaveInterval) {
            try {
                saveConfiguration();
                lastAutoSave = now;
            } catch (...) {
                // Log error but continue
            }
        }
        
        // Auto-backup if enabled and interval has passed
        if (systemConfig.enableBackups && 
            (now - lastBackup) >= systemConfig.backupInterval) {
            try {
                createBackup();
                lastBackup = now;
                maintainBackupVersions();
            } catch (...) {
                // Log error but continue
            }
        }
        
        // Sleep for a short interval
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

inline void configuration_management::AdvancedConfigurationManagementSystem::maintainBackupVersions() {
    // In a full implementation, this would clean up old backup files
    // to maintain the maximum number of backup versions
}

// Additional methods already implemented above

#endif // ADVANCED_CONFIGURATION_MANAGEMENT_H