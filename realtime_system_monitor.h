/**
 * @file realtime_system_monitor.h
 * @author adzetto
 * @brief Advanced Real-Time System Monitor for Electric Vehicle Applications
 * @version 1.0
 * @date 2025-08-31
 * 
 * @copyright Copyright (c) 2025
 * 
 * @details This module provides comprehensive real-time system monitoring capabilities
 * including CPU, memory, network, I/O, temperature, and EV-specific metrics monitoring.
 * Features advanced alerting, historical data collection, and performance analytics.
 */

#ifndef REALTIME_SYSTEM_MONITOR_H
#define REALTIME_SYSTEM_MONITOR_H

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <map>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <functional>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <sstream>
#include <regex>
#include <future>

namespace system_monitor {

// Forward declarations
class SystemResourceMonitor;
class PerformanceAnalyzer;
class AlertManager;
class MetricsCollector;
class SystemProfiler;
class ResourcePredictor;
class TelemetryExporter;

enum class ResourceType {
    CPU_USAGE,
    MEMORY_USAGE,
    DISK_USAGE,
    NETWORK_BANDWIDTH,
    TEMPERATURE,
    BATTERY_LEVEL,
    CHARGING_RATE,
    MOTOR_TEMPERATURE,
    INVERTER_TEMPERATURE,
    COOLANT_TEMPERATURE,
    TIRE_PRESSURE,
    BRAKE_TEMPERATURE,
    SYSTEM_LOAD,
    THREAD_COUNT,
    FILE_DESCRIPTORS,
    NETWORK_CONNECTIONS
};

enum class AlertLevel {
    INFO,
    WARNING,
    CRITICAL,
    EMERGENCY
};

enum class MonitoringMode {
    REAL_TIME,
    BATCH,
    HYBRID,
    LOW_POWER
};

enum class AggregationType {
    AVERAGE,
    MIN,
    MAX,
    SUM,
    COUNT,
    PERCENTILE_95,
    PERCENTILE_99,
    STANDARD_DEVIATION
};

enum class ExportFormat {
    JSON,
    CSV,
    XML,
    BINARY,
    PROMETHEUS,
    GRAFANA
};

struct SystemMetric {
    ResourceType type;
    double value;
    std::string unit;
    std::chrono::steady_clock::time_point timestamp;
    std::string source;
    std::unordered_map<std::string, std::string> tags;
    double confidence;
    
    SystemMetric() : value(0.0), timestamp(std::chrono::steady_clock::now()), confidence(1.0) {}
    SystemMetric(ResourceType t, double v, const std::string& u) :
        type(t), value(v), unit(u), timestamp(std::chrono::steady_clock::now()), confidence(1.0) {}
};

struct AlertRule {
    std::string name;
    ResourceType resourceType;
    double threshold;
    AlertLevel level;
    std::function<bool(const SystemMetric&)> condition;
    std::chrono::seconds cooldownPeriod;
    std::chrono::steady_clock::time_point lastTriggered;
    bool enabled;
    std::string description;
    
    AlertRule() : threshold(0.0), level(AlertLevel::WARNING), 
                  cooldownPeriod(std::chrono::seconds(60)), enabled(true) {}
};

struct SystemAlert {
    std::string alertId;
    AlertRule rule;
    SystemMetric metric;
    std::chrono::steady_clock::time_point timestamp;
    AlertLevel severity;
    std::string message;
    bool acknowledged;
    std::string acknowledgedBy;
    
    SystemAlert() : timestamp(std::chrono::steady_clock::now()), 
                    severity(AlertLevel::INFO), acknowledged(false) {}
};

struct PerformanceSnapshot {
    std::chrono::steady_clock::time_point timestamp;
    std::unordered_map<ResourceType, SystemMetric> metrics;
    double overallScore;
    std::vector<std::string> recommendations;
    
    PerformanceSnapshot() : timestamp(std::chrono::steady_clock::now()), overallScore(0.0) {}
};

struct SystemProfile {
    std::string profileName;
    std::unordered_map<ResourceType, double> baselineValues;
    std::unordered_map<ResourceType, double> thresholds;
    MonitoringMode mode;
    std::chrono::milliseconds samplingInterval;
    bool adaptiveThresholds;
    
    SystemProfile() : mode(MonitoringMode::REAL_TIME), 
                      samplingInterval(std::chrono::milliseconds(1000)),
                      adaptiveThresholds(false) {}
};

struct PredictionModel {
    ResourceType targetResource;
    std::vector<double> historicalValues;
    std::chrono::minutes predictionHorizon;
    double accuracy;
    std::chrono::steady_clock::time_point lastUpdate;
    std::string algorithmType;
    
    PredictionModel() : predictionHorizon(std::chrono::minutes(30)), accuracy(0.0),
                        lastUpdate(std::chrono::steady_clock::now()) {}
};

class SystemResourceMonitor {
private:
    std::atomic<bool> monitoringActive;
    std::thread monitoringThread;
    std::mutex metricsMutex;
    std::condition_variable metricsCondition;
    
    std::unordered_map<ResourceType, std::queue<SystemMetric>> metricsHistory;
    std::unordered_map<ResourceType, SystemMetric> currentMetrics;
    std::chrono::milliseconds samplingInterval;
    size_t maxHistorySize;
    
    // Platform-specific resource readers
    std::unordered_map<ResourceType, std::function<SystemMetric()>> resourceReaders;
    
public:
    SystemResourceMonitor(std::chrono::milliseconds interval = std::chrono::milliseconds(1000));
    ~SystemResourceMonitor();
    
    void start();
    void stop();
    bool isActive() const { return monitoringActive.load(); }
    
    void setSamplingInterval(std::chrono::milliseconds interval) { samplingInterval = interval; }
    void setMaxHistorySize(size_t size) { maxHistorySize = size; }
    
    SystemMetric getCurrentMetric(ResourceType type) const;
    std::vector<SystemMetric> getMetricHistory(ResourceType type, size_t count = 100) const;
    std::unordered_map<ResourceType, SystemMetric> getAllCurrentMetrics() const;
    
    void registerCustomResource(ResourceType type, std::function<SystemMetric()> reader);
    void unregisterCustomResource(ResourceType type);
    
    double getAverageValue(ResourceType type, std::chrono::minutes duration) const;
    double getPeakValue(ResourceType type, std::chrono::minutes duration) const;
    std::vector<double> getTrendData(ResourceType type, size_t points) const;
    
private:
    void monitoringLoop();
    void collectSystemMetrics();
    SystemMetric readCPUUsage();
    SystemMetric readMemoryUsage();
    SystemMetric readDiskUsage();
    SystemMetric readNetworkBandwidth();
    SystemMetric readSystemTemperature();
    SystemMetric readBatteryLevel();
    SystemMetric readChargingRate();
    SystemMetric readMotorTemperature();
    void updateMetricHistory(const SystemMetric& metric);
};

class PerformanceAnalyzer {
private:
    std::unique_ptr<SystemResourceMonitor> resourceMonitor;
    std::vector<PerformanceSnapshot> snapshots;
    std::mutex analysisMutex;
    
    // Analysis algorithms
    std::unordered_map<std::string, std::function<double(const PerformanceSnapshot&)>> scoringFunctions;
    std::unordered_map<ResourceType, double> resourceWeights;
    
public:
    PerformanceAnalyzer(std::unique_ptr<SystemResourceMonitor> monitor);
    ~PerformanceAnalyzer();
    
    void startAnalysis();
    void stopAnalysis();
    
    PerformanceSnapshot createSnapshot();
    std::vector<PerformanceSnapshot> getRecentSnapshots(size_t count = 10) const;
    
    double calculateOverallScore(const PerformanceSnapshot& snapshot) const;
    std::vector<std::string> generateRecommendations(const PerformanceSnapshot& snapshot) const;
    
    void setResourceWeight(ResourceType type, double weight);
    double getResourceWeight(ResourceType type) const;
    
    // Trend analysis
    bool detectPerformanceDegradation(std::chrono::minutes lookbackPeriod) const;
    std::vector<ResourceType> identifyBottlenecks() const;
    double calculateEfficiencyScore() const;
    
    // Comparative analysis
    int compareWithBaseline(const SystemProfile& baseline) const;
    std::vector<ResourceType> findAnomalies(double threshold = 2.0) const;
    
private:
    double calculateCPUScore(const PerformanceSnapshot& snapshot) const;
    double calculateMemoryScore(const PerformanceSnapshot& snapshot) const;
    double calculateNetworkScore(const PerformanceSnapshot& snapshot) const;
    double calculateThermalScore(const PerformanceSnapshot& snapshot) const;
};

class AlertManager {
private:
    std::vector<AlertRule> rules;
    std::queue<SystemAlert> alertQueue;
    std::vector<SystemAlert> alertHistory;
    std::mutex alertMutex;
    std::condition_variable alertCondition;
    
    std::thread alertProcessingThread;
    std::atomic<bool> processingActive;
    
    // Alert handlers
    std::unordered_map<AlertLevel, std::vector<std::function<void(const SystemAlert&)>>> alertHandlers;
    
    // Rate limiting
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> lastAlertTimes;
    
public:
    AlertManager();
    ~AlertManager();
    
    void start();
    void stop();
    
    void addRule(const AlertRule& rule);
    void removeRule(const std::string& ruleName);
    void updateRule(const std::string& ruleName, const AlertRule& newRule);
    std::vector<AlertRule> getAllRules() const;
    
    void checkMetric(const SystemMetric& metric);
    void processAlerts();
    
    void addAlertHandler(AlertLevel level, std::function<void(const SystemAlert&)> handler);
    void removeAlertHandlers(AlertLevel level);
    
    std::vector<SystemAlert> getActiveAlerts() const;
    std::vector<SystemAlert> getAlertHistory(size_t count = 100) const;
    
    void acknowledgeAlert(const std::string& alertId, const std::string& user);
    void clearAcknowledgedAlerts();
    
    // Predefined rule templates
    void addCPUThresholdRule(double threshold, AlertLevel level = AlertLevel::WARNING);
    void addMemoryThresholdRule(double threshold, AlertLevel level = AlertLevel::WARNING);
    void addTemperatureThresholdRule(double threshold, AlertLevel level = AlertLevel::CRITICAL);
    void addBatteryLowRule(double threshold, AlertLevel level = AlertLevel::WARNING);
    
private:
    void alertProcessingLoop();
    void triggerAlert(const AlertRule& rule, const SystemMetric& metric);
    bool isRateLimited(const std::string& ruleName) const;
    std::string generateAlertId() const;
};

class MetricsCollector {
private:
    std::unique_ptr<SystemResourceMonitor> resourceMonitor;
    std::unordered_map<ResourceType, std::vector<SystemMetric>> aggregatedMetrics;
    std::mutex collectorMutex;
    
    // Aggregation settings
    std::chrono::seconds aggregationInterval;
    std::unordered_map<ResourceType, AggregationType> aggregationTypes;
    
    std::thread collectionThread;
    std::atomic<bool> collectionActive;
    
public:
    MetricsCollector(std::unique_ptr<SystemResourceMonitor> monitor);
    ~MetricsCollector();
    
    void start();
    void stop();
    
    void setAggregationInterval(std::chrono::seconds interval);
    void setAggregationType(ResourceType type, AggregationType aggregation);
    
    std::vector<SystemMetric> getAggregatedMetrics(ResourceType type, 
                                                   std::chrono::minutes period) const;
    std::unordered_map<ResourceType, double> getCurrentAggregates() const;
    
    // Statistical functions
    double calculateMean(const std::vector<SystemMetric>& metrics) const;
    double calculateStandardDeviation(const std::vector<SystemMetric>& metrics) const;
    double calculatePercentile(const std::vector<SystemMetric>& metrics, double percentile) const;
    
    // Time-series operations
    std::vector<double> interpolateTimeSeries(const std::vector<SystemMetric>& metrics, 
                                            size_t targetPoints) const;
    std::vector<double> smoothTimeSeries(const std::vector<SystemMetric>& metrics,
                                       size_t windowSize) const;
    
private:
    void collectionLoop();
    void performAggregation();
    SystemMetric aggregateMetrics(const std::vector<SystemMetric>& metrics, 
                                AggregationType type) const;
};

class SystemProfiler {
private:
    std::vector<SystemProfile> profiles;
    SystemProfile* activeProfile;
    std::mutex profilerMutex;
    
    std::unique_ptr<PerformanceAnalyzer> performanceAnalyzer;
    std::unique_ptr<MetricsCollector> metricsCollector;
    
public:
    SystemProfiler();
    ~SystemProfiler();
    
    void createProfile(const std::string& name, const SystemProfile& profile);
    void deleteProfile(const std::string& name);
    void activateProfile(const std::string& name);
    SystemProfile* getActiveProfile();
    
    std::vector<std::string> getProfileNames() const;
    SystemProfile getProfile(const std::string& name) const;
    
    // Automatic profile creation
    SystemProfile createBaselineProfile(std::chrono::minutes samplingDuration);
    SystemProfile createOptimalProfile(const std::vector<PerformanceSnapshot>& snapshots);
    
    // Profile comparison
    double compareProfiles(const SystemProfile& profile1, const SystemProfile& profile2) const;
    SystemProfile mergeProfiles(const std::vector<SystemProfile>& profiles) const;
    
    // Adaptive profiling
    void enableAdaptiveThresholds(bool enable);
    void updateProfileThresholds(const std::string& profileName);
    
private:
    SystemProfile* findProfile(const std::string& name);
    const SystemProfile* findProfile(const std::string& name) const;
};

class ResourcePredictor {
private:
    std::unordered_map<ResourceType, PredictionModel> models;
    std::unique_ptr<SystemResourceMonitor> resourceMonitor;
    std::mutex predictorMutex;
    
    // Prediction algorithms
    std::unordered_map<std::string, std::function<std::vector<double>(const std::vector<double>&, size_t)>> algorithms;
    
public:
    ResourcePredictor(std::unique_ptr<SystemResourceMonitor> monitor);
    ~ResourcePredictor();
    
    void trainModel(ResourceType type, const std::string& algorithm = "linear_regression");
    void updateModel(ResourceType type);
    
    std::vector<double> predictValues(ResourceType type, std::chrono::minutes horizon) const;
    double predictNextValue(ResourceType type) const;
    
    bool willExceedThreshold(ResourceType type, double threshold, 
                           std::chrono::minutes timeFrame) const;
    std::chrono::steady_clock::time_point estimateThresholdCrossing(ResourceType type, 
                                                                  double threshold) const;
    
    double getModelAccuracy(ResourceType type) const;
    void setModelAccuracy(ResourceType type, double accuracy);
    
    // Prediction algorithms
    std::vector<double> linearRegression(const std::vector<double>& data, size_t points) const;
    std::vector<double> exponentialSmoothing(const std::vector<double>& data, size_t points) const;
    std::vector<double> movingAverage(const std::vector<double>& data, size_t points) const;
    std::vector<double> polynomialRegression(const std::vector<double>& data, size_t points) const;
    
private:
    void initializeAlgorithms();
    std::vector<double> extractValues(const std::vector<SystemMetric>& metrics) const;
};

class TelemetryExporter {
private:
    std::unique_ptr<MetricsCollector> metricsCollector;
    std::string outputDirectory;
    std::mutex exportMutex;
    
    // Export settings
    ExportFormat defaultFormat;
    std::chrono::seconds exportInterval;
    bool enableRealTimeExport;
    
    std::thread exportThread;
    std::atomic<bool> exportActive;
    
public:
    TelemetryExporter(std::unique_ptr<MetricsCollector> collector);
    ~TelemetryExporter();
    
    void start();
    void stop();
    
    void setOutputDirectory(const std::string& directory);
    void setDefaultFormat(ExportFormat format);
    void setExportInterval(std::chrono::seconds interval);
    void enableRealTime(bool enable);
    
    void exportMetrics(const std::vector<SystemMetric>& metrics, 
                      ExportFormat format, const std::string& filename);
    void exportSnapshot(const PerformanceSnapshot& snapshot, 
                       ExportFormat format, const std::string& filename);
    
    // Format-specific exporters
    std::string exportToJSON(const std::vector<SystemMetric>& metrics) const;
    std::string exportToCSV(const std::vector<SystemMetric>& metrics) const;
    std::string exportToXML(const std::vector<SystemMetric>& metrics) const;
    std::vector<uint8_t> exportToBinary(const std::vector<SystemMetric>& metrics) const;
    std::string exportToPrometheus(const std::vector<SystemMetric>& metrics) const;
    
    // Real-time streaming
    void startRealTimeStream(const std::string& endpoint);
    void stopRealTimeStream();
    
private:
    void exportLoop();
    void performScheduledExport();
    std::string generateFilename(ExportFormat format) const;
    std::string formatToString(ExportFormat format) const;
};

class RealTimeSystemMonitor {
private:
    std::unique_ptr<SystemResourceMonitor> resourceMonitor;
    std::unique_ptr<PerformanceAnalyzer> performanceAnalyzer;
    std::unique_ptr<AlertManager> alertManager;
    std::unique_ptr<MetricsCollector> metricsCollector;
    std::unique_ptr<SystemProfiler> systemProfiler;
    std::unique_ptr<ResourcePredictor> resourcePredictor;
    std::unique_ptr<TelemetryExporter> telemetryExporter;
    
    std::atomic<bool> systemActive;
    MonitoringMode currentMode;
    std::mutex systemMutex;
    
    // Configuration
    struct {
        bool enablePrediction;
        bool enableExport;
        bool enableAlerting;
        bool enableProfiling;
        std::chrono::milliseconds updateInterval;
        std::string configFile;
    } config;
    
public:
    RealTimeSystemMonitor();
    ~RealTimeSystemMonitor();
    
    void initialize();
    void start();
    void stop();
    void shutdown();
    
    bool isActive() const { return systemActive.load(); }
    MonitoringMode getCurrentMode() const { return currentMode; }
    
    // Configuration
    void loadConfiguration(const std::string& configFile);
    void saveConfiguration(const std::string& configFile);
    void setMonitoringMode(MonitoringMode mode);
    
    // Component access
    SystemResourceMonitor* getResourceMonitor() { return resourceMonitor.get(); }
    PerformanceAnalyzer* getPerformanceAnalyzer() { return performanceAnalyzer.get(); }
    AlertManager* getAlertManager() { return alertManager.get(); }
    MetricsCollector* getMetricsCollector() { return metricsCollector.get(); }
    SystemProfiler* getSystemProfiler() { return systemProfiler.get(); }
    ResourcePredictor* getResourcePredictor() { return resourcePredictor.get(); }
    TelemetryExporter* getTelemetryExporter() { return telemetryExporter.get(); }
    
    // High-level operations
    PerformanceSnapshot getCurrentSnapshot();
    std::vector<SystemAlert> getActiveAlerts();
    std::string getSystemStatus();
    std::unordered_map<ResourceType, double> getPredictedValues(std::chrono::minutes horizon);
    
    // EV-specific monitoring
    void enableEVSpecificMonitoring();
    void disableEVSpecificMonitoring();
    double getBatteryHealthScore();
    double getChargingEfficiency();
    double getThermalManagementScore();
    std::vector<std::string> getMaintenanceRecommendations();
    
private:
    void initializeComponents();
    void setupDefaultRules();
    void setupEVSpecificRules();
    void configureComponents();
    
    double calculateBatteryHealth();
    double calculateChargingEfficiency();
    double calculateThermalScore();
};

// Utility functions
std::string resourceTypeToString(ResourceType type);
ResourceType stringToResourceType(const std::string& str);
std::string alertLevelToString(AlertLevel level);
AlertLevel stringToAlertLevel(const std::string& str);

// Implementation begins here

inline SystemResourceMonitor::SystemResourceMonitor(std::chrono::milliseconds interval)
    : monitoringActive(false), samplingInterval(interval), maxHistorySize(1000) {
    
    // Register default resource readers
    resourceReaders[ResourceType::CPU_USAGE] = [this]() { return readCPUUsage(); };
    resourceReaders[ResourceType::MEMORY_USAGE] = [this]() { return readMemoryUsage(); };
    resourceReaders[ResourceType::DISK_USAGE] = [this]() { return readDiskUsage(); };
    resourceReaders[ResourceType::NETWORK_BANDWIDTH] = [this]() { return readNetworkBandwidth(); };
    resourceReaders[ResourceType::TEMPERATURE] = [this]() { return readSystemTemperature(); };
    resourceReaders[ResourceType::BATTERY_LEVEL] = [this]() { return readBatteryLevel(); };
    resourceReaders[ResourceType::CHARGING_RATE] = [this]() { return readChargingRate(); };
    resourceReaders[ResourceType::MOTOR_TEMPERATURE] = [this]() { return readMotorTemperature(); };
}

inline SystemResourceMonitor::~SystemResourceMonitor() {
    stop();
}

inline void SystemResourceMonitor::start() {
    if (monitoringActive.load()) return;
    
    monitoringActive.store(true);
    monitoringThread = std::thread(&SystemResourceMonitor::monitoringLoop, this);
    
    std::cout << "[SystemMonitor] Resource monitoring started\n";
}

inline void SystemResourceMonitor::stop() {
    if (!monitoringActive.load()) return;
    
    monitoringActive.store(false);
    metricsCondition.notify_all();
    
    if (monitoringThread.joinable()) {
        monitoringThread.join();
    }
    
    std::cout << "[SystemMonitor] Resource monitoring stopped\n";
}

inline SystemMetric SystemResourceMonitor::getCurrentMetric(ResourceType type) const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(metricsMutex));
    
    auto it = currentMetrics.find(type);
    if (it != currentMetrics.end()) {
        return it->second;
    }
    
    return SystemMetric();
}

inline std::vector<SystemMetric> SystemResourceMonitor::getMetricHistory(ResourceType type, size_t count) const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(metricsMutex));
    
    std::vector<SystemMetric> history;
    auto it = metricsHistory.find(type);
    if (it != metricsHistory.end()) {
        auto queue = it->second;
        size_t actualCount = std::min(count, queue.size());
        
        // Convert queue to vector (most recent first)
        std::vector<SystemMetric> temp;
        while (!queue.empty() && temp.size() < actualCount) {
            temp.push_back(queue.front());
            queue.pop();
        }
        
        // Reverse to get chronological order
        std::reverse(temp.begin(), temp.end());
        history = std::move(temp);
    }
    
    return history;
}

inline std::unordered_map<ResourceType, SystemMetric> SystemResourceMonitor::getAllCurrentMetrics() const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(metricsMutex));
    return currentMetrics;
}

inline void SystemResourceMonitor::registerCustomResource(ResourceType type, std::function<SystemMetric()> reader) {
    resourceReaders[type] = reader;
}

inline void SystemResourceMonitor::unregisterCustomResource(ResourceType type) {
    resourceReaders.erase(type);
}

inline void SystemResourceMonitor::monitoringLoop() {
    while (monitoringActive.load()) {
        auto startTime = std::chrono::steady_clock::now();
        
        collectSystemMetrics();
        
        auto endTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        
        if (elapsed < samplingInterval) {
            std::unique_lock<std::mutex> lock(metricsMutex);
            metricsCondition.wait_for(lock, samplingInterval - elapsed, 
                                    [this] { return !monitoringActive.load(); });
        }
    }
}

inline void SystemResourceMonitor::collectSystemMetrics() {
    std::lock_guard<std::mutex> lock(metricsMutex);
    
    for (const auto& [type, reader] : resourceReaders) {
        try {
            SystemMetric metric = reader();
            currentMetrics[type] = metric;
            updateMetricHistory(metric);
        } catch (const std::exception& e) {
            std::cerr << "[SystemMonitor] Error reading " << static_cast<int>(type) 
                      << ": " << e.what() << "\n";
        }
    }
}

inline SystemMetric SystemResourceMonitor::readCPUUsage() {
    // Simplified CPU usage reading - in real implementation would read from /proc/stat
    static double simulatedUsage = 20.0;
    simulatedUsage += (rand() % 20 - 10) * 0.1;  // Simulate fluctuation
    simulatedUsage = std::max(0.0, std::min(100.0, simulatedUsage));
    
    return SystemMetric(ResourceType::CPU_USAGE, simulatedUsage, "%");
}

inline SystemMetric SystemResourceMonitor::readMemoryUsage() {
    // Simplified memory usage reading - in real implementation would read from /proc/meminfo
    static double simulatedUsage = 45.0;
    simulatedUsage += (rand() % 10 - 5) * 0.1;
    simulatedUsage = std::max(0.0, std::min(100.0, simulatedUsage));
    
    return SystemMetric(ResourceType::MEMORY_USAGE, simulatedUsage, "%");
}

inline SystemMetric SystemResourceMonitor::readDiskUsage() {
    // Simplified disk usage reading
    static double simulatedUsage = 60.0;
    simulatedUsage += (rand() % 4 - 2) * 0.1;
    simulatedUsage = std::max(0.0, std::min(100.0, simulatedUsage));
    
    return SystemMetric(ResourceType::DISK_USAGE, simulatedUsage, "%");
}

inline SystemMetric SystemResourceMonitor::readNetworkBandwidth() {
    // Simplified network bandwidth reading
    static double simulatedBandwidth = 15.5;
    simulatedBandwidth += (rand() % 40 - 20) * 0.1;
    simulatedBandwidth = std::max(0.0, simulatedBandwidth);
    
    return SystemMetric(ResourceType::NETWORK_BANDWIDTH, simulatedBandwidth, "MB/s");
}

inline SystemMetric SystemResourceMonitor::readSystemTemperature() {
    // Simplified temperature reading
    static double simulatedTemp = 45.0;
    simulatedTemp += (rand() % 10 - 5) * 0.2;
    simulatedTemp = std::max(20.0, std::min(85.0, simulatedTemp));
    
    return SystemMetric(ResourceType::TEMPERATURE, simulatedTemp, "°C");
}

inline SystemMetric SystemResourceMonitor::readBatteryLevel() {
    // EV-specific: Battery level reading
    static double simulatedLevel = 75.0;
    simulatedLevel += (rand() % 6 - 3) * 0.1;
    simulatedLevel = std::max(0.0, std::min(100.0, simulatedLevel));
    
    return SystemMetric(ResourceType::BATTERY_LEVEL, simulatedLevel, "%");
}

inline SystemMetric SystemResourceMonitor::readChargingRate() {
    // EV-specific: Charging rate reading
    static double simulatedRate = 22.0;
    simulatedRate += (rand() % 8 - 4) * 0.2;
    simulatedRate = std::max(0.0, std::min(50.0, simulatedRate));
    
    return SystemMetric(ResourceType::CHARGING_RATE, simulatedRate, "kW");
}

inline SystemMetric SystemResourceMonitor::readMotorTemperature() {
    // EV-specific: Motor temperature reading
    static double simulatedTemp = 65.0;
    simulatedTemp += (rand() % 12 - 6) * 0.3;
    simulatedTemp = std::max(25.0, std::min(120.0, simulatedTemp));
    
    return SystemMetric(ResourceType::MOTOR_TEMPERATURE, simulatedTemp, "°C");
}

inline void SystemResourceMonitor::updateMetricHistory(const SystemMetric& metric) {
    auto& history = metricsHistory[metric.type];
    history.push(metric);
    
    // Maintain max history size
    while (history.size() > maxHistorySize) {
        history.pop();
    }
}

inline double SystemResourceMonitor::getAverageValue(ResourceType type, std::chrono::minutes duration) const {
    auto history = getMetricHistory(type, 1000); // Get sufficient history
    if (history.empty()) return 0.0;
    
    auto cutoffTime = std::chrono::steady_clock::now() - duration;
    double sum = 0.0;
    size_t count = 0;
    
    for (const auto& metric : history) {
        if (metric.timestamp >= cutoffTime) {
            sum += metric.value;
            count++;
        }
    }
    
    return count > 0 ? sum / count : 0.0;
}

inline double SystemResourceMonitor::getPeakValue(ResourceType type, std::chrono::minutes duration) const {
    auto history = getMetricHistory(type, 1000);
    if (history.empty()) return 0.0;
    
    auto cutoffTime = std::chrono::steady_clock::now() - duration;
    double peak = std::numeric_limits<double>::lowest();
    
    for (const auto& metric : history) {
        if (metric.timestamp >= cutoffTime) {
            peak = std::max(peak, metric.value);
        }
    }
    
    return peak == std::numeric_limits<double>::lowest() ? 0.0 : peak;
}

inline std::vector<double> SystemResourceMonitor::getTrendData(ResourceType type, size_t points) const {
    auto history = getMetricHistory(type, points);
    std::vector<double> trend;
    trend.reserve(history.size());
    
    for (const auto& metric : history) {
        trend.push_back(metric.value);
    }
    
    return trend;
}

inline PerformanceAnalyzer::PerformanceAnalyzer(std::unique_ptr<SystemResourceMonitor> monitor)
    : resourceMonitor(std::move(monitor)) {
    
    // Initialize scoring functions
    scoringFunctions["cpu"] = [this](const PerformanceSnapshot& s) { return calculateCPUScore(s); };
    scoringFunctions["memory"] = [this](const PerformanceSnapshot& s) { return calculateMemoryScore(s); };
    scoringFunctions["network"] = [this](const PerformanceSnapshot& s) { return calculateNetworkScore(s); };
    scoringFunctions["thermal"] = [this](const PerformanceSnapshot& s) { return calculateThermalScore(s); };
    
    // Initialize default resource weights
    resourceWeights[ResourceType::CPU_USAGE] = 0.3;
    resourceWeights[ResourceType::MEMORY_USAGE] = 0.25;
    resourceWeights[ResourceType::TEMPERATURE] = 0.2;
    resourceWeights[ResourceType::NETWORK_BANDWIDTH] = 0.15;
    resourceWeights[ResourceType::BATTERY_LEVEL] = 0.1;
}

inline PerformanceAnalyzer::~PerformanceAnalyzer() = default;

inline void PerformanceAnalyzer::startAnalysis() {
    if (resourceMonitor && !resourceMonitor->isActive()) {
        resourceMonitor->start();
    }
    std::cout << "[PerformanceAnalyzer] Analysis started\n";
}

inline void PerformanceAnalyzer::stopAnalysis() {
    if (resourceMonitor && resourceMonitor->isActive()) {
        resourceMonitor->stop();
    }
    std::cout << "[PerformanceAnalyzer] Analysis stopped\n";
}

inline PerformanceSnapshot PerformanceAnalyzer::createSnapshot() {
    PerformanceSnapshot snapshot;
    
    if (resourceMonitor) {
        snapshot.metrics = resourceMonitor->getAllCurrentMetrics();
        snapshot.overallScore = calculateOverallScore(snapshot);
        snapshot.recommendations = generateRecommendations(snapshot);
    }
    
    std::lock_guard<std::mutex> lock(analysisMutex);
    snapshots.push_back(snapshot);
    
    // Keep only recent snapshots
    if (snapshots.size() > 1000) {
        snapshots.erase(snapshots.begin());
    }
    
    return snapshot;
}

inline std::vector<PerformanceSnapshot> PerformanceAnalyzer::getRecentSnapshots(size_t count) const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(analysisMutex));
    
    size_t start = snapshots.size() > count ? snapshots.size() - count : 0;
    return std::vector<PerformanceSnapshot>(snapshots.begin() + start, snapshots.end());
}

inline double PerformanceAnalyzer::calculateOverallScore(const PerformanceSnapshot& snapshot) const {
    double totalScore = 0.0;
    double totalWeight = 0.0;
    
    for (const auto& [type, metric] : snapshot.metrics) {
        auto weightIt = resourceWeights.find(type);
        if (weightIt != resourceWeights.end()) {
            double weight = weightIt->second;
            double score = 100.0; // Default perfect score
            
            // Calculate score based on resource type
            switch (type) {
                case ResourceType::CPU_USAGE:
                    score = std::max(0.0, 100.0 - metric.value); // Lower usage = higher score
                    break;
                case ResourceType::MEMORY_USAGE:
                    score = std::max(0.0, 100.0 - metric.value);
                    break;
                case ResourceType::TEMPERATURE:
                    score = metric.value < 70.0 ? 100.0 : std::max(0.0, 100.0 - (metric.value - 70.0) * 2);
                    break;
                case ResourceType::BATTERY_LEVEL:
                    score = metric.value; // Higher battery = higher score
                    break;
                default:
                    score = 100.0; // Default score for unhandled types
                    break;
            }
            
            totalScore += score * weight;
            totalWeight += weight;
        }
    }
    
    return totalWeight > 0.0 ? totalScore / totalWeight : 0.0;
}

inline std::vector<std::string> PerformanceAnalyzer::generateRecommendations(const PerformanceSnapshot& snapshot) const {
    std::vector<std::string> recommendations;
    
    for (const auto& [type, metric] : snapshot.metrics) {
        switch (type) {
            case ResourceType::CPU_USAGE:
                if (metric.value > 80.0) {
                    recommendations.push_back("High CPU usage detected. Consider reducing computational load.");
                }
                break;
            case ResourceType::MEMORY_USAGE:
                if (metric.value > 85.0) {
                    recommendations.push_back("Memory usage is high. Consider freeing unused resources.");
                }
                break;
            case ResourceType::TEMPERATURE:
                if (metric.value > 75.0) {
                    recommendations.push_back("System temperature is elevated. Check cooling systems.");
                }
                break;
            case ResourceType::BATTERY_LEVEL:
                if (metric.value < 20.0) {
                    recommendations.push_back("Battery level is low. Consider charging soon.");
                }
                break;
            default:
                break;
        }
    }
    
    return recommendations;
}

inline void PerformanceAnalyzer::setResourceWeight(ResourceType type, double weight) {
    resourceWeights[type] = std::max(0.0, std::min(1.0, weight));
}

inline double PerformanceAnalyzer::getResourceWeight(ResourceType type) const {
    auto it = resourceWeights.find(type);
    return it != resourceWeights.end() ? it->second : 0.0;
}

inline double PerformanceAnalyzer::calculateCPUScore(const PerformanceSnapshot& snapshot) const {
    auto it = snapshot.metrics.find(ResourceType::CPU_USAGE);
    if (it != snapshot.metrics.end()) {
        return std::max(0.0, 100.0 - it->second.value);
    }
    return 50.0; // Default neutral score
}

inline double PerformanceAnalyzer::calculateMemoryScore(const PerformanceSnapshot& snapshot) const {
    auto it = snapshot.metrics.find(ResourceType::MEMORY_USAGE);
    if (it != snapshot.metrics.end()) {
        return std::max(0.0, 100.0 - it->second.value);
    }
    return 50.0;
}

inline double PerformanceAnalyzer::calculateNetworkScore(const PerformanceSnapshot& snapshot) const {
    auto it = snapshot.metrics.find(ResourceType::NETWORK_BANDWIDTH);
    if (it != snapshot.metrics.end()) {
        // Higher bandwidth utilization can be good (up to a point)
        double usage = it->second.value;
        if (usage < 50.0) return 50.0 + usage; // 50-100 score range
        else return std::max(0.0, 150.0 - usage); // Diminishing returns after 50%
    }
    return 50.0;
}

inline double PerformanceAnalyzer::calculateThermalScore(const PerformanceSnapshot& snapshot) const {
    auto it = snapshot.metrics.find(ResourceType::TEMPERATURE);
    if (it != snapshot.metrics.end()) {
        double temp = it->second.value;
        if (temp < 50.0) return 100.0;
        else if (temp < 70.0) return 100.0 - (temp - 50.0) * 2;
        else return std::max(0.0, 60.0 - (temp - 70.0) * 4);
    }
    return 50.0;
}

inline RealTimeSystemMonitor::RealTimeSystemMonitor() : systemActive(false), currentMode(MonitoringMode::REAL_TIME) {
    // Initialize configuration
    config.enablePrediction = true;
    config.enableExport = false;
    config.enableAlerting = true;
    config.enableProfiling = true;
    config.updateInterval = std::chrono::milliseconds(1000);
}

inline RealTimeSystemMonitor::~RealTimeSystemMonitor() {
    shutdown();
}

inline void RealTimeSystemMonitor::initialize() {
    std::lock_guard<std::mutex> lock(systemMutex);
    
    initializeComponents();
    setupDefaultRules();
    configureComponents();
    
    std::cout << "[RealTimeSystemMonitor] Initialization complete\n";
}

inline void RealTimeSystemMonitor::start() {
    if (systemActive.load()) return;
    
    systemActive.store(true);
    
    // Start all components
    if (resourceMonitor) resourceMonitor->start();
    if (performanceAnalyzer) performanceAnalyzer->startAnalysis();
    if (alertManager && config.enableAlerting) alertManager->start();
    if (metricsCollector) metricsCollector->start();
    if (telemetryExporter && config.enableExport) telemetryExporter->start();
    
    std::cout << "[RealTimeSystemMonitor] System monitoring started in " 
              << (currentMode == MonitoringMode::REAL_TIME ? "real-time" : "batch") 
              << " mode\n";
}

inline void RealTimeSystemMonitor::stop() {
    if (!systemActive.load()) return;
    
    systemActive.store(false);
    
    // Stop all components
    if (telemetryExporter) telemetryExporter->stop();
    if (metricsCollector) metricsCollector->stop();
    if (alertManager) alertManager->stop();
    if (performanceAnalyzer) performanceAnalyzer->stopAnalysis();
    if (resourceMonitor) resourceMonitor->stop();
    
    std::cout << "[RealTimeSystemMonitor] System monitoring stopped\n";
}

inline void RealTimeSystemMonitor::shutdown() {
    stop();
    
    std::lock_guard<std::mutex> lock(systemMutex);
    
    // Reset all components
    telemetryExporter.reset();
    resourcePredictor.reset();
    systemProfiler.reset();
    metricsCollector.reset();
    alertManager.reset();
    performanceAnalyzer.reset();
    resourceMonitor.reset();
    
    std::cout << "[RealTimeSystemMonitor] System shutdown complete\n";
}

inline void RealTimeSystemMonitor::setMonitoringMode(MonitoringMode mode) {
    currentMode = mode;
    
    if (resourceMonitor) {
        switch (mode) {
            case MonitoringMode::REAL_TIME:
                resourceMonitor->setSamplingInterval(std::chrono::milliseconds(100));
                break;
            case MonitoringMode::BATCH:
                resourceMonitor->setSamplingInterval(std::chrono::seconds(5));
                break;
            case MonitoringMode::LOW_POWER:
                resourceMonitor->setSamplingInterval(std::chrono::seconds(30));
                break;
            default:
                resourceMonitor->setSamplingInterval(std::chrono::seconds(1));
                break;
        }
    }
}

inline PerformanceSnapshot RealTimeSystemMonitor::getCurrentSnapshot() {
    if (performanceAnalyzer) {
        return performanceAnalyzer->createSnapshot();
    }
    return PerformanceSnapshot();
}

inline std::string RealTimeSystemMonitor::getSystemStatus() {
    std::ostringstream status;
    status << "=== Real-Time System Monitor Status ===\n";
    status << "Active: " << (systemActive.load() ? "Yes" : "No") << "\n";
    status << "Mode: ";
    switch (currentMode) {
        case MonitoringMode::REAL_TIME: status << "Real-Time"; break;
        case MonitoringMode::BATCH: status << "Batch"; break;
        case MonitoringMode::HYBRID: status << "Hybrid"; break;
        case MonitoringMode::LOW_POWER: status << "Low Power"; break;
    }
    status << "\n";
    
    if (resourceMonitor && resourceMonitor->isActive()) {
        auto metrics = resourceMonitor->getAllCurrentMetrics();
        status << "\nCurrent Metrics:\n";
        for (const auto& [type, metric] : metrics) {
            status << "  " << resourceTypeToString(type) << ": " 
                   << metric.value << " " << metric.unit << "\n";
        }
    }
    
    if (performanceAnalyzer) {
        auto snapshot = performanceAnalyzer->createSnapshot();
        status << "\nOverall Performance Score: " << snapshot.overallScore << "/100\n";
        
        if (!snapshot.recommendations.empty()) {
            status << "\nRecommendations:\n";
            for (const auto& rec : snapshot.recommendations) {
                status << "  - " << rec << "\n";
            }
        }
    }
    
    return status.str();
}

inline void RealTimeSystemMonitor::initializeComponents() {
    // Create resource monitor
    resourceMonitor = std::make_unique<SystemResourceMonitor>(config.updateInterval);
    
    // Create performance analyzer
    auto monitorCopy = std::make_unique<SystemResourceMonitor>(config.updateInterval);
    performanceAnalyzer = std::make_unique<PerformanceAnalyzer>(std::move(monitorCopy));
    
    // Create alert manager
    alertManager = std::make_unique<AlertManager>();
    
    // Create metrics collector
    auto monitorCopy2 = std::make_unique<SystemResourceMonitor>(config.updateInterval);
    metricsCollector = std::make_unique<MetricsCollector>(std::move(monitorCopy2));
    
    // Create system profiler
    systemProfiler = std::make_unique<SystemProfiler>();
    
    // Create resource predictor if enabled
    if (config.enablePrediction) {
        auto monitorCopy3 = std::make_unique<SystemResourceMonitor>(config.updateInterval);
        resourcePredictor = std::make_unique<ResourcePredictor>(std::move(monitorCopy3));
    }
    
    // Create telemetry exporter if enabled
    if (config.enableExport) {
        auto collectorCopy = std::make_unique<MetricsCollector>(
            std::make_unique<SystemResourceMonitor>(config.updateInterval));
        telemetryExporter = std::make_unique<TelemetryExporter>(std::move(collectorCopy));
    }
}

inline void RealTimeSystemMonitor::setupDefaultRules() {
    if (alertManager) {
        alertManager->addCPUThresholdRule(80.0, AlertLevel::WARNING);
        alertManager->addCPUThresholdRule(95.0, AlertLevel::CRITICAL);
        alertManager->addMemoryThresholdRule(85.0, AlertLevel::WARNING);
        alertManager->addMemoryThresholdRule(95.0, AlertLevel::CRITICAL);
        alertManager->addTemperatureThresholdRule(75.0, AlertLevel::WARNING);
        alertManager->addTemperatureThresholdRule(85.0, AlertLevel::CRITICAL);
        alertManager->addBatteryLowRule(20.0, AlertLevel::WARNING);
        alertManager->addBatteryLowRule(10.0, AlertLevel::CRITICAL);
    }
}

inline void RealTimeSystemMonitor::configureComponents() {
    // Configure components based on current settings
    if (metricsCollector) {
        metricsCollector->setAggregationInterval(std::chrono::seconds(60));
    }
    
    if (telemetryExporter && config.enableExport) {
        telemetryExporter->setExportInterval(std::chrono::minutes(5));
        telemetryExporter->setDefaultFormat(ExportFormat::JSON);
    }
}

inline std::vector<SystemAlert> RealTimeSystemMonitor::getActiveAlerts() {
    if (alertManager) {
        return alertManager->getActiveAlerts();
    }
    return std::vector<SystemAlert>();
}

inline void RealTimeSystemMonitor::enableEVSpecificMonitoring() {
    setupEVSpecificRules();
    std::cout << "[RealTimeSystemMonitor] EV-specific monitoring enabled\n";
}

inline void RealTimeSystemMonitor::disableEVSpecificMonitoring() {
    std::cout << "[RealTimeSystemMonitor] EV-specific monitoring disabled\n";
}

inline double RealTimeSystemMonitor::getBatteryHealthScore() {
    return calculateBatteryHealth();
}

inline double RealTimeSystemMonitor::getChargingEfficiency() {
    return calculateChargingEfficiency();
}

inline double RealTimeSystemMonitor::getThermalManagementScore() {
    return calculateThermalScore();
}

inline std::vector<std::string> RealTimeSystemMonitor::getMaintenanceRecommendations() {
    std::vector<std::string> recommendations;
    
    if (resourceMonitor) {
        auto batteryLevel = resourceMonitor->getCurrentMetric(ResourceType::BATTERY_LEVEL);
        auto motorTemp = resourceMonitor->getCurrentMetric(ResourceType::MOTOR_TEMPERATURE);
        auto systemTemp = resourceMonitor->getCurrentMetric(ResourceType::TEMPERATURE);
        
        if (batteryLevel.value < 30.0) {
            recommendations.push_back("Battery level is low - plan for charging");
        }
        
        if (motorTemp.value > 100.0) {
            recommendations.push_back("Motor temperature is high - check cooling system");
        }
        
        if (systemTemp.value > 80.0) {
            recommendations.push_back("System temperature elevated - reduce computational load");
        }
        
        // Add periodic maintenance recommendations
        recommendations.push_back("Regular battery health check recommended");
        recommendations.push_back("Motor efficiency analysis due");
    }
    
    return recommendations;
}

inline void RealTimeSystemMonitor::setupEVSpecificRules() {
    if (alertManager) {
        // Add EV-specific alert rules
        AlertRule motorTempRule;
        motorTempRule.name = "Motor_Temperature_High";
        motorTempRule.resourceType = ResourceType::MOTOR_TEMPERATURE;
        motorTempRule.threshold = 110.0;
        motorTempRule.level = AlertLevel::CRITICAL;
        motorTempRule.condition = [](const SystemMetric& metric) { return metric.value > 110.0; };
        motorTempRule.description = "Motor temperature critically high";
        alertManager->addRule(motorTempRule);
        
        AlertRule chargingRule;
        chargingRule.name = "Charging_Rate_Anomaly";
        chargingRule.resourceType = ResourceType::CHARGING_RATE;
        chargingRule.threshold = 1.0;
        chargingRule.level = AlertLevel::WARNING;
        chargingRule.condition = [](const SystemMetric& metric) { return metric.value < 1.0; };
        chargingRule.description = "Charging rate unusually low";
        alertManager->addRule(chargingRule);
    }
}

inline double RealTimeSystemMonitor::calculateBatteryHealth() {
    if (resourceMonitor) {
        auto batteryLevel = resourceMonitor->getCurrentMetric(ResourceType::BATTERY_LEVEL);
        auto chargingRate = resourceMonitor->getCurrentMetric(ResourceType::CHARGING_RATE);
        
        // Simplified battery health calculation
        double levelScore = batteryLevel.value;
        double chargingScore = std::min(100.0, chargingRate.value * 4.0); // Normalize to 0-100
        
        return (levelScore * 0.7 + chargingScore * 0.3);
    }
    return 50.0; // Default neutral score
}

inline double RealTimeSystemMonitor::calculateChargingEfficiency() {
    if (resourceMonitor) {
        auto chargingRate = resourceMonitor->getCurrentMetric(ResourceType::CHARGING_RATE);
        auto systemTemp = resourceMonitor->getCurrentMetric(ResourceType::TEMPERATURE);
        
        // Simplified efficiency calculation
        double rateEfficiency = std::min(100.0, chargingRate.value * 2.0);
        double tempPenalty = systemTemp.value > 60.0 ? (systemTemp.value - 60.0) * 2.0 : 0.0;
        
        return std::max(0.0, rateEfficiency - tempPenalty);
    }
    return 50.0;
}

inline double RealTimeSystemMonitor::calculateThermalScore() {
    if (resourceMonitor) {
        auto systemTemp = resourceMonitor->getCurrentMetric(ResourceType::TEMPERATURE);
        auto motorTemp = resourceMonitor->getCurrentMetric(ResourceType::MOTOR_TEMPERATURE);
        
        // Calculate thermal management score
        double systemScore = systemTemp.value < 60.0 ? 100.0 : std::max(0.0, 100.0 - (systemTemp.value - 60.0) * 2.0);
        double motorScore = motorTemp.value < 80.0 ? 100.0 : std::max(0.0, 100.0 - (motorTemp.value - 80.0) * 1.5);
        
        return (systemScore + motorScore) / 2.0;
    }
    return 50.0;
}

// Utility function implementations

inline std::string resourceTypeToString(ResourceType type) {
    switch (type) {
        case ResourceType::CPU_USAGE: return "CPU Usage";
        case ResourceType::MEMORY_USAGE: return "Memory Usage";
        case ResourceType::DISK_USAGE: return "Disk Usage";
        case ResourceType::NETWORK_BANDWIDTH: return "Network Bandwidth";
        case ResourceType::TEMPERATURE: return "Temperature";
        case ResourceType::BATTERY_LEVEL: return "Battery Level";
        case ResourceType::CHARGING_RATE: return "Charging Rate";
        case ResourceType::MOTOR_TEMPERATURE: return "Motor Temperature";
        case ResourceType::INVERTER_TEMPERATURE: return "Inverter Temperature";
        case ResourceType::COOLANT_TEMPERATURE: return "Coolant Temperature";
        case ResourceType::TIRE_PRESSURE: return "Tire Pressure";
        case ResourceType::BRAKE_TEMPERATURE: return "Brake Temperature";
        case ResourceType::SYSTEM_LOAD: return "System Load";
        case ResourceType::THREAD_COUNT: return "Thread Count";
        case ResourceType::FILE_DESCRIPTORS: return "File Descriptors";
        case ResourceType::NETWORK_CONNECTIONS: return "Network Connections";
        default: return "Unknown";
    }
}

inline std::string alertLevelToString(AlertLevel level) {
    switch (level) {
        case AlertLevel::INFO: return "INFO";
        case AlertLevel::WARNING: return "WARNING";
        case AlertLevel::CRITICAL: return "CRITICAL";
        case AlertLevel::EMERGENCY: return "EMERGENCY";
        default: return "UNKNOWN";
    }
}

// Stub implementations for remaining classes to satisfy linker

inline AlertManager::AlertManager() : processingActive(false) {}

inline AlertManager::~AlertManager() {
    stop();
}

inline void AlertManager::start() {
    if (processingActive.load()) return;
    processingActive.store(true);
    alertProcessingThread = std::thread(&AlertManager::alertProcessingLoop, this);
    std::cout << "[AlertManager] Alert processing started\n";
}

inline void AlertManager::stop() {
    if (!processingActive.load()) return;
    processingActive.store(false);
    alertCondition.notify_all();
    if (alertProcessingThread.joinable()) {
        alertProcessingThread.join();
    }
    std::cout << "[AlertManager] Alert processing stopped\n";
}

inline void AlertManager::addCPUThresholdRule(double threshold, AlertLevel level) {
    AlertRule rule;
    rule.name = "CPU_Threshold_" + std::to_string(static_cast<int>(threshold));
    rule.resourceType = ResourceType::CPU_USAGE;
    rule.threshold = threshold;
    rule.level = level;
    rule.condition = [threshold](const SystemMetric& metric) { return metric.value > threshold; };
    rule.description = "CPU usage threshold alert";
    addRule(rule);
}

inline void AlertManager::addMemoryThresholdRule(double threshold, AlertLevel level) {
    AlertRule rule;
    rule.name = "Memory_Threshold_" + std::to_string(static_cast<int>(threshold));
    rule.resourceType = ResourceType::MEMORY_USAGE;
    rule.threshold = threshold;
    rule.level = level;
    rule.condition = [threshold](const SystemMetric& metric) { return metric.value > threshold; };
    rule.description = "Memory usage threshold alert";
    addRule(rule);
}

inline void AlertManager::addTemperatureThresholdRule(double threshold, AlertLevel level) {
    AlertRule rule;
    rule.name = "Temperature_Threshold_" + std::to_string(static_cast<int>(threshold));
    rule.resourceType = ResourceType::TEMPERATURE;
    rule.threshold = threshold;
    rule.level = level;
    rule.condition = [threshold](const SystemMetric& metric) { return metric.value > threshold; };
    rule.description = "Temperature threshold alert";
    addRule(rule);
}

inline void AlertManager::addBatteryLowRule(double threshold, AlertLevel level) {
    AlertRule rule;
    rule.name = "Battery_Low_" + std::to_string(static_cast<int>(threshold));
    rule.resourceType = ResourceType::BATTERY_LEVEL;
    rule.threshold = threshold;
    rule.level = level;
    rule.condition = [threshold](const SystemMetric& metric) { return metric.value < threshold; };
    rule.description = "Low battery alert";
    addRule(rule);
}

inline void AlertManager::addRule(const AlertRule& rule) {
    std::lock_guard<std::mutex> lock(alertMutex);
    rules.push_back(rule);
}

inline void AlertManager::alertProcessingLoop() {
    // Simplified processing loop
    while (processingActive.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

inline std::vector<SystemAlert> AlertManager::getActiveAlerts() const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(alertMutex));
    
    std::vector<SystemAlert> activeAlerts;
    for (const auto& alert : alertHistory) {
        if (!alert.acknowledged) {
            activeAlerts.push_back(alert);
        }
    }
    return activeAlerts;
}

inline void AlertManager::removeRule(const std::string& ruleName) {
    std::lock_guard<std::mutex> lock(alertMutex);
    rules.erase(std::remove_if(rules.begin(), rules.end(),
        [&ruleName](const AlertRule& rule) { return rule.name == ruleName; }), rules.end());
}

inline void AlertManager::updateRule(const std::string& ruleName, const AlertRule& newRule) {
    std::lock_guard<std::mutex> lock(alertMutex);
    for (auto& rule : rules) {
        if (rule.name == ruleName) {
            rule = newRule;
            break;
        }
    }
}

inline std::vector<AlertRule> AlertManager::getAllRules() const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(alertMutex));
    return rules;
}

inline void AlertManager::checkMetric(const SystemMetric& metric) {
    std::lock_guard<std::mutex> lock(alertMutex);
    for (const auto& rule : rules) {
        if (rule.enabled && rule.resourceType == metric.type && rule.condition(metric)) {
            if (!isRateLimited(rule.name)) {
                triggerAlert(rule, metric);
            }
        }
    }
}

inline void AlertManager::processAlerts() {
    // Process queued alerts
    std::lock_guard<std::mutex> lock(alertMutex);
    while (!alertQueue.empty()) {
        auto alert = alertQueue.front();
        alertQueue.pop();
        alertHistory.push_back(alert);
        
        // Trigger handlers
        auto it = alertHandlers.find(alert.severity);
        if (it != alertHandlers.end()) {
            for (const auto& handler : it->second) {
                handler(alert);
            }
        }
    }
}

inline void AlertManager::addAlertHandler(AlertLevel level, std::function<void(const SystemAlert&)> handler) {
    alertHandlers[level].push_back(handler);
}

inline void AlertManager::removeAlertHandlers(AlertLevel level) {
    alertHandlers[level].clear();
}

inline std::vector<SystemAlert> AlertManager::getAlertHistory(size_t count) const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(alertMutex));
    
    size_t start = alertHistory.size() > count ? alertHistory.size() - count : 0;
    return std::vector<SystemAlert>(alertHistory.begin() + start, alertHistory.end());
}

inline void AlertManager::acknowledgeAlert(const std::string& alertId, const std::string& user) {
    std::lock_guard<std::mutex> lock(alertMutex);
    for (auto& alert : alertHistory) {
        if (alert.alertId == alertId) {
            alert.acknowledged = true;
            alert.acknowledgedBy = user;
            break;
        }
    }
}

inline void AlertManager::clearAcknowledgedAlerts() {
    std::lock_guard<std::mutex> lock(alertMutex);
    alertHistory.erase(std::remove_if(alertHistory.begin(), alertHistory.end(),
        [](const SystemAlert& alert) { return alert.acknowledged; }), alertHistory.end());
}

inline void AlertManager::triggerAlert(const AlertRule& rule, const SystemMetric& metric) {
    SystemAlert alert;
    alert.alertId = generateAlertId();
    alert.rule = rule;
    alert.metric = metric;
    alert.severity = rule.level;
    alert.message = rule.description + " (" + resourceTypeToString(metric.type) + 
                   ": " + std::to_string(metric.value) + " " + metric.unit + ")";
    
    alertQueue.push(alert);
    lastAlertTimes[rule.name] = std::chrono::steady_clock::now();
}

inline bool AlertManager::isRateLimited(const std::string& ruleName) const {
    auto it = lastAlertTimes.find(ruleName);
    if (it != lastAlertTimes.end()) {
        auto elapsed = std::chrono::steady_clock::now() - it->second;
        return elapsed < std::chrono::seconds(60); // 1 minute rate limit
    }
    return false;
}

inline std::string AlertManager::generateAlertId() const {
    static int counter = 0;
    return "alert_" + std::to_string(++counter);
}

inline MetricsCollector::MetricsCollector(std::unique_ptr<SystemResourceMonitor> monitor) 
    : resourceMonitor(std::move(monitor)), collectionActive(false),
      aggregationInterval(std::chrono::seconds(60)) {}

inline MetricsCollector::~MetricsCollector() {
    stop();
}

inline void MetricsCollector::start() {
    if (collectionActive.load()) return;
    collectionActive.store(true);
    if (resourceMonitor) resourceMonitor->start();
    collectionThread = std::thread(&MetricsCollector::collectionLoop, this);
    std::cout << "[MetricsCollector] Collection started\n";
}

inline void MetricsCollector::stop() {
    if (!collectionActive.load()) return;
    collectionActive.store(false);
    if (collectionThread.joinable()) {
        collectionThread.join();
    }
    if (resourceMonitor) resourceMonitor->stop();
    std::cout << "[MetricsCollector] Collection stopped\n";
}

inline void MetricsCollector::setAggregationInterval(std::chrono::seconds interval) {
    aggregationInterval = interval;
}

inline void MetricsCollector::collectionLoop() {
    while (collectionActive.load()) {
        performAggregation();
        std::this_thread::sleep_for(aggregationInterval);
    }
}

inline void MetricsCollector::performAggregation() {
    // Simplified aggregation implementation
    if (resourceMonitor) {
        auto metrics = resourceMonitor->getAllCurrentMetrics();
        std::lock_guard<std::mutex> lock(collectorMutex);
        for (const auto& [type, metric] : metrics) {
            aggregatedMetrics[type].push_back(metric);
        }
    }
}

inline SystemProfiler::SystemProfiler() : activeProfile(nullptr) {}

inline SystemProfiler::~SystemProfiler() = default;

inline ResourcePredictor::ResourcePredictor(std::unique_ptr<SystemResourceMonitor> monitor)
    : resourceMonitor(std::move(monitor)) {}

inline ResourcePredictor::~ResourcePredictor() = default;

inline TelemetryExporter::TelemetryExporter(std::unique_ptr<MetricsCollector> collector)
    : metricsCollector(std::move(collector)), exportActive(false), 
      defaultFormat(ExportFormat::JSON), exportInterval(std::chrono::minutes(5)),
      enableRealTimeExport(false) {}

inline TelemetryExporter::~TelemetryExporter() {
    stop();
}

inline void TelemetryExporter::start() {
    if (exportActive.load()) return;
    exportActive.store(true);
    exportThread = std::thread(&TelemetryExporter::exportLoop, this);
    std::cout << "[TelemetryExporter] Export started\n";
}

inline void TelemetryExporter::stop() {
    if (!exportActive.load()) return;
    exportActive.store(false);
    if (exportThread.joinable()) {
        exportThread.join();
    }
    std::cout << "[TelemetryExporter] Export stopped\n";
}

inline void TelemetryExporter::setExportInterval(std::chrono::seconds interval) {
    exportInterval = interval;
}

inline void TelemetryExporter::setDefaultFormat(ExportFormat format) {
    defaultFormat = format;
}

inline void TelemetryExporter::exportLoop() {
    while (exportActive.load()) {
        performScheduledExport();
        std::this_thread::sleep_for(exportInterval);
    }
}

inline void TelemetryExporter::performScheduledExport() {
    // Simplified export implementation
    std::cout << "[TelemetryExporter] Performing scheduled export\n";
}

} // namespace system_monitor

#endif // REALTIME_SYSTEM_MONITOR_H