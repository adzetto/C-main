/**
 * @file realtime_monitor_demo.cpp
 * @author adzetto
 * @brief Demo application for Real-Time System Monitor
 * @version 1.0
 * @date 2025-08-31
 * 
 * @copyright Copyright (c) 2025
 */

#include "realtime_system_monitor.h"
#include <iostream>
#include <chrono>
#include <thread>

using namespace system_monitor;

class RealTimeMonitorDemo {
private:
    std::unique_ptr<RealTimeSystemMonitor> systemMonitor;
    
public:
    RealTimeMonitorDemo() {
        systemMonitor = std::make_unique<RealTimeSystemMonitor>();
    }
    
    void runDemo() {
        std::cout << "=== Real-Time System Monitor Demo ===\n";
        std::cout << "Author: adzetto\n";
        std::cout << "Version: 1.0\n";
        std::cout << "Date: 2025-08-31\n\n";
        
        // Initialize and start the system monitor
        std::cout << "Initializing Real-Time System Monitor...\n";
        systemMonitor->initialize();
        
        std::cout << "Starting system monitoring...\n";
        systemMonitor->start();
        
        // Demonstrate different monitoring modes
        demonstrateMonitoringModes();
        
        // Demonstrate real-time metrics
        demonstrateRealTimeMetrics();
        
        // Demonstrate performance analysis
        demonstratePerformanceAnalysis();
        
        // Demonstrate alerting system
        demonstrateAlerting();
        
        // Demonstrate EV-specific monitoring
        demonstrateEVMonitoring();
        
        // Clean shutdown
        std::cout << "\nShutting down system monitor...\n";
        systemMonitor->shutdown();
        
        std::cout << "\n=== Demo Completed Successfully ===\n";
    }
    
private:
    void demonstrateMonitoringModes() {
        std::cout << "\n--- Monitoring Modes Demo ---\n";
        
        // Test different monitoring modes
        systemMonitor->setMonitoringMode(MonitoringMode::REAL_TIME);
        std::cout << "Switched to Real-Time mode\n";
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        systemMonitor->setMonitoringMode(MonitoringMode::BATCH);
        std::cout << "Switched to Batch mode\n";
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        systemMonitor->setMonitoringMode(MonitoringMode::LOW_POWER);
        std::cout << "Switched to Low Power mode\n";
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Switch back to real-time for rest of demo
        systemMonitor->setMonitoringMode(MonitoringMode::REAL_TIME);
        std::cout << "Switched back to Real-Time mode\n";
    }
    
    void demonstrateRealTimeMetrics() {
        std::cout << "\n--- Real-Time Metrics Demo ---\n";
        
        auto resourceMonitor = systemMonitor->getResourceMonitor();
        if (!resourceMonitor) {
            std::cout << "Resource monitor not available\n";
            return;
        }
        
        std::cout << "Collecting real-time metrics for 10 seconds...\n";
        for (int i = 0; i < 10; ++i) {
            auto metrics = resourceMonitor->getAllCurrentMetrics();
            std::cout << "Sample " << (i + 1) << ":\n";
            
            for (const auto& [type, metric] : metrics) {
                std::cout << "  " << resourceTypeToString(type) 
                          << ": " << metric.value << " " << metric.unit << "\n";
            }
            std::cout << "\n";
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        
        // Demonstrate trend analysis
        std::cout << "CPU Usage Trend (last 5 points): ";
        auto cpuTrend = resourceMonitor->getTrendData(ResourceType::CPU_USAGE, 5);
        for (size_t i = 0; i < cpuTrend.size(); ++i) {
            std::cout << cpuTrend[i];
            if (i < cpuTrend.size() - 1) std::cout << " -> ";
        }
        std::cout << "\n";
        
        // Demonstrate average values
        auto avgCPU = resourceMonitor->getAverageValue(ResourceType::CPU_USAGE, std::chrono::minutes(1));
        auto peakCPU = resourceMonitor->getPeakValue(ResourceType::CPU_USAGE, std::chrono::minutes(1));
        std::cout << "CPU Usage - Average: " << avgCPU << "%, Peak: " << peakCPU << "%\n";
    }
    
    void demonstratePerformanceAnalysis() {
        std::cout << "\n--- Performance Analysis Demo ---\n";
        
        auto performanceAnalyzer = systemMonitor->getPerformanceAnalyzer();
        if (!performanceAnalyzer) {
            std::cout << "Performance analyzer not available\n";
            return;
        }
        
        // Create performance snapshots
        std::cout << "Creating performance snapshots...\n";
        for (int i = 0; i < 5; ++i) {
            auto snapshot = performanceAnalyzer->createSnapshot();
            std::cout << "Snapshot " << (i + 1) << ":\n";
            std::cout << "  Overall Score: " << snapshot.overallScore << "/100\n";
            
            if (!snapshot.recommendations.empty()) {
                std::cout << "  Recommendations:\n";
                for (const auto& rec : snapshot.recommendations) {
                    std::cout << "    - " << rec << "\n";
                }
            }
            std::cout << "\n";
            
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        
        // Demonstrate resource weights
        std::cout << "Current resource weights:\n";
        std::vector<ResourceType> types = {
            ResourceType::CPU_USAGE,
            ResourceType::MEMORY_USAGE,
            ResourceType::TEMPERATURE,
            ResourceType::BATTERY_LEVEL
        };
        
        for (auto type : types) {
            double weight = performanceAnalyzer->getResourceWeight(type);
            std::cout << "  " << resourceTypeToString(type) << ": " << weight << "\n";
        }
    }
    
    void demonstrateAlerting() {
        std::cout << "\n--- Alerting System Demo ---\n";
        
        auto alertManager = systemMonitor->getAlertManager();
        if (!alertManager) {
            std::cout << "Alert manager not available\n";
            return;
        }
        
        std::cout << "Alert system is active and monitoring thresholds\n";
        std::cout << "Default rules configured for:\n";
        std::cout << "  - CPU Usage: >80% (Warning), >95% (Critical)\n";
        std::cout << "  - Memory Usage: >85% (Warning), >95% (Critical)\n";
        std::cout << "  - Temperature: >75°C (Warning), >85°C (Critical)\n";
        std::cout << "  - Battery: <20% (Warning), <10% (Critical)\n";
        
        // Add a custom alert rule
        AlertRule customRule;
        customRule.name = "Demo_High_Network_Usage";
        customRule.resourceType = ResourceType::NETWORK_BANDWIDTH;
        customRule.threshold = 30.0;
        customRule.level = AlertLevel::WARNING;
        customRule.condition = [](const SystemMetric& metric) { return metric.value > 30.0; };
        customRule.description = "High network bandwidth usage detected";
        
        alertManager->addRule(customRule);
        std::cout << "Added custom network usage alert rule\n";
        
        // Simulate checking for alerts
        std::cout << "Monitoring for alerts (checking every 2 seconds)...\n";
        for (int i = 0; i < 5; ++i) {
            auto activeAlerts = systemMonitor->getActiveAlerts();
            if (activeAlerts.empty()) {
                std::cout << "No active alerts at sample " << (i + 1) << "\n";
            } else {
                std::cout << "Active alerts found at sample " << (i + 1) << ":\n";
                for (const auto& alert : activeAlerts) {
                    std::cout << "  - " << alertLevelToString(alert.severity) 
                              << ": " << alert.message << "\n";
                }
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
    
    void demonstrateEVMonitoring() {
        std::cout << "\n--- EV-Specific Monitoring Demo ---\n";
        
        systemMonitor->enableEVSpecificMonitoring();
        std::cout << "EV-specific monitoring enabled\n";
        
        auto resourceMonitor = systemMonitor->getResourceMonitor();
        if (resourceMonitor) {
            std::cout << "EV-specific metrics:\n";
            
            // Display EV-specific metrics
            auto batteryMetric = resourceMonitor->getCurrentMetric(ResourceType::BATTERY_LEVEL);
            std::cout << "  Battery Level: " << batteryMetric.value << "%\n";
            
            auto chargingMetric = resourceMonitor->getCurrentMetric(ResourceType::CHARGING_RATE);
            std::cout << "  Charging Rate: " << chargingMetric.value << " kW\n";
            
            auto motorTempMetric = resourceMonitor->getCurrentMetric(ResourceType::MOTOR_TEMPERATURE);
            std::cout << "  Motor Temperature: " << motorTempMetric.value << "°C\n";
            
            // Calculate EV-specific scores
            double batteryHealth = systemMonitor->getBatteryHealthScore();
            double chargingEfficiency = systemMonitor->getChargingEfficiency();
            double thermalScore = systemMonitor->getThermalManagementScore();
            
            std::cout << "\nEV Health Scores:\n";
            std::cout << "  Battery Health: " << batteryHealth << "/100\n";
            std::cout << "  Charging Efficiency: " << chargingEfficiency << "/100\n";
            std::cout << "  Thermal Management: " << thermalScore << "/100\n";
            
            // Get maintenance recommendations
            auto recommendations = systemMonitor->getMaintenanceRecommendations();
            if (!recommendations.empty()) {
                std::cout << "\nMaintenance Recommendations:\n";
                for (const auto& rec : recommendations) {
                    std::cout << "  - " << rec << "\n";
                }
            }
        }
    }
};

int main() {
    try {
        RealTimeMonitorDemo demo;
        demo.runDemo();
    } catch (const std::exception& e) {
        std::cerr << "Demo error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}