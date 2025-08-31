/**
 * @file advanced_utilities_demo.cpp
 * @author adzetto
 * @brief Demonstration of Advanced Utility Systems for Electric Vehicle Applications
 * @version 1.0
 * @date 2025-08-31
 * 
 * @copyright Copyright (c) 2025
 * 
 * @details This program demonstrates the comprehensive advanced utility systems
 * including data analytics, networking, cryptographic security, signal processing,
 * memory management, configuration management, and logging framework.
 */

#include "advanced_data_analytics.h"
#include "advanced_network_stack.h"
#include "advanced_cryptographic_security.h"
#include "advanced_signal_processing.h"
#include "advanced_memory_management.h"
#include "advanced_configuration_management.h"
#include "advanced_logging_framework.h"

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <random>

using namespace std::chrono_literals;

class AdvancedUtilitiesDemo {
private:
    // System components
    std::unique_ptr<analytics::AdvancedDataAnalyticsSystem> dataAnalyticsSystem;
    std::unique_ptr<network::AdvancedNetworkSystem> networkSystem;
    std::unique_ptr<security::AdvancedCryptographicSecuritySystem> securitySystem;
    std::unique_ptr<signal_processing::AdvancedSignalProcessingSystem> signalProcessingSystem;
    std::unique_ptr<memory_management::AdvancedMemoryManagementSystem> memorySystem;
    std::unique_ptr<configuration_management::AdvancedConfigurationManagementSystem> configSystem;
    logging_framework::Logger* logger;
    
    // Demo data
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<double> dist;
    
public:
    AdvancedUtilitiesDemo() : gen(rd()), dist(0.0, 100.0) {
        // Initialize logging first
        logger = logging_framework::createLogger("AdvancedUtilitiesDemo");
        setupLogging();
        
        LOG_INFO(logger, "Initializing Advanced Utilities Demo System");
        
        // Initialize all subsystems
        initializeSubsystems();
    }
    
    ~AdvancedUtilitiesDemo() {
        LOG_INFO(logger, "Shutting down Advanced Utilities Demo System");
        shutdownSubsystems();
    }
    
    void runDemo() {
        std::cout << "=== Advanced Utilities Demo for Electric Vehicle Systems ===\n";
        std::cout << "Author: adzetto\n";
        std::cout << "Version: 1.0\n";
        std::cout << "Date: 2025-08-31\n\n";
        
        LOG_INFO(logger, "Starting comprehensive demo");
        
        // Start all systems
        startAllSystems();
        
        // Run demonstrations
        demonstrateDataAnalytics();
        demonstrateNetworking();
        demonstrateSecurity();
        demonstrateSignalProcessing();
        demonstrateMemoryManagement();
        demonstrateConfiguration();
        demonstrateLogging();
        
        // Show system integration
        demonstrateSystemIntegration();
        
        // Performance and monitoring
        demonstratePerformanceMonitoring();
        
        // Stop all systems
        stopAllSystems();
        
        std::cout << "\n=== Demo Completed Successfully ===\n";
        LOG_INFO(logger, "Demo completed successfully");
    }

private:
    void setupLogging() {
        // Configure advanced logging
        logger->setMinimumLevel(logging_framework::LogLevel::DEBUG);
        logger->setAsynchronous(true);
        logger->setBufferSize(10000);
        
        // Add file destination
        auto fileDestination = std::make_unique<logging_framework::FileLogDestination>(
            "advanced_utilities_demo.log",
            logging_framework::LogRotationPolicy::SIZE_BASED,
            5 * 1024 * 1024  // 5MB max file size
        );
        logger->addDestination(std::move(fileDestination));
        
        // Add JSON formatter for structured logging
        auto jsonFormatter = std::make_unique<logging_framework::JSONFormatter>(true);
        logger->setFormatter(std::move(jsonFormatter));
        
        logger->start();
    }
    
    void initializeSubsystems() {
        dataAnalyticsSystem = std::make_unique<analytics::AdvancedDataAnalyticsSystem>();
        networkSystem = std::make_unique<network::AdvancedNetworkSystem>();
        securitySystem = std::make_unique<security::AdvancedCryptographicSecuritySystem>();
        signalProcessingSystem = std::make_unique<signal_processing::AdvancedSignalProcessingSystem>();
        memorySystem = std::make_unique<memory_management::AdvancedMemoryManagementSystem>();
        configSystem = std::make_unique<configuration_management::AdvancedConfigurationManagementSystem>();
        
        LOG_INFO(logger, "All subsystems initialized successfully");
    }
    
    void startAllSystems() {
        std::cout << "Starting all advanced utility systems...\n";
        
        dataAnalyticsSystem->start();
        networkSystem->start();
        securitySystem->start();
        signalProcessingSystem->start();
        memorySystem->start();
        configSystem->start();
        
        // Wait for systems to fully start
        std::this_thread::sleep_for(2s);
        
        std::cout << "All systems started successfully!\n\n";
        LOG_INFO(logger, "All systems started successfully");
    }
    
    void stopAllSystems() {
        std::cout << "Stopping all advanced utility systems...\n";
        
        dataAnalyticsSystem->stop();
        networkSystem->stop();
        securitySystem->stop();
        signalProcessingSystem->stop();
        memorySystem->stop();
        configSystem->stop();
        
        std::cout << "All systems stopped gracefully.\n";
        LOG_INFO(logger, "All systems stopped gracefully");
    }
    
    void shutdownSubsystems() {
        dataAnalyticsSystem.reset();
        networkSystem.reset();
        securitySystem.reset();
        signalProcessingSystem.reset();
        memorySystem.reset();
        configSystem.reset();
        
        if (logger) {
            logger->stop();
        }
    }
    
    void demonstrateDataAnalytics() {
        std::cout << "=== Data Analytics System Demo ===\n";
        LOG_INFO(logger, "Starting Data Analytics demonstration");
        
        // Generate sample EV data
        std::vector<analytics::DataPoint> batteryVoltageData;
        std::vector<analytics::DataPoint> motorCurrentData;
        std::vector<analytics::DataPoint> temperatureData;
        
        for (int i = 0; i < 1000; ++i) {
            double voltage = 350.0 + 10.0 * std::sin(i * 0.1) + dist(gen) * 0.1;
            double current = 100.0 + 50.0 * std::cos(i * 0.05) + dist(gen) * 0.5;
            double temperature = 25.0 + 15.0 * std::sin(i * 0.02) + dist(gen) * 0.2;
            
            batteryVoltageData.emplace_back("battery_voltage", voltage);
            motorCurrentData.emplace_back("motor_current", current);
            temperatureData.emplace_back("temperature", temperature);
            
            dataAnalyticsSystem->ingestDataPoint(batteryVoltageData.back());
            dataAnalyticsSystem->ingestDataPoint(motorCurrentData.back());
            dataAnalyticsSystem->ingestDataPoint(temperatureData.back());
        }
        
        // Perform statistical analysis
        auto batteryStats = dataAnalyticsSystem->getStatisticalSummary("battery_voltage");
        auto motorStats = dataAnalyticsSystem->getStatisticalSummary("motor_current");
        
        std::cout << "Battery Voltage Statistics:\n";
        std::cout << "  Mean: " << batteryStats.mean << " V\n";
        std::cout << "  Std Dev: " << batteryStats.stddev << " V\n";
        std::cout << "  Min/Max: " << batteryStats.min << "/" << batteryStats.max << " V\n";
        
        std::cout << "Motor Current Statistics:\n";
        std::cout << "  Mean: " << motorStats.mean << " A\n";
        std::cout << "  Std Dev: " << motorStats.stddev << " A\n";
        std::cout << "  Min/Max: " << motorStats.min << "/" << motorStats.max << " A\n";
        
        // Predictive analysis
        auto prediction = dataAnalyticsSystem->getPrediction("battery_voltage", 50);
        std::cout << "Battery voltage prediction (next 50 samples): " << prediction.predictions.size() << " values\n";
        std::cout << "Prediction accuracy: " << (prediction.accuracy * 100.0) << "%\n";
        
        // Anomaly detection
        auto anomalies = dataAnalyticsSystem->getAnomalies("temperature");
        std::cout << "Temperature anomalies detected: " << anomalies.size() << "\n";
        
        std::cout << "Data Analytics Demo completed.\n\n";
        LOG_INFO(logger, "Data Analytics demonstration completed");
    }
    
    void demonstrateNetworking() {
        std::cout << "=== Network Stack System Demo ===\n";
        LOG_INFO(logger, "Starting Network Stack demonstration");
        
        // Enable HTTP server
        networkSystem->enableHTTPServer(true, network::NetworkEndpoint("localhost", 8080, network::ProtocolType::HTTP));
        
        // Configure HTTP server
        std::cout << "HTTP server enabled on localhost:8080\n";
        
        // Demonstrate load balancing
        networkSystem->addBackendServer(network::NetworkEndpoint("server1.example.com", 80), 1);
        networkSystem->addBackendServer(network::NetworkEndpoint("server2.example.com", 80), 2);
        networkSystem->setLoadBalancingStrategy(network::LoadBalancingStrategy::WEIGHTED_ROUND_ROBIN);
        
        auto selectedEndpoint = networkSystem->getBalancedEndpoint();
        std::cout << "Load balancer selected: " << selectedEndpoint.toString() << "\n";
        
        // Create QoS flow
        auto flowId = networkSystem->createQoSFlow(
            network::NetworkEndpoint("vehicle.local", 5000),
            network::NetworkEndpoint("cloud.service.com", 443),
            network::QoSLevel::CRITICAL
        );
        std::cout << "Created QoS flow: " << flowId << "\n";
        
        // Protocol analysis
        networkSystem->startPacketCapture();
        std::this_thread::sleep_for(1s);
        networkSystem->stopPacketCapture();
        
        std::cout << "Network Stack Demo completed.\n\n";
        LOG_INFO(logger, "Network Stack demonstration completed");
    }
    
    void demonstrateSecurity() {
        std::cout << "=== Cryptographic Security System Demo ===\n";
        LOG_INFO(logger, "Starting Cryptographic Security demonstration");
        
        // Generate encryption key
        auto encryptionKeyId = securitySystem->generateEncryptionKey(
            security::CipherType::AES_256, 
            security::SecurityLevel::HIGH
        );
        std::cout << "Generated encryption key: " << encryptionKeyId << "\n";
        
        // Encrypt some data
        std::string testData = "This is sensitive EV telemetry data that needs encryption";
        std::vector<uint8_t> dataToEncrypt(testData.begin(), testData.end());
        
        auto encryptedData = securitySystem->encryptData(dataToEncrypt, encryptionKeyId);
        std::cout << "Data encrypted successfully. Ciphertext size: " << encryptedData.ciphertext.size() << " bytes\n";
        
        // Decrypt the data
        auto decryptedData = securitySystem->decryptData(encryptedData, encryptionKeyId);
        std::string decryptedString(decryptedData.begin(), decryptedData.end());
        std::cout << "Data decrypted: " << (decryptedString == testData ? "SUCCESS" : "FAILED") << "\n";
        
        // Generate digital signature key
        auto signatureKeyId = securitySystem->generateSigningKey(
            security::SignatureAlgorithm::RSA_PSS, 
            security::SecurityLevel::HIGH
        );
        std::cout << "Generated signature key: " << signatureKeyId << "\n";
        
        // Sign data
        auto signature = securitySystem->signData(dataToEncrypt, signatureKeyId);
        std::cout << "Data signed successfully. Signature size: " << signature.signature.size() << " bytes\n";
        
        // Verify signature
        bool isValid = securitySystem->verifySignature(dataToEncrypt, signature);
        std::cout << "Signature verification: " << (isValid ? "VALID" : "INVALID") << "\n";
        
        // Establish secure channel
        auto channelId = securitySystem->establishSecureChannel("remote.server.com:443");
        std::cout << "Secure channel established: " << channelId << "\n";
        
        // Get security report
        auto threats = securitySystem->getActiveThreats();
        std::cout << "Active security threats: " << threats.size() << "\n";
        
        std::cout << "Cryptographic Security Demo completed.\n\n";
        LOG_INFO(logger, "Cryptographic Security demonstration completed");
    }
    
    void demonstrateSignalProcessing() {
        std::cout << "=== Signal Processing System Demo ===\n";
        LOG_INFO(logger, "Starting Signal Processing demonstration");
        
        // Generate test signals
        double samplingRate = 1000.0; // 1kHz
        double duration = 2.0; // 2 seconds
        
        // Generate sine wave with noise
        auto sineWave = signalProcessingSystem->generateSineWave(50.0, 1.0, duration, samplingRate);
        auto noise = signalProcessingSystem->generateWhiteNoise(0.1, duration, samplingRate);
        
        // Add noise to signal
        auto noisySignal = sineWave;
        for (size_t i = 0; i < noisySignal.size() && i < noise.size(); ++i) {
            noisySignal[i] += noise[i];
        }
        
        signalProcessingSystem->addSignal("clean_sine", sineWave, samplingRate);
        signalProcessingSystem->addSignal("noisy_sine", noisySignal, samplingRate);
        
        // Design and apply filter
        auto filterId = signalProcessingSystem->createDigitalFilter(
            "lowpass_filter",
            signal_processing::FilterType::LOW_PASS,
            signal_processing::FilterDesign::BUTTERWORTH,
            4, // order
            100.0, // cutoff frequency
            samplingRate
        );
        
        auto filteredSignal = signalProcessingSystem->applyDigitalFilter("noisy_sine", filterId);
        signalProcessingSystem->addSignal("filtered_sine", filteredSignal, samplingRate);
        
        // Perform FFT analysis
        auto spectrum = signalProcessingSystem->performFFTAnalysis("noisy_sine");
        std::cout << "FFT analysis completed. Found " << spectrum.harmonics.size() << " harmonics\n";
        std::cout << "Fundamental frequency: " << spectrum.fundamentalFrequency << " Hz\n";
        std::cout << "Spectral centroid: " << spectrum.spectralCentroid << " Hz\n";
        
        // Wavelet analysis
        auto waveletResult = signalProcessingSystem->performWaveletDecomposition("noisy_sine", 
            signal_processing::WaveletType::DAUBECHIES);
        std::cout << "Wavelet decomposition completed. Approximation size: " << waveletResult.first.size() << "\n";
        
        // Perform denoising
        auto denoisedSignal = signalProcessingSystem->performWaveletDenoising("noisy_sine", 
            signal_processing::WaveletType::DAUBECHIES, 4, 0.1);
        signalProcessingSystem->addSignal("denoised_sine", denoisedSignal, samplingRate);
        
        // Signal analysis
        auto signalProps = signalProcessingSystem->analyzeSignalProperties(noisySignal, samplingRate);
        std::cout << "Signal Properties:\n";
        std::cout << "  RMS Value: " << signalProps.rmsValue << "\n";
        std::cout << "  Peak Value: " << signalProps.peakValue << "\n";
        std::cout << "  Crest Factor: " << signalProps.crestFactor << "\n";
        
        // Create real-time processing chain
        auto chainId = signalProcessingSystem->createRealTimeChain("motor_analysis", samplingRate);
        signalProcessingSystem->addRealTimeFilter(chainId, filterId);
        signalProcessingSystem->activateRealTimeChain(chainId, true);
        
        // Process some real-time samples
        for (int i = 0; i < 100; ++i) {
            double sample = std::sin(2.0 * M_PI * 60.0 * i / samplingRate) + 0.1 * dist(gen);
            signalProcessingSystem->inputRealTimeSample(chainId, sample);
            double processed = signalProcessingSystem->outputRealTimeSample(chainId);
            (void)processed; // Suppress unused variable warning
        }
        
        std::cout << "Signal Processing Demo completed.\n\n";
        LOG_INFO(logger, "Signal Processing demonstration completed");
    }
    
    void demonstrateMemoryManagement() {
        std::cout << "=== Memory Management System Demo ===\n";
        LOG_INFO(logger, "Starting Memory Management demonstration");
        
        // Create different types of allocators
        auto poolId = memorySystem->createPoolAllocator("test_pool", 1024 * 1024, 1024); // 1MB pool, 1KB blocks
        auto stackId = memorySystem->createStackAllocator("test_stack", 512 * 1024); // 512KB stack
        auto ringId = memorySystem->createRingBufferAllocator("test_ring", 256 * 1024, 256); // 256KB ring, 256B elements
        auto buddyId = memorySystem->createBuddyAllocator("test_buddy", 1024 * 1024); // 1MB buddy system
        
        std::cout << "Created allocators: " << poolId << ", " << stackId << ", " << ringId << ", " << buddyId << "\n";
        
        // Test allocations
        void* ptr1 = memorySystem->allocate(poolId, 512);
        void* ptr2 = memorySystem->allocate(stackId, 1024);
        void* ptr3 = memorySystem->allocate(ringId, 256);
        void* ptr4 = memorySystem->allocate(buddyId, 2048);
        
        std::cout << "Allocated memory from all allocators successfully\n";
        
        // Cache some data
        std::string cacheData = "This is cached EV configuration data";
        std::vector<uint8_t> dataToCache(cacheData.begin(), cacheData.end());
        
        bool cached = memorySystem->cacheData("config_cache", dataToCache.data(), dataToCache.size());
        std::cout << "Data cached: " << (cached ? "SUCCESS" : "FAILED") << "\n";
        
        // Retrieve cached data
        size_t retrievedSize;
        void* retrievedData = memorySystem->getCachedData("config_cache", retrievedSize);
        if (retrievedData && retrievedSize == dataToCache.size()) {
            std::string retrievedString(static_cast<char*>(retrievedData), retrievedSize);
            std::cout << "Cache retrieval: " << (retrievedString == cacheData ? "SUCCESS" : "FAILED") << "\n";
        }
        
        // Memory mapping
        void* mappedMemory = memorySystem->mapMemory("test_mapping", 64 * 1024);
        std::cout << "Memory mapped: " << (mappedMemory ? "SUCCESS" : "FAILED") << "\n";
        
        // Start memory profiling
        memorySystem->startMemoryProfiling();
        
        // Perform some allocations for profiling
        std::vector<void*> testAllocations;
        for (int i = 0; i < 100; ++i) {
            void* ptr = memorySystem->allocate(poolId, 64 + (i % 512));
            if (ptr) {
                testAllocations.push_back(ptr);
            }
        }
        
        // Clean up some allocations
        for (size_t i = 0; i < testAllocations.size() / 2; ++i) {
            memorySystem->deallocate(poolId, testAllocations[i]);
        }
        
        // Get memory analysis
        auto analysis = memorySystem->getMemoryAnalysis();
        std::cout << "Memory Analysis:\n";
        std::cout << "  Total Allocations: " << analysis.totalAllocations << "\n";
        std::cout << "  Active Allocations: " << analysis.activeAllocations << "\n";
        std::cout << "  Total Bytes Allocated: " << analysis.totalBytesAllocated << "\n";
        std::cout << "  Average Allocation Size: " << analysis.averageAllocationSize << " bytes\n";
        
        // Detect memory leaks
        auto leaks = memorySystem->detectMemoryLeaks();
        std::cout << "Memory leaks detected: " << leaks.size() << "\n";
        
        // Get system statistics
        auto stats = memorySystem->getAllocatorStatistics();
        std::cout << "Allocator statistics collected for " << stats.size() << " allocators\n";
        
        // Cleanup
        for (void* ptr : testAllocations) {
            memorySystem->deallocate(poolId, ptr);
        }
        
        memorySystem->deallocate(poolId, ptr1);
        memorySystem->deallocate(stackId, ptr2);
        memorySystem->deallocate(ringId, ptr3);
        memorySystem->deallocate(buddyId, ptr4);
        
        memorySystem->unmapMemory("test_mapping");
        memorySystem->stopMemoryProfiling();
        
        std::cout << "Memory Management Demo completed.\n\n";
        LOG_INFO(logger, "Memory Management demonstration completed");
    }
    
    void demonstrateConfiguration() {
        std::cout << "=== Configuration Management System Demo ===\n";
        LOG_INFO(logger, "Starting Configuration Management demonstration");
        
        // Set various configuration values
        configSystem->setConfigValue("vehicle.battery.max_voltage", 400.0);
        configSystem->setConfigValue("vehicle.battery.min_voltage", 300.0);
        configSystem->setConfigValue("vehicle.motor.max_current", 200.0);
        configSystem->setConfigValue("vehicle.thermal.max_temperature", 80.0);
        configSystem->setConfigValue("system.logging.level", std::string("DEBUG"));
        configSystem->setConfigValue("system.network.timeout", 30000);
        configSystem->setConfigValue("system.security.encryption_enabled", true);
        
        // Create configuration sections
        configSystem->createConfigSection("vehicle.charging");
        configSystem->setConfigValue("vehicle.charging.max_power", 150.0);
        configSystem->setConfigValue("vehicle.charging.efficiency", 0.95);
        
        configSystem->createConfigSection("system.performance");
        configSystem->setConfigValue("system.performance.cpu_limit", 80.0);
        configSystem->setConfigValue("system.performance.memory_limit", 4096);
        
        // Retrieve configuration values
        auto maxVoltage = configSystem->getConfigValue<double>("vehicle.battery.max_voltage", 0.0);
        auto logLevel = configSystem->getConfigValue<std::string>("system.logging.level", "INFO");
        auto encryptionEnabled = configSystem->getConfigValue<bool>("system.security.encryption_enabled", false);
        
        std::cout << "Configuration values:\n";
        std::cout << "  Max Battery Voltage: " << maxVoltage << " V\n";
        std::cout << "  Logging Level: " << logLevel << "\n";
        std::cout << "  Encryption Enabled: " << (encryptionEnabled ? "Yes" : "No") << "\n";
        
        // List configuration sections and keys
        auto sections = configSystem->getConfigSections();
        std::cout << "Configuration sections: ";
        for (const auto& section : sections) {
            std::cout << section << " ";
        }
        std::cout << "\n";
        
        auto vehicleKeys = configSystem->getConfigKeys("vehicle.battery");
        std::cout << "Vehicle battery configuration keys: ";
        for (const auto& key : vehicleKeys) {
            std::cout << key << " ";
        }
        std::cout << "\n";
        
        // Save configuration
        try {
            configSystem->saveConfiguration("demo_config.json", "json");
            std::cout << "Configuration saved to demo_config.json\n";
        } catch (const std::exception& e) {
            std::cout << "Configuration save failed: " << e.what() << "\n";
        }
        
        // Export configuration
        auto exportedConfig = configSystem->exportConfiguration("json");
        std::cout << "Configuration exported (" << exportedConfig.size() << " bytes)\n";
        
        // Create backup
        configSystem->createBackup("demo_backup");
        std::cout << "Configuration backup created\n";
        
        // Register event callback
        configSystem->registerEventCallback(
            configuration_management::ConfigEventType::VALUE_CHANGED,
            [](const configuration_management::ConfigEvent& event) {
                std::cout << "Config changed: " << event.configPath << "\n";
            }
        );
        
        // Modify a value to trigger callback
        configSystem->setConfigValue("vehicle.battery.max_voltage", 420.0);
        
        std::cout << "Configuration Management Demo completed.\n\n";
        LOG_INFO(logger, "Configuration Management demonstration completed");
    }
    
    void demonstrateLogging() {
        std::cout << "=== Logging Framework Demo ===\n";
        LOG_INFO(logger, "Starting Logging Framework demonstration");
        
        // Create specialized loggers
        auto securityLogger = logging_framework::createLogger("Security");
        auto performanceLogger = logging_framework::createLogger("Performance");
        auto diagnosticsLogger = logging_framework::createLogger("Diagnostics");
        
        // Configure security logger with different format
        securityLogger->setMinimumLevel(logging_framework::LogLevel::SECURITY);
        auto plainFormatter = std::make_unique<logging_framework::PlainTextFormatter>();
        securityLogger->setFormatter(std::move(plainFormatter));
        
        // Add memory destination to performance logger
        auto memoryDestination = std::make_unique<logging_framework::MemoryLogDestination>(1000, true);
        performanceLogger->addDestination(std::move(memoryDestination));
        
        // Start specialized loggers
        securityLogger->start();
        performanceLogger->start();
        diagnosticsLogger->start();
        
        // Demonstrate various log levels and structured logging
        logger->trace("This is a trace message for debugging");
        logger->debug("System initialization complete");
        logger->info("Vehicle system started successfully");
        logger->warn("Battery temperature approaching upper limit");
        logger->error("Motor controller communication timeout");
        logger->fatal("Critical system failure detected");
        logger->audit("User authentication successful");
        logger->metric("Battery charge level: 85%");
        logger->security("Unauthorized access attempt detected");
        
        // Structured logging with fields and tags
        std::unordered_map<std::string, std::any> fields;
        fields["battery_voltage"] = 385.7;
        fields["motor_current"] = 127.3;
        fields["temperature"] = 42.8;
        
        std::unordered_map<std::string, std::string> tags;
        tags["component"] = "BatteryManagement";
        tags["severity"] = "medium";
        tags["action_required"] = "monitor";
        
        logger->logStructured(
            logging_framework::LogLevel::INFO,
            "Battery system status update",
            fields,
            tags,
            "VehicleDiagnostics",
            "BatteryController"
        );
        
        // Log with specialized loggers
        securityLogger->security("Intrusion detection system activated");
        performanceLogger->metric("CPU utilization: 67%");
        diagnosticsLogger->info("Diagnostic scan completed successfully");
        
        // Wait for async logging
        std::this_thread::sleep_for(1s);
        
        // Get logger metrics
        auto metrics = logger->getMetrics();
        std::cout << "Logging metrics:\n";
        std::cout << "  Total entries: " << metrics.totalLogEntries << "\n";
        std::cout << "  Dropped entries: " << metrics.droppedLogEntries << "\n";
        std::cout << "  Errors: " << metrics.errorCount << "\n";
        std::cout << "  Entries per second: " << metrics.entriesPerSecond << "\n";
        std::cout << "  Average processing time: " << metrics.averageProcessingTime << " μs\n";
        
        // Stop specialized loggers
        securityLogger->stop();
        performanceLogger->stop();
        diagnosticsLogger->stop();
        
        std::cout << "Logging Framework Demo completed.\n\n";
        LOG_INFO(logger, "Logging Framework demonstration completed");
    }
    
    void demonstrateSystemIntegration() {
        std::cout << "=== System Integration Demo ===\n";
        LOG_INFO(logger, "Starting System Integration demonstration");
        
        // Simulate integrated EV data processing workflow
        
        // 1. Generate sensor data
        auto motorCurrentSignal = signalProcessingSystem->generateSineWave(60.0, 100.0, 1.0, 1000.0);
        auto batteryVoltageSignal = signalProcessingSystem->generateSineWave(0.1, 350.0, 10.0, 100.0);
        
        // 2. Process signals in real-time
        auto signalChain = signalProcessingSystem->createRealTimeChain("integration_demo", 1000.0);
        auto filterName = signalProcessingSystem->createDigitalFilter("integration_filter",
            signal_processing::FilterType::LOW_PASS,
            signal_processing::FilterDesign::BUTTERWORTH, 4, 30.0, 1000.0);
        signalProcessingSystem->addRealTimeFilter(signalChain, filterName);
        
        // 3. Store processed data with memory management
        auto dataPoolId = memorySystem->createPoolAllocator("sensor_data", 64 * 1024, 64);
        std::vector<void*> sensorDataBuffers;
        
        for (size_t i = 0; i < std::min(motorCurrentSignal.size(), size_t(100)); ++i) {
            void* buffer = memorySystem->allocate(dataPoolId, sizeof(double));
            if (buffer) {
                *static_cast<double*>(buffer) = motorCurrentSignal[i];
                sensorDataBuffers.push_back(buffer);
                
                // Feed to analytics
                analytics::DataPoint point("motor_current_filtered", motorCurrentSignal[i]);
                dataAnalyticsSystem->ingestDataPoint(point);
            }
        }
        
        // 4. Apply security to sensitive data
        std::string sensitiveData = "Vehicle Location: 40.7128 N, 74.0060 W";
        std::vector<uint8_t> dataToSecure(sensitiveData.begin(), sensitiveData.end());
        
        auto secureKeyId = securitySystem->generateEncryptionKey(security::CipherType::AES_256);
        auto secureData = securitySystem->encryptData(dataToSecure, secureKeyId);
        
        // 5. Store configuration for the integration
        configSystem->setConfigValue("integration.demo.active", true);
        configSystem->setConfigValue("integration.demo.sample_rate", 1000.0);
        configSystem->setConfigValue("integration.demo.filter_cutoff", 30.0);
        
        // 6. Log the integrated operation
        std::unordered_map<std::string, std::any> integrationFields;
        integrationFields["processed_samples"] = static_cast<int>(sensorDataBuffers.size());
        integrationFields["encrypted_data_size"] = static_cast<int>(secureData.ciphertext.size());
        integrationFields["memory_allocated"] = static_cast<int>(sensorDataBuffers.size() * sizeof(double));
        
        logger->logStructured(
            logging_framework::LogLevel::INFO,
            "Integrated system processing completed",
            integrationFields,
            {{"operation", "sensor_processing"}, {"security_level", "high"}},
            "SystemIntegration",
            "DemoController"
        );
        
        // 7. Network communication of results
        networkSystem->enableLoadBalancing(true);
        auto balancedEndpoint = networkSystem->getBalancedEndpoint();
        
        // 8. Analytics on the complete workflow
        auto statistics = dataAnalyticsSystem->getStatisticalSummary("motor_current_filtered");
        std::cout << "Integrated processing results:\n";
        std::cout << "  Processed samples: " << sensorDataBuffers.size() << "\n";
        std::cout << "  Signal mean: " << statistics.mean << "\n";
        std::cout << "  Signal std dev: " << statistics.stddev << "\n";
        std::cout << "  Encrypted data size: " << secureData.ciphertext.size() << " bytes\n";
        std::cout << "  Load balanced endpoint: " << balancedEndpoint.toString() << "\n";
        
        // Cleanup
        for (void* buffer : sensorDataBuffers) {
            memorySystem->deallocate(dataPoolId, buffer);
        }
        
        std::cout << "System Integration Demo completed.\n\n";
        LOG_INFO(logger, "System Integration demonstration completed");
    }
    
    void demonstratePerformanceMonitoring() {
        std::cout << "=== Performance Monitoring Demo ===\n";
        LOG_INFO(logger, "Starting Performance Monitoring demonstration");
        
        // Collect performance metrics from all systems
        std::cout << "System Performance Metrics:\n\n";
        
        // Data Analytics System
        std::cout << dataAnalyticsSystem->getSystemStatus() << "\n";
        
        // Network System
        std::cout << networkSystem->getSystemStatus() << "\n";
        
        // Security System
        std::cout << securitySystem->getSecurityStatus() << "\n";
        
        // Signal Processing System
        std::cout << signalProcessingSystem->getSystemStatus() << "\n";
        
        // Memory Management System
        std::cout << memorySystem->getSystemStatus() << "\n";
        
        // Configuration Management System
        std::cout << configSystem->getSystemStatus() << "\n";
        
        // Logging System
        auto& loggerManager = logging_framework::getGlobalLoggerManager();
        std::cout << loggerManager.getSystemStatus() << "\n";
        
        // Overall system health check
        bool allSystemsHealthy = 
            dataAnalyticsSystem->isRunning() &&
            networkSystem->isNetworkSystemRunning() &&
            securitySystem->isSystemRunning() &&
            signalProcessingSystem->isRunning() &&
            memorySystem->isRunning() &&
            configSystem->isRunning();
        
        std::cout << "Overall System Health: " << (allSystemsHealthy ? "HEALTHY" : "DEGRADED") << "\n";
        
        // Resource utilization summary
        auto memStats = memorySystem->getAllocatorStatistics();
        size_t totalMemoryUsed = 0;
        for (const auto& stat : memStats) {
            totalMemoryUsed += stat.second.currentAllocatedBytes;
        }
        
        std::cout << "Resource Utilization Summary:\n";
        std::cout << "  Total Memory Used: " << (totalMemoryUsed / 1024) << " KB\n";
        std::cout << "  Active Allocators: " << memStats.size() << "\n";
        std::cout << "  Configuration Items: " << configSystem->getConfigurationSize() << "\n";
        
        std::cout << "Performance Monitoring Demo completed.\n\n";
        LOG_INFO(logger, "Performance Monitoring demonstration completed");
    }
};

int main() {
    try {
        AdvancedUtilitiesDemo demo;
        demo.runDemo();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Demo failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Demo failed with unknown exception" << std::endl;
        return 1;
    }
}