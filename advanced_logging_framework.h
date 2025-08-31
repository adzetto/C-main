/**
 * @file advanced_logging_framework.h
 * @author adzetto
 * @brief Advanced Logging Framework for Electric Vehicle Systems
 * @version 1.0
 * @date 2025-08-31
 * 
 * @copyright Copyright (c) 2025
 * 
 * @details This module provides comprehensive logging capabilities including
 * multi-level logging, structured logging, log rotation, remote logging,
 * performance monitoring, and real-time log analysis.
 */

#ifndef ADVANCED_LOGGING_FRAMEWORK_H
#define ADVANCED_LOGGING_FRAMEWORK_H

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

namespace logging_framework {

// Logging levels and types
enum class LogLevel {
    TRACE,      // Detailed information for debugging
    DEBUG,      // Debug information
    INFO,       // General information
    WARN,       // Warning conditions
    ERROR,      // Error conditions
    FATAL,      // Fatal error conditions
    AUDIT,      // Audit trail
    METRIC,     // Performance metrics
    SECURITY,   // Security events
    CUSTOM      // Custom log levels
};

enum class LogFormat {
    PLAIN_TEXT,     // Simple text format
    JSON,           // JSON structured format
    XML,            // XML format
    CSV,            // Comma-separated values
    BINARY,         // Binary format for performance
    CUSTOM          // Custom formatter
};

enum class LogDestination {
    CONSOLE,        // Standard output/error
    FILE,           // Log file
    SYSLOG,         // System log daemon
    DATABASE,       // Database storage
    NETWORK,        // Network/remote logging
    MEMORY,         // In-memory buffer
    CUSTOM          // Custom destination
};

enum class LogRotationPolicy {
    SIZE_BASED,     // Rotate based on file size
    TIME_BASED,     // Rotate based on time intervals
    DATE_BASED,     // Rotate daily/weekly/monthly
    COUNT_BASED,    // Rotate after N log entries
    HYBRID,         // Combination of policies
    NONE            // No rotation
};

enum class LogCompression {
    NONE,
    GZIP,
    BZIP2,
    LZ4,
    CUSTOM
};

struct LogEntry {
    LogLevel level;
    std::chrono::system_clock::time_point timestamp;
    std::string message;
    std::string category;
    std::string component;
    std::string threadId;
    std::string processId;
    std::string fileName;
    int lineNumber;
    std::string functionName;
    std::unordered_map<std::string, std::any> fields;  // Structured fields
    std::unordered_map<std::string, std::string> tags;
    size_t sequenceNumber;
    
    LogEntry() : level(LogLevel::INFO), 
                timestamp(std::chrono::system_clock::now()),
                lineNumber(0), sequenceNumber(0) {}
    
    template<typename T>
    void addField(const std::string& key, const T& value) {
        fields[key] = value;
    }
    
    template<typename T>
    T getField(const std::string& key, const T& defaultValue = T{}) const {
        auto it = fields.find(key);
        if (it != fields.end()) {
            try {
                return std::any_cast<T>(it->second);
            } catch (...) {
                return defaultValue;
            }
        }
        return defaultValue;
    }
    
    void addTag(const std::string& key, const std::string& value) {
        tags[key] = value;
    }
    
    std::string getTag(const std::string& key, const std::string& defaultValue = "") const {
        auto it = tags.find(key);
        return (it != tags.end()) ? it->second : defaultValue;
    }
};

struct LogFilter {
    std::vector<LogLevel> allowedLevels;
    std::vector<std::string> allowedCategories;
    std::vector<std::string> allowedComponents;
    std::regex messagePattern;
    std::function<bool(const LogEntry&)> customFilter;
    bool isEnabled;
    
    LogFilter() : isEnabled(true) {}
    
    bool shouldLog(const LogEntry& entry) const {
        if (!isEnabled) return false;
        
        // Check level filter
        if (!allowedLevels.empty()) {
            if (std::find(allowedLevels.begin(), allowedLevels.end(), entry.level) == allowedLevels.end()) {
                return false;
            }
        }
        
        // Check category filter
        if (!allowedCategories.empty()) {
            if (std::find(allowedCategories.begin(), allowedCategories.end(), entry.category) == allowedCategories.end()) {
                return false;
            }
        }
        
        // Check component filter
        if (!allowedComponents.empty()) {
            if (std::find(allowedComponents.begin(), allowedComponents.end(), entry.component) == allowedComponents.end()) {
                return false;
            }
        }
        
        // Check message pattern
        if (!messagePattern.mark_count() == 0) {  // Check if pattern is set
            if (!std::regex_search(entry.message, messagePattern)) {
                return false;
            }
        }
        
        // Check custom filter
        if (customFilter) {
            return customFilter(entry);
        }
        
        return true;
    }
};

// Log Formatter Interface
class LogFormatter {
public:
    virtual ~LogFormatter() = default;
    virtual std::string format(const LogEntry& entry) = 0;
    virtual std::string getFormatName() const = 0;
};

// Plain Text Formatter
class PlainTextFormatter : public LogFormatter {
private:
    std::string timeFormat;
    bool includeThreadId;
    bool includeProcessId;
    bool includeSourceLocation;
    
public:
    PlainTextFormatter(const std::string& timeFormat = "%Y-%m-%d %H:%M:%S",
                      bool includeThread = true,
                      bool includeProcess = true,
                      bool includeSource = true) :
        timeFormat(timeFormat), includeThreadId(includeThread),
        includeProcessId(includeProcess), includeSourceLocation(includeSource) {}
    
    std::string format(const LogEntry& entry) override {
        std::ostringstream oss;
        
        // Timestamp
        auto time_t = std::chrono::system_clock::to_time_t(entry.timestamp);
        oss << std::put_time(std::localtime(&time_t), timeFormat.c_str());
        
        // Level
        oss << " [" << levelToString(entry.level) << "]";
        
        // Thread/Process ID
        if (includeThreadId && !entry.threadId.empty()) {
            oss << " [T:" << entry.threadId << "]";
        }
        if (includeProcessId && !entry.processId.empty()) {
            oss << " [P:" << entry.processId << "]";
        }
        
        // Category/Component
        if (!entry.category.empty()) {
            oss << " [" << entry.category << "]";
        }
        if (!entry.component.empty()) {
            oss << " (" << entry.component << ")";
        }
        
        // Source location
        if (includeSourceLocation && !entry.fileName.empty()) {
            oss << " {" << entry.fileName;
            if (entry.lineNumber > 0) {
                oss << ":" << entry.lineNumber;
            }
            if (!entry.functionName.empty()) {
                oss << " in " << entry.functionName << "()";
            }
            oss << "}";
        }
        
        // Message
        oss << " - " << entry.message;
        
        // Tags
        if (!entry.tags.empty()) {
            oss << " [";
            bool first = true;
            for (const auto& tag : entry.tags) {
                if (!first) oss << ", ";
                oss << tag.first << "=" << tag.second;
                first = false;
            }
            oss << "]";
        }
        
        return oss.str();
    }
    
    std::string getFormatName() const override {
        return "PlainText";
    }
    
private:
    std::string levelToString(LogLevel level) const {
        switch (level) {
            case LogLevel::TRACE: return "TRACE";
            case LogLevel::DEBUG: return "DEBUG";
            case LogLevel::INFO: return "INFO";
            case LogLevel::WARN: return "WARN";
            case LogLevel::ERROR: return "ERROR";
            case LogLevel::FATAL: return "FATAL";
            case LogLevel::AUDIT: return "AUDIT";
            case LogLevel::METRIC: return "METRIC";
            case LogLevel::SECURITY: return "SECURITY";
            case LogLevel::CUSTOM: return "CUSTOM";
            default: return "UNKNOWN";
        }
    }
};

// JSON Formatter
class JSONFormatter : public LogFormatter {
private:
    bool prettyPrint;
    
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
    JSONFormatter(bool pretty = false) : prettyPrint(pretty) {}
    
    std::string format(const LogEntry& entry) override {
        std::ostringstream json;
        
        json << "{";
        if (prettyPrint) json << "\n  ";
        
        // Timestamp (ISO 8601 format)
        auto time_t = std::chrono::system_clock::to_time_t(entry.timestamp);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            entry.timestamp.time_since_epoch()) % 1000;
        
        json << "\"timestamp\":\"";
        json << std::put_time(std::gmtime(&time_t), "%Y-%m-%dT%H:%M:%S");
        json << "." << std::setfill('0') << std::setw(3) << ms.count() << "Z\"";
        
        json << ",";
        if (prettyPrint) json << "\n  ";
        json << "\"level\":\"" << levelToString(entry.level) << "\"";
        
        json << ",";
        if (prettyPrint) json << "\n  ";
        json << "\"message\":\"" << escapeJsonString(entry.message) << "\"";
        
        if (!entry.category.empty()) {
            json << ",";
            if (prettyPrint) json << "\n  ";
            json << "\"category\":\"" << escapeJsonString(entry.category) << "\"";
        }
        
        if (!entry.component.empty()) {
            json << ",";
            if (prettyPrint) json << "\n  ";
            json << "\"component\":\"" << escapeJsonString(entry.component) << "\"";
        }
        
        if (!entry.threadId.empty()) {
            json << ",";
            if (prettyPrint) json << "\n  ";
            json << "\"thread_id\":\"" << entry.threadId << "\"";
        }
        
        if (!entry.fileName.empty()) {
            json << ",";
            if (prettyPrint) json << "\n  ";
            json << "\"source\":{";
            json << "\"file\":\"" << escapeJsonString(entry.fileName) << "\"";
            if (entry.lineNumber > 0) {
                json << ",\"line\":" << entry.lineNumber;
            }
            if (!entry.functionName.empty()) {
                json << ",\"function\":\"" << escapeJsonString(entry.functionName) << "\"";
            }
            json << "}";
        }
        
        if (!entry.fields.empty()) {
            json << ",";
            if (prettyPrint) json << "\n  ";
            json << "\"fields\":{";
            bool firstField = true;
            for (const auto& field : entry.fields) {
                if (!firstField) json << ",";
                if (prettyPrint) json << "\n    ";
                json << "\"" << escapeJsonString(field.first) << "\":";
                
                // Try to convert std::any to string representation
                // This is simplified - a full implementation would handle various types
                json << "\"[complex_value]\"";
                
                firstField = false;
            }
            if (prettyPrint) json << "\n  ";
            json << "}";
        }
        
        if (!entry.tags.empty()) {
            json << ",";
            if (prettyPrint) json << "\n  ";
            json << "\"tags\":{";
            bool firstTag = true;
            for (const auto& tag : entry.tags) {
                if (!firstTag) json << ",";
                if (prettyPrint) json << "\n    ";
                json << "\"" << escapeJsonString(tag.first) << "\":\"" << escapeJsonString(tag.second) << "\"";
                firstTag = false;
            }
            if (prettyPrint) json << "\n  ";
            json << "}";
        }
        
        json << ",";
        if (prettyPrint) json << "\n  ";
        json << "\"sequence\":" << entry.sequenceNumber;
        
        if (prettyPrint) json << "\n";
        json << "}";
        
        return json.str();
    }
    
    std::string getFormatName() const override {
        return "JSON";
    }
    
private:
    std::string levelToString(LogLevel level) const {
        switch (level) {
            case LogLevel::TRACE: return "TRACE";
            case LogLevel::DEBUG: return "DEBUG";
            case LogLevel::INFO: return "INFO";
            case LogLevel::WARN: return "WARN";
            case LogLevel::ERROR: return "ERROR";
            case LogLevel::FATAL: return "FATAL";
            case LogLevel::AUDIT: return "AUDIT";
            case LogLevel::METRIC: return "METRIC";
            case LogLevel::SECURITY: return "SECURITY";
            case LogLevel::CUSTOM: return "CUSTOM";
            default: return "UNKNOWN";
        }
    }
};

// Log Destination Interface
class LogDestinationHandler {
public:
    virtual ~LogDestinationHandler() = default;
    virtual void write(const std::string& formattedEntry) = 0;
    virtual void flush() = 0;
    virtual void close() = 0;
    virtual std::string getDestinationName() const = 0;
    virtual bool isHealthy() const = 0;
};

// Console Log Destination
class ConsoleLogDestination : public LogDestinationHandler {
private:
    bool useStdErr;
    std::mutex outputMutex;
    
public:
    ConsoleLogDestination(bool useErrorStream = false) : useStdErr(useErrorStream) {}
    
    void write(const std::string& formattedEntry) override {
        std::lock_guard<std::mutex> lock(outputMutex);
        if (useStdErr) {
            std::cerr << formattedEntry << std::endl;
        } else {
            std::cout << formattedEntry << std::endl;
        }
    }
    
    void flush() override {
        std::lock_guard<std::mutex> lock(outputMutex);
        if (useStdErr) {
            std::cerr.flush();
        } else {
            std::cout.flush();
        }
    }
    
    void close() override {
        // Console streams don't need to be closed
    }
    
    std::string getDestinationName() const override {
        return useStdErr ? "Console(stderr)" : "Console(stdout)";
    }
    
    bool isHealthy() const override {
        return true; // Console is always available
    }
};

// File Log Destination with Rotation
class FileLogDestination : public LogDestinationHandler {
private:
    std::string baseFileName;
    std::string currentFileName;
    std::ofstream currentFile;
    std::mutex fileMutex;
    
    LogRotationPolicy rotationPolicy;
    size_t maxFileSize;
    std::chrono::hours rotationInterval;
    int maxBackupFiles;
    LogCompression compression;
    
    size_t currentFileSize;
    std::chrono::system_clock::time_point lastRotation;
    size_t entryCount;
    
    void rotateIfNeeded() {
        bool shouldRotate = false;
        
        switch (rotationPolicy) {
            case LogRotationPolicy::SIZE_BASED:
                shouldRotate = (currentFileSize >= maxFileSize);
                break;
            case LogRotationPolicy::TIME_BASED:
                shouldRotate = (std::chrono::system_clock::now() - lastRotation) >= rotationInterval;
                break;
            case LogRotationPolicy::COUNT_BASED:
                shouldRotate = (entryCount >= maxFileSize); // Reusing maxFileSize as max count
                break;
            case LogRotationPolicy::HYBRID:
                shouldRotate = (currentFileSize >= maxFileSize) || 
                              ((std::chrono::system_clock::now() - lastRotation) >= rotationInterval);
                break;
            default:
                break;
        }
        
        if (shouldRotate) {
            performRotation();
        }
    }
    
    void performRotation() {
        if (currentFile.is_open()) {
            currentFile.close();
        }
        
        // Generate backup filename with timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::ostringstream backupName;
        backupName << baseFileName << "." << std::put_time(std::gmtime(&time_t), "%Y%m%d_%H%M%S");
        
        // Rename current file to backup
        std::rename(currentFileName.c_str(), backupName.str().c_str());
        
        // Compress if needed
        if (compression != LogCompression::NONE) {
            compressFile(backupName.str());
        }
        
        // Clean up old backup files
        cleanupOldBackups();
        
        // Open new current file
        currentFile.open(currentFileName, std::ios::out | std::ios::app);
        currentFileSize = 0;
        lastRotation = std::chrono::system_clock::now();
        entryCount = 0;
    }
    
    void compressFile(const std::string& filename) {
        // Simplified compression - would implement actual compression algorithms
        switch (compression) {
            case LogCompression::GZIP:
                // Implement gzip compression
                break;
            case LogCompression::BZIP2:
                // Implement bzip2 compression
                break;
            case LogCompression::LZ4:
                // Implement LZ4 compression
                break;
            default:
                break;
        }
    }
    
    void cleanupOldBackups() {
        // Implementation would scan for backup files and remove oldest ones
        // beyond maxBackupFiles limit
    }
    
public:
    FileLogDestination(const std::string& fileName,
                      LogRotationPolicy policy = LogRotationPolicy::SIZE_BASED,
                      size_t maxSize = 10 * 1024 * 1024,  // 10MB default
                      int maxBackups = 5) :
        baseFileName(fileName), currentFileName(fileName),
        rotationPolicy(policy), maxFileSize(maxSize),
        rotationInterval(std::chrono::hours(24)),
        maxBackupFiles(maxBackups), compression(LogCompression::NONE),
        currentFileSize(0), lastRotation(std::chrono::system_clock::now()),
        entryCount(0) {
        
        currentFile.open(currentFileName, std::ios::out | std::ios::app);
        if (!currentFile.is_open()) {
            throw std::runtime_error("Cannot open log file: " + currentFileName);
        }
    }
    
    ~FileLogDestination() override {
        close();
    }
    
    void write(const std::string& formattedEntry) override {
        std::lock_guard<std::mutex> lock(fileMutex);
        
        rotateIfNeeded();
        
        if (currentFile.is_open()) {
            currentFile << formattedEntry << std::endl;
            currentFileSize += formattedEntry.length() + 1; // +1 for newline
            entryCount++;
        }
    }
    
    void flush() override {
        std::lock_guard<std::mutex> lock(fileMutex);
        if (currentFile.is_open()) {
            currentFile.flush();
        }
    }
    
    void close() override {
        std::lock_guard<std::mutex> lock(fileMutex);
        if (currentFile.is_open()) {
            currentFile.close();
        }
    }
    
    std::string getDestinationName() const override {
        return "File(" + currentFileName + ")";
    }
    
    bool isHealthy() const override {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(fileMutex));
        return currentFile.is_open() && currentFile.good();
    }
    
    // Configuration methods
    void setRotationPolicy(LogRotationPolicy policy) { rotationPolicy = policy; }
    void setMaxFileSize(size_t size) { maxFileSize = size; }
    void setRotationInterval(std::chrono::hours interval) { rotationInterval = interval; }
    void setMaxBackupFiles(int count) { maxBackupFiles = count; }
    void setCompression(LogCompression comp) { compression = comp; }
};

// Memory Buffer Log Destination
class MemoryLogDestination : public LogDestinationHandler {
private:
    std::deque<std::string> buffer;
    std::mutex bufferMutex;
    size_t maxBufferSize;
    bool circularBuffer;
    
public:
    MemoryLogDestination(size_t maxSize = 10000, bool circular = true) :
        maxBufferSize(maxSize), circularBuffer(circular) {}
    
    void write(const std::string& formattedEntry) override {
        std::lock_guard<std::mutex> lock(bufferMutex);
        
        buffer.push_back(formattedEntry);
        
        if (buffer.size() > maxBufferSize) {
            if (circularBuffer) {
                buffer.pop_front();
            } else {
                // Stop logging if buffer is full and not circular
                return;
            }
        }
    }
    
    void flush() override {
        // Memory buffer doesn't need flushing
    }
    
    void close() override {
        std::lock_guard<std::mutex> lock(bufferMutex);
        buffer.clear();
    }
    
    std::string getDestinationName() const override {
        return "Memory";
    }
    
    bool isHealthy() const override {
        return true;
    }
    
    // Memory-specific methods
    std::vector<std::string> getBufferContents() const {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(bufferMutex));
        return std::vector<std::string>(buffer.begin(), buffer.end());
    }
    
    void clearBuffer() {
        std::lock_guard<std::mutex> lock(bufferMutex);
        buffer.clear();
    }
    
    size_t getBufferSize() const {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(bufferMutex));
        return buffer.size();
    }
    
    void setMaxBufferSize(size_t size) {
        std::lock_guard<std::mutex> lock(bufferMutex);
        maxBufferSize = size;
        
        // Trim buffer if necessary
        while (buffer.size() > maxBufferSize) {
            buffer.pop_front();
        }
    }
};

// Logger Configuration
struct LoggerConfiguration {
    LogLevel minimumLevel;
    std::vector<std::unique_ptr<LogDestinationHandler>> destinations;
    std::unique_ptr<LogFormatter> formatter;
    LogFilter filter;
    bool asynchronous;
    size_t bufferSize;
    std::chrono::milliseconds flushInterval;
    bool enableMetrics;
    bool enableSampling;
    double samplingRate;
    
    LoggerConfiguration() :
        minimumLevel(LogLevel::INFO),
        asynchronous(true),
        bufferSize(10000),
        flushInterval(std::chrono::milliseconds(1000)),
        enableMetrics(true),
        enableSampling(false),
        samplingRate(1.0) {}
};

// Performance Metrics
struct LoggingMetrics {
    std::atomic<size_t> totalLogEntries{0};
    std::atomic<size_t> droppedLogEntries{0};
    std::atomic<size_t> errorCount{0};
    std::atomic<size_t> entriesPerSecond{0};
    std::atomic<double> averageProcessingTime{0.0};
    std::atomic<size_t> bufferUtilization{0};
    std::chrono::steady_clock::time_point lastReset;
    
    LoggingMetrics() : lastReset(std::chrono::steady_clock::now()) {}
    
    void reset() {
        totalLogEntries = 0;
        droppedLogEntries = 0;
        errorCount = 0;
        entriesPerSecond = 0;
        averageProcessingTime = 0.0;
        bufferUtilization = 0;
        lastReset = std::chrono::steady_clock::now();
    }
};

struct LoggingMetricsCopy {
    size_t totalLogEntries;
    size_t droppedLogEntries;
    size_t errorCount;
    size_t entriesPerSecond;
    double averageProcessingTime;
    size_t bufferUtilization;
    std::chrono::steady_clock::time_point lastReset;
    
    LoggingMetricsCopy() : totalLogEntries(0), droppedLogEntries(0), errorCount(0),
                          entriesPerSecond(0), averageProcessingTime(0.0), bufferUtilization(0),
                          lastReset(std::chrono::steady_clock::now()) {}
};

// Main Logger Class
class Logger {
private:
    std::string loggerName;
    LoggerConfiguration config;
    std::queue<LogEntry> logQueue;
    std::mutex queueMutex;
    std::condition_variable queueCondition;
    std::thread loggingThread;
    std::atomic<bool> isRunning{false};
    std::atomic<size_t> sequenceCounter{0};
    LoggingMetrics metrics;
    
    // Sampling support
    std::random_device randomDevice;
    mutable std::mt19937 randomGenerator;
    std::uniform_real_distribution<double> distribution;
    
    void loggingWorker() {
        auto lastFlush = std::chrono::steady_clock::now();
        auto lastMetricsUpdate = std::chrono::steady_clock::now();
        size_t recentEntries = 0;
        
        while (isRunning.load()) {
            std::unique_lock<std::mutex> lock(queueMutex);
            
            // Wait for log entries or timeout
            queueCondition.wait_for(lock, std::chrono::milliseconds(100), [this] {
                return !logQueue.empty() || !isRunning.load();
            });
            
            // Process all available log entries
            while (!logQueue.empty() && isRunning.load()) {
                LogEntry entry = logQueue.front();
                logQueue.pop();
                lock.unlock();
                
                auto startTime = std::chrono::steady_clock::now();
                
                try {
                    // Apply filter
                    if (config.filter.shouldLog(entry)) {
                        // Format the entry
                        std::string formattedEntry = config.formatter->format(entry);
                        
                        // Write to all destinations
                        for (const auto& destination : config.destinations) {
                            if (destination->isHealthy()) {
                                destination->write(formattedEntry);
                            } else {
                                metrics.errorCount++;
                            }
                        }
                        
                        metrics.totalLogEntries++;
                        recentEntries++;
                    } else {
                        metrics.droppedLogEntries++;
                    }
                } catch (...) {
                    metrics.errorCount++;
                }
                
                // Update processing time metrics
                auto endTime = std::chrono::steady_clock::now();
                auto processingTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
                metrics.averageProcessingTime = 
                    (metrics.averageProcessingTime.load() + processingTime.count()) / 2.0;
                
                lock.lock();
            }
            
            auto now = std::chrono::steady_clock::now();
            
            // Periodic flush
            if (now - lastFlush >= config.flushInterval) {
                lock.unlock();
                flushAll();
                lastFlush = now;
                lock.lock();
            }
            
            // Update metrics
            if (now - lastMetricsUpdate >= std::chrono::seconds(1)) {
                metrics.entriesPerSecond = recentEntries;
                metrics.bufferUtilization = (logQueue.size() * 100) / config.bufferSize;
                recentEntries = 0;
                lastMetricsUpdate = now;
            }
            
            lock.unlock();
        }
        
        // Final flush
        flushAll();
    }
    
    void flushAll() {
        for (const auto& destination : config.destinations) {
            try {
                destination->flush();
            } catch (...) {
                // Continue with other destinations
            }
        }
    }
    
    bool shouldSample() const {
        if (!config.enableSampling || config.samplingRate >= 1.0) {
            return true;
        }
        return const_cast<std::uniform_real_distribution<double>&>(distribution)(const_cast<std::mt19937&>(randomGenerator)) < config.samplingRate;
    }
    
public:
    Logger(const std::string& name) :
        loggerName(name),
        randomGenerator(randomDevice()),
        distribution(0.0, 1.0) {
        
        // Set default formatter
        config.formatter = std::make_unique<PlainTextFormatter>();
        
        // Add default console destination
        config.destinations.push_back(std::make_unique<ConsoleLogDestination>());
    }
    
    ~Logger() {
        stop();
    }
    
    void start() {
        if (isRunning.load()) return;
        
        isRunning.store(true);
        loggingThread = std::thread(&Logger::loggingWorker, this);
    }
    
    void stop() {
        if (!isRunning.load()) return;
        
        isRunning.store(false);
        queueCondition.notify_all();
        
        if (loggingThread.joinable()) {
            loggingThread.join();
        }
        
        // Close all destinations
        for (auto& destination : config.destinations) {
            destination->close();
        }
    }
    
    // Configuration methods
    void setMinimumLevel(LogLevel level) {
        config.minimumLevel = level;
    }
    
    void setFormatter(std::unique_ptr<LogFormatter> formatter) {
        config.formatter = std::move(formatter);
    }
    
    void addDestination(std::unique_ptr<LogDestinationHandler> destination) {
        config.destinations.push_back(std::move(destination));
    }
    
    void clearDestinations() {
        for (auto& destination : config.destinations) {
            destination->close();
        }
        config.destinations.clear();
    }
    
    void setFilter(const LogFilter& filter) {
        config.filter = filter;
    }
    
    void setAsynchronous(bool async) {
        config.asynchronous = async;
    }
    
    void setBufferSize(size_t size) {
        config.bufferSize = size;
    }
    
    void setFlushInterval(std::chrono::milliseconds interval) {
        config.flushInterval = interval;
    }
    
    void enableSampling(bool enable, double rate = 1.0) {
        config.enableSampling = enable;
        config.samplingRate = std::clamp(rate, 0.0, 1.0);
    }
    
    // Logging methods
    void log(LogLevel level, const std::string& message, 
             const std::string& category = "", const std::string& component = "",
             const std::string& fileName = "", int lineNumber = 0, 
             const std::string& functionName = "") {
        
        // Check level threshold
        if (level < config.minimumLevel) return;
        
        // Apply sampling
        if (!shouldSample()) return;
        
        LogEntry entry;
        entry.level = level;
        entry.message = message;
        entry.category = category;
        entry.component = component;
        entry.fileName = fileName;
        entry.lineNumber = lineNumber;
        entry.functionName = functionName;
        entry.sequenceNumber = sequenceCounter++;
        
        // Add thread and process information
        std::ostringstream threadIdStream;
        threadIdStream << std::this_thread::get_id();
        entry.threadId = threadIdStream.str();
        
        // Process ID would be obtained from system call
        entry.processId = "process_id";
        
        if (config.asynchronous) {
            std::lock_guard<std::mutex> lock(queueMutex);
            
            // Drop entries if buffer is full
            if (logQueue.size() >= config.bufferSize) {
                metrics.droppedLogEntries++;
                return;
            }
            
            logQueue.push(entry);
            queueCondition.notify_one();
        } else {
            // Synchronous logging
            if (config.filter.shouldLog(entry)) {
                try {
                    std::string formattedEntry = config.formatter->format(entry);
                    for (const auto& destination : config.destinations) {
                        if (destination->isHealthy()) {
                            destination->write(formattedEntry);
                        }
                    }
                    metrics.totalLogEntries++;
                } catch (...) {
                    metrics.errorCount++;
                }
            }
        }
    }
    
    // Structured logging
    void logStructured(LogLevel level, const std::string& message,
                      const std::unordered_map<std::string, std::any>& fields = {},
                      const std::unordered_map<std::string, std::string>& tags = {},
                      const std::string& category = "", const std::string& component = "") {
        
        if (level < config.minimumLevel) return;
        if (!shouldSample()) return;
        
        LogEntry entry;
        entry.level = level;
        entry.message = message;
        entry.category = category;
        entry.component = component;
        entry.fields = fields;
        entry.tags = tags;
        entry.sequenceNumber = sequenceCounter++;
        
        std::ostringstream threadIdStream;
        threadIdStream << std::this_thread::get_id();
        entry.threadId = threadIdStream.str();
        entry.processId = "process_id";
        
        if (config.asynchronous) {
            std::lock_guard<std::mutex> lock(queueMutex);
            
            if (logQueue.size() >= config.bufferSize) {
                metrics.droppedLogEntries++;
                return;
            }
            
            logQueue.push(entry);
            queueCondition.notify_one();
        } else {
            if (config.filter.shouldLog(entry)) {
                try {
                    std::string formattedEntry = config.formatter->format(entry);
                    for (const auto& destination : config.destinations) {
                        if (destination->isHealthy()) {
                            destination->write(formattedEntry);
                        }
                    }
                    metrics.totalLogEntries++;
                } catch (...) {
                    metrics.errorCount++;
                }
            }
        }
    }
    
    // Convenience methods
    void trace(const std::string& message, const std::string& category = "", const std::string& component = "") {
        log(LogLevel::TRACE, message, category, component);
    }
    
    void debug(const std::string& message, const std::string& category = "", const std::string& component = "") {
        log(LogLevel::DEBUG, message, category, component);
    }
    
    void info(const std::string& message, const std::string& category = "", const std::string& component = "") {
        log(LogLevel::INFO, message, category, component);
    }
    
    void warn(const std::string& message, const std::string& category = "", const std::string& component = "") {
        log(LogLevel::WARN, message, category, component);
    }
    
    void error(const std::string& message, const std::string& category = "", const std::string& component = "") {
        log(LogLevel::ERROR, message, category, component);
    }
    
    void fatal(const std::string& message, const std::string& category = "", const std::string& component = "") {
        log(LogLevel::FATAL, message, category, component);
    }
    
    void audit(const std::string& message, const std::string& category = "", const std::string& component = "") {
        log(LogLevel::AUDIT, message, category, component);
    }
    
    void metric(const std::string& message, const std::string& category = "", const std::string& component = "") {
        log(LogLevel::METRIC, message, category, component);
    }
    
    void security(const std::string& message, const std::string& category = "", const std::string& component = "") {
        log(LogLevel::SECURITY, message, category, component);
    }
    
    // Metrics and status
    LoggingMetricsCopy getMetrics() const {
        LoggingMetricsCopy copy;
        copy.totalLogEntries = metrics.totalLogEntries.load();
        copy.droppedLogEntries = metrics.droppedLogEntries.load();
        copy.errorCount = metrics.errorCount.load();
        copy.entriesPerSecond = metrics.entriesPerSecond.load();
        copy.averageProcessingTime = metrics.averageProcessingTime.load();
        copy.bufferUtilization = metrics.bufferUtilization.load();
        copy.lastReset = metrics.lastReset;
        return copy;
    }
    
    void resetMetrics() {
        metrics.reset();
    }
    
    std::string getStatus() const {
        std::ostringstream status;
        status << "Logger '" << loggerName << "' Status:\n";
        status << "  Running: " << (isRunning.load() ? "Yes" : "No") << "\n";
        status << "  Minimum Level: " << static_cast<int>(config.minimumLevel) << "\n";
        status << "  Asynchronous: " << (config.asynchronous ? "Yes" : "No") << "\n";
        status << "  Buffer Size: " << config.bufferSize << "\n";
        status << "  Destinations: " << config.destinations.size() << "\n";
        status << "  Sampling: " << (config.enableSampling ? "Enabled" : "Disabled") << "\n";
        if (config.enableSampling) {
            status << "  Sampling Rate: " << config.samplingRate << "\n";
        }
        
        status << "  Metrics:\n";
        status << "    Total Entries: " << metrics.totalLogEntries.load() << "\n";
        status << "    Dropped Entries: " << metrics.droppedLogEntries.load() << "\n";
        status << "    Errors: " << metrics.errorCount.load() << "\n";
        status << "    Entries/sec: " << metrics.entriesPerSecond.load() << "\n";
        status << "    Avg Processing Time: " << metrics.averageProcessingTime.load() << " μs\n";
        status << "    Buffer Utilization: " << metrics.bufferUtilization.load() << "%\n";
        
        return status.str();
    }
    
    std::string getName() const {
        return loggerName;
    }
    
    bool isRunningStatus() const {
        return isRunning.load();
    }
};

// Logger Manager for multiple loggers
class LoggerManager {
private:
    std::unordered_map<std::string, std::unique_ptr<Logger>> loggers;
    std::unique_ptr<Logger> rootLogger;
    mutable std::mutex managerMutex;
    
public:
    LoggerManager() {
        // Create root logger
        rootLogger = std::make_unique<Logger>("root");
        rootLogger->start();
    }
    
    ~LoggerManager() {
        // Stop all loggers
        for (auto& pair : loggers) {
            if (pair.second->isRunningStatus()) {
                pair.second->stop();
            }
        }
        if (rootLogger && rootLogger->isRunningStatus()) {
            rootLogger->stop();
        }
    }
    
    Logger* createLogger(const std::string& name) {
        std::lock_guard<std::mutex> lock(managerMutex);
        
        auto logger = std::make_unique<Logger>(name);
        Logger* loggerPtr = logger.get();
        loggers[name] = std::move(logger);
        
        return loggerPtr;
    }
    
    Logger* getLogger(const std::string& name) {
        std::lock_guard<std::mutex> lock(managerMutex);
        
        auto it = loggers.find(name);
        if (it != loggers.end()) {
            return it->second.get();
        }
        
        return nullptr;
    }
    
    Logger* getRootLogger() {
        return rootLogger.get();
    }
    
    void removeLogger(const std::string& name) {
        std::lock_guard<std::mutex> lock(managerMutex);
        
        auto it = loggers.find(name);
        if (it != loggers.end()) {
            if (it->second->isRunningStatus()) {
                it->second->stop();
            }
            loggers.erase(it);
        }
    }
    
    std::vector<std::string> getLoggerNames() const {
        std::lock_guard<std::mutex> lock(managerMutex);
        
        std::vector<std::string> names;
        for (const auto& pair : loggers) {
            names.push_back(pair.first);
        }
        return names;
    }
    
    void startAllLoggers() {
        std::lock_guard<std::mutex> lock(managerMutex);
        
        for (auto& pair : loggers) {
            if (!pair.second->isRunningStatus()) {
                pair.second->start();
            }
        }
    }
    
    void stopAllLoggers() {
        std::lock_guard<std::mutex> lock(managerMutex);
        
        for (auto& pair : loggers) {
            if (pair.second->isRunningStatus()) {
                pair.second->stop();
            }
        }
    }
    
    std::string getSystemStatus() const {
        std::lock_guard<std::mutex> lock(managerMutex);
        
        std::ostringstream status;
        status << "=== Logger Manager Status ===\n";
        status << "Active Loggers: " << loggers.size() << "\n";
        status << "Root Logger: " << (rootLogger->isRunningStatus() ? "Running" : "Stopped") << "\n\n";
        
        // Root logger status
        status << rootLogger->getStatus() << "\n\n";
        
        // Individual logger statuses
        for (const auto& pair : loggers) {
            status << pair.second->getStatus() << "\n";
        }
        
        return status.str();
    }
};

// Macros for convenient logging with source location
#define LOG_TRACE(logger, message) \
    (logger)->log(logging_framework::LogLevel::TRACE, message, "", "", __FILE__, __LINE__, __FUNCTION__)

#define LOG_DEBUG(logger, message) \
    (logger)->log(logging_framework::LogLevel::DEBUG, message, "", "", __FILE__, __LINE__, __FUNCTION__)

#define LOG_INFO(logger, message) \
    (logger)->log(logging_framework::LogLevel::INFO, message, "", "", __FILE__, __LINE__, __FUNCTION__)

#define LOG_WARN(logger, message) \
    (logger)->log(logging_framework::LogLevel::WARN, message, "", "", __FILE__, __LINE__, __FUNCTION__)

#define LOG_ERROR(logger, message) \
    (logger)->log(logging_framework::LogLevel::ERROR, message, "", "", __FILE__, __LINE__, __FUNCTION__)

#define LOG_FATAL(logger, message) \
    (logger)->log(logging_framework::LogLevel::FATAL, message, "", "", __FILE__, __LINE__, __FUNCTION__)

// Global logger manager instance
inline LoggerManager& getGlobalLoggerManager() {
    static LoggerManager instance;
    return instance;
}

// Convenience functions for global logging
inline Logger* getLogger(const std::string& name = "root") {
    if (name == "root") {
        return getGlobalLoggerManager().getRootLogger();
    }
    return getGlobalLoggerManager().getLogger(name);
}

inline Logger* createLogger(const std::string& name) {
    return getGlobalLoggerManager().createLogger(name);
}

// Additional methods would be implemented here if needed

} // namespace logging_framework

#endif // ADVANCED_LOGGING_FRAMEWORK_H