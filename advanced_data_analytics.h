/**
 * @file advanced_data_analytics.h
 * @author adzetto
 * @brief Advanced Data Analytics Engine for Electric Vehicle Systems
 * @version 1.0
 * @date 2025-08-31
 * 
 * @copyright Copyright (c) 2025
 * 
 * @details This module provides comprehensive data analytics capabilities including
 * real-time stream processing, statistical analysis, predictive modeling,
 * anomaly detection, and advanced visualization for EV systems.
 */

#ifndef ADVANCED_DATA_ANALYTICS_H
#define ADVANCED_DATA_ANALYTICS_H

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
#include <complex>
#include <tuple>
#include <condition_variable>

// Data types and structures
namespace analytics {

enum class DataType {
    INTEGER,
    DOUBLE,
    STRING,
    BOOLEAN,
    TIMESTAMP,
    COMPLEX,
    VECTOR,
    MATRIX
};

enum class AggregationType {
    SUM,
    AVERAGE,
    MEDIAN,
    MODE,
    MIN,
    MAX,
    STDDEV,
    VARIANCE,
    PERCENTILE,
    COUNT,
    DISTINCT_COUNT
};

enum class ModelType {
    LINEAR_REGRESSION,
    POLYNOMIAL_REGRESSION,
    EXPONENTIAL_SMOOTHING,
    MOVING_AVERAGE,
    ARIMA,
    KALMAN_FILTER,
    NEURAL_NETWORK,
    DECISION_TREE,
    RANDOM_FOREST,
    SVM,
    CLUSTERING,
    PCA
};

enum class AnomalyType {
    STATISTICAL_OUTLIER,
    TREND_DEVIATION,
    SEASONAL_ANOMALY,
    CORRELATION_BREAK,
    FREQUENCY_ANOMALY,
    PATTERN_DEVIATION
};

struct DataPoint {
    std::chrono::steady_clock::time_point timestamp;
    DataType type;
    std::string key;
    std::vector<double> values;
    std::unordered_map<std::string, std::string> metadata;
    
    DataPoint() : timestamp(std::chrono::steady_clock::now()), type(DataType::DOUBLE) {}
    DataPoint(const std::string& k, double v) : 
        timestamp(std::chrono::steady_clock::now()), type(DataType::DOUBLE), key(k), values({v}) {}
    DataPoint(const std::string& k, const std::vector<double>& v) : 
        timestamp(std::chrono::steady_clock::now()), type(DataType::VECTOR), key(k), values(v) {}
};

struct StatisticalSummary {
    double mean;
    double median;
    double mode;
    double stddev;
    double variance;
    double min;
    double max;
    double skewness;
    double kurtosis;
    size_t count;
    std::vector<double> percentiles;
    
    StatisticalSummary() : mean(0), median(0), mode(0), stddev(0), variance(0), 
                          min(0), max(0), skewness(0), kurtosis(0), count(0) {}
};

struct PredictionResult {
    std::vector<double> predictions;
    std::vector<double> confidenceIntervals;
    double accuracy;
    double mse;
    double mae;
    std::chrono::steady_clock::time_point predictionTime;
    std::string modelUsed;
};

struct AnomalyDetection {
    bool isAnomaly;
    AnomalyType type;
    double severity;
    double confidence;
    std::string description;
    std::chrono::steady_clock::time_point detectionTime;
    std::vector<double> contributingFactors;
};

// Time Series Analysis Classes
class TimeSeriesAnalyzer {
private:
    std::deque<DataPoint> timeSeriesData;
    size_t maxSize;
    mutable std::mutex dataMutex;
    
    // Internal analysis methods
    std::vector<double> calculateMovingAverage(const std::vector<double>& data, size_t window) const;
    std::vector<double> calculateExponentialSmoothing(const std::vector<double>& data, double alpha) const;
    std::vector<double> detectSeasonality(const std::vector<double>& data) const;
    std::vector<double> detrend(const std::vector<double>& data) const;
    double calculateAutocorrelation(const std::vector<double>& data, size_t lag) const;
    
public:
    TimeSeriesAnalyzer(size_t maxDataPoints = 10000) : maxSize(maxDataPoints) {}
    
    void addDataPoint(const DataPoint& point) {
        std::lock_guard<std::mutex> lock(dataMutex);
        timeSeriesData.push_back(point);
        if (timeSeriesData.size() > maxSize) {
            timeSeriesData.pop_front();
        }
    }
    
    std::vector<double> extractValues(const std::string& key) const {
        std::lock_guard<std::mutex> lock(dataMutex);
        std::vector<double> values;
        for (const auto& point : timeSeriesData) {
            if (point.key == key && !point.values.empty()) {
                values.push_back(point.values[0]);
            }
        }
        return values;
    }
    
    StatisticalSummary calculateStatistics(const std::string& key) const;
    PredictionResult predict(const std::string& key, size_t forecastSteps, ModelType model) const;
    std::vector<double> detectTrends(const std::string& key) const;
    std::vector<double> calculateCorrelation(const std::string& key1, const std::string& key2) const;
    AnomalyDetection detectAnomalies(const std::string& key) const;
};

// Statistical Computing Engine
class StatisticalEngine {
private:
    std::random_device rd;
    mutable std::mt19937 gen;
    
    // Statistical distribution functions
    double normalPDF(double x, double mean, double stddev) const;
    double normalCDF(double x, double mean, double stddev) const;
    double tTestStatistic(const std::vector<double>& sample1, const std::vector<double>& sample2) const;
    double chiSquareTest(const std::vector<double>& observed, const std::vector<double>& expected) const;
    
public:
    StatisticalEngine() : gen(rd()) {}
    
    // Descriptive statistics
    double calculateMean(const std::vector<double>& data) const;
    double calculateMedian(std::vector<double> data) const;
    double calculateMode(const std::vector<double>& data) const;
    double calculateStandardDeviation(const std::vector<double>& data) const;
    double calculateVariance(const std::vector<double>& data) const;
    double calculateSkewness(const std::vector<double>& data) const;
    double calculateKurtosis(const std::vector<double>& data) const;
    double calculatePercentile(std::vector<double> data, double percentile) const;
    
    // Inferential statistics
    bool performTTest(const std::vector<double>& sample1, const std::vector<double>& sample2, double alpha = 0.05) const;
    bool performChiSquareTest(const std::vector<double>& observed, const std::vector<double>& expected, double alpha = 0.05) const;
    std::pair<double, double> calculateConfidenceInterval(const std::vector<double>& data, double confidence = 0.95) const;
    double calculateCorrelationCoefficient(const std::vector<double>& x, const std::vector<double>& y) const;
    
    // Regression analysis
    std::pair<double, double> linearRegression(const std::vector<double>& x, const std::vector<double>& y) const;
    std::vector<double> polynomialRegression(const std::vector<double>& x, const std::vector<double>& y, int degree) const;
    double calculateRSquared(const std::vector<double>& actual, const std::vector<double>& predicted) const;
    
    // Distribution fitting
    std::pair<double, double> fitNormalDistribution(const std::vector<double>& data) const;
    std::pair<double, double> fitExponentialDistribution(const std::vector<double>& data) const;
    bool goodnessOfFitTest(const std::vector<double>& data, const std::string& distribution) const;
};

// Machine Learning Pipeline
class MachineLearningPipeline {
private:
    struct NeuralLayer {
        std::vector<std::vector<double>> weights;
        std::vector<double> biases;
        std::function<double(double)> activationFunction;
        
        NeuralLayer(size_t inputSize, size_t outputSize, std::function<double(double)> activation) :
            activationFunction(activation) {
            weights.resize(outputSize, std::vector<double>(inputSize));
            biases.resize(outputSize);
            // Initialize weights randomly
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<double> dis(-1.0, 1.0);
            for (auto& row : weights) {
                for (auto& w : row) {
                    w = dis(gen);
                }
            }
            for (auto& b : biases) {
                b = dis(gen);
            }
        }
    };
    
    std::vector<NeuralLayer> neuralLayers;
    std::unordered_map<std::string, std::vector<std::vector<double>>> trainingData;
    std::unordered_map<std::string, std::vector<double>> trainingLabels;
    
    // Activation functions
    static double sigmoid(double x) { return 1.0 / (1.0 + std::exp(-x)); }
    static double tanh_activation(double x) { return std::tanh(x); }
    static double relu(double x) { return std::max(0.0, x); }
    static double leakyRelu(double x) { return x > 0 ? x : 0.01 * x; }
    
    // Helper methods
    std::vector<double> forwardPass(const std::vector<double>& input) const;
    void backwardPass(const std::vector<double>& input, const std::vector<double>& expected, double learningRate);
    std::vector<double> normalizeFeatures(const std::vector<double>& features) const;
    std::vector<std::vector<double>> principalComponentAnalysis(const std::vector<std::vector<double>>& data, int components) const;
    
public:
    MachineLearningPipeline() = default;
    
    // Model construction
    void addNeuralLayer(size_t inputSize, size_t outputSize, const std::string& activationType);
    void setTrainingData(const std::string& key, const std::vector<std::vector<double>>& data, const std::vector<double>& labels);
    
    // Training and prediction
    void trainNeuralNetwork(const std::string& dataKey, int epochs, double learningRate);
    PredictionResult predict(const std::vector<double>& features) const;
    double evaluateModel(const std::vector<std::vector<double>>& testData, const std::vector<double>& testLabels) const;
    
    // Feature engineering
    std::vector<double> extractFeatures(const std::vector<DataPoint>& data) const;
    std::vector<std::vector<double>> performPCA(const std::vector<std::vector<double>>& data, int components) const;
    std::vector<double> selectFeatures(const std::vector<std::vector<double>>& data, const std::vector<double>& labels, int topFeatures) const;
    
    // Clustering
    std::vector<int> kMeansClustering(const std::vector<std::vector<double>>& data, int k, int maxIterations = 100) const;
    std::vector<int> dbscanClustering(const std::vector<std::vector<double>>& data, double eps, int minPoints) const;
    double calculateSilhouetteScore(const std::vector<std::vector<double>>& data, const std::vector<int>& clusters) const;
};

// Real-time Stream Processing
class StreamProcessor {
private:
    std::queue<DataPoint> streamBuffer;
    std::unordered_map<std::string, std::function<void(const DataPoint&)>> processors;
    std::thread processingThread;
    std::mutex bufferMutex;
    std::condition_variable bufferCondition;
    std::atomic<bool> isRunning{false};
    size_t maxBufferSize;
    
    void processStream();
    
public:
    StreamProcessor(size_t maxBuffer = 100000) : maxBufferSize(maxBuffer) {}
    ~StreamProcessor() { stop(); }
    
    void start();
    void stop();
    void addDataPoint(const DataPoint& point);
    void registerProcessor(const std::string& key, std::function<void(const DataPoint&)> processor);
    void unregisterProcessor(const std::string& key);
    size_t getBufferSize() const;
    void clearBuffer();
};

// Advanced Visualization Engine
class VisualizationEngine {
private:
    struct ChartConfig {
        std::string title;
        std::string xLabel;
        std::string yLabel;
        int width;
        int height;
        std::vector<std::string> colors;
        bool showGrid;
        bool showLegend;
        
        ChartConfig() : width(800), height(600), showGrid(true), showLegend(true) {}
    };
    
    std::unordered_map<std::string, std::vector<DataPoint>> visualizationData;
    mutable std::mutex dataMutex;
    
    // Chart generation helpers
    std::string generateASCIIChart(const std::vector<double>& data, const ChartConfig& config) const;
    std::string generateHistogram(const std::vector<double>& data, int bins) const;
    std::string generateBoxPlot(const std::vector<double>& data) const;
    std::string generateHeatMap(const std::vector<std::vector<double>>& matrix) const;
    
public:
    void addVisualizationData(const std::string& seriesName, const std::vector<DataPoint>& data);
    std::string generateLineChart(const std::string& seriesName, const ChartConfig& config) const;
    std::string generateBarChart(const std::unordered_map<std::string, double>& data, const ChartConfig& config) const;
    std::string generateScatterPlot(const std::string& xSeries, const std::string& ySeries, const ChartConfig& config) const;
    std::string generateHistogram(const std::string& seriesName, int bins, const ChartConfig& config) const;
    std::string generateBoxPlot(const std::string& seriesName, const ChartConfig& config) const;
    std::string generateCorrelationMatrix(const std::vector<std::string>& seriesNames) const;
    
    // Export capabilities
    void exportToCSV(const std::string& filename, const std::vector<std::string>& seriesNames) const;
    void exportToJSON(const std::string& filename, const std::vector<std::string>& seriesNames) const;
    void generateReport(const std::string& filename, const std::vector<std::string>& seriesNames) const;
};

// Data Quality Assessment
class DataQualityAssessment {
private:
    struct QualityMetrics {
        double completeness;     // Percentage of non-null values
        double accuracy;         // Percentage of values within expected ranges
        double consistency;      // Percentage of values consistent with patterns
        double timeliness;       // Percentage of recent/current data
        double validity;         // Percentage of values conforming to format
        double uniqueness;       // Percentage of unique values (for keys that should be unique)
        
        QualityMetrics() : completeness(0), accuracy(0), consistency(0), 
                          timeliness(0), validity(0), uniqueness(0) {}
    };
    
    std::unordered_map<std::string, QualityMetrics> qualityMetrics;
    std::unordered_map<std::string, std::pair<double, double>> expectedRanges;
    std::unordered_map<std::string, std::vector<std::string>> validFormats;
    mutable std::mutex assessmentMutex;
    
    bool isValueInRange(double value, const std::pair<double, double>& range) const;
    bool isFormatValid(const std::string& value, const std::vector<std::string>& formats) const;
    double calculateTimeliness(const std::chrono::steady_clock::time_point& timestamp) const;
    
public:
    void setExpectedRange(const std::string& key, double min, double max);
    void addValidFormat(const std::string& key, const std::string& format);
    QualityMetrics assessDataQuality(const std::vector<DataPoint>& data, const std::string& key) const;
    std::unordered_map<std::string, QualityMetrics> assessAllDataQuality(const std::vector<DataPoint>& data) const;
    std::string generateQualityReport(const std::unordered_map<std::string, QualityMetrics>& metrics) const;
    std::vector<DataPoint> cleanData(const std::vector<DataPoint>& data) const;
    std::vector<DataPoint> imputeMissingValues(const std::vector<DataPoint>& data, const std::string& method = "mean") const;
};

// Main Advanced Data Analytics System
class AdvancedDataAnalyticsSystem {
private:
    std::unique_ptr<TimeSeriesAnalyzer> timeSeriesAnalyzer;
    std::unique_ptr<StatisticalEngine> statisticalEngine;
    std::unique_ptr<MachineLearningPipeline> mlPipeline;
    std::unique_ptr<StreamProcessor> streamProcessor;
    std::unique_ptr<VisualizationEngine> visualizationEngine;
    std::unique_ptr<DataQualityAssessment> qualityAssessment;
    
    std::unordered_map<std::string, std::vector<DataPoint>> historicalData;
    std::unordered_map<std::string, PredictionResult> cachedPredictions;
    std::unordered_map<std::string, AnomalyDetection> recentAnomalies;
    
    mutable std::mutex systemMutex;
    std::atomic<bool> systemRunning{false};
    std::thread mainAnalyticsThread;
    
    // Configuration
    struct AnalyticsConfig {
        size_t maxHistoricalDataPoints;
        int predictionHorizon;
        double anomalyThreshold;
        bool enableRealTimeProcessing;
        bool enablePredictiveAnalytics;
        bool enableAnomalyDetection;
        std::chrono::milliseconds processingInterval;
        
        AnalyticsConfig() : maxHistoricalDataPoints(100000), predictionHorizon(24),
                           anomalyThreshold(0.05), enableRealTimeProcessing(true),
                           enablePredictiveAnalytics(true), enableAnomalyDetection(true),
                           processingInterval(std::chrono::milliseconds(1000)) {}
    } config;
    
    void runMainAnalytics();
    void performScheduledAnalysis();
    void updatePredictiveModels();
    void checkForAnomalies();
    
public:
    AdvancedDataAnalyticsSystem();
    ~AdvancedDataAnalyticsSystem();
    
    // System control
    void start();
    void stop();
    bool isRunning() const { return systemRunning.load(); }
    
    // Configuration
    void setMaxHistoricalDataPoints(size_t maxPoints) { config.maxHistoricalDataPoints = maxPoints; }
    void setPredictionHorizon(int horizon) { config.predictionHorizon = horizon; }
    void setAnomalyThreshold(double threshold) { config.anomalyThreshold = threshold; }
    void setProcessingInterval(std::chrono::milliseconds interval) { config.processingInterval = interval; }
    void enableRealTimeProcessing(bool enable) { config.enableRealTimeProcessing = enable; }
    void enablePredictiveAnalytics(bool enable) { config.enablePredictiveAnalytics = enable; }
    void enableAnomalyDetection(bool enable) { config.enableAnomalyDetection = enable; }
    
    // Data ingestion
    void ingestDataPoint(const DataPoint& point);
    void ingestBatchData(const std::vector<DataPoint>& data);
    void ingestFromCSV(const std::string& filename);
    void ingestFromJSON(const std::string& filename);
    
    // Analytics operations
    StatisticalSummary getStatisticalSummary(const std::string& key) const;
    PredictionResult getPrediction(const std::string& key, size_t steps, ModelType model = ModelType::LINEAR_REGRESSION) const;
    std::vector<AnomalyDetection> getAnomalies(const std::string& key = "") const;
    std::vector<double> getCorrelation(const std::string& key1, const std::string& key2) const;
    
    // Machine learning
    void trainModel(const std::string& modelName, const std::string& dataKey, ModelType type);
    PredictionResult predictWithModel(const std::string& modelName, const std::vector<double>& features) const;
    double evaluateModelPerformance(const std::string& modelName, const std::string& testDataKey) const;
    
    // Visualization
    std::string generateChart(const std::string& chartType, const std::string& dataKey, const std::unordered_map<std::string, std::string>& options = {}) const;
    void exportAnalysisReport(const std::string& filename) const;
    void exportDataVisualization(const std::string& filename, const std::string& format = "html") const;
    
    // Data quality
    std::string getDataQualityReport(const std::string& key = "") const;
    std::vector<DataPoint> getCleanedData(const std::string& key) const;
    
    // System status and diagnostics
    std::string getSystemStatus() const;
    std::unordered_map<std::string, size_t> getDataSummary() const;
    void clearCache();
    void resetAnalytics();
    
    // Advanced analytics
    std::unordered_map<std::string, double> performSensitivityAnalysis(const std::string& targetKey, const std::vector<std::string>& inputKeys) const;
    std::vector<std::string> identifyKeyFactors(const std::string& targetKey, int topN = 5) const;
    std::unordered_map<std::string, std::vector<double>> performWhatIfAnalysis(const std::string& targetKey, const std::unordered_map<std::string, double>& scenarios) const;
    
    // Real-time processing
    void registerRealTimeProcessor(const std::string& key, std::function<void(const DataPoint&)> processor);
    void unregisterRealTimeProcessor(const std::string& key);
    
    // Alerting system
    using AlertCallback = std::function<void(const AnomalyDetection&)>;
    void registerAnomalyAlert(const std::string& key, AlertCallback callback);
    void unregisterAnomalyAlert(const std::string& key);
};

} // namespace analytics

// Implementation of key methods

inline double analytics::StatisticalEngine::calculateMean(const std::vector<double>& data) const {
    if (data.empty()) return 0.0;
    return std::accumulate(data.begin(), data.end(), 0.0) / data.size();
}

inline double analytics::StatisticalEngine::calculateMedian(std::vector<double> data) const {
    if (data.empty()) return 0.0;
    std::sort(data.begin(), data.end());
    size_t n = data.size();
    if (n % 2 == 0) {
        return (data[n/2 - 1] + data[n/2]) / 2.0;
    } else {
        return data[n/2];
    }
}

inline double analytics::StatisticalEngine::calculateStandardDeviation(const std::vector<double>& data) const {
    if (data.size() < 2) return 0.0;
    double mean = calculateMean(data);
    double sum = 0.0;
    for (double value : data) {
        sum += (value - mean) * (value - mean);
    }
    return std::sqrt(sum / (data.size() - 1));
}

inline double analytics::StatisticalEngine::calculateVariance(const std::vector<double>& data) const {
    double stddev = calculateStandardDeviation(data);
    return stddev * stddev;
}

inline double analytics::StatisticalEngine::calculateCorrelationCoefficient(const std::vector<double>& x, const std::vector<double>& y) const {
    if (x.size() != y.size() || x.empty()) return 0.0;
    
    double meanX = calculateMean(x);
    double meanY = calculateMean(y);
    
    double numerator = 0.0;
    double denomX = 0.0;
    double denomY = 0.0;
    
    for (size_t i = 0; i < x.size(); ++i) {
        double diffX = x[i] - meanX;
        double diffY = y[i] - meanY;
        numerator += diffX * diffY;
        denomX += diffX * diffX;
        denomY += diffY * diffY;
    }
    
    if (denomX == 0.0 || denomY == 0.0) return 0.0;
    return numerator / std::sqrt(denomX * denomY);
}

inline std::pair<double, double> analytics::StatisticalEngine::linearRegression(const std::vector<double>& x, const std::vector<double>& y) const {
    if (x.size() != y.size() || x.empty()) return {0.0, 0.0};
    
    double meanX = calculateMean(x);
    double meanY = calculateMean(y);
    
    double numerator = 0.0;
    double denominator = 0.0;
    
    for (size_t i = 0; i < x.size(); ++i) {
        double diffX = x[i] - meanX;
        numerator += diffX * (y[i] - meanY);
        denominator += diffX * diffX;
    }
    
    if (denominator == 0.0) return {0.0, meanY};
    
    double slope = numerator / denominator;
    double intercept = meanY - slope * meanX;
    
    return {slope, intercept};
}

inline analytics::StatisticalSummary analytics::TimeSeriesAnalyzer::calculateStatistics(const std::string& key) const {
    std::vector<double> values = extractValues(key);
    StatisticalSummary summary;
    
    if (values.empty()) return summary;
    
    // Use the statistical engine for calculations
    StatisticalEngine engine;
    summary.mean = engine.calculateMean(values);
    summary.median = engine.calculateMedian(values);
    summary.stddev = engine.calculateStandardDeviation(values);
    summary.variance = engine.calculateVariance(values);
    summary.count = values.size();
    
    auto minMax = std::minmax_element(values.begin(), values.end());
    summary.min = *minMax.first;
    summary.max = *minMax.second;
    
    // Calculate percentiles
    summary.percentiles.resize(11); // 0%, 10%, 20%, ..., 100%
    for (int i = 0; i <= 10; ++i) {
        summary.percentiles[i] = engine.calculatePercentile(values, i * 10.0);
    }
    
    return summary;
}

inline std::vector<double> analytics::TimeSeriesAnalyzer::calculateMovingAverage(const std::vector<double>& data, size_t window) const {
    std::vector<double> result;
    if (data.size() < window) return result;
    
    result.reserve(data.size() - window + 1);
    for (size_t i = window - 1; i < data.size(); ++i) {
        double sum = 0.0;
        for (size_t j = i - window + 1; j <= i; ++j) {
            sum += data[j];
        }
        result.push_back(sum / window);
    }
    
    return result;
}

inline std::vector<double> analytics::TimeSeriesAnalyzer::calculateExponentialSmoothing(const std::vector<double>& data, double alpha) const {
    std::vector<double> result;
    if (data.empty()) return result;
    
    result.reserve(data.size());
    result.push_back(data[0]); // Initialize with first value
    
    for (size_t i = 1; i < data.size(); ++i) {
        result.push_back(alpha * data[i] + (1 - alpha) * result[i-1]);
    }
    
    return result;
}

// Constructor and Destructor implementations
inline analytics::AdvancedDataAnalyticsSystem::AdvancedDataAnalyticsSystem() 
    : timeSeriesAnalyzer(std::make_unique<TimeSeriesAnalyzer>())
    , statisticalEngine(std::make_unique<StatisticalEngine>())
    , mlPipeline(std::make_unique<MachineLearningPipeline>())
    , streamProcessor(std::make_unique<StreamProcessor>())
    , visualizationEngine(std::make_unique<VisualizationEngine>())
    , qualityAssessment(std::make_unique<DataQualityAssessment>()) {
}

inline analytics::AdvancedDataAnalyticsSystem::~AdvancedDataAnalyticsSystem() {
    stop();
}

inline void analytics::AdvancedDataAnalyticsSystem::start() {
    if (systemRunning.load()) return;
    
    systemRunning.store(true);
    if (config.enableRealTimeProcessing) {
        streamProcessor->start();
    }
    
    mainAnalyticsThread = std::thread(&AdvancedDataAnalyticsSystem::runMainAnalytics, this);
    
    std::cout << "[AdvancedDataAnalytics] System started successfully\n";
}

inline void analytics::AdvancedDataAnalyticsSystem::stop() {
    if (!systemRunning.load()) return;
    
    systemRunning.store(false);
    
    if (config.enableRealTimeProcessing) {
        streamProcessor->stop();
    }
    
    if (mainAnalyticsThread.joinable()) {
        mainAnalyticsThread.join();
    }
    
    std::cout << "[AdvancedDataAnalytics] System stopped successfully\n";
}

inline void analytics::AdvancedDataAnalyticsSystem::ingestDataPoint(const DataPoint& point) {
    {
        std::lock_guard<std::mutex> lock(systemMutex);
        historicalData[point.key].push_back(point);
        
        // Maintain size limit
        if (historicalData[point.key].size() > config.maxHistoricalDataPoints) {
            historicalData[point.key].erase(historicalData[point.key].begin());
        }
    }
    
    timeSeriesAnalyzer->addDataPoint(point);
    
    if (config.enableRealTimeProcessing) {
        streamProcessor->addDataPoint(point);
    }
}

inline analytics::StatisticalSummary analytics::AdvancedDataAnalyticsSystem::getStatisticalSummary(const std::string& key) const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(systemMutex));
    
    StatisticalSummary summary;
    auto it = historicalData.find(key);
    if (it == historicalData.end() || it->second.empty()) {
        return summary;
    }
    
    const auto& data = it->second;
    std::vector<double> values;
    values.reserve(data.size());
    for (const auto& point : data) {
        if (!point.values.empty()) {
            values.push_back(point.values[0]);
        }
    }
    
    StatisticalSummary result;
    if (!values.empty()) {
        double sum = 0.0;
        for (double val : values) sum += val;
        result.mean = sum / values.size();
        result.count = values.size();
    }
    return result;
}

inline std::string analytics::AdvancedDataAnalyticsSystem::getSystemStatus() const {
    std::ostringstream status;
    status << "=== Advanced Data Analytics System Status ===\n";
    status << "System Running: " << (systemRunning.load() ? "Yes" : "No") << "\n";
    status << "Real-time Processing: " << (config.enableRealTimeProcessing ? "Enabled" : "Disabled") << "\n";
    status << "Predictive Analytics: " << (config.enablePredictiveAnalytics ? "Enabled" : "Disabled") << "\n";
    status << "Anomaly Detection: " << (config.enableAnomalyDetection ? "Enabled" : "Disabled") << "\n";
    status << "Historical Data Points: " << config.maxHistoricalDataPoints << "\n";
    status << "Prediction Horizon: " << config.predictionHorizon << "\n";
    status << "Processing Interval: " << config.processingInterval.count() << " ms\n";
    
    {
        std::lock_guard<std::mutex> lock(systemMutex);
        status << "Data Series Count: " << historicalData.size() << "\n";
        status << "Cached Predictions: " << cachedPredictions.size() << "\n";
        status << "Recent Anomalies: " << recentAnomalies.size() << "\n";
    }
    
    return status.str();
}

// ===== Inline implementations for real-time StreamProcessor =====
inline void analytics::StreamProcessor::processStream() {
    while (isRunning.load()) {
        std::unique_lock<std::mutex> lock(bufferMutex);
        bufferCondition.wait_for(lock, std::chrono::milliseconds(50), [this]{
            return !streamBuffer.empty() || !isRunning.load();
        });
        while (!streamBuffer.empty()) {
            auto point = streamBuffer.front();
            streamBuffer.pop();
            lock.unlock();
            // dispatch
            auto it = processors.find(point.key);
            if (it != processors.end() && it->second) {
                try { it->second(point); } catch (...) { /* swallow in stub */ }
            }
            lock.lock();
        }
    }
}

inline void analytics::StreamProcessor::start() {
    if (isRunning.load()) return;
    isRunning.store(true);
    processingThread = std::thread(&StreamProcessor::processStream, this);
}

inline void analytics::StreamProcessor::stop() {
    if (!isRunning.load()) return;
    isRunning.store(false);
    bufferCondition.notify_all();
    if (processingThread.joinable()) processingThread.join();
}

inline void analytics::StreamProcessor::addDataPoint(const DataPoint& point) {
    std::lock_guard<std::mutex> lock(bufferMutex);
    if (streamBuffer.size() >= maxBufferSize) {
        // drop oldest
        streamBuffer.pop();
    }
    streamBuffer.push(point);
    bufferCondition.notify_one();
}

inline void analytics::StreamProcessor::registerProcessor(const std::string& key, std::function<void(const DataPoint&)> processor) {
    std::lock_guard<std::mutex> lock(bufferMutex);
    processors[key] = std::move(processor);
}

inline void analytics::StreamProcessor::unregisterProcessor(const std::string& key) {
    std::lock_guard<std::mutex> lock(bufferMutex);
    processors.erase(key);
}

inline size_t analytics::StreamProcessor::getBufferSize() const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(bufferMutex));
    return streamBuffer.size();
}

inline void analytics::StreamProcessor::clearBuffer() {
    std::lock_guard<std::mutex> lock(bufferMutex);
    while (!streamBuffer.empty()) streamBuffer.pop();
}

// ===== Inline implementations for AdvancedDataAnalyticsSystem loop and helpers =====
inline void analytics::AdvancedDataAnalyticsSystem::runMainAnalytics() {
    while (systemRunning.load()) {
        // In a full implementation, we would perform periodic analysis
        // Here, keep it lightweight to avoid heavy CPU usage
        std::this_thread::sleep_for(config.processingInterval);
        performScheduledAnalysis();
        if (config.enablePredictiveAnalytics) updatePredictiveModels();
        if (config.enableAnomalyDetection) checkForAnomalies();
    }
}

inline void analytics::AdvancedDataAnalyticsSystem::performScheduledAnalysis() {
    // Placeholder: aggregate stats or rollups
}

inline void analytics::AdvancedDataAnalyticsSystem::updatePredictiveModels() {
    // Placeholder: retrain or refresh cached predictions
}

inline void analytics::AdvancedDataAnalyticsSystem::checkForAnomalies() {
    // Placeholder: run quick anomaly checks
}

#endif // ADVANCED_DATA_ANALYTICS_H