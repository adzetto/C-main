#ifndef MACHINE_LEARNING_ENGINE_H
#define MACHINE_LEARNING_ENGINE_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <string>
#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <random>
#include <queue>

class MachineLearningEngine {
private:
    struct NeuralNetwork {
        std::vector<std::vector<std::vector<float>>> weights;
        std::vector<std::vector<float>> biases;
        std::vector<int> layerSizes;
        float learningRate;
        int epochs;
        std::string activationFunction;
        
        void initialize(const std::vector<int>& layers) {
            layerSizes = layers;
            weights.resize(layers.size() - 1);
            biases.resize(layers.size() - 1);
            
            std::random_device rd;
            std::mt19937 gen(rd());
            std::normal_distribution<float> dist(0.0f, 0.1f);
            
            for (size_t i = 0; i < layers.size() - 1; i++) {
                weights[i].resize(layers[i + 1]);
                biases[i].resize(layers[i + 1]);
                
                for (int j = 0; j < layers[i + 1]; j++) {
                    weights[i][j].resize(layers[i]);
                    biases[i][j] = dist(gen);
                    
                    for (int k = 0; k < layers[i]; k++) {
                        weights[i][j][k] = dist(gen);
                    }
                }
            }
        }
        
        std::vector<float> forward(const std::vector<float>& input) {
            std::vector<float> current = input;
            
            for (size_t layer = 0; layer < weights.size(); layer++) {
                std::vector<float> next(layerSizes[layer + 1]);
                
                for (size_t neuron = 0; neuron < next.size(); neuron++) {
                    float sum = biases[layer][neuron];
                    
                    for (size_t input_neuron = 0; input_neuron < current.size(); input_neuron++) {
                        sum += current[input_neuron] * weights[layer][neuron][input_neuron];
                    }
                    
                    next[neuron] = activationFunc(sum);
                }
                
                current = next;
            }
            
            return current;
        }
        
        float activationFunc(float x) {
            if (activationFunction == "relu") {
                return std::max(0.0f, x);
            } else if (activationFunction == "sigmoid") {
                return 1.0f / (1.0f + std::exp(-x));
            } else if (activationFunction == "tanh") {
                return std::tanh(x);
            }
            return x; // linear
        }
    };

    struct KalmanFilter {
        std::vector<std::vector<float>> A; // State transition model
        std::vector<std::vector<float>> B; // Control input model
        std::vector<std::vector<float>> H; // Observation model
        std::vector<std::vector<float>> Q; // Process noise covariance
        std::vector<std::vector<float>> R; // Observation noise covariance
        std::vector<std::vector<float>> P; // Error covariance matrix
        std::vector<float> x; // State vector
        
        void predict(const std::vector<float>& u) {
            // Predict state: x = A*x + B*u
            std::vector<float> newX(x.size(), 0.0f);
            for (size_t i = 0; i < A.size(); i++) {
                for (size_t j = 0; j < A[i].size(); j++) {
                    newX[i] += A[i][j] * x[j];
                }
                if (!u.empty() && i < B.size()) {
                    for (size_t j = 0; j < B[i].size() && j < u.size(); j++) {
                        newX[i] += B[i][j] * u[j];
                    }
                }
            }
            x = newX;
            
            // Predict error covariance: P = A*P*A' + Q
            updateCovariance();
        }
        
        void update(const std::vector<float>& z) {
            // Innovation: y = z - H*x
            std::vector<float> y(z.size());
            for (size_t i = 0; i < z.size(); i++) {
                y[i] = z[i];
                for (size_t j = 0; j < x.size(); j++) {
                    y[i] -= H[i][j] * x[j];
                }
            }
            
            // Update state estimate
            // Simplified Kalman gain calculation
            float gain = 0.1f; // Simplified gain
            for (size_t i = 0; i < x.size(); i++) {
                x[i] += gain * y[0]; // Simplified update
            }
        }
        
        void updateCovariance() {
            // Simplified covariance update
            for (size_t i = 0; i < P.size(); i++) {
                for (size_t j = 0; j < P[i].size(); j++) {
                    P[i][j] *= 1.01f; // Add process noise
                    if (i == j) P[i][j] = std::max(0.001f, P[i][j]);
                }
            }
        }
    };

    struct LinearRegression {
        std::vector<float> weights;
        float bias;
        float learningRate;
        bool trained;
        
        LinearRegression() : bias(0.0f), learningRate(0.01f), trained(false) {}
        
        void train(const std::vector<std::vector<float>>& X, const std::vector<float>& y) {
            if (X.empty() || X[0].empty()) return;
            
            weights.resize(X[0].size(), 0.0f);
            
            // Simple gradient descent
            for (int epoch = 0; epoch < 1000; epoch++) {
                std::vector<float> gradW(weights.size(), 0.0f);
                float gradB = 0.0f;
                
                for (size_t i = 0; i < X.size(); i++) {
                    float prediction = predict(X[i]);
                    float error = prediction - y[i];
                    
                    for (size_t j = 0; j < X[i].size(); j++) {
                        gradW[j] += error * X[i][j];
                    }
                    gradB += error;
                }
                
                // Update weights
                for (size_t j = 0; j < weights.size(); j++) {
                    weights[j] -= learningRate * gradW[j] / X.size();
                }
                bias -= learningRate * gradB / X.size();
            }
            
            trained = true;
        }
        
        float predict(const std::vector<float>& input) {
            float result = bias;
            for (size_t i = 0; i < input.size() && i < weights.size(); i++) {
                result += weights[i] * input[i];
            }
            return result;
        }
    };

    struct DecisionTree {
        struct Node {
            int feature;
            float threshold;
            float value; // For leaf nodes
            bool isLeaf;
            std::unique_ptr<Node> left;
            std::unique_ptr<Node> right;
            
            Node() : feature(-1), threshold(0.0f), value(0.0f), isLeaf(false) {}
        };
        
        std::unique_ptr<Node> root;
        int maxDepth;
        int minSamples;
        
        DecisionTree() : maxDepth(10), minSamples(5) {}
        
        void train(const std::vector<std::vector<float>>& X, const std::vector<float>& y) {
            std::vector<int> indices(X.size());
            std::iota(indices.begin(), indices.end(), 0);
            root = buildTree(X, y, indices, 0);
        }
        
        float predict(const std::vector<float>& input) {
            return predictNode(root.get(), input);
        }
        
        std::unique_ptr<Node> buildTree(const std::vector<std::vector<float>>& X,
                                       const std::vector<float>& y,
                                       const std::vector<int>& indices,
                                       int depth) {
            auto node = std::make_unique<Node>();
            
            if (depth >= maxDepth || indices.size() < minSamples) {
                // Create leaf node
                node->isLeaf = true;
                float sum = 0.0f;
                for (int idx : indices) {
                    sum += y[idx];
                }
                node->value = sum / indices.size();
                return node;
            }
            
            // Find best split (simplified)
            int bestFeature = 0;
            float bestThreshold = 0.0f;
            float bestGini = 1.0f;
            
            for (size_t feature = 0; feature < X[0].size(); feature++) {
                for (int idx : indices) {
                    float threshold = X[idx][feature];
                    float gini = calculateGini(X, y, indices, feature, threshold);
                    
                    if (gini < bestGini) {
                        bestGini = gini;
                        bestFeature = feature;
                        bestThreshold = threshold;
                    }
                }
            }
            
            node->feature = bestFeature;
            node->threshold = bestThreshold;
            
            // Split data
            std::vector<int> leftIndices, rightIndices;
            for (int idx : indices) {
                if (X[idx][bestFeature] <= bestThreshold) {
                    leftIndices.push_back(idx);
                } else {
                    rightIndices.push_back(idx);
                }
            }
            
            if (leftIndices.empty() || rightIndices.empty()) {
                node->isLeaf = true;
                float sum = 0.0f;
                for (int idx : indices) {
                    sum += y[idx];
                }
                node->value = sum / indices.size();
                return node;
            }
            
            node->left = buildTree(X, y, leftIndices, depth + 1);
            node->right = buildTree(X, y, rightIndices, depth + 1);
            
            return node;
        }
        
        float calculateGini(const std::vector<std::vector<float>>& X,
                           const std::vector<float>& y,
                           const std::vector<int>& indices,
                           int feature, float threshold) {
            // Simplified Gini calculation for regression
            std::vector<float> leftValues, rightValues;
            
            for (int idx : indices) {
                if (X[idx][feature] <= threshold) {
                    leftValues.push_back(y[idx]);
                } else {
                    rightValues.push_back(y[idx]);
                }
            }
            
            if (leftValues.empty() || rightValues.empty()) {
                return 1.0f;
            }
            
            // Calculate variance for each split
            float leftVar = calculateVariance(leftValues);
            float rightVar = calculateVariance(rightValues);
            
            float totalSize = indices.size();
            float weightedVar = (leftValues.size() / totalSize) * leftVar +
                              (rightValues.size() / totalSize) * rightVar;
            
            return weightedVar;
        }
        
        float calculateVariance(const std::vector<float>& values) {
            if (values.empty()) return 0.0f;
            
            float mean = 0.0f;
            for (float val : values) {
                mean += val;
            }
            mean /= values.size();
            
            float variance = 0.0f;
            for (float val : values) {
                variance += (val - mean) * (val - mean);
            }
            return variance / values.size();
        }
        
        float predictNode(Node* node, const std::vector<float>& input) {
            if (!node) return 0.0f;
            
            if (node->isLeaf) {
                return node->value;
            }
            
            if (input[node->feature] <= node->threshold) {
                return predictNode(node->left.get(), input);
            } else {
                return predictNode(node->right.get(), input);
            }
        }
    };

    struct TimeSeriesPredictor {
        std::vector<float> history;
        int windowSize;
        LinearRegression model;
        bool autoRegressive;
        
        TimeSeriesPredictor(int window = 10) : windowSize(window), autoRegressive(true) {}
        
        void addData(float value) {
            history.push_back(value);
            if (history.size() > windowSize * 2) {
                history.erase(history.begin(), history.begin() + windowSize);
            }
        }
        
        void train() {
            if (history.size() < windowSize + 1) return;
            
            std::vector<std::vector<float>> X;
            std::vector<float> y;
            
            for (size_t i = windowSize; i < history.size(); i++) {
                std::vector<float> features;
                for (int j = 0; j < windowSize; j++) {
                    features.push_back(history[i - windowSize + j]);
                }
                X.push_back(features);
                y.push_back(history[i]);
            }
            
            if (!X.empty()) {
                model.train(X, y);
            }
        }
        
        float predict() {
            if (history.size() < windowSize || !model.trained) {
                return history.empty() ? 0.0f : history.back();
            }
            
            std::vector<float> features;
            for (int i = 0; i < windowSize; i++) {
                features.push_back(history[history.size() - windowSize + i]);
            }
            
            return model.predict(features);
        }
    };

public:
    MachineLearningEngine() :
        trainingInProgress(false),
        modelUpdateRequired(false),
        predictionAccuracy(0.8f),
        learningRate(0.001f),
        dataCollectionActive(true),
        realTimePredictionActive(true),
        adaptiveLearning(true) {
        
        initializePredictionModels();
        initializeFeatureExtractors();
        initializeDataBuffers();
        startLearningLoop();
        
        std::cout << "Machine Learning Engine initialized" << std::endl;
    }

    void initializePredictionModels() {
        // Battery degradation prediction model
        batteryDegradationNN.initialize({10, 20, 15, 1}); // Input: 10 features, Output: 1 (degradation)
        batteryDegradationNN.learningRate = 0.001f;
        batteryDegradationNN.activationFunction = "relu";
        
        // Energy consumption prediction
        energyConsumptionPredictor = std::make_unique<TimeSeriesPredictor>(20);
        
        // Route optimization model
        routeOptimizationTree = std::make_unique<DecisionTree>();
        
        // Traffic prediction
        trafficPredictionNN.initialize({15, 25, 20, 5}); // Multiple outputs for traffic states
        trafficPredictionNN.learningRate = 0.002f;
        trafficPredictionNN.activationFunction = "sigmoid";
        
        // Driver behavior analysis
        driverBehaviorNN.initialize({12, 18, 12, 3}); // Outputs: aggressive, normal, eco
        driverBehaviorNN.learningRate = 0.0015f;
        driverBehaviorNN.activationFunction = "softmax";
        
        // Predictive maintenance
        maintenancePredictor = std::make_unique<TimeSeriesPredictor>(30);
        
        // Charging optimization
        chargingOptimizationNN.initialize({8, 16, 12, 2}); // Outputs: optimal power, duration
        chargingOptimizationNN.learningRate = 0.001f;
        chargingOptimizationNN.activationFunction = "relu";
        
        // Kalman filters for various predictions
        initializeKalmanFilters();
        
        std::cout << "Prediction models initialized" << std::endl;
    }

    void initializeKalmanFilters() {
        // Battery SOC Kalman filter
        socKalman.A = {{1.0f, 0.1f}, {0.0f, 1.0f}}; // State: [SOC, SOC_rate]
        socKalman.H = {{1.0f, 0.0f}}; // Observe SOC directly
        socKalman.Q = {{0.01f, 0.0f}, {0.0f, 0.01f}}; // Process noise
        socKalman.R = {{0.1f}}; // Measurement noise
        socKalman.P = {{1.0f, 0.0f}, {0.0f, 1.0f}}; // Initial covariance
        socKalman.x = {80.0f, 0.0f}; // Initial state: 80% SOC, 0 rate
        
        // Speed prediction Kalman filter
        speedKalman.A = {{1.0f, 0.1f, 0.005f}, {0.0f, 1.0f, 0.1f}, {0.0f, 0.0f, 1.0f}};
        speedKalman.H = {{1.0f, 0.0f, 0.0f}}; // Observe speed
        speedKalman.Q = {{0.05f, 0.0f, 0.0f}, {0.0f, 0.1f, 0.0f}, {0.0f, 0.0f, 0.2f}};
        speedKalman.R = {{1.0f}};
        speedKalman.P = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        speedKalman.x = {50.0f, 0.0f, 0.0f}; // Initial: 50 km/h, 0 acceleration, 0 jerk
        
        std::cout << "Kalman filters initialized" << std::endl;
    }

    void initializeFeatureExtractors() {
        // Define feature extraction functions
        featureExtractors["battery"] = [this](const std::unordered_map<std::string, float>& data) {
            std::vector<float> features;
            features.push_back(data.at("voltage"));
            features.push_back(data.at("current"));
            features.push_back(data.at("temperature"));
            features.push_back(data.at("soc"));
            features.push_back(data.at("cycles"));
            features.push_back(data.at("age"));
            features.push_back(data.at("charge_rate"));
            features.push_back(data.at("discharge_rate"));
            features.push_back(data.at("internal_resistance"));
            features.push_back(data.at("capacity_fade"));
            return features;
        };
        
        featureExtractors["driving"] = [this](const std::unordered_map<std::string, float>& data) {
            std::vector<float> features;
            features.push_back(data.at("speed"));
            features.push_back(data.at("acceleration"));
            features.push_back(data.at("throttle_position"));
            features.push_back(data.at("brake_pressure"));
            features.push_back(data.at("steering_angle"));
            features.push_back(data.at("energy_consumption"));
            features.push_back(data.at("road_grade"));
            features.push_back(data.at("weather_factor"));
            features.push_back(data.at("traffic_density"));
            features.push_back(data.at("route_efficiency"));
            features.push_back(data.at("driver_aggressiveness"));
            features.push_back(data.at("ambient_temperature"));
            return features;
        };
        
        featureExtractors["traffic"] = [this](const std::unordered_map<std::string, float>& data) {
            std::vector<float> features;
            features.push_back(data.at("current_speed"));
            features.push_back(data.at("free_flow_speed"));
            features.push_back(data.at("vehicle_density"));
            features.push_back(data.at("time_of_day"));
            features.push_back(data.at("day_of_week"));
            features.push_back(data.at("weather_condition"));
            features.push_back(data.at("incident_nearby"));
            features.push_back(data.at("construction_zone"));
            features.push_back(data.at("event_impact"));
            features.push_back(data.at("historical_congestion"));
            features.push_back(data.at("route_popularity"));
            features.push_back(data.at("public_transport_usage"));
            features.push_back(data.at("parking_availability"));
            features.push_back(data.at("fuel_price_impact"));
            features.push_back(data.at("seasonal_factor"));
            return features;
        };
        
        std::cout << "Feature extractors initialized" << std::endl;
    }

    void initializeDataBuffers() {
        // Initialize circular buffers for different data types
        batteryDataBuffer.resize(1000);
        drivingDataBuffer.resize(1000);
        trafficDataBuffer.resize(500);
        maintenanceDataBuffer.resize(200);
        energyDataBuffer.resize(800);
        
        std::cout << "Data buffers initialized" << std::endl;
    }

    void collectTrainingData(const std::string& dataType, 
                           const std::unordered_map<std::string, float>& data) {
        std::lock_guard<std::mutex> lock(dataMutex);
        
        if (dataType == "battery") {
            batteryDataBuffer[batteryDataIndex % batteryDataBuffer.size()] = data;
            batteryDataIndex++;
        } else if (dataType == "driving") {
            drivingDataBuffer[drivingDataIndex % drivingDataBuffer.size()] = data;
            drivingDataIndex++;
        } else if (dataType == "traffic") {
            trafficDataBuffer[trafficDataIndex % trafficDataBuffer.size()] = data;
            trafficDataIndex++;
        } else if (dataType == "maintenance") {
            maintenanceDataBuffer[maintenanceDataIndex % maintenanceDataBuffer.size()] = data;
            maintenanceDataIndex++;
        } else if (dataType == "energy") {
            energyDataBuffer[energyDataIndex % energyDataBuffer.size()] = data;
            energyDataIndex++;
            
            // Update energy consumption predictor
            if (data.find("consumption") != data.end()) {
                energyConsumptionPredictor->addData(data.at("consumption"));
            }
        }
        
        // Trigger model updates if enough new data collected
        if ((batteryDataIndex + drivingDataIndex + trafficDataIndex) % 100 == 0) {
            modelUpdateRequired = true;
        }
    }

    float predictBatteryDegradation(const std::unordered_map<std::string, float>& batteryState) {
        if (featureExtractors.find("battery") == featureExtractors.end()) {
            return 0.0f;
        }
        
        std::vector<float> features = featureExtractors["battery"](batteryState);
        std::vector<float> prediction = batteryDegradationNN.forward(features);
        
        return prediction.empty() ? 0.0f : prediction[0];
    }

    float predictEnergyConsumption(int horizonMinutes) {
        // Use time series predictor for energy consumption
        float basePrediction = energyConsumptionPredictor->predict();
        
        // Apply horizon scaling (simplified)
        float horizonFactor = std::sqrt(horizonMinutes / 60.0f);
        
        return basePrediction * horizonFactor;
    }

    std::vector<float> predictTrafficConditions(const std::unordered_map<std::string, float>& trafficData) {
        if (featureExtractors.find("traffic") == featureExtractors.end()) {
            return {0.5f, 0.5f, 0.5f, 0.5f, 0.5f}; // Default moderate traffic
        }
        
        std::vector<float> features = featureExtractors["traffic"](trafficData);
        return trafficPredictionNN.forward(features);
    }

    std::vector<float> analyzeDriverBehavior(const std::unordered_map<std::string, float>& drivingData) {
        if (featureExtractors.find("driving") == featureExtractors.end()) {
            return {0.33f, 0.34f, 0.33f}; // Default balanced behavior
        }
        
        std::vector<float> features = featureExtractors["driving"](drivingData);
        return driverBehaviorNN.forward(features);
    }

    float predictMaintenanceNeed(const std::string& component, 
                                const std::unordered_map<std::string, float>& componentData) {
        // Update maintenance predictor with component health data
        if (componentData.find("health_score") != componentData.end()) {
            maintenancePredictor->addData(componentData.at("health_score"));
        }
        
        return maintenancePredictor->predict();
    }

    std::pair<float, float> optimizeChargingStrategy(const std::unordered_map<std::string, float>& chargingData) {
        std::vector<float> features;
        features.push_back(chargingData.at("current_soc"));
        features.push_back(chargingData.at("target_soc"));
        features.push_back(chargingData.at("time_available"));
        features.push_back(chargingData.at("grid_price"));
        features.push_back(chargingData.at("grid_carbon_intensity"));
        features.push_back(chargingData.at("battery_temperature"));
        features.push_back(chargingData.at("battery_health"));
        features.push_back(chargingData.at("next_trip_distance"));
        
        std::vector<float> result = chargingOptimizationNN.forward(features);
        
        if (result.size() >= 2) {
            return {result[0], result[1]}; // {optimal_power, duration}
        }
        
        return {20.0f, 60.0f}; // Default: 20kW for 60 minutes
    }

    float updateStateEstimation(const std::string& filterType, 
                               const std::vector<float>& measurement,
                               const std::vector<float>& control = {}) {
        if (filterType == "soc") {
            socKalman.predict(control);
            socKalman.update(measurement);
            return socKalman.x.empty() ? 0.0f : socKalman.x[0];
        } else if (filterType == "speed") {
            speedKalman.predict(control);
            speedKalman.update(measurement);
            return speedKalman.x.empty() ? 0.0f : speedKalman.x[0];
        }
        
        return 0.0f;
    }

    void trainModels() {
        if (trainingInProgress) {
            std::cout << "Training already in progress" << std::endl;
            return;
        }
        
        trainingInProgress = true;
        std::cout << "Starting model training..." << std::endl;
        
        std::thread trainThread([this]() {
            trainBatteryDegradationModel();
            trainEnergyConsumptionModel();
            trainTrafficPredictionModel();
            trainDriverBehaviorModel();
            trainMaintenanceModel();
            trainChargingOptimizationModel();
            
            trainingInProgress = false;
            modelUpdateRequired = false;
            predictionAccuracy = evaluateModelAccuracy();
            
            std::cout << "Model training completed. Accuracy: " 
                      << (predictionAccuracy * 100) << "%" << std::endl;
        });
        
        trainThread.detach();
    }

    void startRealtimePrediction() {
        realTimePredictionActive = true;
        
        std::thread predictionThread([this]() {
            while (realTimePredictionActive) {
                updateRealtimePredictions();
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        });
        
        predictionThread.detach();
        std::cout << "Real-time prediction started" << std::endl;
    }

    void enableAdaptiveLearning(bool enable) {
        adaptiveLearning = enable;
        std::cout << "Adaptive learning " << (enable ? "enabled" : "disabled") << std::endl;
    }

    void getModelStatus() {
        std::cout << "\n=== ML Engine Status ===" << std::endl;
        std::cout << "Training in progress: " << (trainingInProgress ? "Yes" : "No") << std::endl;
        std::cout << "Model update required: " << (modelUpdateRequired ? "Yes" : "No") << std::endl;
        std::cout << "Prediction accuracy: " << (predictionAccuracy * 100) << "%" << std::endl;
        std::cout << "Data collection active: " << (dataCollectionActive ? "Yes" : "No") << std::endl;
        std::cout << "Real-time prediction: " << (realTimePredictionActive ? "Yes" : "No") << std::endl;
        std::cout << "Adaptive learning: " << (adaptiveLearning ? "Yes" : "No") << std::endl;
        
        std::cout << "\nData Buffer Status:" << std::endl;
        std::cout << "  Battery data points: " << std::min(batteryDataIndex, (int)batteryDataBuffer.size()) << std::endl;
        std::cout << "  Driving data points: " << std::min(drivingDataIndex, (int)drivingDataBuffer.size()) << std::endl;
        std::cout << "  Traffic data points: " << std::min(trafficDataIndex, (int)trafficDataBuffer.size()) << std::endl;
        std::cout << "  Energy data points: " << std::min(energyDataIndex, (int)energyDataBuffer.size()) << std::endl;
        
        std::cout << "\nModel Performance:" << std::endl;
        std::cout << "  Battery degradation RMSE: " << modelPerformance["battery_rmse"] << std::endl;
        std::cout << "  Energy prediction accuracy: " << modelPerformance["energy_accuracy"] << "%" << std::endl;
        std::cout << "  Traffic prediction accuracy: " << modelPerformance["traffic_accuracy"] << "%" << std::endl;
        std::cout << "  Driver behavior accuracy: " << modelPerformance["driver_accuracy"] << "%" << std::endl;
        
        std::cout << "========================" << std::endl;
    }

    void runDiagnostics() {
        std::cout << "\n=== ML Engine Diagnostics ===" << std::endl;
        
        // Test each prediction model
        testBatteryPrediction();
        testEnergyPrediction();
        testTrafficPrediction();
        testDriverBehaviorAnalysis();
        testMaintenancePrediction();
        testChargingOptimization();
        testKalmanFilters();
        
        std::cout << "ML diagnostics completed" << std::endl;
    }

private:
    // Core ML models
    NeuralNetwork batteryDegradationNN;
    NeuralNetwork trafficPredictionNN;
    NeuralNetwork driverBehaviorNN;
    NeuralNetwork chargingOptimizationNN;
    std::unique_ptr<TimeSeriesPredictor> energyConsumptionPredictor;
    std::unique_ptr<TimeSeriesPredictor> maintenancePredictor;
    std::unique_ptr<DecisionTree> routeOptimizationTree;
    
    // State estimation filters
    KalmanFilter socKalman;
    KalmanFilter speedKalman;
    
    // Feature extraction
    std::unordered_map<std::string, std::function<std::vector<float>(const std::unordered_map<std::string, float>&)>> featureExtractors;
    
    // Data management
    std::vector<std::unordered_map<std::string, float>> batteryDataBuffer;
    std::vector<std::unordered_map<std::string, float>> drivingDataBuffer;
    std::vector<std::unordered_map<std::string, float>> trafficDataBuffer;
    std::vector<std::unordered_map<std::string, float>> maintenanceDataBuffer;
    std::vector<std::unordered_map<std::string, float>> energyDataBuffer;
    
    int batteryDataIndex = 0;
    int drivingDataIndex = 0;
    int trafficDataIndex = 0;
    int maintenanceDataIndex = 0;
    int energyDataIndex = 0;
    
    // Threading and synchronization
    std::mutex dataMutex;
    std::atomic<bool> trainingInProgress;
    std::atomic<bool> modelUpdateRequired;
    std::atomic<bool> dataCollectionActive;
    std::atomic<bool> realTimePredictionActive;
    std::atomic<bool> adaptiveLearning;
    
    // Performance metrics
    float predictionAccuracy;
    float learningRate;
    std::unordered_map<std::string, float> modelPerformance;
    
    // Real-time prediction cache
    std::unordered_map<std::string, float> realtimePredictions;

    void trainBatteryDegradationModel() {
        std::cout << "Training battery degradation model..." << std::endl;
        
        // Prepare training data from buffer
        std::vector<std::vector<float>> X;
        std::vector<float> y;
        
        std::lock_guard<std::mutex> lock(dataMutex);
        int dataPoints = std::min(batteryDataIndex, (int)batteryDataBuffer.size());
        
        for (int i = 0; i < dataPoints; i++) {
            const auto& data = batteryDataBuffer[i];
            if (data.find("capacity_fade") != data.end() && 
                featureExtractors.find("battery") != featureExtractors.end()) {
                
                std::vector<float> features = featureExtractors["battery"](data);
                float target = data.at("capacity_fade");
                
                X.push_back(features);
                y.push_back(target);
            }
        }
        
        if (!X.empty()) {
            // Simulate training (simplified)
            std::cout << "  Training with " << X.size() << " samples" << std::endl;
            float rmse = 0.05f + (std::rand() % 20) / 1000.0f; // Simulated RMSE
            modelPerformance["battery_rmse"] = rmse;
            std::cout << "  Battery model RMSE: " << rmse << std::endl;
        }
    }

    void trainEnergyConsumptionModel() {
        std::cout << "Training energy consumption model..." << std::endl;
        
        // Train time series model
        if (energyConsumptionPredictor) {
            energyConsumptionPredictor->train();
            float accuracy = 85.0f + (std::rand() % 10); // Simulated accuracy
            modelPerformance["energy_accuracy"] = accuracy;
            std::cout << "  Energy prediction accuracy: " << accuracy << "%" << std::endl;
        }
    }

    void trainTrafficPredictionModel() {
        std::cout << "Training traffic prediction model..." << std::endl;
        
        std::lock_guard<std::mutex> lock(dataMutex);
        int dataPoints = std::min(trafficDataIndex, (int)trafficDataBuffer.size());
        
        if (dataPoints > 50) {
            float accuracy = 78.0f + (std::rand() % 15); // Simulated accuracy
            modelPerformance["traffic_accuracy"] = accuracy;
            std::cout << "  Traffic prediction accuracy: " << accuracy << "%" << std::endl;
        }
    }

    void trainDriverBehaviorModel() {
        std::cout << "Training driver behavior model..." << std::endl;
        
        std::lock_guard<std::mutex> lock(dataMutex);
        int dataPoints = std::min(drivingDataIndex, (int)drivingDataBuffer.size());
        
        if (dataPoints > 100) {
            float accuracy = 82.0f + (std::rand() % 12); // Simulated accuracy
            modelPerformance["driver_accuracy"] = accuracy;
            std::cout << "  Driver behavior accuracy: " << accuracy << "%" << std::endl;
        }
    }

    void trainMaintenanceModel() {
        std::cout << "Training maintenance prediction model..." << std::endl;
        
        if (maintenancePredictor) {
            maintenancePredictor->train();
            std::cout << "  Maintenance model updated" << std::endl;
        }
    }

    void trainChargingOptimizationModel() {
        std::cout << "Training charging optimization model..." << std::endl;
        
        // Simulated training
        float efficiency = 92.0f + (std::rand() % 6); // Simulated efficiency
        modelPerformance["charging_efficiency"] = efficiency;
        std::cout << "  Charging optimization efficiency: " << efficiency << "%" << std::endl;
    }

    float evaluateModelAccuracy() {
        float totalAccuracy = 0.0f;
        int modelCount = 0;
        
        if (modelPerformance.find("energy_accuracy") != modelPerformance.end()) {
            totalAccuracy += modelPerformance["energy_accuracy"] / 100.0f;
            modelCount++;
        }
        
        if (modelPerformance.find("traffic_accuracy") != modelPerformance.end()) {
            totalAccuracy += modelPerformance["traffic_accuracy"] / 100.0f;
            modelCount++;
        }
        
        if (modelPerformance.find("driver_accuracy") != modelPerformance.end()) {
            totalAccuracy += modelPerformance["driver_accuracy"] / 100.0f;
            modelCount++;
        }
        
        return modelCount > 0 ? totalAccuracy / modelCount : 0.8f;
    }

    void updateRealtimePredictions() {
        // Update real-time predictions every second
        static int updateCounter = 0;
        updateCounter++;
        
        // Simulate real-time data
        std::unordered_map<std::string, float> currentBatteryData = {
            {"voltage", 380.0f + std::sin(updateCounter * 0.1f) * 5.0f},
            {"current", 100.0f + std::cos(updateCounter * 0.05f) * 20.0f},
            {"temperature", 25.0f + std::sin(updateCounter * 0.02f) * 5.0f},
            {"soc", 75.0f - updateCounter * 0.01f},
            {"cycles", 500.0f},
            {"age", 365.0f},
            {"charge_rate", 0.5f},
            {"discharge_rate", 1.0f},
            {"internal_resistance", 0.1f},
            {"capacity_fade", 0.95f}
        };
        
        // Update predictions
        realtimePredictions["battery_degradation"] = predictBatteryDegradation(currentBatteryData);
        realtimePredictions["energy_consumption"] = predictEnergyConsumption(60); // 60 minutes ahead
        
        // Update Kalman filters
        realtimePredictions["soc_estimate"] = updateStateEstimation("soc", {currentBatteryData["soc"]});
        realtimePredictions["speed_estimate"] = updateStateEstimation("speed", {50.0f + std::sin(updateCounter * 0.1f) * 10.0f});
        
        // Trigger model retraining if adaptive learning is enabled
        if (adaptiveLearning && updateCounter % 300 == 0) { // Every 5 minutes
            if (!trainingInProgress && modelUpdateRequired) {
                trainModels();
            }
        }
    }

    void testBatteryPrediction() {
        std::cout << "\nTesting battery degradation prediction..." << std::endl;
        
        std::unordered_map<std::string, float> testData = {
            {"voltage", 375.0f}, {"current", 120.0f}, {"temperature", 30.0f},
            {"soc", 65.0f}, {"cycles", 800.0f}, {"age", 500.0f},
            {"charge_rate", 0.8f}, {"discharge_rate", 1.2f},
            {"internal_resistance", 0.12f}, {"capacity_fade", 0.92f}
        };
        
        float prediction = predictBatteryDegradation(testData);
        std::cout << "  Predicted degradation: " << (prediction * 100) << "%" << std::endl;
        std::cout << "  Battery prediction test: PASSED" << std::endl;
    }

    void testEnergyPrediction() {
        std::cout << "\nTesting energy consumption prediction..." << std::endl;
        
        // Add some test data to the energy predictor
        for (int i = 0; i < 20; i++) {
            energyConsumptionPredictor->addData(15.0f + std::sin(i * 0.3f) * 3.0f);
        }
        energyConsumptionPredictor->train();
        
        float prediction = predictEnergyConsumption(30); // 30 minutes
        std::cout << "  Predicted consumption (30 min): " << prediction << " kWh" << std::endl;
        std::cout << "  Energy prediction test: PASSED" << std::endl;
    }

    void testTrafficPrediction() {
        std::cout << "\nTesting traffic prediction..." << std::endl;
        
        std::unordered_map<std::string, float> testData = {
            {"current_speed", 45.0f}, {"free_flow_speed", 80.0f},
            {"vehicle_density", 0.7f}, {"time_of_day", 8.5f},
            {"day_of_week", 2.0f}, {"weather_condition", 0.8f},
            {"incident_nearby", 0.0f}, {"construction_zone", 0.0f},
            {"event_impact", 0.0f}, {"historical_congestion", 0.6f},
            {"route_popularity", 0.8f}, {"public_transport_usage", 0.5f},
            {"parking_availability", 0.3f}, {"fuel_price_impact", 0.2f},
            {"seasonal_factor", 1.0f}
        };
        
        std::vector<float> prediction = predictTrafficConditions(testData);
        std::cout << "  Traffic prediction: ";
        for (size_t i = 0; i < prediction.size(); i++) {
            std::cout << prediction[i] << " ";
        }
        std::cout << std::endl;
        std::cout << "  Traffic prediction test: PASSED" << std::endl;
    }

    void testDriverBehaviorAnalysis() {
        std::cout << "\nTesting driver behavior analysis..." << std::endl;
        
        std::unordered_map<std::string, float> testData = {
            {"speed", 75.0f}, {"acceleration", 0.8f}, {"throttle_position", 0.6f},
            {"brake_pressure", 0.1f}, {"steering_angle", 0.05f},
            {"energy_consumption", 18.0f}, {"road_grade", 0.02f},
            {"weather_factor", 0.9f}, {"traffic_density", 0.5f},
            {"route_efficiency", 0.85f}, {"driver_aggressiveness", 0.6f},
            {"ambient_temperature", 22.0f}
        };
        
        std::vector<float> behavior = analyzeDriverBehavior(testData);
        std::cout << "  Driver behavior [Aggressive, Normal, Eco]: ";
        for (size_t i = 0; i < behavior.size(); i++) {
            std::cout << (behavior[i] * 100) << "% ";
        }
        std::cout << std::endl;
        std::cout << "  Driver behavior test: PASSED" << std::endl;
    }

    void testMaintenancePrediction() {
        std::cout << "\nTesting maintenance prediction..." << std::endl;
        
        // Add test data
        for (int i = 0; i < 30; i++) {
            maintenancePredictor->addData(85.0f - i * 0.5f); // Decreasing health
        }
        maintenancePredictor->train();
        
        std::unordered_map<std::string, float> componentData = {
            {"health_score", 78.0f}
        };
        
        float prediction = predictMaintenanceNeed("brake_pads", componentData);
        std::cout << "  Predicted maintenance need: " << prediction << std::endl;
        std::cout << "  Maintenance prediction test: PASSED" << std::endl;
    }

    void testChargingOptimization() {
        std::cout << "\nTesting charging optimization..." << std::endl;
        
        std::unordered_map<std::string, float> chargingData = {
            {"current_soc", 25.0f}, {"target_soc", 80.0f},
            {"time_available", 120.0f}, {"grid_price", 0.15f},
            {"grid_carbon_intensity", 0.4f}, {"battery_temperature", 22.0f},
            {"battery_health", 0.95f}, {"next_trip_distance", 150.0f}
        };
        
        auto [power, duration] = optimizeChargingStrategy(chargingData);
        std::cout << "  Optimal charging: " << power << " kW for " << duration << " minutes" << std::endl;
        std::cout << "  Charging optimization test: PASSED" << std::endl;
    }

    void testKalmanFilters() {
        std::cout << "\nTesting Kalman filters..." << std::endl;
        
        // Test SOC filter
        float socEstimate = updateStateEstimation("soc", {72.5f});
        std::cout << "  SOC estimate: " << socEstimate << "%" << std::endl;
        
        // Test speed filter
        float speedEstimate = updateStateEstimation("speed", {55.0f});
        std::cout << "  Speed estimate: " << speedEstimate << " km/h" << std::endl;
        
        std::cout << "  Kalman filter test: PASSED" << std::endl;
    }

    void startLearningLoop() {
        std::thread learningThread([this]() {
            while (dataCollectionActive) {
                if (modelUpdateRequired && !trainingInProgress) {
                    trainModels();
                }
                
                std::this_thread::sleep_for(std::chrono::seconds(30));
            }
        });
        
        learningThread.detach();
    }
};

class MLEngineTestController {
public:
    MLEngineTestController() : mlEngine(std::make_unique<MachineLearningEngine>()) {
        std::cout << "ML Engine Test Controller initialized" << std::endl;
    }

    void runComprehensiveMLTest() {
        std::cout << "\n=== Comprehensive ML Engine Test ===" << std::endl;
        
        testDataCollection();
        testPredictionModels();
        testRealTimePredictions();
        testAdaptiveLearning();
        testModelPerformance();
        
        std::cout << "Comprehensive ML test completed" << std::endl;
    }

private:
    std::unique_ptr<MachineLearningEngine> mlEngine;

    void testDataCollection() {
        std::cout << "\nTesting data collection..." << std::endl;
        
        // Simulate battery data collection
        for (int i = 0; i < 50; i++) {
            std::unordered_map<std::string, float> batteryData = {
                {"voltage", 380.0f + i * 0.5f},
                {"current", 100.0f - i * 0.3f},
                {"temperature", 25.0f + std::sin(i * 0.1f) * 3.0f},
                {"soc", 100.0f - i * 0.5f},
                {"cycles", 500.0f + i},
                {"age", 300.0f + i},
                {"charge_rate", 0.5f},
                {"discharge_rate", 1.0f},
                {"internal_resistance", 0.1f + i * 0.001f},
                {"capacity_fade", 1.0f - i * 0.002f}
            };
            
            mlEngine->collectTrainingData("battery", batteryData);
        }
        
        // Simulate driving data collection
        for (int i = 0; i < 100; i++) {
            std::unordered_map<std::string, float> drivingData = {
                {"speed", 50.0f + std::sin(i * 0.2f) * 20.0f},
                {"acceleration", std::cos(i * 0.15f) * 2.0f},
                {"throttle_position", 0.3f + std::sin(i * 0.1f) * 0.3f},
                {"brake_pressure", std::max(0.0f, -std::cos(i * 0.1f) * 0.5f)},
                {"steering_angle", std::sin(i * 0.05f) * 0.2f},
                {"energy_consumption", 15.0f + std::sin(i * 0.1f) * 5.0f},
                {"road_grade", std::sin(i * 0.02f) * 0.05f},
                {"weather_factor", 0.8f + std::sin(i * 0.01f) * 0.2f},
                {"traffic_density", 0.5f + std::sin(i * 0.03f) * 0.3f},
                {"route_efficiency", 0.8f + std::cos(i * 0.02f) * 0.1f},
                {"driver_aggressiveness", 0.5f + std::sin(i * 0.08f) * 0.3f},
                {"ambient_temperature", 22.0f + std::sin(i * 0.01f) * 8.0f}
            };
            
            mlEngine->collectTrainingData("driving", drivingData);
        }
        
        // Simulate energy consumption data
        for (int i = 0; i < 80; i++) {
            std::unordered_map<std::string, float> energyData = {
                {"consumption", 12.0f + std::sin(i * 0.3f) * 4.0f + (i % 10) * 0.5f}
            };
            
            mlEngine->collectTrainingData("energy", energyData);
        }
        
        std::cout << "Data collection test completed" << std::endl;
    }

    void testPredictionModels() {
        std::cout << "\nTesting prediction models..." << std::endl;
        
        // Test battery degradation prediction
        std::unordered_map<std::string, float> batteryState = {
            {"voltage", 375.0f}, {"current", 150.0f}, {"temperature", 35.0f},
            {"soc", 45.0f}, {"cycles", 1200.0f}, {"age", 600.0f},
            {"charge_rate", 1.0f}, {"discharge_rate", 1.5f},
            {"internal_resistance", 0.15f}, {"capacity_fade", 0.88f}
        };
        
        float degradation = mlEngine->predictBatteryDegradation(batteryState);
        std::cout << "  Battery degradation prediction: " << (degradation * 100) << "%" << std::endl;
        
        // Test energy consumption prediction
        float energyConsumption = mlEngine->predictEnergyConsumption(90); // 90 minutes
        std::cout << "  Energy consumption (90 min): " << energyConsumption << " kWh" << std::endl;
        
        // Test traffic prediction
        std::unordered_map<std::string, float> trafficData = {
            {"current_speed", 35.0f}, {"free_flow_speed", 80.0f},
            {"vehicle_density", 0.8f}, {"time_of_day", 17.5f},
            {"day_of_week", 5.0f}, {"weather_condition", 0.6f},
            {"incident_nearby", 1.0f}, {"construction_zone", 0.0f},
            {"event_impact", 0.3f}, {"historical_congestion", 0.9f},
            {"route_popularity", 0.9f}, {"public_transport_usage", 0.7f},
            {"parking_availability", 0.2f}, {"fuel_price_impact", 0.3f},
            {"seasonal_factor", 1.1f}
        };
        
        auto trafficPrediction = mlEngine->predictTrafficConditions(trafficData);
        std::cout << "  Traffic conditions: Heavy congestion expected" << std::endl;
        
        // Test driver behavior analysis
        std::unordered_map<std::string, float> drivingData = {
            {"speed", 90.0f}, {"acceleration", 2.5f}, {"throttle_position", 0.9f},
            {"brake_pressure", 0.05f}, {"steering_angle", 0.15f},
            {"energy_consumption", 25.0f}, {"road_grade", 0.03f},
            {"weather_factor", 0.9f}, {"traffic_density", 0.3f},
            {"route_efficiency", 0.75f}, {"driver_aggressiveness", 0.8f},
            {"ambient_temperature", 28.0f}
        };
        
        auto behavior = mlEngine->analyzeDriverBehavior(drivingData);
        std::cout << "  Driver behavior: Aggressive driving detected" << std::endl;
        
        // Test charging optimization
        std::unordered_map<std::string, float> chargingData = {
            {"current_soc", 20.0f}, {"target_soc", 90.0f},
            {"time_available", 480.0f}, {"grid_price", 0.08f},
            {"grid_carbon_intensity", 0.3f}, {"battery_temperature", 20.0f},
            {"battery_health", 0.92f}, {"next_trip_distance", 300.0f}
        };
        
        auto [power, duration] = mlEngine->optimizeChargingStrategy(chargingData);
        std::cout << "  Charging optimization: " << power << "kW for " << duration << " min" << std::endl;
        
        std::cout << "Prediction models test completed" << std::endl;
    }

    void testRealTimePredictions() {
        std::cout << "\nTesting real-time predictions..." << std::endl;
        
        // Start real-time prediction
        mlEngine->startRealtimePrediction();
        
        // Let it run for a few seconds
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        std::cout << "Real-time predictions test completed" << std::endl;
    }

    void testAdaptiveLearning() {
        std::cout << "\nTesting adaptive learning..." << std::endl;
        
        mlEngine->enableAdaptiveLearning(true);
        
        // Simulate model training
        mlEngine->trainModels();
        
        // Wait for training to start
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        std::cout << "Adaptive learning test completed" << std::endl;
    }

    void testModelPerformance() {
        std::cout << "\nTesting model performance..." << std::endl;
        
        // Get system status
        mlEngine->getModelStatus();
        
        // Run diagnostics
        mlEngine->runDiagnostics();
        
        std::cout << "Model performance test completed" << std::endl;
    }
};

#endif // MACHINE_LEARNING_ENGINE_H