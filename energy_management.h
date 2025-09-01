/**
 * Advanced Energy Management System for Electric Vehicles
 * Author: adzetto
 * Features: Smart charging, route optimization, energy prediction, grid integration
 */

#ifndef ENERGY_MANAGEMENT_H
#define ENERGY_MANAGEMENT_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <string>
#include <chrono>
#include <thread>
#include <ctime>
#include <memory>
#include <algorithm>
#include <cmath>
#include <queue>
#include <numeric>

enum class DrivingMode {
    ECO,
    COMFORT,
    SPORT,
    CUSTOM,
    WINTER,
    TOWING
};

enum class ChargingStrategy {
    IMMEDIATE,
    SCHEDULED,
    SMART_GRID,
    SOLAR_OPTIMIZED,
    COST_OPTIMIZED,
    FAST_CHARGE_ONLY
};

enum class EnergySource {
    BATTERY_MAIN,
    BATTERY_AUX,
    SOLAR_PANEL,
    REGENERATIVE_BRAKING,
    GRID_CHARGING,
    WIRELESS_CHARGING
};

enum class PowerDistributionMode {
    PERFORMANCE,
    EFFICIENCY,
    BALANCED,
    EMERGENCY
};

struct EnergyConsumer {
    std::string name;
    double powerRating;        // Maximum power consumption in kW
    double currentPower;       // Current power consumption in kW
    double efficiency;         // Efficiency factor (0-1)
    int priority;              // Priority level (0-10, 10 being highest)
    bool isEssential;          // Cannot be turned off
    bool isActive;             // Currently active
    double totalEnergyUsed;    // Cumulative energy usage in kWh
    
    EnergyConsumer(const std::string& n, double rating, int prio = 5) 
        : name(n), powerRating(rating), currentPower(0.0), efficiency(0.9),
          priority(prio), isEssential(false), isActive(false), totalEnergyUsed(0.0) {}
};

struct RouteSegment {
    double distance;           // km
    double elevation;          // m
    double speedLimit;         // km/h
    double trafficFactor;      // 0.5-1.5 (traffic density multiplier)
    std::string roadType;      // "highway", "city", "rural"
    double weatherFactor;      // Energy consumption multiplier
    
    RouteSegment() : distance(0), elevation(0), speedLimit(50), trafficFactor(1.0),
                    roadType("city"), weatherFactor(1.0) {}
};

struct ChargingEvent {
    std::chrono::steady_clock::time_point startTime;
    std::chrono::steady_clock::time_point endTime;
    double energyDelivered;    // kWh
    double peakPower;          // kW
    double averagePower;       // kW
    double cost;               // Currency units
    std::string stationType;   // "AC_L2", "DC_Fast", "Supercharger"
    double efficiency;         // Charging efficiency
    
    ChargingEvent() : energyDelivered(0), peakPower(0), averagePower(0),
                     cost(0), stationType("AC_L2"), efficiency(0.9) {
        startTime = std::chrono::steady_clock::now();
        endTime = startTime;
    }
};

struct EnergyPrediction {
    double remainingRange;     // km
    double energyToDestination;// kWh
    double arrivalSOC;         // %
    std::vector<double> consumptionProfile; // kW per segment
    double confidence;         // 0-1
    std::string warnings;      // Energy warnings
    
    EnergyPrediction() : remainingRange(0), energyToDestination(0),
                        arrivalSOC(0), confidence(0.8) {}
};

class EnergyModel {
private:
    // Vehicle specifications
    double vehicleMass;        // kg
    double dragCoefficient;    // Cd
    double frontalArea;        // m²
    double rollingResistance;  // Coefficient
    double motorEfficiency;    // Overall drivetrain efficiency
    double regenEfficiency;    // Regenerative braking efficiency
    
    // Environmental factors
    double airDensity;         // kg/m³
    double gravity;            // m/s²
    
public:
    EnergyModel(double mass = 1800.0, double cd = 0.28, double area = 2.3) 
        : vehicleMass(mass), dragCoefficient(cd), frontalArea(area),
          rollingResistance(0.008), motorEfficiency(0.92), regenEfficiency(0.85),
          airDensity(1.225), gravity(9.81) {}
    
    double calculateEnergyConsumption(double distance, double speed, double elevation,
                                     double trafficFactor = 1.0, double weatherFactor = 1.0) const {
        // Rolling resistance force
        double rollForce = rollingResistance * vehicleMass * gravity * cos(atan(elevation / distance));
        
        // Aerodynamic drag force
        double dragForce = 0.5 * airDensity * dragCoefficient * frontalArea * pow(speed / 3.6, 2);
        
        // Gravitational force (hill climbing)
        double gravForce = vehicleMass * gravity * sin(atan(elevation / distance));
        
        // Total force
        double totalForce = rollForce + dragForce + gravForce;
        
        // Power required (including inefficiencies)
        double powerRequired = totalForce * (speed / 3.6) / motorEfficiency;
        
        // Apply traffic and weather factors
        powerRequired *= trafficFactor * weatherFactor;
        
        // Energy consumption in kWh
        double time = distance / speed; // hours
        return std::max(0.0, powerRequired * time / 1000.0); // Convert W to kW
    }
    
    double calculateRegenerativeEnergy(double distance, double speed, double elevation,
                                      double brakingIntensity = 0.3) const {
        if (elevation >= 0) return 0.0; // No regen when going uphill
        
        // Available gravitational energy
        double gravEnergy = vehicleMass * gravity * abs(elevation) / 3600000.0; // Convert to kWh
        
        // Regenerative efficiency and braking intensity
        return gravEnergy * regenEfficiency * brakingIntensity;
    }
    
    void setVehicleParameters(double mass, double cd, double area) {
        vehicleMass = mass;
        dragCoefficient = cd;
        frontalArea = area;
    }
    
    void setEfficiencyParameters(double motorEff, double regenEff) {
        motorEfficiency = motorEff;
        regenEfficiency = regenEff;
    }
};

class PowerDistributor {
private:
    std::vector<std::unique_ptr<EnergyConsumer>> consumers;
    double totalAvailablePower;
    double currentPowerDemand;
    PowerDistributionMode mode;
    std::unordered_map<std::string, double> priorityWeights;
    
public:
    PowerDistributor(double maxPower = 150.0) : totalAvailablePower(maxPower),
                                               currentPowerDemand(0.0),
                                               mode(PowerDistributionMode::BALANCED) {
        initializeConsumers();
        setupPriorityWeights();
    }
    
    void addConsumer(const std::string& name, double rating, int priority = 5) {
        consumers.push_back(std::make_unique<EnergyConsumer>(name, rating, priority));
    }
    
    void updatePowerDemand(const std::string& consumerName, double requestedPower) {
        for (auto& consumer : consumers) {
            if (consumer->name == consumerName) {
                consumer->currentPower = std::min(requestedPower, consumer->powerRating);
                break;
            }
        }
        distributePower();
    }
    
    void setDistributionMode(PowerDistributionMode newMode) {
        mode = newMode;
        distributePower();
        std::cout << "Power distribution mode set to: " << getModeString(newMode) << "\n";
    }
    
    void distributePower() {
        // Calculate total demand
        currentPowerDemand = 0.0;
        for (const auto& consumer : consumers) {
            currentPowerDemand += consumer->currentPower;
        }
        
        // If demand exceeds supply, apply load shedding
        if (currentPowerDemand > totalAvailablePower) {
            applyLoadShedding();
        }
        
        // Update energy consumption
        updateEnergyConsumption();
    }
    
    void applyLoadShedding() {
        std::cout << "Power demand (" << currentPowerDemand 
                  << " kW) exceeds available power (" << totalAvailablePower 
                  << " kW). Applying load shedding.\n";
        
        // Sort consumers by priority (descending) and essential status
        std::vector<EnergyConsumer*> sortedConsumers;
        for (auto& consumer : consumers) {
            if (consumer->isActive) {
                sortedConsumers.push_back(consumer.get());
            }
        }
        
        std::sort(sortedConsumers.begin(), sortedConsumers.end(),
                 [](const EnergyConsumer* a, const EnergyConsumer* b) {
                     if (a->isEssential != b->isEssential) return a->isEssential;
                     return a->priority > b->priority;
                 });
        
        double remainingPower = totalAvailablePower;
        
        for (auto* consumer : sortedConsumers) {
            if (consumer->isEssential || remainingPower >= consumer->currentPower) {
                remainingPower -= consumer->currentPower;
            } else {
                // Reduce power for non-essential consumers
                double reductionFactor = getReductionFactor(consumer->priority);
                double reducedPower = consumer->currentPower * reductionFactor;
                
                if (remainingPower >= reducedPower) {
                    consumer->currentPower = reducedPower;
                    remainingPower -= reducedPower;
                    std::cout << "Reduced power for " << consumer->name 
                              << " to " << reducedPower << " kW\n";
                } else {
                    consumer->currentPower = 0.0;
                    consumer->isActive = false;
                    std::cout << "Temporarily disabled " << consumer->name << "\n";
                }
            }
        }
    }
    
    void displayPowerDistribution() const {
        std::cout << "\n=== Power Distribution Status ===\n";
        std::cout << "Mode: " << getModeString(mode) << "\n";
        std::cout << "Available Power: " << totalAvailablePower << " kW\n";
        std::cout << "Current Demand: " << currentPowerDemand << " kW\n";
        std::cout << "Utilization: " << (currentPowerDemand / totalAvailablePower * 100.0) << "%\n\n";
        
        std::cout << "Consumer Status:\n";
        for (const auto& consumer : consumers) {
            std::cout << "  " << consumer->name << ": " << consumer->currentPower 
                      << "/" << consumer->powerRating << " kW";
            if (!consumer->isActive) std::cout << " [DISABLED]";
            if (consumer->isEssential) std::cout << " [ESSENTIAL]";
            std::cout << " (Priority: " << consumer->priority << ")\n";
        }
        std::cout << "================================\n\n";
    }
    
    double getTotalPowerConsumption() const { return currentPowerDemand; }
    double getAvailablePower() const { return totalAvailablePower - currentPowerDemand; }
    
    void setTotalAvailablePower(double power) { totalAvailablePower = power; }
    
private:
    void initializeConsumers() {
        // Propulsion system
        addConsumer("Motor Drive", 120.0, 10);
        
        // Essential systems
        addConsumer("Vehicle Control Unit", 0.5, 10);
        addConsumer("Safety Systems", 1.0, 10);
        addConsumer("Lights", 0.3, 9);
        
        // Comfort systems
        addConsumer("HVAC", 5.0, 7);
        addConsumer("Infotainment", 0.8, 5);
        addConsumer("Seat Heating", 1.2, 4);
        addConsumer("Audio System", 0.4, 3);
        
        // Auxiliary systems
        addConsumer("12V Systems", 2.0, 8);
        addConsumer("Charging Port", 0.2, 6);
        
        // Mark essential consumers
        for (auto& consumer : consumers) {
            if (consumer->name == "Vehicle Control Unit" || 
                consumer->name == "Safety Systems" ||
                consumer->name == "Motor Drive" ||
                consumer->name == "Lights") {
                consumer->isEssential = true;
            }
        }
    }
    
    void setupPriorityWeights() {
        priorityWeights[getModeString(PowerDistributionMode::PERFORMANCE)] = 1.0;
        priorityWeights[getModeString(PowerDistributionMode::EFFICIENCY)] = 0.8;
        priorityWeights[getModeString(PowerDistributionMode::BALANCED)] = 0.9;
        priorityWeights[getModeString(PowerDistributionMode::EMERGENCY)] = 0.5;
    }
    
    double getReductionFactor(int priority) const {
        switch (mode) {
            case PowerDistributionMode::EFFICIENCY:
                return priority >= 7 ? 0.9 : 0.6;
            case PowerDistributionMode::PERFORMANCE:
                return priority >= 5 ? 0.95 : 0.8;
            case PowerDistributionMode::EMERGENCY:
                return priority >= 8 ? 0.8 : 0.3;
            default:
                return priority >= 6 ? 0.85 : 0.7;
        }
    }
    
    void updateEnergyConsumption() {
        static auto lastUpdate = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastUpdate);
        
        if (duration.count() > 0) {
            double hours = duration.count() / 3600000.0;
            for (auto& consumer : consumers) {
                if (consumer->isActive) {
                    consumer->totalEnergyUsed += consumer->currentPower * hours;
                }
            }
            lastUpdate = now;
        }
    }
    
    std::string getModeString(PowerDistributionMode mode) const {
        switch (mode) {
            case PowerDistributionMode::PERFORMANCE: return "PERFORMANCE";
            case PowerDistributionMode::EFFICIENCY: return "EFFICIENCY";
            case PowerDistributionMode::BALANCED: return "BALANCED";
            case PowerDistributionMode::EMERGENCY: return "EMERGENCY";
            default: return "UNKNOWN";
        }
    }
};

class RouteEnergyOptimizer {
private:
    EnergyModel energyModel;
    std::vector<RouteSegment> currentRoute;
    DrivingMode drivingMode;
    double targetArrivalSOC;
    
    // Optimization parameters
    double speedOptimizationFactor;
    double elevationWeight;
    double trafficWeight;
    
public:
    RouteEnergyOptimizer() : drivingMode(DrivingMode::ECO), targetArrivalSOC(20.0),
                            speedOptimizationFactor(0.8), elevationWeight(1.2),
                            trafficWeight(1.1) {}
    
    void setRoute(const std::vector<RouteSegment>& route) {
        currentRoute = route;
    }
    
    void setDrivingMode(DrivingMode mode) {
        drivingMode = mode;
        updateOptimizationParameters();
        std::cout << "Driving mode set to: " << getModeString(mode) << "\n";
    }
    
    EnergyPrediction predictEnergyConsumption(double currentSOC, double batteryCapacity) {
        EnergyPrediction prediction;
        
        if (currentRoute.empty()) {
            prediction.warnings = "No route data available";
            return prediction;
        }
        
        double totalEnergyConsumption = 0.0;
        double totalDistance = 0.0;
        prediction.consumptionProfile.clear();
        
        for (const auto& segment : currentRoute) {
            // Optimize speed for this segment
            double optimizedSpeed = optimizeSpeedForSegment(segment);
            
            // Calculate energy consumption for segment
            double segmentEnergy = energyModel.calculateEnergyConsumption(
                segment.distance, optimizedSpeed, segment.elevation,
                segment.trafficFactor, segment.weatherFactor
            );
            
            // Add regenerative energy if applicable
            double regenEnergy = energyModel.calculateRegenerativeEnergy(
                segment.distance, optimizedSpeed, segment.elevation
            );
            
            double netEnergy = segmentEnergy - regenEnergy;
            totalEnergyConsumption += netEnergy;
            totalDistance += segment.distance;
            
            prediction.consumptionProfile.push_back(netEnergy);
        }
        
        // Calculate predictions
        prediction.energyToDestination = totalEnergyConsumption;
        prediction.arrivalSOC = currentSOC - (totalEnergyConsumption / batteryCapacity * 100.0);
        
        // Calculate remaining range based on average consumption
        double avgConsumption = totalEnergyConsumption / totalDistance; // kWh/km
        prediction.remainingRange = (currentSOC / 100.0 * batteryCapacity) / avgConsumption;
        
        // Generate warnings
        generateEnergyWarnings(prediction, currentSOC);
        
        // Set confidence based on route complexity and data quality
        prediction.confidence = calculatePredictionConfidence();
        
        return prediction;
    }
    
    std::vector<std::string> generateEcoRoutingRecommendations(const EnergyPrediction& prediction) {
        std::vector<std::string> recommendations;
        
        if (prediction.arrivalSOC < targetArrivalSOC) {
            recommendations.push_back("Consider charging before departure or en route");
            recommendations.push_back("Reduce HVAC usage to save energy");
            recommendations.push_back("Drive in ECO mode for maximum efficiency");
        }
        
        if (prediction.energyToDestination > prediction.remainingRange * 0.8) {
            recommendations.push_back("Route is energy-intensive, plan charging stops");
        }
        
        // Analyze route segments for specific recommendations
        for (size_t i = 0; i < currentRoute.size(); i++) {
            const auto& segment = currentRoute[i];
            
            if (segment.elevation > 100) {
                recommendations.push_back("Segment " + std::to_string(i+1) + ": Steep climb ahead, pre-cool cabin");
            }
            
            if (segment.trafficFactor > 1.3) {
                recommendations.push_back("Segment " + std::to_string(i+1) + ": Heavy traffic, consider alternate route");
            }
            
            if (segment.weatherFactor > 1.2) {
                recommendations.push_back("Segment " + std::to_string(i+1) + ": Adverse weather, reduce speed for safety");
            }
        }
        
        return recommendations;
    }
    
    void displayRouteAnalysis(const EnergyPrediction& prediction) {
        std::cout << "\n=== Route Energy Analysis ===\n";
        std::cout << "Driving Mode: " << getModeString(drivingMode) << "\n";
        std::cout << "Total Distance: " << getTotalDistance() << " km\n";
        std::cout << "Energy to Destination: " << prediction.energyToDestination << " kWh\n";
        std::cout << "Predicted Arrival SOC: " << prediction.arrivalSOC << "%\n";
        std::cout << "Remaining Range: " << prediction.remainingRange << " km\n";
        std::cout << "Prediction Confidence: " << (prediction.confidence * 100.0) << "%\n";
        
        if (!prediction.warnings.empty()) {
            std::cout << "Warnings: " << prediction.warnings << "\n";
        }
        
        std::cout << "\nSegment Analysis:\n";
        for (size_t i = 0; i < currentRoute.size() && i < prediction.consumptionProfile.size(); i++) {
            const auto& segment = currentRoute[i];
            std::cout << "  Segment " << (i+1) << ": " << segment.distance << " km, "
                      << prediction.consumptionProfile[i] << " kWh (" 
                      << segment.roadType << ")\n";
        }
        
        std::cout << "============================\n\n";
    }
    
private:
    void updateOptimizationParameters() {
        switch (drivingMode) {
            case DrivingMode::ECO:
                speedOptimizationFactor = 0.75;
                elevationWeight = 1.3;
                trafficWeight = 1.2;
                break;
            case DrivingMode::COMFORT:
                speedOptimizationFactor = 0.85;
                elevationWeight = 1.1;
                trafficWeight = 1.0;
                break;
            case DrivingMode::SPORT:
                speedOptimizationFactor = 1.0;
                elevationWeight = 1.0;
                trafficWeight = 0.9;
                break;
            case DrivingMode::WINTER:
                speedOptimizationFactor = 0.8;
                elevationWeight = 1.4;
                trafficWeight = 1.3;
                break;
            default:
                speedOptimizationFactor = 0.85;
                elevationWeight = 1.1;
                trafficWeight = 1.0;
                break;
        }
    }
    
    double optimizeSpeedForSegment(const RouteSegment& segment) {
        double baseSpeed = segment.speedLimit * speedOptimizationFactor;
        
        // Adjust for elevation
        if (segment.elevation > 0) {
            baseSpeed *= (1.0 / elevationWeight);
        }
        
        // Adjust for traffic
        baseSpeed *= (1.0 / (segment.trafficFactor * trafficWeight));
        
        // Adjust for weather
        baseSpeed *= (1.0 / segment.weatherFactor);
        
        // Ensure speed is reasonable
        return std::max(30.0, std::min(baseSpeed, segment.speedLimit));
    }
    
    void generateEnergyWarnings(EnergyPrediction& prediction, double currentSOC) {
        std::vector<std::string> warnings;
        
        if (prediction.arrivalSOC < 10.0) {
            warnings.push_back("CRITICAL: Very low arrival SOC predicted");
        } else if (prediction.arrivalSOC < 20.0) {
            warnings.push_back("WARNING: Low arrival SOC predicted");
        }
        
        if (prediction.energyToDestination > currentSOC * 0.8) {
            warnings.push_back("High energy consumption route");
        }
        
        // Check for challenging segments
        for (size_t i = 0; i < currentRoute.size(); i++) {
            const auto& segment = currentRoute[i];
            if (segment.elevation > 200) {
                warnings.push_back("Steep climb in segment " + std::to_string(i+1));
            }
        }
        
        // Combine warnings into single string
        if (!warnings.empty()) {
            prediction.warnings = warnings[0];
            for (size_t i = 1; i < warnings.size(); i++) {
                prediction.warnings += "; " + warnings[i];
            }
        }
    }
    
    double calculatePredictionConfidence() {
        // Base confidence
        double confidence = 0.8;
        
        // Reduce confidence for complex routes
        int complexSegments = 0;
        for (const auto& segment : currentRoute) {
            if (abs(segment.elevation) > 100 || segment.trafficFactor > 1.2) {
                complexSegments++;
            }
        }
        
        confidence -= (complexSegments * 0.05);
        
        // Weather reduces confidence
        for (const auto& segment : currentRoute) {
            if (segment.weatherFactor > 1.1) {
                confidence -= 0.02;
            }
        }
        
        return std::max(0.5, std::min(1.0, confidence));
    }
    
    double getTotalDistance() const {
        double total = 0.0;
        for (const auto& segment : currentRoute) {
            total += segment.distance;
        }
        return total;
    }
    
    std::string getModeString(DrivingMode mode) const {
        switch (mode) {
            case DrivingMode::ECO: return "ECO";
            case DrivingMode::COMFORT: return "COMFORT";
            case DrivingMode::SPORT: return "SPORT";
            case DrivingMode::CUSTOM: return "CUSTOM";
            case DrivingMode::WINTER: return "WINTER";
            case DrivingMode::TOWING: return "TOWING";
            default: return "UNKNOWN";
        }
    }
};

class SmartChargingManager {
private:
    ChargingStrategy strategy;
    std::vector<ChargingEvent> chargingHistory;
    std::queue<ChargingEvent> scheduledSessions;
    
    // Grid integration parameters
    double gridLoadFactor;
    double electricityPrice;
    double solarAvailability;
    std::vector<double> demandForecast; // 24-hour demand forecast
    
    // Charging parameters
    double maxChargingPower;
    double preferredSOC;
    bool preconditioning;
    
public:
    SmartChargingManager() : strategy(ChargingStrategy::SMART_GRID), gridLoadFactor(1.0),
                            electricityPrice(0.15), solarAvailability(0.0),
                            maxChargingPower(50.0), preferredSOC(80.0), preconditioning(true) {
        initializeDemandForecast();
    }
    
    void setChargingStrategy(ChargingStrategy newStrategy) {
        strategy = newStrategy;
        std::cout << "Charging strategy set to: " << getStrategyString(newStrategy) << "\n";
    }
    
    ChargingEvent optimizeChargingSession(double currentSOC, double targetSOC, 
                                         double timeAvailable, double batteryCapacity) {
        ChargingEvent session;
        session.stationType = determineOptimalStationType(targetSOC - currentSOC, timeAvailable);
        
        // Calculate required energy
        double energyNeeded = (targetSOC - currentSOC) / 100.0 * batteryCapacity;
        
        // Optimize charging profile based on strategy
        switch (strategy) {
            case ChargingStrategy::IMMEDIATE:
                session = planImmediateCharging(energyNeeded, timeAvailable);
                break;
            case ChargingStrategy::SCHEDULED:
                session = planScheduledCharging(energyNeeded, timeAvailable);
                break;
            case ChargingStrategy::SMART_GRID:
                session = planSmartGridCharging(energyNeeded, timeAvailable);
                break;
            case ChargingStrategy::SOLAR_OPTIMIZED:
                session = planSolarOptimizedCharging(energyNeeded, timeAvailable);
                break;
            case ChargingStrategy::COST_OPTIMIZED:
                session = planCostOptimizedCharging(energyNeeded, timeAvailable);
                break;
            default:
                session = planImmediateCharging(energyNeeded, timeAvailable);
                break;
        }
        
        session.energyDelivered = energyNeeded;
        calculateChargingCost(session);
        
        return session;
    }
    
    void scheduleChargingSession(const ChargingEvent& session) {
        scheduledSessions.push(session);
        std::cout << "Charging session scheduled: " << session.energyDelivered 
                  << " kWh at " << session.averagePower << " kW\n";
    }
    
    void executeScheduledCharging() {
        if (scheduledSessions.empty()) {
            return;
        }
        
        ChargingEvent session = scheduledSessions.front();
        scheduledSessions.pop();
        
        std::cout << "Executing scheduled charging session...\n";
        
        // Simulate charging process
        auto startTime = std::chrono::steady_clock::now();
        session.startTime = startTime;
        
        // Charging simulation with variable power
        double totalEnergy = 0.0;
        double timeStep = 0.1; // 6-minute intervals
        std::vector<double> powerProfile = generateChargingProfile(session);
        
        for (double power : powerProfile) {
            totalEnergy += power * timeStep;
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Accelerated simulation
            
            if (totalEnergy >= session.energyDelivered) {
                break;
            }
        }
        
        session.endTime = std::chrono::steady_clock::now();
        chargingHistory.push_back(session);
        
        std::cout << "Charging session completed: " << session.energyDelivered 
                  << " kWh delivered\n";
    }
    
    void updateGridConditions(double loadFactor, double price, double solarGen) {
        gridLoadFactor = loadFactor;
        electricityPrice = price;
        solarAvailability = solarGen;
    }
    
    std::vector<std::string> getChargingRecommendations(double currentSOC, double targetSOC) {
        std::vector<std::string> recommendations;
        
        // SOC-based recommendations
        if (currentSOC < 20.0) {
            recommendations.push_back("Low battery - consider immediate charging");
        } else if (currentSOC > 80.0 && targetSOC > 90.0) {
            recommendations.push_back("Battery already high - slow charging recommended");
        }
        
        // Grid-based recommendations
        if (gridLoadFactor < 0.7) {
            recommendations.push_back("Grid load is low - good time for fast charging");
        } else if (gridLoadFactor > 1.3) {
            recommendations.push_back("High grid demand - consider delaying non-urgent charging");
        }
        
        // Price-based recommendations
        if (electricityPrice < 0.10) {
            recommendations.push_back("Electricity price is low - optimal charging window");
        } else if (electricityPrice > 0.25) {
            recommendations.push_back("High electricity price - consider waiting or using solar");
        }
        
        // Solar recommendations
        if (solarAvailability > 0.7) {
            recommendations.push_back("High solar generation - excellent time for eco-friendly charging");
        }
        
        return recommendations;
    }
    
    void displayChargingStatus() {
        std::cout << "\n=== Smart Charging Manager Status ===\n";
        std::cout << "Strategy: " << getStrategyString(strategy) << "\n";
        std::cout << "Scheduled Sessions: " << scheduledSessions.size() << "\n";
        std::cout << "Charging History: " << chargingHistory.size() << " sessions\n";
        std::cout << "Grid Load Factor: " << gridLoadFactor << "\n";
        std::cout << "Electricity Price: $" << electricityPrice << "/kWh\n";
        std::cout << "Solar Availability: " << (solarAvailability * 100.0) << "%\n";
        std::cout << "Max Charging Power: " << maxChargingPower << " kW\n";
        std::cout << "Preferred SOC: " << preferredSOC << "%\n";
        
        if (!chargingHistory.empty()) {
            double totalEnergy = 0.0;
            double totalCost = 0.0;
            for (const auto& event : chargingHistory) {
                totalEnergy += event.energyDelivered;
                totalCost += event.cost;
            }
            std::cout << "Total Energy Charged: " << totalEnergy << " kWh\n";
            std::cout << "Total Charging Cost: $" << totalCost << "\n";
            std::cout << "Average Cost per kWh: $" << (totalCost / totalEnergy) << "\n";
        }
        
        std::cout << "====================================\n\n";
    }
    
private:
    void initializeDemandForecast() {
        // Generate 24-hour demand forecast (normalized 0-1)
        demandForecast.resize(24);
        
        // Typical daily pattern: low at night, peaks in evening
        for (int hour = 0; hour < 24; hour++) {
            // Define PI locally to avoid platform-specific M_PI issues
            constexpr double PI = 3.14159265358979323846;
            if (hour >= 0 && hour < 6) {
                demandForecast[hour] = 0.3 + 0.1 * sin(hour * PI / 12.0);
            } else if (hour >= 6 && hour < 18) {
                demandForecast[hour] = 0.6 + 0.2 * sin((hour - 6) * PI / 12.0);
            } else {
                demandForecast[hour] = 0.8 + 0.2 * sin((hour - 18) * PI / 6.0);
            }
        }
    }
    
    std::string determineOptimalStationType(double energyNeeded, double timeAvailable) {
        double requiredPower = energyNeeded / timeAvailable;
        
        if (requiredPower > 100.0) {
            return "Supercharger";
        } else if (requiredPower > 20.0) {
            return "DC_Fast";
        } else {
            return "AC_L2";
        }
    }
    
    ChargingEvent planImmediateCharging(double energyNeeded, double timeAvailable) {
        ChargingEvent event;
        event.averagePower = std::min(maxChargingPower, energyNeeded / timeAvailable);
        event.peakPower = event.averagePower;
        event.efficiency = 0.92; // High efficiency for immediate charging
        return event;
    }
    
    ChargingEvent planScheduledCharging(double energyNeeded, double timeAvailable) {
        ChargingEvent event;
        
        // Find optimal time window based on grid demand
        int optimalHour = 0;
        double minDemand = demandForecast[0];
        for (int i = 1; i < 24; i++) {
            if (demandForecast[i] < minDemand) {
                minDemand = demandForecast[i];
                optimalHour = i;
            }
        }
        
        event.averagePower = energyNeeded / timeAvailable;
        event.peakPower = event.averagePower;
        event.efficiency = 0.90;
        
        std::cout << "Optimal charging window: " << optimalHour << ":00 - " 
                  << (optimalHour + 1) << ":00\n";
        
        return event;
    }
    
    ChargingEvent planSmartGridCharging(double energyNeeded, double timeAvailable) {
        ChargingEvent event;
        
        // Adjust power based on grid conditions
        double gridAdjustment = 1.0 / gridLoadFactor;
        event.averagePower = std::min(maxChargingPower * gridAdjustment, 
                                     energyNeeded / timeAvailable);
        event.peakPower = event.averagePower * 1.2;
        event.efficiency = 0.88; // Slightly lower due to grid optimization
        
        return event;
    }
    
    ChargingEvent planSolarOptimizedCharging(double energyNeeded, double timeAvailable) {
        ChargingEvent event;
        
        // Optimize for solar availability
        event.averagePower = maxChargingPower * solarAvailability;
        if (event.averagePower < energyNeeded / timeAvailable) {
            // Supplement with grid power
            double gridPower = (energyNeeded / timeAvailable) - event.averagePower;
            event.averagePower += gridPower * 0.5; // Reduce grid usage
        }
        
        event.peakPower = event.averagePower;
        event.efficiency = 0.94; // High efficiency for solar charging
        
        return event;
    }
    
    ChargingEvent planCostOptimizedCharging(double energyNeeded, double timeAvailable) {
        ChargingEvent event;
        
        // Optimize for lowest cost periods
        double priceAdjustment = 0.20 / electricityPrice; // Normalize to $0.20/kWh
        event.averagePower = std::min(maxChargingPower * priceAdjustment,
                                     energyNeeded / timeAvailable);
        event.peakPower = event.averagePower;
        event.efficiency = 0.89;
        
        return event;
    }
    
    void calculateChargingCost(ChargingEvent& event) {
        double baseCost = event.energyDelivered * electricityPrice;
        
        // Apply demand charges for high power charging
        if (event.peakPower > 50.0) {
            baseCost += (event.peakPower - 50.0) * 0.05; // Demand charge
        }
        
        // Apply time-of-use multipliers
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto* tm = std::localtime(&time_t);
        
        if (tm->tm_hour >= 17 && tm->tm_hour <= 21) {
            baseCost *= 1.5; // Peak hours
        } else if (tm->tm_hour >= 22 || tm->tm_hour <= 6) {
            baseCost *= 0.7; // Off-peak hours
        }
        
        event.cost = baseCost;
    }
    
    std::vector<double> generateChargingProfile(const ChargingEvent& event) {
        std::vector<double> profile;
        double duration = event.energyDelivered / event.averagePower; // hours
        int steps = static_cast<int>(duration / 0.1); // 6-minute steps
        
        for (int i = 0; i < steps; i++) {
            double progress = static_cast<double>(i) / steps;
            
            // Typical charging curve: high power initially, tapering as SOC increases
            double power = event.peakPower * (1.0 - 0.3 * progress * progress);
            profile.push_back(power);
        }
        
        return profile;
    }
    
    std::string getStrategyString(ChargingStrategy strategy) const {
        switch (strategy) {
            case ChargingStrategy::IMMEDIATE: return "IMMEDIATE";
            case ChargingStrategy::SCHEDULED: return "SCHEDULED";
            case ChargingStrategy::SMART_GRID: return "SMART_GRID";
            case ChargingStrategy::SOLAR_OPTIMIZED: return "SOLAR_OPTIMIZED";
            case ChargingStrategy::COST_OPTIMIZED: return "COST_OPTIMIZED";
            case ChargingStrategy::FAST_CHARGE_ONLY: return "FAST_CHARGE_ONLY";
            default: return "UNKNOWN";
        }
    }
};

class EnergyManagementSystem {
private:
    std::unique_ptr<PowerDistributor> powerDistributor;
    std::unique_ptr<RouteEnergyOptimizer> routeOptimizer;
    std::unique_ptr<SmartChargingManager> chargingManager;
    
    // System state
    double currentSOC;
    double batteryCapacity;
    double currentPower;
    DrivingMode currentDrivingMode;
    
    // Energy statistics
    double totalEnergyConsumed;
    double totalEnergyRegenerated;
    double totalChargingCost;
    double averageEfficiency;
    
public:
    EnergyManagementSystem(double batteryCapacity = 75.0) 
        : currentSOC(80.0), batteryCapacity(batteryCapacity), currentPower(0.0),
          currentDrivingMode(DrivingMode::ECO), totalEnergyConsumed(0.0),
          totalEnergyRegenerated(0.0), totalChargingCost(0.0), averageEfficiency(92.0) {
        
        powerDistributor = std::make_unique<PowerDistributor>();
        routeOptimizer = std::make_unique<RouteEnergyOptimizer>();
        chargingManager = std::make_unique<SmartChargingManager>();
        
        std::cout << "Energy Management System initialized\n";
        std::cout << "Battery Capacity: " << batteryCapacity << " kWh\n";
        std::cout << "Initial SOC: " << currentSOC << "%\n";
    }
    
    void setDrivingMode(DrivingMode mode) {
        currentDrivingMode = mode;
        routeOptimizer->setDrivingMode(mode);
        
        // Adjust power distribution based on driving mode
        switch (mode) {
            case DrivingMode::ECO:
                powerDistributor->setDistributionMode(PowerDistributionMode::EFFICIENCY);
                break;
            case DrivingMode::SPORT:
                powerDistributor->setDistributionMode(PowerDistributionMode::PERFORMANCE);
                break;
            default:
                powerDistributor->setDistributionMode(PowerDistributionMode::BALANCED);
                break;
        }
    }
    
    void updateEnergyConsumption(double powerDemand, double regenPower = 0.0) {
        currentPower = powerDemand;
        
        // Update power distributor
        powerDistributor->updatePowerDemand("Motor Drive", powerDemand);
        
        // Update energy statistics
        static auto lastUpdate = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastUpdate);
        
        if (duration.count() > 0) {
            double hours = duration.count() / 3600000.0;
            double energyConsumed = powerDemand * hours;
            double energyRegen = regenPower * hours;
            
            totalEnergyConsumed += energyConsumed;
            totalEnergyRegenerated += energyRegen;
            
            // Update SOC
            double netEnergyChange = energyRegen - energyConsumed;
            currentSOC += (netEnergyChange / batteryCapacity) * 100.0;
            currentSOC = std::max(0.0, std::min(100.0, currentSOC));
            
            lastUpdate = now;
        }
    }
    
    EnergyPrediction planRoute(const std::vector<RouteSegment>& route) {
        routeOptimizer->setRoute(route);
        EnergyPrediction prediction = routeOptimizer->predictEnergyConsumption(currentSOC, batteryCapacity);
        
        routeOptimizer->displayRouteAnalysis(prediction);
        
        // Generate recommendations
        auto recommendations = routeOptimizer->generateEcoRoutingRecommendations(prediction);
        std::cout << "\nRoute Recommendations:\n";
        for (const auto& rec : recommendations) {
            std::cout << "  • " << rec << "\n";
        }
        
        return prediction;
    }
    
    void planChargingSession(double targetSOC, double timeAvailable) {
        auto session = chargingManager->optimizeChargingSession(currentSOC, targetSOC, 
                                                               timeAvailable, batteryCapacity);
        chargingManager->scheduleChargingSession(session);
        
        // Display charging recommendations
        auto recommendations = chargingManager->getChargingRecommendations(currentSOC, targetSOC);
        std::cout << "\nCharging Recommendations:\n";
        for (const auto& rec : recommendations) {
            std::cout << "  • " << rec << "\n";
        }
    }
    
    void executeChargingSession() {
        chargingManager->executeScheduledCharging();
        // Note: In real implementation, this would update currentSOC
    }
    
    void updateGridConditions(double loadFactor, double price, double solarAvailability) {
        chargingManager->updateGridConditions(loadFactor, price, solarAvailability);
    }
    
    void displaySystemStatus() {
        std::cout << "\n" << std::string(60, '=') << "\n";
        std::cout << "ENERGY MANAGEMENT SYSTEM STATUS\n";
        std::cout << std::string(60, '=') << "\n";
        
        std::cout << "Battery Status:\n";
        std::cout << "  SOC: " << currentSOC << "%\n";
        std::cout << "  Capacity: " << batteryCapacity << " kWh\n";
        std::cout << "  Available Energy: " << (currentSOC / 100.0 * batteryCapacity) << " kWh\n";
        std::cout << "  Current Power: " << currentPower << " kW\n";
        
        std::cout << "\nDriving Mode: " << getDrivingModeString(currentDrivingMode) << "\n";
        
        std::cout << "\nEnergy Statistics:\n";
        std::cout << "  Total Consumed: " << totalEnergyConsumed << " kWh\n";
        std::cout << "  Total Regenerated: " << totalEnergyRegenerated << " kWh\n";
        std::cout << "  Net Consumption: " << (totalEnergyConsumed - totalEnergyRegenerated) << " kWh\n";
        std::cout << "  System Efficiency: " << averageEfficiency << "%\n";
        std::cout << "  Total Charging Cost: $" << totalChargingCost << "\n";
        
        std::cout << "\n";
        powerDistributor->displayPowerDistribution();
        chargingManager->displayChargingStatus();
    }
    
    // Getters
    double getCurrentSOC() const { return currentSOC; }
    double getBatteryCapacity() const { return batteryCapacity; }
    double getCurrentPower() const { return currentPower; }
    double getAvailableEnergy() const { return currentSOC / 100.0 * batteryCapacity; }
    double getSystemEfficiency() const { return averageEfficiency; }
    
    // Setters for testing
    void setSOC(double soc) { currentSOC = std::max(0.0, std::min(100.0, soc)); }
    void setBatteryCapacity(double capacity) { batteryCapacity = capacity; }
    
private:
    std::string getDrivingModeString(DrivingMode mode) const {
        switch (mode) {
            case DrivingMode::ECO: return "ECO";
            case DrivingMode::COMFORT: return "COMFORT";
            case DrivingMode::SPORT: return "SPORT";
            case DrivingMode::CUSTOM: return "CUSTOM";
            case DrivingMode::WINTER: return "WINTER";
            case DrivingMode::TOWING: return "TOWING";
            default: return "UNKNOWN";
        }
    }
};

#endif // ENERGY_MANAGEMENT_H