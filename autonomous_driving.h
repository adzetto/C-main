/**
 * Autonomous Driving Framework for Electric Vehicles
 * Author: adzetto
 * Features: Path planning, decision making, sensor fusion, motion control, safety systems
 */

#ifndef AUTONOMOUS_DRIVING_H
#define AUTONOMOUS_DRIVING_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <string>
#include <chrono>
#include <memory>
#include <queue>
#include <algorithm>
#include <cmath>
#include <thread>
#include <mutex>
#include <atomic>

enum class AutonomyLevel {
    LEVEL_0_NO_AUTOMATION,
    LEVEL_1_DRIVER_ASSISTANCE,
    LEVEL_2_PARTIAL_AUTOMATION,
    LEVEL_3_CONDITIONAL_AUTOMATION,
    LEVEL_4_HIGH_AUTOMATION,
    LEVEL_5_FULL_AUTOMATION
};

enum class DrivingState {
    MANUAL_CONTROL,
    MONITORING,
    ENGAGED,
    INTERVENTION_REQUIRED,
    EMERGENCY_STOP,
    SYSTEM_FAULT
};

enum class PathPlanningMode {
    GLOBAL_PLANNER,
    LOCAL_PLANNER,
    EMERGENCY_PLANNER,
    PARKING_PLANNER
};

enum class ManeuverType {
    LANE_FOLLOWING,
    LANE_CHANGE_LEFT,
    LANE_CHANGE_RIGHT,
    OVERTAKING,
    MERGING,
    TURNING_LEFT,
    TURNING_RIGHT,
    U_TURN,
    PARKING,
    EMERGENCY_STOP
};

struct Waypoint {
    double x, y;           // Global coordinates (m)
    double heading;        // Heading angle (radians)
    double speed;          // Target speed (m/s)
    double curvature;      // Path curvature (1/m)
    double timeStamp;      // Time to reach waypoint (s)
    
    Waypoint(double x = 0, double y = 0, double h = 0, double s = 0) 
        : x(x), y(y), heading(h), speed(s), curvature(0), timeStamp(0) {}
    
    double distanceTo(const Waypoint& other) const {
        return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
    }
};

struct DetectedObject {
    int id;
    double x, y;           // Position (m)
    double vx, vy;         // Velocity (m/s)
    double ax, ay;         // Acceleration (m/s²)
    double width, length;  // Dimensions (m)
    double confidence;     // Detection confidence (0-1)
    std::string type;      // "vehicle", "pedestrian", "cyclist", "static"
    std::chrono::steady_clock::time_point lastSeen;
    std::vector<Waypoint> predictedPath;
    
    DetectedObject() : id(0), x(0), y(0), vx(0), vy(0), ax(0), ay(0),
                      width(1.8), length(4.5), confidence(0), type("unknown") {
        lastSeen = std::chrono::steady_clock::now();
    }
    
    void predictFuturePath(double timeHorizon = 5.0, double timeStep = 0.1) {
        predictedPath.clear();
        
        for (double t = 0; t <= timeHorizon; t += timeStep) {
            Waypoint futurePoint;
            futurePoint.x = x + vx * t + 0.5 * ax * t * t;
            futurePoint.y = y + vy * t + 0.5 * ay * t * t;
            futurePoint.speed = sqrt(pow(vx + ax * t, 2) + pow(vy + ay * t, 2));
            futurePoint.heading = atan2(vy + ay * t, vx + ax * t);
            futurePoint.timeStamp = t;
            
            predictedPath.push_back(futurePoint);
        }
    }
};

struct DrivingScenario {
    std::string scenarioId;
    std::string description;
    std::vector<DetectedObject> objects;
    double complexity;         // Complexity score (0-1)
    double riskLevel;         // Risk assessment (0-1)
    ManeuverType requiredManeuver;
    std::vector<std::string> constraints;
    
    DrivingScenario() : complexity(0), riskLevel(0), requiredManeuver(ManeuverType::LANE_FOLLOWING) {}
};

class PathPlanner {
private:
    std::vector<Waypoint> globalPath;
    std::vector<Waypoint> localPath;
    PathPlanningMode currentMode;
    
    // Planning parameters
    double planningHorizon;    // Planning distance (m)
    double timeHorizon;        // Planning time (s)
    double lateralResolution;  // Lateral sampling resolution (m)
    double longitudinalResolution; // Longitudinal sampling resolution (m)
    
    // Vehicle constraints
    double maxSpeed;           // m/s
    double maxAcceleration;    // m/s²
    double maxDeceleration;    // m/s²
    double maxSteeringAngle;   // radians
    double wheelbase;          // m
    
public:
    PathPlanner() : currentMode(PathPlanningMode::GLOBAL_PLANNER),
                   planningHorizon(100.0), timeHorizon(5.0),
                   lateralResolution(0.5), longitudinalResolution(1.0),
                   maxSpeed(30.0), maxAcceleration(3.0), maxDeceleration(-8.0),
                   maxSteeringAngle(0.6), wheelbase(2.8) {}
    
    void setGlobalPath(const std::vector<Waypoint>& path) {
        globalPath = path;
        std::cout << "Global path set with " << path.size() << " waypoints\n";
    }
    
    std::vector<Waypoint> planLocalPath(const Waypoint& currentPose,
                                       const std::vector<DetectedObject>& obstacles) {
        localPath.clear();
        
        switch (currentMode) {
            case PathPlanningMode::GLOBAL_PLANNER:
                localPath = planGlobalPath(currentPose);
                break;
            case PathPlanningMode::LOCAL_PLANNER:
                localPath = planObstacleAvoidance(currentPose, obstacles);
                break;
            case PathPlanningMode::EMERGENCY_PLANNER:
                localPath = planEmergencyPath(currentPose, obstacles);
                break;
            case PathPlanningMode::PARKING_PLANNER:
                localPath = planParkingPath(currentPose);
                break;
        }
        
        // Smooth and validate path
        smoothPath(localPath);
        validatePath(localPath);
        
        return localPath;
    }
    
    void setPlanningMode(PathPlanningMode mode) {
        currentMode = mode;
        std::cout << "Path planning mode set to: " << getPlanningModeString(mode) << "\n";
    }
    
    double calculatePathCost(const std::vector<Waypoint>& path,
                           const std::vector<DetectedObject>& obstacles) {
        double cost = 0.0;
        
        // Distance cost
        for (size_t i = 1; i < path.size(); i++) {
            cost += path[i-1].distanceTo(path[i]);
        }
        
        // Curvature cost
        for (const auto& waypoint : path) {
            cost += abs(waypoint.curvature) * 10.0;
        }
        
        // Obstacle proximity cost
        for (const auto& waypoint : path) {
            for (const auto& obstacle : obstacles) {
                double distance = sqrt(pow(waypoint.x - obstacle.x, 2) + 
                                     pow(waypoint.y - obstacle.y, 2));
                if (distance < 5.0) { // Within 5m
                    cost += (5.0 - distance) * 50.0; // High penalty for close obstacles
                }
            }
        }
        
        // Speed variation cost
        for (size_t i = 1; i < path.size(); i++) {
            double speedChange = abs(path[i].speed - path[i-1].speed);
            cost += speedChange * 2.0;
        }
        
        return cost;
    }
    
    void displayPlannerStatus() const {
        std::cout << "\n=== Path Planner Status ===\n";
        std::cout << "Current Mode: " << getPlanningModeString(currentMode) << "\n";
        std::cout << "Global Path Points: " << globalPath.size() << "\n";
        std::cout << "Local Path Points: " << localPath.size() << "\n";
        std::cout << "Planning Horizon: " << planningHorizon << " m\n";
        std::cout << "Time Horizon: " << timeHorizon << " s\n";
        std::cout << "Max Speed: " << maxSpeed << " m/s\n";
        std::cout << "Max Acceleration: " << maxAcceleration << " m/s²\n";
        std::cout << "==========================\n\n";
    }
    
private:
    std::vector<Waypoint> planGlobalPath(const Waypoint& currentPose) {
        std::vector<Waypoint> path;
        
        // Find nearest point on global path
        if (globalPath.empty()) {
            // Create simple straight path ahead
            for (int i = 0; i < 20; i++) {
                Waypoint wp;
                wp.x = currentPose.x + i * 5.0 * cos(currentPose.heading);
                wp.y = currentPose.y + i * 5.0 * sin(currentPose.heading);
                wp.heading = currentPose.heading;
                wp.speed = maxSpeed * 0.8; // 80% of max speed
                path.push_back(wp);
            }
            return path;
        }
        
        // Extract local segment from global path
        int startIndex = findNearestWaypoint(currentPose, globalPath);
        int endIndex = std::min(static_cast<int>(globalPath.size()), 
                               startIndex + static_cast<int>(planningHorizon / 5.0));
        
        for (int i = startIndex; i < endIndex; i++) {
            path.push_back(globalPath[i]);
        }
        
        return path;
    }
    
    std::vector<Waypoint> planObstacleAvoidance(const Waypoint& currentPose,
                                               const std::vector<DetectedObject>& obstacles) {
        std::vector<Waypoint> bestPath;
        double minCost = std::numeric_limits<double>::max();
        
        // Generate multiple candidate paths
        for (double lateralOffset = -2.0; lateralOffset <= 2.0; lateralOffset += lateralResolution) {
            std::vector<Waypoint> candidatePath = generateCandidatePath(currentPose, lateralOffset);
            
            if (isPathFeasible(candidatePath, obstacles)) {
                double cost = calculatePathCost(candidatePath, obstacles);
                if (cost < minCost) {
                    minCost = cost;
                    bestPath = candidatePath;
                }
            }
        }
        
        // If no feasible path found, use emergency planning
        if (bestPath.empty()) {
            bestPath = planEmergencyPath(currentPose, obstacles);
        }
        
        return bestPath;
    }
    
    std::vector<Waypoint> planEmergencyPath(const Waypoint& currentPose,
                                           const std::vector<DetectedObject>& obstacles) {
        std::vector<Waypoint> emergencyPath;
        
        // Create emergency stop path
        for (int i = 0; i < 10; i++) {
            Waypoint wp;
            wp.x = currentPose.x + i * 2.0 * cos(currentPose.heading);
            wp.y = currentPose.y + i * 2.0 * sin(currentPose.heading);
            wp.heading = currentPose.heading;
            wp.speed = std::max(0.0, currentPose.speed - i * 2.0); // Gradual deceleration
            emergencyPath.push_back(wp);
        }
        
        return emergencyPath;
    }
    
    std::vector<Waypoint> planParkingPath(const Waypoint& currentPose) {
        std::vector<Waypoint> parkingPath;
        
        // Simple parallel parking maneuver
        double parkingLength = 6.0; // m
        double parkingWidth = 2.5;  // m
        
        // Generate parking waypoints (simplified)
        for (int i = 0; i < 15; i++) {
            Waypoint wp;
            double t = i / 14.0; // Normalized time
            
            // S-curve for parking
            wp.x = currentPose.x + parkingLength * t;
            wp.y = currentPose.y + parkingWidth * sin(M_PI * t);
            wp.heading = currentPose.heading + M_PI/4 * sin(2 * M_PI * t);
            wp.speed = 1.0; // Slow parking speed
            
            parkingPath.push_back(wp);
        }
        
        return parkingPath;
    }
    
    std::vector<Waypoint> generateCandidatePath(const Waypoint& currentPose, double lateralOffset) {
        std::vector<Waypoint> path;
        
        for (int i = 0; i < 20; i++) {
            Waypoint wp;
            double distance = i * longitudinalResolution;
            
            wp.x = currentPose.x + distance * cos(currentPose.heading) - 
                   lateralOffset * sin(currentPose.heading);
            wp.y = currentPose.y + distance * sin(currentPose.heading) + 
                   lateralOffset * cos(currentPose.heading);
            wp.heading = currentPose.heading + lateralOffset * 0.1; // Slight heading change
            wp.speed = maxSpeed * 0.8;
            wp.curvature = lateralOffset / (distance + 1.0); // Avoid division by zero
            
            path.push_back(wp);
        }
        
        return path;
    }
    
    bool isPathFeasible(const std::vector<Waypoint>& path,
                       const std::vector<DetectedObject>& obstacles) {
        // Check kinematic feasibility
        for (size_t i = 1; i < path.size(); i++) {
            double ds = path[i-1].distanceTo(path[i]);
            double dheading = abs(path[i].heading - path[i-1].heading);
            
            // Check maximum steering constraint
            if (ds > 0 && dheading / ds > maxSteeringAngle / wheelbase) {
                return false;
            }
            
            // Check acceleration constraints
            double dv = path[i].speed - path[i-1].speed;
            if (ds > 0) {
                double acceleration = dv * path[i].speed / ds;
                if (acceleration > maxAcceleration || acceleration < maxDeceleration) {
                    return false;
                }
            }
        }
        
        // Check obstacle collisions
        for (const auto& wp : path) {
            for (const auto& obstacle : obstacles) {
                double distance = sqrt(pow(wp.x - obstacle.x, 2) + pow(wp.y - obstacle.y, 2));
                double safetyMargin = (obstacle.width + 1.8) / 2.0; // Vehicle width + margin
                
                if (distance < safetyMargin) {
                    return false;
                }
            }
        }
        
        return true;
    }
    
    void smoothPath(std::vector<Waypoint>& path) {
        if (path.size() < 3) return;
        
        // Apply simple smoothing filter
        std::vector<Waypoint> smoothedPath = path;
        
        for (size_t i = 1; i < path.size() - 1; i++) {
            smoothedPath[i].x = 0.25 * path[i-1].x + 0.5 * path[i].x + 0.25 * path[i+1].x;
            smoothedPath[i].y = 0.25 * path[i-1].y + 0.5 * path[i].y + 0.25 * path[i+1].y;
            
            // Recalculate heading and curvature
            if (i > 0) {
                double dx = smoothedPath[i].x - smoothedPath[i-1].x;
                double dy = smoothedPath[i].y - smoothedPath[i-1].y;
                smoothedPath[i].heading = atan2(dy, dx);
            }
        }
        
        path = smoothedPath;
    }
    
    void validatePath(std::vector<Waypoint>& path) {
        // Ensure path meets safety requirements
        for (auto& wp : path) {
            wp.speed = std::max(0.0, std::min(maxSpeed, wp.speed));
            wp.heading = fmod(wp.heading + M_PI, 2 * M_PI) - M_PI; // Normalize to [-π, π]
        }
    }
    
    int findNearestWaypoint(const Waypoint& pose, const std::vector<Waypoint>& path) {
        if (path.empty()) return 0;
        
        int nearestIndex = 0;
        double minDistance = pose.distanceTo(path[0]);
        
        for (size_t i = 1; i < path.size(); i++) {
            double distance = pose.distanceTo(path[i]);
            if (distance < minDistance) {
                minDistance = distance;
                nearestIndex = i;
            }
        }
        
        return nearestIndex;
    }
    
    std::string getPlanningModeString(PathPlanningMode mode) const {
        switch (mode) {
            case PathPlanningMode::GLOBAL_PLANNER: return "GLOBAL_PLANNER";
            case PathPlanningMode::LOCAL_PLANNER: return "LOCAL_PLANNER";
            case PathPlanningMode::EMERGENCY_PLANNER: return "EMERGENCY_PLANNER";
            case PathPlanningMode::PARKING_PLANNER: return "PARKING_PLANNER";
            default: return "UNKNOWN";
        }
    }
};

class DecisionMaker {
private:
    DrivingState currentState;
    std::queue<ManeuverType> maneuverQueue;
    std::unordered_map<std::string, double> behaviorWeights;
    
    // Decision parameters
    double riskTolerance;
    double aggressiveness;
    double safetyMargin;
    
    // Decision history for learning
    std::vector<DrivingScenario> scenarioHistory;
    std::vector<ManeuverType> decisionHistory;
    
public:
    DecisionMaker() : currentState(DrivingState::MANUAL_CONTROL),
                     riskTolerance(0.3), aggressiveness(0.5), safetyMargin(2.0) {
        initializeBehaviorWeights();
        std::cout << "Decision Making system initialized\n";
    }
    
    ManeuverType decideManeuver(const DrivingScenario& scenario,
                               const Waypoint& currentPose,
                               const std::vector<Waypoint>& plannedPath) {
        // Analyze current situation
        double riskScore = assessRisk(scenario);
        double urgency = calculateUrgency(scenario, currentPose);
        
        ManeuverType decision = ManeuverType::LANE_FOLLOWING;
        
        // Decision logic based on scenario
        if (riskScore > 0.8) {
            decision = ManeuverType::EMERGENCY_STOP;
        } else if (scenario.requiredManeuver != ManeuverType::LANE_FOLLOWING) {
            decision = evaluateManeuver(scenario.requiredManeuver, scenario);
        } else {
            decision = selectOptimalManeuver(scenario, plannedPath);
        }
        
        // Store decision for learning
        scenarioHistory.push_back(scenario);
        decisionHistory.push_back(decision);
        
        // Keep history size manageable
        if (scenarioHistory.size() > 1000) {
            scenarioHistory.erase(scenarioHistory.begin());
            decisionHistory.erase(decisionHistory.begin());
        }
        
        std::cout << "Decision made: " << getManeuverString(decision) 
                  << " (Risk: " << riskScore << ")\n";
        
        return decision;
    }
    
    void updateDrivingState(DrivingState newState) {
        if (currentState != newState) {
            std::cout << "Driving state changed: " << getStateString(currentState) 
                      << " -> " << getStateString(newState) << "\n";
            currentState = newState;
        }
    }
    
    bool shouldInterventionBeRequested(const DrivingScenario& scenario) {
        // Check for conditions requiring human intervention
        if (scenario.riskLevel > 0.9) return true;
        if (scenario.complexity > 0.95) return true;
        
        // Check for unknown scenarios
        bool isKnownScenario = false;
        for (const auto& knownScenario : scenarioHistory) {
            if (isSimilarScenario(scenario, knownScenario, 0.8)) {
                isKnownScenario = true;
                break;
            }
        }
        
        if (!isKnownScenario && scenario.complexity > 0.7) {
            return true;
        }
        
        return false;
    }
    
    void setBehaviorParameters(double risk, double aggression, double safety) {
        riskTolerance = std::max(0.0, std::min(1.0, risk));
        aggressiveness = std::max(0.0, std::min(1.0, aggression));
        safetyMargin = std::max(1.0, std::min(5.0, safety));
        
        std::cout << "Behavior parameters updated: Risk=" << riskTolerance 
                  << ", Aggression=" << aggressiveness 
                  << ", Safety=" << safetyMargin << "\n";
    }
    
    void displayDecisionStatus() const {
        std::cout << "\n=== Decision Making Status ===\n";
        std::cout << "Current State: " << getStateString(currentState) << "\n";
        std::cout << "Queued Maneuvers: " << maneuverQueue.size() << "\n";
        std::cout << "Risk Tolerance: " << riskTolerance << "\n";
        std::cout << "Aggressiveness: " << aggressiveness << "\n";
        std::cout << "Safety Margin: " << safetyMargin << " m\n";
        std::cout << "Decision History: " << decisionHistory.size() << " decisions\n";
        std::cout << "=============================\n\n";
    }
    
private:
    void initializeBehaviorWeights() {
        behaviorWeights["safety"] = 1.0;
        behaviorWeights["efficiency"] = 0.7;
        behaviorWeights["comfort"] = 0.8;
        behaviorWeights["progress"] = 0.6;
        behaviorWeights["social"] = 0.5;
    }
    
    double assessRisk(const DrivingScenario& scenario) {
        double risk = scenario.riskLevel;
        
        // Increase risk based on complexity
        risk += scenario.complexity * 0.3;
        
        // Check for high-risk objects
        for (const auto& obj : scenario.objects) {
            if (obj.type == "pedestrian" && obj.confidence > 0.8) {
                risk += 0.2;
            }
            if (obj.type == "cyclist" && obj.confidence > 0.8) {
                risk += 0.15;
            }
        }
        
        return std::min(1.0, risk);
    }
    
    double calculateUrgency(const DrivingScenario& scenario, const Waypoint& currentPose) {
        double urgency = 0.0;
        
        // Time to collision with objects
        for (const auto& obj : scenario.objects) {
            double distance = sqrt(pow(currentPose.x - obj.x, 2) + pow(currentPose.y - obj.y, 2));
            double relativeVelocity = currentPose.speed - obj.vx;
            
            if (relativeVelocity > 0 && distance > 0) {
                double ttc = distance / relativeVelocity;
                if (ttc < 5.0) { // Less than 5 seconds
                    urgency = std::max(urgency, 1.0 - ttc / 5.0);
                }
            }
        }
        
        return urgency;
    }
    
    ManeuverType evaluateManeuver(ManeuverType proposedManeuver, const DrivingScenario& scenario) {
        // Check if proposed maneuver is safe and feasible
        double maneuverRisk = calculateManeuverRisk(proposedManeuver, scenario);
        
        if (maneuverRisk > riskTolerance) {
            // Find safer alternative
            return findSaferAlternative(proposedManeuver, scenario);
        }
        
        return proposedManeuver;
    }
    
    ManeuverType selectOptimalManeuver(const DrivingScenario& scenario,
                                     const std::vector<Waypoint>& plannedPath) {
        std::vector<ManeuverType> candidates = {
            ManeuverType::LANE_FOLLOWING,
            ManeuverType::LANE_CHANGE_LEFT,
            ManeuverType::LANE_CHANGE_RIGHT
        };
        
        ManeuverType bestManeuver = ManeuverType::LANE_FOLLOWING;
        double bestScore = calculateManeuverScore(ManeuverType::LANE_FOLLOWING, scenario);
        
        for (ManeuverType candidate : candidates) {
            double score = calculateManeuverScore(candidate, scenario);
            if (score > bestScore) {
                bestScore = score;
                bestManeuver = candidate;
            }
        }
        
        return bestManeuver;
    }
    
    double calculateManeuverRisk(ManeuverType maneuver, const DrivingScenario& scenario) {
        double baseRisk = 0.1; // Base risk for any maneuver
        
        switch (maneuver) {
            case ManeuverType::LANE_FOLLOWING:
                baseRisk = 0.1;
                break;
            case ManeuverType::LANE_CHANGE_LEFT:
            case ManeuverType::LANE_CHANGE_RIGHT:
                baseRisk = 0.3;
                break;
            case ManeuverType::OVERTAKING:
                baseRisk = 0.5;
                break;
            case ManeuverType::U_TURN:
                baseRisk = 0.7;
                break;
            case ManeuverType::EMERGENCY_STOP:
                baseRisk = 0.4;
                break;
            default:
                baseRisk = 0.2;
                break;
        }
        
        // Adjust based on scenario complexity and object proximity
        baseRisk += scenario.complexity * 0.2;
        baseRisk += scenario.objects.size() * 0.05;
        
        return std::min(1.0, baseRisk);
    }
    
    double calculateManeuverScore(ManeuverType maneuver, const DrivingScenario& scenario) {
        double score = 1.0;
        
        // Safety component
        double risk = calculateManeuverRisk(maneuver, scenario);
        score -= risk * behaviorWeights["safety"];
        
        // Efficiency component (progress toward goal)
        if (maneuver == ManeuverType::LANE_FOLLOWING) {
            score += 0.8 * behaviorWeights["efficiency"];
        } else if (maneuver == ManeuverType::LANE_CHANGE_LEFT || 
                   maneuver == ManeuverType::LANE_CHANGE_RIGHT) {
            score += 0.6 * behaviorWeights["efficiency"];
        }
        
        // Comfort component (smooth driving)
        if (maneuver == ManeuverType::LANE_FOLLOWING) {
            score += 0.9 * behaviorWeights["comfort"];
        } else if (maneuver == ManeuverType::EMERGENCY_STOP) {
            score -= 0.5 * behaviorWeights["comfort"];
        }
        
        return std::max(0.0, score);
    }
    
    ManeuverType findSaferAlternative(ManeuverType originalManeuver, const DrivingScenario& scenario) {
        // If original maneuver is too risky, find safer option
        std::vector<ManeuverType> alternatives;
        
        switch (originalManeuver) {
            case ManeuverType::LANE_CHANGE_LEFT:
            case ManeuverType::LANE_CHANGE_RIGHT:
                alternatives = {ManeuverType::LANE_FOLLOWING};
                break;
            case ManeuverType::OVERTAKING:
                alternatives = {ManeuverType::LANE_FOLLOWING, ManeuverType::LANE_CHANGE_LEFT};
                break;
            default:
                alternatives = {ManeuverType::LANE_FOLLOWING, ManeuverType::EMERGENCY_STOP};
                break;
        }
        
        ManeuverType safestManeuver = ManeuverType::EMERGENCY_STOP;
        double lowestRisk = 1.0;
        
        for (ManeuverType alternative : alternatives) {
            double risk = calculateManeuverRisk(alternative, scenario);
            if (risk < lowestRisk) {
                lowestRisk = risk;
                safestManeuver = alternative;
            }
        }
        
        return safestManeuver;
    }
    
    bool isSimilarScenario(const DrivingScenario& scenario1, const DrivingScenario& scenario2,
                          double threshold) {
        // Simple similarity metric based on object count and complexity
        double complexityDiff = abs(scenario1.complexity - scenario2.complexity);
        double riskDiff = abs(scenario1.riskLevel - scenario2.riskLevel);
        double objectCountDiff = abs(static_cast<double>(scenario1.objects.size()) - 
                                   static_cast<double>(scenario2.objects.size()));
        
        double similarity = 1.0 - (complexityDiff + riskDiff + objectCountDiff * 0.1) / 3.0;
        return similarity > threshold;
    }
    
    std::string getStateString(DrivingState state) const {
        switch (state) {
            case DrivingState::MANUAL_CONTROL: return "MANUAL_CONTROL";
            case DrivingState::MONITORING: return "MONITORING";
            case DrivingState::ENGAGED: return "ENGAGED";
            case DrivingState::INTERVENTION_REQUIRED: return "INTERVENTION_REQUIRED";
            case DrivingState::EMERGENCY_STOP: return "EMERGENCY_STOP";
            case DrivingState::SYSTEM_FAULT: return "SYSTEM_FAULT";
            default: return "UNKNOWN";
        }
    }
    
    std::string getManeuverString(ManeuverType maneuver) const {
        switch (maneuver) {
            case ManeuverType::LANE_FOLLOWING: return "LANE_FOLLOWING";
            case ManeuverType::LANE_CHANGE_LEFT: return "LANE_CHANGE_LEFT";
            case ManeuverType::LANE_CHANGE_RIGHT: return "LANE_CHANGE_RIGHT";
            case ManeuverType::OVERTAKING: return "OVERTAKING";
            case ManeuverType::MERGING: return "MERGING";
            case ManeuverType::TURNING_LEFT: return "TURNING_LEFT";
            case ManeuverType::TURNING_RIGHT: return "TURNING_RIGHT";
            case ManeuverType::U_TURN: return "U_TURN";
            case ManeuverType::PARKING: return "PARKING";
            case ManeuverType::EMERGENCY_STOP: return "EMERGENCY_STOP";
            default: return "UNKNOWN";
        }
    }
};

class MotionController {
private:
    double targetSpeed;
    double targetSteering;
    double currentSpeed;
    double currentSteering;
    
    // PID controllers
    struct PIDController {
        double kp, ki, kd;
        double integral, previousError;
        double output;
        
        PIDController(double p = 1.0, double i = 0.1, double d = 0.01) 
            : kp(p), ki(i), kd(d), integral(0), previousError(0), output(0) {}
        
        double calculate(double setpoint, double actual, double dt) {
            double error = setpoint - actual;
            integral += error * dt;
            double derivative = (error - previousError) / dt;
            
            output = kp * error + ki * integral + kd * derivative;
            previousError = error;
            
            return output;
        }
    };
    
    PIDController speedController;
    PIDController steeringController;
    
    // Control limits
    double maxAcceleration;
    double maxDeceleration;
    double maxSteeringRate;
    
    // Vehicle model parameters
    double wheelbase;
    double mass;
    double inertia;
    
public:
    MotionController() : targetSpeed(0), targetSteering(0), currentSpeed(0), currentSteering(0),
                        speedController(2.0, 0.5, 0.1), steeringController(3.0, 0.2, 0.05),
                        maxAcceleration(3.0), maxDeceleration(8.0), maxSteeringRate(1.0),
                        wheelbase(2.8), mass(1800.0), inertia(3000.0) {}
    
    void setTargets(double speed, double steering) {
        targetSpeed = speed;
        targetSteering = steering;
    }
    
    void updateControl(double dt) {
        // Speed control
        double speedError = targetSpeed - currentSpeed;
        double accelerationCommand = speedController.calculate(targetSpeed, currentSpeed, dt);
        
        // Apply acceleration limits
        accelerationCommand = std::max(-maxDeceleration, 
                                     std::min(maxAcceleration, accelerationCommand));
        
        // Update current speed (simplified dynamics)
        currentSpeed += accelerationCommand * dt;
        currentSpeed = std::max(0.0, currentSpeed);
        
        // Steering control
        double steeringCommand = steeringController.calculate(targetSteering, currentSteering, dt);
        
        // Apply steering rate limits
        double steeringRate = steeringCommand - currentSteering;
        steeringRate = std::max(-maxSteeringRate * dt, 
                               std::min(maxSteeringRate * dt, steeringRate));
        
        currentSteering += steeringRate;
        currentSteering = std::max(-0.6, std::min(0.6, currentSteering)); // ±35 degrees
    }
    
    std::pair<double, double> getControlOutputs() const {
        return {currentSpeed, currentSteering};
    }
    
    void displayControlStatus() const {
        std::cout << "\n=== Motion Controller Status ===\n";
        std::cout << "Target Speed: " << targetSpeed << " m/s\n";
        std::cout << "Current Speed: " << currentSpeed << " m/s\n";
        std::cout << "Speed Error: " << (targetSpeed - currentSpeed) << " m/s\n";
        std::cout << "Target Steering: " << targetSteering << " rad\n";
        std::cout << "Current Steering: " << currentSteering << " rad\n";
        std::cout << "Steering Error: " << (targetSteering - currentSteering) << " rad\n";
        std::cout << "===============================\n\n";
    }
};

class AutonomousDrivingSystem {
private:
    std::unique_ptr<PathPlanner> pathPlanner;
    std::unique_ptr<DecisionMaker> decisionMaker;
    std::unique_ptr<MotionController> motionController;
    
    AutonomyLevel currentLevel;
    DrivingState systemState;
    std::atomic<bool> systemActive;
    std::thread controlThread;
    
    // Current vehicle state
    Waypoint currentPose;
    std::vector<DetectedObject> perceivedObjects;
    
    // System statistics
    uint32_t totalDecisions;
    uint32_t interventionsRequested;
    uint32_t emergencyStops;
    double totalAutonomousDistance;
    
public:
    AutonomousDrivingSystem() : currentLevel(AutonomyLevel::LEVEL_0_NO_AUTOMATION),
                               systemState(DrivingState::MANUAL_CONTROL),
                               systemActive(false), totalDecisions(0),
                               interventionsRequested(0), emergencyStops(0),
                               totalAutonomousDistance(0.0) {
        
        pathPlanner = std::make_unique<PathPlanner>();
        decisionMaker = std::make_unique<DecisionMaker>();
        motionController = std::make_unique<MotionController>();
        
        std::cout << "Autonomous Driving System initialized\n";
    }
    
    ~AutonomousDrivingSystem() {
        disableAutonomy();
    }
    
    void setAutonomyLevel(AutonomyLevel level) {
        currentLevel = level;
        updateSystemCapabilities();
        std::cout << "Autonomy level set to: " << getAutonomyLevelString(level) << "\n";
    }
    
    void enableAutonomy() {
        if (currentLevel == AutonomyLevel::LEVEL_0_NO_AUTOMATION) {
            std::cout << "Cannot enable autonomy at Level 0\n";
            return;
        }
        
        systemActive = true;
        systemState = DrivingState::MONITORING;
        controlThread = std::thread(&AutonomousDrivingSystem::autonomousControlLoop, this);
        
        std::cout << "Autonomous driving enabled\n";
    }
    
    void disableAutonomy() {
        systemActive = false;
        systemState = DrivingState::MANUAL_CONTROL;
        
        if (controlThread.joinable()) {
            controlThread.join();
        }
        
        std::cout << "Autonomous driving disabled\n";
    }
    
    void updateVehicleState(double x, double y, double heading, double speed) {
        currentPose.x = x;
        currentPose.y = y;
        currentPose.heading = heading;
        currentPose.speed = speed;
    }
    
    void updatePerception(const std::vector<DetectedObject>& objects) {
        perceivedObjects = objects;
        
        // Predict future paths for all objects
        for (auto& obj : perceivedObjects) {
            obj.predictFuturePath();
        }
    }
    
    void setDestination(double x, double y) {
        std::vector<Waypoint> globalPath;
        
        // Simple path from current position to destination
        double distance = sqrt(pow(x - currentPose.x, 2) + pow(y - currentPose.y, 2));
        int numWaypoints = static_cast<int>(distance / 10.0) + 1; // Every 10m
        
        for (int i = 0; i <= numWaypoints; i++) {
            Waypoint wp;
            double t = static_cast<double>(i) / numWaypoints;
            wp.x = currentPose.x + t * (x - currentPose.x);
            wp.y = currentPose.y + t * (y - currentPose.y);
            wp.heading = atan2(y - currentPose.y, x - currentPose.x);
            wp.speed = 15.0; // 15 m/s target speed
            
            globalPath.push_back(wp);
        }
        
        pathPlanner->setGlobalPath(globalPath);
        std::cout << "Destination set to (" << x << ", " << y << ")\n";
    }
    
    void requestIntervention() {
        systemState = DrivingState::INTERVENTION_REQUIRED;
        interventionsRequested++;
        std::cout << "INTERVENTION REQUESTED - Human takeover required\n";
    }
    
    void emergencyStop() {
        systemState = DrivingState::EMERGENCY_STOP;
        emergencyStops++;
        motionController->setTargets(0.0, 0.0); // Stop and straight
        std::cout << "EMERGENCY STOP ACTIVATED\n";
    }
    
    void displaySystemStatus() const {
        std::cout << "\n" << std::string(50, '=') << "\n";
        std::cout << "AUTONOMOUS DRIVING SYSTEM STATUS\n";
        std::cout << std::string(50, '=') << "\n";
        
        std::cout << "Autonomy Level: " << getAutonomyLevelString(currentLevel) << "\n";
        std::cout << "System State: " << getSystemStateString(systemState) << "\n";
        std::cout << "System Active: " << (systemActive ? "YES" : "NO") << "\n";
        
        std::cout << "\nVehicle State:\n";
        std::cout << "  Position: (" << currentPose.x << ", " << currentPose.y << ")\n";
        std::cout << "  Heading: " << currentPose.heading << " rad\n";
        std::cout << "  Speed: " << currentPose.speed << " m/s\n";
        
        std::cout << "\nPerception:\n";
        std::cout << "  Detected Objects: " << perceivedObjects.size() << "\n";
        
        std::cout << "\nSystem Statistics:\n";
        std::cout << "  Total Decisions: " << totalDecisions << "\n";
        std::cout << "  Interventions Requested: " << interventionsRequested << "\n";
        std::cout << "  Emergency Stops: " << emergencyStops << "\n";
        std::cout << "  Autonomous Distance: " << totalAutonomousDistance << " m\n";
        
        if (totalDecisions > 0) {
            double interventionRate = (interventionsRequested * 100.0) / totalDecisions;
            std::cout << "  Intervention Rate: " << interventionRate << "%\n";
        }
        
        std::cout << "\n";
        pathPlanner->displayPlannerStatus();
        decisionMaker->displayDecisionStatus();
        motionController->displayControlStatus();
    }
    
private:
    void autonomousControlLoop() {
        const double controlFrequency = 10.0; // 10 Hz
        const double dt = 1.0 / controlFrequency;
        
        while (systemActive) {
            auto loopStart = std::chrono::steady_clock::now();
            
            // Check system health
            if (systemState == DrivingState::SYSTEM_FAULT) {
                emergencyStop();
                break;
            }
            
            // Create driving scenario
            DrivingScenario scenario;
            scenario.objects = perceivedObjects;
            scenario.complexity = calculateScenarioComplexity();
            scenario.riskLevel = calculateRiskLevel();
            
            // Check if intervention is needed
            if (decisionMaker->shouldInterventionBeRequested(scenario)) {
                requestIntervention();
                continue;
            }
            
            // Plan path
            std::vector<Waypoint> localPath = pathPlanner->planLocalPath(currentPose, perceivedObjects);
            
            if (localPath.empty()) {
                emergencyStop();
                continue;
            }
            
            // Make decision
            ManeuverType decision = decisionMaker->decideManeuver(scenario, currentPose, localPath);
            totalDecisions++;
            
            // Handle emergency situations
            if (decision == ManeuverType::EMERGENCY_STOP) {
                emergencyStop();
                continue;
            }
            
            // Execute motion control
            executeManeuver(decision, localPath, dt);
            
            // Update statistics
            updateStatistics(dt);
            
            // Maintain control frequency
            auto loopEnd = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(loopEnd - loopStart);
            auto sleepTime = std::chrono::microseconds(static_cast<int>(dt * 1000000)) - elapsed;
            
            if (sleepTime > std::chrono::microseconds(0)) {
                std::this_thread::sleep_for(sleepTime);
            }
        }
    }
    
    void updateSystemCapabilities() {
        switch (currentLevel) {
            case AutonomyLevel::LEVEL_0_NO_AUTOMATION:
                // No autonomous capabilities
                break;
            case AutonomyLevel::LEVEL_1_DRIVER_ASSISTANCE:
                // Basic assistance (ACC, LKA)
                decisionMaker->setBehaviorParameters(0.9, 0.3, 3.0); // Conservative
                break;
            case AutonomyLevel::LEVEL_2_PARTIAL_AUTOMATION:
                // Combined steering and acceleration
                decisionMaker->setBehaviorParameters(0.7, 0.4, 2.5);
                break;
            case AutonomyLevel::LEVEL_3_CONDITIONAL_AUTOMATION:
                // System can handle most scenarios
                decisionMaker->setBehaviorParameters(0.5, 0.5, 2.0);
                break;
            case AutonomyLevel::LEVEL_4_HIGH_AUTOMATION:
                // Full autonomy in defined conditions
                decisionMaker->setBehaviorParameters(0.4, 0.6, 2.0);
                break;
            case AutonomyLevel::LEVEL_5_FULL_AUTOMATION:
                // Full autonomy everywhere
                decisionMaker->setBehaviorParameters(0.3, 0.7, 1.5);
                break;
        }
    }
    
    double calculateScenarioComplexity() {
        double complexity = 0.0;
        
        // Base complexity from object count
        complexity += perceivedObjects.size() * 0.1;
        
        // Increase complexity for dynamic objects
        for (const auto& obj : perceivedObjects) {
            if (obj.type == "vehicle" && obj.vx > 1.0) {
                complexity += 0.15;
            } else if (obj.type == "pedestrian") {
                complexity += 0.2;
            } else if (obj.type == "cyclist") {
                complexity += 0.18;
            }
        }
        
        // Environmental factors (simplified)
        complexity += 0.1; // Base environmental complexity
        
        return std::min(1.0, complexity);
    }
    
    double calculateRiskLevel() {
        double risk = 0.0;
        
        // Check proximity to objects
        for (const auto& obj : perceivedObjects) {
            double distance = sqrt(pow(currentPose.x - obj.x, 2) + pow(currentPose.y - obj.y, 2));
            double safetyDistance = currentPose.speed * 2.0 + 5.0; // 2-second rule + 5m
            
            if (distance < safetyDistance) {
                risk += (safetyDistance - distance) / safetyDistance;
            }
        }
        
        return std::min(1.0, risk);
    }
    
    void executeManeuver(ManeuverType maneuver, const std::vector<Waypoint>& path, double dt) {
        if (path.empty()) return;
        
        // Find target waypoint (look ahead)
        int lookaheadIndex = std::min(static_cast<int>(path.size()) - 1, 
                                     static_cast<int>(currentPose.speed * 0.5)); // 0.5s lookahead
        const Waypoint& targetWaypoint = path[lookaheadIndex];
        
        // Calculate steering command
        double headingError = targetWaypoint.heading - currentPose.heading;
        // Normalize angle to [-π, π]
        while (headingError > M_PI) headingError -= 2 * M_PI;
        while (headingError < -M_PI) headingError += 2 * M_PI;
        
        double steeringCommand = headingError * 0.8; // Proportional steering
        
        // Set control targets
        motionController->setTargets(targetWaypoint.speed, steeringCommand);
        motionController->updateControl(dt);
        
        // Update current pose based on control outputs
        auto controlOutputs = motionController->getControlOutputs();
        currentPose.speed = controlOutputs.first;
        
        // Simple kinematic model update
        currentPose.x += currentPose.speed * cos(currentPose.heading) * dt;
        currentPose.y += currentPose.speed * sin(currentPose.heading) * dt;
        currentPose.heading += currentPose.speed * tan(controlOutputs.second) / 2.8 * dt; // wheelbase = 2.8m
    }
    
    void updateStatistics(double dt) {
        if (systemState == DrivingState::ENGAGED || systemState == DrivingState::MONITORING) {
            totalAutonomousDistance += currentPose.speed * dt;
        }
    }
    
    std::string getAutonomyLevelString(AutonomyLevel level) const {
        switch (level) {
            case AutonomyLevel::LEVEL_0_NO_AUTOMATION: return "Level 0 - No Automation";
            case AutonomyLevel::LEVEL_1_DRIVER_ASSISTANCE: return "Level 1 - Driver Assistance";
            case AutonomyLevel::LEVEL_2_PARTIAL_AUTOMATION: return "Level 2 - Partial Automation";
            case AutonomyLevel::LEVEL_3_CONDITIONAL_AUTOMATION: return "Level 3 - Conditional Automation";
            case AutonomyLevel::LEVEL_4_HIGH_AUTOMATION: return "Level 4 - High Automation";
            case AutonomyLevel::LEVEL_5_FULL_AUTOMATION: return "Level 5 - Full Automation";
            default: return "Unknown";
        }
    }
    
    std::string getSystemStateString(DrivingState state) const {
        switch (state) {
            case DrivingState::MANUAL_CONTROL: return "MANUAL_CONTROL";
            case DrivingState::MONITORING: return "MONITORING";
            case DrivingState::ENGAGED: return "ENGAGED";
            case DrivingState::INTERVENTION_REQUIRED: return "INTERVENTION_REQUIRED";
            case DrivingState::EMERGENCY_STOP: return "EMERGENCY_STOP";
            case DrivingState::SYSTEM_FAULT: return "SYSTEM_FAULT";
            default: return "UNKNOWN";
        }
    }
};

#endif // AUTONOMOUS_DRIVING_H