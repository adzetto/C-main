/**
 * @file simulation_toolkit.h
 * @author adzetto
 * @brief Advanced Simulation Toolkit for Electric Vehicle Systems
 * @version 1.0
 * @date 2025-08-31
 *
 * @copyright Copyright (c) 2025
 *
 * @details This toolkit provides a comprehensive framework for creating, running, 
 *          and analyzing simulations of electric vehicle components and systems. It
 *          supports discrete-time simulation, event-driven simulation, and various
 *          modeling techniques for key EV subsystems like battery, motor, and powertrain.
 */

#ifndef SIMULATION_TOOLKIT_H
#define SIMULATION_TOOLKIT_H

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <map>
#include <functional>
#include <chrono>
#include <thread>
#include <random>
#include <algorithm>
#include <stdexcept>
#include <limits>

namespace simulation {

// Forward declarations
class SimulationModel;
class SimulationEngine;
class EventScheduler;
class DataLogger;
class ReportGenerator;

/**
 * @brief Represents the state of a simulation at a given time.
 */
struct SimulationState {
    double currentTime = 0.0;
    std::map<std::string, double> variables;
    std::string statusMessage = "Initialized";
};

/**
 * @brief Base class for all simulation models.
 *
 * Defines the interface for models that can be integrated into the simulation engine.
 */
class SimulationModel {
public:
    virtual ~SimulationModel() = default;

    /**
     * @brief Gets the name of the model.
     * @return The model's name.
     */
    virtual std::string getName() const = 0;

    /**
     * @brief Initializes the model at the start of a simulation.
     * @param initialState The initial state of the simulation.
     */
    virtual void initialize(const SimulationState& initialState) = 0;

    /**
     * @brief Updates the model's state for a given time step.
     * @param state The current simulation state.
     * @param dt The time step duration.
     */
    virtual void update(SimulationState& state, double dt) = 0;

    /**
     * @brief Provides the current state of the model as a map of key-value pairs.
     * @return A map representing the model's state.
     */
    virtual std::map<std::string, double> getModelData() const = 0;
};

/**
 * @brief Manages the scheduling and execution of events in an event-driven simulation.
 */
class EventScheduler {
public:
    using EventAction = std::function<void(SimulationState&)>;

    struct ScheduledEvent {
        double time;
        int priority;
        EventAction action;

        bool operator>(const ScheduledEvent& other) const {
            if (time != other.time) {
                return time > other.time;
            }
            return priority > other.priority;
        }
    };

    /**
     * @brief Schedules a new event.
     * @param time The simulation time at which the event should occur.
     * @param action The action to perform when the event is triggered.
     * @param priority The priority of the event (lower value = higher priority).
     */
    void schedule(double time, EventAction action, int priority = 10) {
        events.push_back({time, priority, action});
        std::push_heap(events.begin(), events.end(), std::greater<ScheduledEvent>());
    }

    /**
     * @brief Gets the time of the next scheduled event.
     * @return The time of the next event. Returns infinity if no events are scheduled.
     */
    double getNextEventTime() const {
        return events.empty() ? std::numeric_limits<double>::infinity() : events.front().time;
    }

    /**
     * @brief Processes the next event in the queue.
     * @param state The current simulation state.
     */
    void processNextEvent(SimulationState& state) {
        if (events.empty()) return;

        std::pop_heap(events.begin(), events.end(), std::greater<ScheduledEvent>());
        ScheduledEvent event = events.back();
        events.pop_back();

        state.currentTime = event.time;
        event.action(state);
    }

    /**
     * @brief Checks if there are any pending events.
     * @return True if there are events in the queue, false otherwise.
     */
    bool hasEvents() const {
        return !events.empty();
    }

private:
    std::vector<ScheduledEvent> events;
};

/**
 * @brief Logs simulation data for analysis.
 */
class DataLogger {
public:
    /**
     * @brief Initializes the logger with the data fields to be recorded.
     * @param fields A vector of strings representing the data column headers.
     */
    void initialize(const std::vector<std::string>& fields) {
        headers = fields;
        logData.clear();
        logData.push_back(headers);
    }

    /**
     * @brief Logs a new row of data.
     * @param time The simulation time for this data point.
     * @param data A map of data variable names to their values.
     */
    void log(double time, const std::map<std::string, double>& data) {
        std::vector<std::string> row;
        row.push_back(std::to_string(time));
        for (size_t i = 1; i < headers.size(); ++i) {
            auto it = data.find(headers[i]);
            if (it != data.end()) {
                row.push_back(std::to_string(it->second));
            } else {
                row.push_back("N/A");
            }
        }
        logData.push_back(row);
    }

    /**
     * @brief Exports the logged data to a CSV file.
     * @param filename The name of the file to export to.
     */
    void exportToCSV(const std::string& filename) const {
        // In a real implementation, this would write to a file.
        // For this demo, we print to console.
        std::cout << "\n--- Exporting log data to " << filename << " ---\
";
        for (const auto& row : logData) {
            for (size_t i = 0; i < row.size(); ++i) {
                std::cout << row[i] << (i == row.size() - 1 ? "" : ",");
            }
            std::cout << "\n";
        }
    }

private:
    std::vector<std::string> headers;
    std::vector<std::vector<std::string>> logData;
};

/**
 * @brief The main simulation engine.
 *
 * Manages the simulation loop, models, and other components.
 */
class SimulationEngine {
public:
    SimulationEngine() {
        scheduler = std::make_unique<EventScheduler>();
        logger = std::make_unique<DataLogger>();
    }

    /**
     * @brief Adds a simulation model to the engine.
     * @param model A shared pointer to the model.
     */
    void addModel(std::shared_ptr<SimulationModel> model) {
        models.push_back(model);
    }

    /**
     * @brief Runs the simulation for a specified duration and time step.
     * @param duration The total simulation time.
     * @param dt The time step for discrete-time updates.
     */
    void run(double duration, double dt) {
        initialize();

        std::cout << "Starting simulation...\n";
        std::cout << "Duration: " << duration << "s, Time Step: " << dt << "s\n";

        while (state.currentTime < duration) {
            double nextDiscreteTime = state.currentTime + dt;
            double nextEventTime = scheduler->getNextEventTime();

            if (nextDiscreteTime < nextEventTime) {
                state.currentTime = nextDiscreteTime;
                updateModels(dt);
            } else {
                scheduler->processNextEvent(state);
            }

            logData();
            printProgress(duration);
        }

        finalize();
    }

    /**
     * @brief Gets a pointer to the event scheduler.
     * @return A pointer to the EventScheduler instance.
     */
    EventScheduler* getScheduler() {
        return scheduler.get();
    }

private:
    void initialize() {
        state = SimulationState();
        for (auto& model : models) {
            model->initialize(state);
        }

        std::vector<std::string> logHeaders = {"Time"};
        for (auto& model : models) {
            auto modelData = model->getModelData();
            for (const auto& pair : modelData) {
                logHeaders.push_back(pair.first);
            }
        }
        logger->initialize(logHeaders);
    }

    void updateModels(double dt) {
        for (auto& model : models) {
            model->update(state, dt);
        }
    }

    void logData() {
        std::map<std::string, double> allData;
        for (auto& model : models) {
            auto modelData = model->getModelData();
            allData.insert(modelData.begin(), modelData.end());
        }
        logger->log(state.currentTime, allData);
    }

    void printProgress(double duration) {
        int progress = static_cast<int>((state.currentTime / duration) * 100);
        std::cout << "\rProgress: [" << std::string(progress / 5, '#') << std::string(20 - progress / 5, ' ') << "] "
                  << progress << "%";
        std::cout.flush();
    }

    void finalize() {
        std::cout << "\nSimulation finished at time " << state.currentTime << "s\n";
        logger->exportToCSV("simulation_log.csv");
    }

    SimulationState state;
    std::vector<std::shared_ptr<SimulationModel>> models;
    std::unique_ptr<EventScheduler> scheduler;
    std::unique_ptr<DataLogger> logger;
};


// --- Example Simulation Models for an Electric Vehicle ---

/**
 * @brief A simple model for an EV battery.
 */
class BatteryModel : public SimulationModel {
public:
    BatteryModel(double capacity_kWh, double initialSoC) 
        : capacity(capacity_kWh), soc(initialSoC) {}

    std::string getName() const override { return "Battery"; }

    void initialize(const SimulationState& initialState) override {
        soc = 100.0;
        voltage = 400.0;
        current = 0.0;
    }

    void update(SimulationState& state, double dt) override {
        // Get power draw from other components (negative for charging)
        double powerDraw_kW = state.variables["motor_power_draw"] + state.variables["aux_power_draw"];
        
        current = powerDraw_kW / (voltage / 1000.0);
        double energyChange_kWh = powerDraw_kW * (dt / 3600.0);
        
        soc -= (energyChange_kWh / capacity) * 100.0;
        soc = std::max(0.0, std::min(100.0, soc));

        // Update voltage based on SoC (simplified)
        voltage = 350.0 + (soc / 100.0) * 50.0;
    }

    std::map<std::string, double> getModelData() const override {
        return {
            {"battery_soc", soc},
            {"battery_voltage", voltage},
            {"battery_current", current}
        };
    }

private:
    double capacity; // in kWh
    double soc;      // in percent
    double voltage;  // in Volts
    double current;  // in Amps
};

/**
 * @brief A simple model for an EV motor and powertrain.
 */
class PowertrainModel : public SimulationModel {
public:
    std::string getName() const override { return "Powertrain"; }

    void initialize(const SimulationState& initialState) override {
        speed_kmh = 0.0;
        power_draw_kW = 0.0;
        throttle_pos = 0.0;
    }

    void update(SimulationState& state, double dt) override {
        // Get throttle position from a driver model or scenario
        throttle_pos = state.variables["throttle_position"];

        // Simplified physics
        double motor_torque = throttle_pos * 250.0; // Max torque 250 Nm
        double wheel_rpm = speed_kmh * 10.0; // Simplified relation
        
        power_draw_kW = (motor_torque * wheel_rpm) / 9549.0; // Power in kW
        
        // Add regenerative braking effect
        if (throttle_pos < 0) {
            power_draw_kW *= 0.8; // 80% regen efficiency
        }

        // Update vehicle speed based on power (very simplified)
        double acceleration = (power_draw_kW / 50.0) - (speed_kmh * 0.01); // Simplified dynamics
        speed_kmh += acceleration * dt;
        speed_kmh = std::max(0.0, speed_kmh);

        state.variables["motor_power_draw"] = power_draw_kW;
    }

    std::map<std::string, double> getModelData() const override {
        return {
            {"vehicle_speed", speed_kmh},
            {"motor_power_draw", power_draw_kW},
            {"throttle_position", throttle_pos}
        };
    }

private:
    double speed_kmh;
    double power_draw_kW;
    double throttle_pos; // 0-1 for acceleration, <0 for braking
};

/**
 * @brief A model for auxiliary systems (HVAC, infotainment).
 */
class AuxSystemsModel : public SimulationModel {
public:
    std::string getName() const override { return "AuxiliarySystems"; }

    void initialize(const SimulationState& initialState) override {
        power_draw_kW = 0.5; // Constant baseline draw
    }

    void update(SimulationState& state, double dt) override {
        // Power draw could be varied by events (e.g., user turns on AC)
        state.variables["aux_power_draw"] = power_draw_kW;
    }

    std::map<std::string, double> getModelData() const override {
        return {{"aux_power_draw", power_draw_kW}};
    }

private:
    double power_draw_kW;
};

/**
 * @brief A scenario manager that controls the simulation via events.
 */
class ScenarioManager {
public:
    ScenarioManager(EventScheduler* scheduler) : scheduler(scheduler) {}

    void loadDrivingCycle() {
        std::cout << "Loading simple driving cycle scenario...\n";

        // Schedule a series of throttle changes
        scheduler->schedule(5.0, [](SimulationState& s){ s.variables["throttle_position"] = 0.8; std::cout << "\nEvent: Accelerate\n"; });
        scheduler->schedule(20.0, [](SimulationState& s){ s.variables["throttle_position"] = 0.2; std::cout << "\nEvent: Cruise\n"; });
        scheduler->schedule(40.0, [](SimulationState& s){ s.variables["throttle_position"] = -0.5; std::cout << "\nEvent: Brake\n"; });
        scheduler->schedule(45.0, [](SimulationState& s){ s.variables["throttle_position"] = 0.0; std::cout << "\nEvent: Stop\n"; });
        scheduler->schedule(50.0, [](SimulationState& s){ s.variables["throttle_position"] = 1.0; std::cout << "\nEvent: Full Throttle\n"; });
        scheduler->schedule(60.0, [](SimulationState& s){ s.variables["throttle_position"] = 0.0; std::cout << "\nEvent: Coast\n"; });
    }

private:
    EventScheduler* scheduler;
};

} // namespace simulation

#endif // SIMULATION_TOOLKIT_H
