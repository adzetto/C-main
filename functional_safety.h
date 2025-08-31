/**
 * @file functional_safety.h
 * @author adzetto
 * @brief Functional Safety (ISO 26262) Framework for Critical EV Systems
 * @version 1.0
 * @date 2025-08-31
 *
 * @copyright Copyright (c) 2025
 *
 * @details This module provides a framework for implementing functional safety concepts
 *          in line with ISO 26262. It includes mechanisms for safety-critical operations,
 *          error handling, and state management to ensure the vehicle operates safely
 *          even in the event of a fault.
 */

#ifndef FUNCTIONAL_SAFETY_H
#define FUNCTIONAL_SAFETY_H

#include <iostream>
#include <string>
#include <functional>
#include <stdexcept>
#include <map>
#include <mutex>

namespace functional_safety {

/**
 * @brief Automotive Safety Integrity Level (ASIL).
 */
enum class ASIL {
    QM,      // Quality Management (no safety requirements)
    A,       // Lowest level of hazard
    B,
    C,
    D        // Highest level of hazard
};

/**
 * @brief Represents the current safety state of a system.
 */
enum class SafetyState {
    NOMINAL,         // Normal operation
    DEGRADED,        // Functionality is limited
    SAFE_STATE,      // System is in a safe, minimal-function state
    FAULT            // A critical fault has occurred
};

/**
 * @brief A wrapper for safety-critical values.
 * 
 * This template class provides a simple mechanism for redundant storage and checking
 * of critical data, a key concept in functional safety.
 */
template<typename T>
class SafetyCriticalValue {
public:
    SafetyCriticalValue(T val) : value(val), value_inverse(~val) {}

    /**
     * @brief Sets a new value, storing it and its bitwise inverse.
     * @param new_val The new value to set.
     */
    void set(T new_val) {
        std::lock_guard<std::mutex> lock(mtx);
        value = new_val;
        value_inverse = ~new_val;
    }

    /**
     * @brief Gets the value after performing a consistency check.
     * @param out_val Reference to store the retrieved value.
     * @return True if the value is consistent, false otherwise.
     */
    bool get(T& out_val) const {
        std::lock_guard<std::mutex> lock(mtx);
        if ((value ^ ~value_inverse) == 0) {
            out_val = value;
            return true;
        }
        return false;
    }

private:
    T value;
    T value_inverse;
    mutable std::mutex mtx;
};

/**
 * @brief Manages the safety lifecycle of a critical system.
 */
class SafetyManager {
public:
    using ErrorHandler = std::function<void(const std::string&)>;

    SafetyManager(const std::string& system_name, ASIL level)
        : system_name(system_name), asil_level(level), state(SafetyState::NOMINAL) {}

    /**
     * @brief Executes a safety-critical function.
     * 
     * The manager will check the system state before execution and handle exceptions.
     * @param func The function to execute.
     * @param required_level The minimum ASIL level required for this operation.
     */
    void execute(std::function<void()> func, ASIL required_level) {
        if (asil_level < required_level) {
            reportError("Attempted to execute operation with insufficient ASIL level.");
            return;
        }

        if (state != SafetyState::NOMINAL) {
            reportError("Cannot execute function in a non-nominal state.");
            return;
        }

        try {
            func();
        } catch (const std::exception& e) {
            std::string error_msg = "Exception caught during critical execution: ";
            error_msg += e.what();
            reportError(error_msg);
            enterSafeState();
        }
    }

    /**
     * @brief Reports an error and transitions the system state if necessary.
     * @param error_message A description of the error.
     */
    void reportError(const std::string& error_message) {
        std::cerr << "[SafetyManager: " << system_name << "] ERROR: " << error_message << std::endl;
        
        // Simple state transition logic based on error
        if (state == SafetyState::NOMINAL) {
            state = SafetyState::DEGRADED;
        } else if (state == SafetyState::DEGRADED) {
            state = SafetyState::SAFE_STATE;
        }

        if (error_handler) {
            error_handler(error_message);
        }
    }

    /**
     * @brief Manually forces the system into a safe state.
     */
    void enterSafeState() {
        std::cout << "[SafetyManager: " << system_name << "] Entering SAFE_STATE.\n";
        state = SafetyState::SAFE_STATE;
    }

    /**
     * @brief Sets a custom error handler.
     * @param handler The function to call when an error is reported.
     */
    void setErrorHandler(ErrorHandler handler) {
        error_handler = handler;
    }

    /**
     * @brief Gets the current safety state of the system.
     * @return The current SafetyState.
     */
    SafetyState getSafetyState() const {
        return state;
    }

private:
    std::string system_name;
    ASIL asil_level;
    std::atomic<SafetyState> state;
    ErrorHandler error_handler;
};

} // namespace functional_safety

#endif // FUNCTIONAL_SAFETY_H