/**
 * @file motor_control_system.h
 * @author adzetto
 * @brief Motor Control System for Electric Vehicles
 * @version 0.1
 * @date 2025-08-29
 *
 * @copyright Copyright (c) 2025
 *
 * @details This file contains the implementation of the Motor Control System
 * for a modern electric vehicle. It includes features like Field-Oriented Control
 * (FOC), torque control, and speed control for the electric motor.
 */

#ifndef MOTOR_CONTROL_SYSTEM_H
#define MOTOR_CONTROL_SYSTEM_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <memory>

/**
 * @brief Represents the electric motor.
 */
class ElectricMotorController {
public:
    ElectricMotorController() : speed(0.0), torque(0.0) {}

    void set_torque(double requested_torque) {
        // Torque control logic using FOC
        this->torque = requested_torque;
    }

    void set_speed(double requested_speed) {
        // Speed control logic
        this->speed = requested_speed;
    }

    double get_speed() const {
        return speed;
    }

    double get_torque() const {
        return torque;
    }

private:
    double speed;
    double torque;
    // Other motor parameters like currents, voltages, etc.
};

/**
 * @brief Main Motor Control System class.
 */
class MotorControlSystem {
public:
    MotorControlSystem() {
        motor_controller = std::make_unique<ElectricMotorController>();
    }

    void update(double torque_request, double speed_request) {
        motor_controller->set_torque(torque_request);
        motor_controller->set_speed(speed_request);
    }

    ElectricMotorController* get_motor_controller() const {
        return motor_controller.get();
    }

private:
    std::unique_ptr<ElectricMotorController> motor_controller;
};

#endif // MOTOR_CONTROL_SYSTEM_H
