# Electric Vehicle Control System

This repository contains C++ code for simulating and controlling electric vehicles, including battery management and charging station operations.

## Files

### 1. ev_controller.cpp
Main electric vehicle controller class that handles:
- Engine start/stop operations
- Speed control and acceleration
- Battery level monitoring
- Charging functionality
- Vehicle status display

### 2. battery_management.h
Battery Management System (BMS) header file containing:
- Individual battery cell monitoring
- Cell balancing algorithms
- Temperature monitoring
- Battery degradation simulation
- Range calculation

### 3. charging_station.cpp
Charging station simulator with features:
- Multiple charging types (Slow, Fast, Rapid, Ultra Rapid)
- Charging time calculation
- Power delivery simulation
- Charging network management

## Compilation

To compile the programs:

```bash
# Compile main EV controller
g++ -std=c++11 -o ev_controller ev_controller.cpp

# Compile charging station simulator
g++ -std=c++11 -o charging_station charging_station.cpp

# For battery management, include the header in your main program
```

## Usage

```bash
# Run EV controller
./ev_controller

# Run charging station simulator
./charging_station
```

## Features

- **Real-time battery monitoring**
- **Multi-threaded charging simulation**
- **Temperature-based safety controls**
- **Cell balancing algorithms**
- **Power management system**
- **Charging network simulation**

## Safety Features

- Low battery warnings
- Temperature monitoring
- Overspeed protection
- Cell health monitoring
- Automatic charging cutoff