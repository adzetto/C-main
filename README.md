# Comprehensive Electric Vehicle Control System

This repository contains advanced C++ frameworks for electric vehicle systems, including battery management, ADAS, motor control, thermal management, and vehicle diagnostics.

**Author: adzetto**

## System Architecture

This project implements a complete EV control system with the following major components:

### Core Systems

1. **Advanced Battery Management System (BMS)** - `advanced_bms.h`
   - 96-cell battery pack monitoring
   - Active/Passive/Hybrid cell balancing
   - State of Charge (SOC) and State of Health (SOH) estimation
   - Temperature management and fault detection
   - Predictive battery degradation modeling

2. **Advanced Driver Assistance Systems (ADAS)** - `adas_system.h`
   - Adaptive Cruise Control (ACC)
   - Lane Keeping Assist (LKA)
   - Automatic Emergency Braking (AEB)
   - Blind Spot Monitoring (BSM)
   - Traffic Sign Recognition (TSR)
   - Sensor fusion (Radar, LiDAR, Camera)

3. **CAN Bus Communication System** - `can_bus_system.h`
   - Multi-node CAN network simulation
   - Message filtering and priority handling
   - Error detection and diagnostics
   - OBD-II compatible interface

4. **Motor Control System** - `motor_control_system.h`
   - PMSM/BLDC motor control with Field Oriented Control (FOC)
   - Space Vector Modulation (SVM)
   - Regenerative braking system
   - Torque vectoring for AWD vehicles
   - Thermal derating protection

5. **Thermal Management System** - `thermal_management.h`
   - Multi-zone cooling system control
   - HVAC system integration
   - Predictive thermal control
   - Battery, motor, and cabin climate control

6. **Vehicle Diagnostics System** - `vehicle_diagnostics.h`
   - Comprehensive fault detection and isolation
   - OBD-II diagnostic trouble codes (DTCs)
   - Predictive maintenance algorithms
   - Real-time system health monitoring

### Legacy Components

7. **Basic EV Controller** - `ev_controller.cpp`
8. **Charging Station Simulator** - `charging_station.cpp`
9. **Basic Battery Management** - `battery_management.h`

## Quick Start

### Build All Systems

```bash
make all
```

### Run Comprehensive Demo

```bash
make run-demo
```

### Run Test Suite

```bash
make test
```

### Run Real-time Simulation

```bash
make demo
```

### Run Complete Test Suite + Simulation

```bash
make full-test
```

## Detailed Usage

### Individual System Testing

```bash
# Basic EV controller
make run-ev

# Charging station simulation
make run-charging

# Comprehensive system demo (interactive)
make run-demo
```

### Advanced Features

The `ev_system_demo.cpp` provides three operation modes:

1. **Test Suite Mode**: Automated testing of all subsystems
2. **Real-time Simulation**: 10-second vehicle simulation with:
   - Dynamic driving scenarios (acceleration, cruise, braking, maneuvering)
   - Real-time system interactions
   - Fault injection and recovery
   - Performance monitoring

3. **Combined Mode**: Both test suite and simulation

## System Features

### Advanced BMS Features
- **96-cell monitoring** with individual cell balancing
- **Kalman filter** based SOC estimation
- **Thermal management** integration
- **Predictive maintenance** with degradation modeling
- **Emergency shutdown** protection

### ADAS Capabilities
- **Multi-sensor fusion** (Radar + LiDAR + Camera)
- **Real-time object tracking** and collision prediction
- **Adaptive cruise control** with following distance management
- **Lane departure prevention** with steering correction
- **Traffic sign recognition** with speed limit adaptation

### Motor Control Features
- **Field Oriented Control (FOC)** for maximum efficiency
- **Space Vector Modulation** for optimal power delivery
- **Multi-mode regenerative braking** (5 levels)
- **All-wheel drive torque vectoring**
- **Thermal protection** with performance derating

### Thermal Management
- **Multi-zone temperature control** (Battery, Motor, Cabin, Power Electronics)
- **Predictive cooling** based on load forecasting
- **HVAC integration** with range optimization
- **Emergency thermal protection**

### Diagnostics & Monitoring
- **Real-time fault detection** with 50+ diagnostic trouble codes
- **OBD-II compatibility** for standard diagnostic tools
- **Predictive maintenance** with trend analysis
- **Comprehensive logging** and report generation

## Technical Specifications

### Performance Metrics
- **Battery Pack**: 96 cells, 400V nominal, 75kWh capacity
- **Motor**: PMSM, 150kW peak power, 8000 RPM max speed
- **Regenerative Braking**: Up to 50kW recovery power
- **Thermal System**: Multi-loop cooling, -20°C to +60°C operation
- **CAN Bus**: 500 kbit/s, 5+ nodes, real-time messaging

### Safety & Standards
- **ISO 26262** functional safety considerations
- **OBD-II** diagnostic standard compatibility
- **CAN 2.0B** communication protocol
- **Emergency shutdown** systems
- **Fault tolerance** and graceful degradation

## Build Requirements

- **C++11** compatible compiler (GCC 4.8+ recommended)
- **POSIX threads** support (pthread)
- **Make** build system

### Dependencies
- Standard C++ library
- Threading support (std::thread)
- Chrono library for timing
- Mathematical functions (cmath)

## File Structure

```
C++ codes/
├── README.md                    # This file
├── Makefile                     # Build system
├── ev_system_demo.cpp           # Comprehensive demo application
├── advanced_bms.h               # Advanced Battery Management System
├── adas_system.h                # Advanced Driver Assistance Systems
├── can_bus_system.h             # CAN Bus Communication System
├── motor_control_system.h       # Motor Control & Regenerative Braking
├── thermal_management.h         # Thermal Management System
├── vehicle_diagnostics.h        # Diagnostics & Fault Detection
├── ev_controller.cpp            # Basic EV controller (legacy)
├── charging_station.cpp         # Charging station simulator (legacy)
└── battery_management.h         # Basic BMS (legacy)
```

## Development & Testing

### Adding New Features

1. Implement new functionality in appropriate header files
2. Add test cases to `ev_system_demo.cpp`
3. Update CAN message definitions in `can_bus_system.h`
4. Add diagnostic codes to `vehicle_diagnostics.h`
5. Update thermal zones in `thermal_management.h`

### Debugging

```bash
# Build with debug symbols
make clean
CXXFLAGS="-std=c++11 -g -O0" make all

# Run with GDB
gdb ./bin/ev_system_demo
```

### Performance Optimization

The system is designed for real-time operation with:
- **Lock-free** data structures where possible
- **Efficient algorithms** (Kalman filters, PID controllers)
- **Memory pool** management for dynamic allocations
- **Optimized control loops** (1ms to 100ms cycles)

## Integration Examples

### Adding Custom Sensors

```cpp
// Add to vehicle_diagnostics.h
diagnostics->addSensor(11, "Custom Sensor", 0.0, 100.0, 50.0);

// Update in main loop
diagnostics->updateSensorReading(11, customSensorValue);
```

### Custom CAN Messages

```cpp
// Define in can_bus_system.h
constexpr uint32_t CUSTOM_MESSAGE = 0x700;

// Send custom data
canSystem->broadcastMessage(CUSTOM_MESSAGE, data, 8, nodeId);
```

## Troubleshooting

### Common Issues

1. **Compilation Errors**: Ensure C++11 support and pthread linking
2. **Runtime Crashes**: Check sensor value ranges and null pointers
3. **CAN Communication**: Verify node IDs and message formats
4. **Thermal Issues**: Check sensor connections and cooling system

### Diagnostic Tools

```bash
# Check system status
make test

# Generate diagnostic report
./bin/ev_system_demo  # Choose option 1 or 3

# View diagnostic report
cat diagnostic_report.txt
```

## Contributing

When contributing to this project:

1. Follow existing code style and patterns
2. Add comprehensive test cases
3. Update documentation
4. Ensure all tests pass before committing
5. Add appropriate diagnostic codes for new failures

## License & Safety Notice

This code is intended for educational and simulation purposes. For production use in actual vehicles, additional safety validation, testing, and certification would be required according to automotive standards (ISO 26262, etc.).

## Future Enhancements

Planned features:
- **V2X Communication** (Vehicle-to-Everything)
- **Machine Learning** integration for predictive systems
- **Cybersecurity** framework
- **Over-the-Air (OTA)** update capability
- **Advanced energy management** algorithms
- **Integration with charging infrastructure** (smart grid)
