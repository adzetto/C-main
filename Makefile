CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -pthread
TARGET_DIR = bin
TARGETS = ev_controller charging_station ev_system_demo advanced_utilities_demo realtime_monitor_demo simulation_demo

.PHONY: all clean test

all: $(TARGET_DIR) $(addprefix $(TARGET_DIR)/, $(TARGETS))

$(TARGET_DIR):
	mkdir -p $(TARGET_DIR)

$(TARGET_DIR)/ev_controller: ev_controller.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

$(TARGET_DIR)/charging_station: charging_station.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

$(TARGET_DIR)/ev_system_demo: ev_system_demo.cpp adas_system.h advanced_bms.h advanced_powertrain_control.h advanced_vehicle_dynamics.h autonomous_driving.h battery_management.h can_bus_system.h charging_station.cpp energy_management.h ev_controller.cpp human_machine_interface.h machine_learning_engine.h motor_control_system.h thermal_management.h vehicle_connectivity.h vehicle_cybersecurity.h vehicle_diagnostics.h
	$(CXX) $(CXXFLAGS) -o $@ $<

# Advanced utilities demo
$(TARGET_DIR)/advanced_utilities_demo: advanced_utilities_demo.cpp advanced_utilities_stubs.cpp advanced_data_analytics.h advanced_network_stack.h advanced_cryptographic_security.h advanced_signal_processing.h advanced_memory_management.h advanced_configuration_management.h advanced_logging_framework.h
	$(CXX) $(CXXFLAGS) -o $@ advanced_utilities_demo.cpp advanced_utilities_stubs.cpp

# Real-time system monitor demo
$(TARGET_DIR)/realtime_monitor_demo: realtime_monitor_demo.cpp realtime_system_monitor.h
	$(CXX) $(CXXFLAGS) -o $@ $<

# Simulation toolkit demo
$(TARGET_DIR)/simulation_demo: simulation_demo.cpp simulation_toolkit.h
	$(CXX) $(CXXFLAGS) -o $@ simulation_demo.cpp

clean:
	rm -rf $(TARGET_DIR)
	rm -f diagnostic_report.txt

run-ev: $(TARGET_DIR)/ev_controller
	./$(TARGET_DIR)/ev_controller

run-charging: $(TARGET_DIR)/charging_station
	./$(TARGET_DIR)/charging_station

run-demo: $(TARGET_DIR)/ev_system_demo
	./$(TARGET_DIR)/ev_system_demo

run-simulation: $(TARGET_DIR)/simulation_demo
	./$(TARGET_DIR)/simulation_demo

run-utilities: $(TARGET_DIR)/advanced_utilities_demo
	./$(TARGET_DIR)/advanced_utilities_demo

run-monitor: $(TARGET_DIR)/realtime_monitor_demo
	./$(TARGET_DIR)/realtime_monitor_demo

test: $(TARGET_DIR)/ev_system_demo
	@echo "Running comprehensive EV system test suite..."
	@echo "1" | ./$(TARGET_DIR)/ev_system_demo

demo: $(TARGET_DIR)/ev_system_demo
	@echo "Running real-time EV system simulation..."
	@echo "2" | ./$(TARGET_DIR)/ev_system_demo

full-test: $(TARGET_DIR)/ev_system_demo
	@echo "Running complete test suite and simulation..."
	@echo "3" | ./$(TARGET_DIR)/ev_system_demo

install:
	sudo cp $(TARGET_DIR)/* /usr/local/bin/

test-utilities: $(TARGET_DIR)/advanced_utilities_demo
	@echo "Running advanced utilities test suite..."
	./$(TARGET_DIR)/advanced_utilities_demo

test-monitor: $(TARGET_DIR)/realtime_monitor_demo
	@echo "Running real-time system monitor test..."
	./$(TARGET_DIR)/realtime_monitor_demo

test-simulation: $(TARGET_DIR)/simulation_demo
	@echo "Running simulation toolkit test..."
	./$(TARGET_DIR)/simulation_demo

help:
	@echo "Available targets:"
	@echo "  all          - Build all programs"
	@echo "  clean        - Remove build artifacts"
	@echo "  run-ev       - Run basic EV controller"
	@echo "  run-charging - Run charging station simulator"
	@echo "  run-demo     - Run comprehensive EV system demo"
	@echo "  run-simulation - Run simulation demo"
	@echo "  run-utilities  - Run advanced utilities demo"
	@echo "  run-monitor    - Run real-time system monitor demo"
	@echo "  test         - Run automated test suite"
	@echo "  test-utilities - Run advanced utilities test suite"
	@echo "  test-monitor   - Run real-time system monitor test"
	@echo "  test-simulation - Run simulation toolkit test"
	@echo "  demo         - Run real-time simulation"
	@echo "  full-test    - Run both test suite and simulation"
	@echo "  install      - Install binaries to system"
	@echo "  help         - Show this help message"
