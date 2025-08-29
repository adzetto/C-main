CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2
TARGET_DIR = bin
TARGETS = ev_controller charging_station

.PHONY: all clean

all: $(TARGET_DIR) $(addprefix $(TARGET_DIR)/, $(TARGETS))

$(TARGET_DIR):
	mkdir -p $(TARGET_DIR)

$(TARGET_DIR)/ev_controller: ev_controller.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

$(TARGET_DIR)/charging_station: charging_station.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -rf $(TARGET_DIR)

run-ev: $(TARGET_DIR)/ev_controller
	./$(TARGET_DIR)/ev_controller

run-charging: $(TARGET_DIR)/charging_station
	./$(TARGET_DIR)/charging_station

install:
	sudo cp $(TARGET_DIR)/* /usr/local/bin/