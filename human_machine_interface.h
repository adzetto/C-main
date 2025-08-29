#ifndef HUMAN_MACHINE_INTERFACE_H
#define HUMAN_MACHINE_INTERFACE_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <string>
#include <chrono>
#include <memory>
#include <queue>
#include <sstream>
#include <thread>
#include <mutex>
#include <functional>
#include <cmath>
#include <algorithm>

class HumanMachineInterface {
private:
    enum class DisplayTheme {
        LIGHT,
        DARK,
        AUTO,
        SPORT,
        ECO
    };

    enum class ScreenOrientation {
        PORTRAIT,
        LANDSCAPE,
        AUTO_ROTATE
    };

public:
    enum class AlertType {
        INFO,
        WARNING,
        CRITICAL,
        MAINTENANCE,
        NAVIGATION
    };
private:

    enum class InputMethod {
        TOUCH,
        VOICE,
        GESTURE,
        PHYSICAL_BUTTON,
        STEERING_WHEEL
    };

    struct TouchPoint {
        float x, y;
        float pressure;
        std::chrono::steady_clock::time_point timestamp;
        bool active;
    };

    struct GestureData {
        enum Type {
            SWIPE_LEFT,
            SWIPE_RIGHT,
            SWIPE_UP,
            SWIPE_DOWN,
            PINCH_ZOOM,
            TAP,
            LONG_PRESS,
            TWO_FINGER_TAP
        } type;
        
        float velocity;
        float magnitude;
        std::chrono::steady_clock::time_point timestamp;
    };

    struct VoiceCommand {
        std::string command;
        float confidence;
        std::string language;
        std::vector<std::string> parameters;
        std::chrono::steady_clock::time_point timestamp;
    };

    struct DisplayWidget {
        std::string id;
        std::string type;
        float x, y, width, height;
        bool visible;
        bool enabled;
        std::unordered_map<std::string, float> properties;
        std::function<void()> updateCallback;
    };

    struct AlertMessage {
        std::string id;
        AlertType type;
        std::string title;
        std::string message;
        int priority;
        std::chrono::steady_clock::time_point timestamp;
        std::chrono::milliseconds duration;
        bool acknowledged;
        std::function<void()> actionCallback;
    };

    struct UserProfile {
        std::string userId;
        std::string name;
        DisplayTheme preferredTheme;
        float displayBrightness;
        float audioVolume;
        std::string language;
        std::vector<std::string> favoriteApps;
        std::unordered_map<std::string, float> customSettings;
        bool voiceEnabled;
        bool gestureEnabled;
        std::chrono::steady_clock::time_point lastLogin;
    };

    struct NavigationDisplay {
        struct MapData {
            double centerLat, centerLon;
            float zoomLevel;
            float bearing;
            bool followVehicle;
            std::vector<std::pair<double, double>> route;
            std::pair<double, double> destination;
        } map;

        struct DirectionData {
            std::string nextInstruction;
            float distanceToNext;
            std::string streetName;
            float remainingDistance;
            std::chrono::minutes estimatedTime;
            bool recalculating;
        } directions;

        bool trafficEnabled;
        bool satelliteView;
        float compassHeading;
    };

    struct ClimateDisplay {
        float targetTemperature;
        float currentTemperature;
        float fanSpeed;
        bool autoMode;
        bool acEnabled;
        bool heaterEnabled;
        bool defrostEnabled;
        std::string airDirection;
        float outsideTemperature;
        bool airQualityFilter;
    };

    struct MediaDisplay {
        std::string currentSource;
        std::string trackTitle;
        std::string artist;
        std::string album;
        float playbackPosition;
        float trackDuration;
        bool playing;
        bool shuffle;
        bool repeat;
        float volume;
        std::vector<std::string> playlist;
        int currentTrackIndex;
    };

    struct VehicleStatusDisplay {
        float speed;
        float batteryLevel;
        float range;
        float odometer;
        std::string driveMode;
        bool parkingBrakeActive;
        bool seatbeltFastened;
        bool doorsClosed;
        float tirePressure[4];
        std::string gearPosition;
        float motorTemperature;
        float batteryTemperature;
    };

    struct DiagnosticDisplay {
        std::vector<std::string> activeFaults;
        std::vector<std::string> warningMessages;
        float systemHealth;
        std::chrono::steady_clock::time_point lastUpdate;
        std::unordered_map<std::string, float> sensorReadings;
        bool maintenanceRequired;
        int daysToMaintenance;
    };

public:
    HumanMachineInterface() : 
        currentTheme(DisplayTheme::AUTO),
        screenOrientation(ScreenOrientation::LANDSCAPE),
        displayBrightness(0.8f),
        audioVolume(0.7f),
        currentLanguage("en"),
        touchCalibrated(true),
        voiceRecognitionActive(false),
        gestureRecognitionActive(true),
        nightModeActive(false),
        screenSaverActive(false),
        emergencyModeActive(false),
        driverAttentionMonitoring(true),
        adaptiveUI(true) {
        
        initializeDisplayWidgets();
        initializeUserProfiles();
        initializeVoiceCommands();
        initializeGestureRecognition();
        startUpdateLoop();
        
        std::cout << "HMI System initialized successfully" << std::endl;
    }

    void initializeDisplayWidgets() {
        widgets.clear();
        
        // Speedometer widget
        DisplayWidget speedometer = {
            "speedometer", "gauge", 100, 50, 200, 200, true, true,
            {{"minValue", 0}, {"maxValue", 200}, {"currentValue", 0}, {"redZone", 160}},
            [this]() { updateSpeedometer(); }
        };
        widgets["speedometer"] = speedometer;

        // Battery level widget
        DisplayWidget battery = {
            "battery", "bar", 350, 50, 150, 50, true, true,
            {{"level", 85}, {"charging", 0}, {"range", 250}},
            [this]() { updateBatteryDisplay(); }
        };
        widgets["battery"] = battery;

        // Navigation widget
        DisplayWidget navigation = {
            "navigation", "map", 0, 300, 800, 400, true, true,
            {{"zoom", 15}, {"bearing", 0}, {"traffic", 1}},
            [this]() { updateNavigationDisplay(); }
        };
        widgets["navigation"] = navigation;

        // Climate control widget
        DisplayWidget climate = {
            "climate", "panel", 550, 50, 200, 200, true, true,
            {{"temperature", 22}, {"fanSpeed", 3}, {"autoMode", 1}},
            [this]() { updateClimateDisplay(); }
        };
        widgets["climate"] = climate;

        // Media player widget
        DisplayWidget media = {
            "media", "panel", 250, 750, 300, 100, true, true,
            {{"volume", 70}, {"playing", 0}, {"trackPosition", 0}},
            [this]() { updateMediaDisplay(); }
        };
        widgets["media"] = media;

        // Alert panel widget
        DisplayWidget alerts = {
            "alerts", "list", 600, 750, 200, 100, true, true,
            {{"alertCount", 0}, {"priority", 0}},
            [this]() { updateAlertDisplay(); }
        };
        widgets["alerts"] = alerts;

        std::cout << "Display widgets initialized: " << widgets.size() << " widgets" << std::endl;
    }

    void initializeUserProfiles() {
        // Default driver profile
        UserProfile defaultProfile = {
            "driver1", "Default Driver", DisplayTheme::AUTO, 0.8f, 0.7f, "en",
            {"navigation", "media", "climate"}, {{"seatPosition", 50}}, 
            true, true, std::chrono::steady_clock::now()
        };
        userProfiles["driver1"] = defaultProfile;

        // Sport driver profile
        UserProfile sportProfile = {
            "driver2", "Sport Driver", DisplayTheme::SPORT, 0.9f, 0.8f, "en",
            {"performance", "diagnostics", "media"}, {{"seatPosition", 30}},
            true, true, std::chrono::steady_clock::now()
        };
        userProfiles["driver2"] = sportProfile;

        currentProfile = "driver1";
        std::cout << "User profiles initialized: " << userProfiles.size() << " profiles" << std::endl;
    }

    void initializeVoiceCommands() {
        // Navigation commands
        voiceCommandMap["navigate to"] = [this](const std::vector<std::string>& params) {
            if (!params.empty()) {
                startNavigation(params[0]);
            }
        };

        voiceCommandMap["cancel navigation"] = [this](const std::vector<std::string>& params) {
            cancelNavigation();
        };

        // Climate commands
        voiceCommandMap["set temperature"] = [this](const std::vector<std::string>& params) {
            if (!params.empty()) {
                try {
                    float temp = std::stof(params[0]);
                    setClimateTemperature(temp);
                } catch (const std::exception& e) {
                    showAlert("Invalid temperature value", AlertType::WARNING);
                }
            }
        };

        voiceCommandMap["turn on air conditioning"] = [this](const std::vector<std::string>& params) {
            setClimateAC(true);
        };

        voiceCommandMap["turn off air conditioning"] = [this](const std::vector<std::string>& params) {
            setClimateAC(false);
        };

        // Media commands
        voiceCommandMap["play music"] = [this](const std::vector<std::string>& params) {
            playMedia();
        };

        voiceCommandMap["pause music"] = [this](const std::vector<std::string>& params) {
            pauseMedia();
        };

        voiceCommandMap["next track"] = [this](const std::vector<std::string>& params) {
            nextTrack();
        };

        voiceCommandMap["previous track"] = [this](const std::vector<std::string>& params) {
            previousTrack();
        };

        voiceCommandMap["volume up"] = [this](const std::vector<std::string>& params) {
            adjustVolume(10);
        };

        voiceCommandMap["volume down"] = [this](const std::vector<std::string>& params) {
            adjustVolume(-10);
        };

        // System commands
        voiceCommandMap["show diagnostics"] = [this](const std::vector<std::string>& params) {
            showDiagnostics();
        };

        voiceCommandMap["call emergency"] = [this](const std::vector<std::string>& params) {
            initiateEmergencyCall();
        };

        std::cout << "Voice commands initialized: " << voiceCommandMap.size() << " commands" << std::endl;
    }

    void initializeGestureRecognition() {
        // Swipe gestures
        gestureHandlers[GestureData::SWIPE_LEFT] = [this](const GestureData& gesture) {
            navigateToNextScreen();
        };

        gestureHandlers[GestureData::SWIPE_RIGHT] = [this](const GestureData& gesture) {
            navigateToPreviousScreen();
        };

        gestureHandlers[GestureData::SWIPE_UP] = [this](const GestureData& gesture) {
            showMainMenu();
        };

        gestureHandlers[GestureData::SWIPE_DOWN] = [this](const GestureData& gesture) {
            showNotificationPanel();
        };

        // Pinch zoom
        gestureHandlers[GestureData::PINCH_ZOOM] = [this](const GestureData& gesture) {
            if (currentScreen == "navigation") {
                adjustMapZoom(gesture.magnitude);
            }
        };

        // Two finger tap
        gestureHandlers[GestureData::TWO_FINGER_TAP] = [this](const GestureData& gesture) {
            activateVoiceControl();
        };

        std::cout << "Gesture recognition initialized: " << gestureHandlers.size() << " gestures" << std::endl;
    }

    void processTouch(float x, float y, float pressure) {
        TouchPoint touch = {x, y, pressure, std::chrono::steady_clock::now(), true};
        currentTouches.push_back(touch);

        // Check if touch hits any widget
        for (auto& [widgetId, widget] : widgets) {
            if (widget.visible && widget.enabled &&
                x >= widget.x && x <= widget.x + widget.width &&
                y >= widget.y && y <= widget.y + widget.height) {
                
                handleWidgetTouch(widgetId, x - widget.x, y - widget.y);
                break;
            }
        }

        // Detect gestures from touch patterns
        detectGestures();
    }

    void processVoiceCommand(const std::string& command, float confidence) {
        if (confidence < 0.7f) {
            showAlert("Voice command not recognized", AlertType::INFO);
            return;
        }

        VoiceCommand voiceCmd = {command, confidence, currentLanguage, {}, 
                               std::chrono::steady_clock::now()};

        // Parse command and parameters
        std::vector<std::string> tokens = parseCommand(command);
        if (tokens.empty()) return;

        std::string baseCommand = tokens[0];
        std::vector<std::string> parameters(tokens.begin() + 1, tokens.end());
        voiceCmd.parameters = parameters;

        // Find matching command
        for (const auto& [cmdPattern, handler] : voiceCommandMap) {
            if (command.find(cmdPattern) != std::string::npos) {
                handler(parameters);
                showAlert("Voice command executed", AlertType::INFO);
                return;
            }
        }

        showAlert("Unknown voice command", AlertType::WARNING);
    }

    void updateDisplay() {
        // Update all widgets
        for (auto& [widgetId, widget] : widgets) {
            if (widget.visible && widget.updateCallback) {
                widget.updateCallback();
            }
        }

        // Update navigation display
        updateNavigationInfo();

        // Update climate display
        updateClimateInfo();

        // Update media display
        updateMediaInfo();

        // Update vehicle status
        updateVehicleStatus();

        // Update diagnostics
        updateDiagnosticsInfo();

        // Process pending alerts
        processAlerts();

        // Adjust brightness based on ambient light
        adjustDisplayBrightness();
    }

    void showAlert(const std::string& message, AlertType type, int priority = 1) {
        AlertMessage alert = {
            "alert_" + std::to_string(alertCounter++),
            type,
            getAlertTypeString(type),
            message,
            priority,
            std::chrono::steady_clock::now(),
            std::chrono::milliseconds(5000),
            false,
            nullptr
        };

        alerts.push(alert);

        // Play alert sound based on type
        playAlertSound(type);

        // Update alert widget
        if (widgets.find("alerts") != widgets.end()) {
            widgets["alerts"].properties["alertCount"] = alerts.size();
            widgets["alerts"].properties["priority"] = priority;
        }
    }

    void setUserProfile(const std::string& profileId) {
        if (userProfiles.find(profileId) != userProfiles.end()) {
            currentProfile = profileId;
            const auto& profile = userProfiles[profileId];
            
            currentTheme = profile.preferredTheme;
            displayBrightness = profile.displayBrightness;
            audioVolume = profile.audioVolume;
            currentLanguage = profile.language;
            
            applyTheme();
            std::cout << "User profile switched to: " << profile.name << std::endl;
        }
    }

    void enableEmergencyMode() {
        emergencyModeActive = true;
        currentTheme = DisplayTheme::LIGHT;
        displayBrightness = 1.0f;
        
        // Show only essential widgets
        for (auto& [widgetId, widget] : widgets) {
            if (widgetId == "speedometer" || widgetId == "battery" || 
                widgetId == "alerts" || widgetId == "navigation") {
                widget.visible = true;
            } else {
                widget.visible = false;
            }
        }
        
        showAlert("Emergency Mode Active", AlertType::CRITICAL, 5);
        std::cout << "Emergency mode activated" << std::endl;
    }

    void disableEmergencyMode() {
        emergencyModeActive = false;
        
        // Restore all widgets
        for (auto& [widgetId, widget] : widgets) {
            widget.visible = true;
        }
        
        // Restore user preferences
        if (userProfiles.find(currentProfile) != userProfiles.end()) {
            const auto& profile = userProfiles[currentProfile];
            currentTheme = profile.preferredTheme;
            displayBrightness = profile.displayBrightness;
        }
        
        applyTheme();
        showAlert("Emergency Mode Deactivated", AlertType::INFO);
        std::cout << "Emergency mode deactivated" << std::endl;
    }

    void updateVehicleData(float speed, float batteryLevel, float range, 
                          const std::string& driveMode) {
        vehicleStatus.speed = speed;
        vehicleStatus.batteryLevel = batteryLevel;
        vehicleStatus.range = range;
        vehicleStatus.driveMode = driveMode;

        // Update speedometer widget
        if (widgets.find("speedometer") != widgets.end()) {
            widgets["speedometer"].properties["currentValue"] = speed;
        }

        // Update battery widget
        if (widgets.find("battery") != widgets.end()) {
            widgets["battery"].properties["level"] = batteryLevel;
            widgets["battery"].properties["range"] = range;
        }

        // Check for low battery warning
        if (batteryLevel < 20.0f && !lowBatteryWarningShown) {
            showAlert("Low Battery - " + std::to_string((int)batteryLevel) + "%", 
                     AlertType::WARNING, 3);
            lowBatteryWarningShown = true;
        } else if (batteryLevel >= 20.0f) {
            lowBatteryWarningShown = false;
        }
    }

    void shutdown() {
        std::cout << "Shutting down HMI system..." << std::endl;
        
        // Save user preferences
        saveUserProfiles();
        
        // Clear alerts
        while (!alerts.empty()) {
            alerts.pop();
        }
        
        // Clear widgets
        widgets.clear();
        
        // Stop update loop
        running = false;
        
        std::cout << "HMI system shutdown complete" << std::endl;
    }

private:
    // Display management
    DisplayTheme currentTheme;
    ScreenOrientation screenOrientation;
    float displayBrightness;
    float audioVolume;
    std::string currentLanguage;
    std::string currentScreen;
    bool nightModeActive;
    bool screenSaverActive;
    bool adaptiveUI;

    // Input management
    std::vector<TouchPoint> currentTouches;
    bool touchCalibrated;
    bool voiceRecognitionActive;
    bool gestureRecognitionActive;

    // System state
    bool emergencyModeActive;
    bool driverAttentionMonitoring;
    bool running = true;
    bool lowBatteryWarningShown = false;
    int alertCounter = 0;

    // User management
    std::unordered_map<std::string, UserProfile> userProfiles;
    std::string currentProfile;

    // UI components
    std::unordered_map<std::string, DisplayWidget> widgets;
    std::queue<AlertMessage> alerts;

    // Navigation data
    NavigationDisplay navigationData;

    // Climate data
    ClimateDisplay climateData;

    // Media data
    MediaDisplay mediaData;

    // Vehicle status data
    VehicleStatusDisplay vehicleStatus;

    // Diagnostic data
    DiagnosticDisplay diagnosticData;

    // Input handlers
    std::unordered_map<std::string, std::function<void(const std::vector<std::string>&)>> voiceCommandMap;
    std::unordered_map<GestureData::Type, std::function<void(const GestureData&)>> gestureHandlers;

    void updateSpeedometer() {
        // Speedometer update logic
    }

    void updateBatteryDisplay() {
        // Battery display update logic
    }

    void updateNavigationDisplay() {
        // Navigation display update logic
    }

    void updateClimateDisplay() {
        // Climate display update logic
    }

    void updateMediaDisplay() {
        // Media display update logic
    }

    void updateAlertDisplay() {
        // Alert display update logic
    }

    void handleWidgetTouch(const std::string& widgetId, float localX, float localY) {
        if (widgetId == "speedometer") {
            // Handle speedometer touch
        } else if (widgetId == "battery") {
            showAlert("Battery: " + std::to_string((int)vehicleStatus.batteryLevel) + 
                     "% - Range: " + std::to_string((int)vehicleStatus.range) + " km", 
                     AlertType::INFO);
        } else if (widgetId == "navigation") {
            // Handle navigation touch
            handleNavigationTouch(localX, localY);
        } else if (widgetId == "climate") {
            // Handle climate touch
            handleClimateTouch(localX, localY);
        } else if (widgetId == "media") {
            // Handle media touch
            handleMediaTouch(localX, localY);
        }
    }

    void detectGestures() {
        // Gesture detection logic from touch patterns
    }

    std::vector<std::string> parseCommand(const std::string& command) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream iss(command);
        while (iss >> token) {
            tokens.push_back(token);
        }
        return tokens;
    }

    std::string getAlertTypeString(AlertType type) {
        switch (type) {
            case AlertType::INFO: return "Info";
            case AlertType::WARNING: return "Warning";
            case AlertType::CRITICAL: return "Critical";
            case AlertType::MAINTENANCE: return "Maintenance";
            case AlertType::NAVIGATION: return "Navigation";
        }
        return "Unknown";
    }

    void playAlertSound(AlertType type) {
        // Play appropriate sound for alert type
        switch (type) {
            case AlertType::INFO:
                // Play info sound
                break;
            case AlertType::WARNING:
                // Play warning sound
                break;
            case AlertType::CRITICAL:
                // Play critical alert sound
                break;
            default:
                break;
        }
    }

    void applyTheme() {
        // Apply theme to all widgets based on currentTheme
        switch (currentTheme) {
            case DisplayTheme::LIGHT:
                // Apply light theme
                break;
            case DisplayTheme::DARK:
                // Apply dark theme
                break;
            case DisplayTheme::SPORT:
                // Apply sport theme with red accents
                break;
            case DisplayTheme::ECO:
                // Apply eco theme with green accents
                break;
            case DisplayTheme::AUTO:
                // Apply theme based on time of day
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                auto tm = *std::localtime(&time_t);
                
                if (tm.tm_hour >= 18 || tm.tm_hour <= 6) {
                    // Night time - use dark theme
                    nightModeActive = true;
                } else {
                    // Day time - use light theme
                    nightModeActive = false;
                }
                break;
        }
    }

    void adjustDisplayBrightness() {
        // Auto-adjust brightness based on ambient light and time of day
        if (nightModeActive) {
            displayBrightness = std::max(0.3f, displayBrightness * 0.7f);
        }
    }

    void processAlerts() {
        while (!alerts.empty()) {
            auto& alert = alerts.front();
            auto now = std::chrono::steady_clock::now();
            
            if (now - alert.timestamp > alert.duration) {
                alerts.pop();
            } else {
                break;
            }
        }
    }

    // Navigation functions
    void startNavigation(const std::string& destination) {
        navigationData.directions.nextInstruction = "Starting navigation to " + destination;
        showAlert("Navigation started to " + destination, AlertType::NAVIGATION);
    }

    void cancelNavigation() {
        navigationData.directions.nextInstruction = "";
        showAlert("Navigation cancelled", AlertType::INFO);
    }

    void handleNavigationTouch(float x, float y) {
        // Handle navigation map interaction
    }

    // Climate functions
    void setClimateTemperature(float temperature) {
        climateData.targetTemperature = temperature;
        showAlert("Temperature set to " + std::to_string((int)temperature) + "°C", AlertType::INFO);
    }

    void setClimateAC(bool enabled) {
        climateData.acEnabled = enabled;
        showAlert(enabled ? "Air conditioning on" : "Air conditioning off", AlertType::INFO);
    }

    void handleClimateTouch(float x, float y) {
        // Handle climate control interaction
    }

    // Media functions
    void playMedia() {
        mediaData.playing = true;
        showAlert("Playing: " + mediaData.trackTitle, AlertType::INFO);
    }

    void pauseMedia() {
        mediaData.playing = false;
        showAlert("Media paused", AlertType::INFO);
    }

    void nextTrack() {
        if (mediaData.currentTrackIndex < mediaData.playlist.size() - 1) {
            mediaData.currentTrackIndex++;
            mediaData.trackTitle = mediaData.playlist[mediaData.currentTrackIndex];
            showAlert("Next track: " + mediaData.trackTitle, AlertType::INFO);
        }
    }

    void previousTrack() {
        if (mediaData.currentTrackIndex > 0) {
            mediaData.currentTrackIndex--;
            mediaData.trackTitle = mediaData.playlist[mediaData.currentTrackIndex];
            showAlert("Previous track: " + mediaData.trackTitle, AlertType::INFO);
        }
    }

    void adjustVolume(int delta) {
        mediaData.volume = std::clamp(mediaData.volume + delta, 0.0f, 100.0f);
        showAlert("Volume: " + std::to_string((int)mediaData.volume) + "%", AlertType::INFO);
    }

    void handleMediaTouch(float x, float y) {
        // Handle media player interaction
    }

    // Screen navigation functions
    void navigateToNextScreen() {
        // Navigate to next screen
    }

    void navigateToPreviousScreen() {
        // Navigate to previous screen
    }

    void showMainMenu() {
        currentScreen = "main_menu";
        showAlert("Main menu", AlertType::INFO);
    }

    void showNotificationPanel() {
        currentScreen = "notifications";
        showAlert("Notifications", AlertType::INFO);
    }

    void adjustMapZoom(float magnitude) {
        navigationData.map.zoomLevel += magnitude * 0.1f;
        navigationData.map.zoomLevel = std::clamp(navigationData.map.zoomLevel, 1.0f, 20.0f);
    }

    void activateVoiceControl() {
        voiceRecognitionActive = true;
        showAlert("Voice control activated - Speak now", AlertType::INFO);
    }

    // System functions
    void showDiagnostics() {
        currentScreen = "diagnostics";
        showAlert("Diagnostics displayed", AlertType::INFO);
    }

    void initiateEmergencyCall() {
        enableEmergencyMode();
        showAlert("Emergency call initiated", AlertType::CRITICAL, 5);
    }

    void updateNavigationInfo() {
        // Update navigation information
    }

    void updateClimateInfo() {
        // Update climate information
    }

    void updateMediaInfo() {
        // Update media information
    }

    void updateVehicleStatus() {
        // Update vehicle status information
    }

    void updateDiagnosticsInfo() {
        // Update diagnostics information
    }

    void saveUserProfiles() {
        // Save user profiles to persistent storage
        std::cout << "User profiles saved" << std::endl;
    }

    void startUpdateLoop() {
        std::thread([this]() {
            while (running) {
                updateDisplay();
                std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20 FPS
            }
        }).detach();
    }
};

class VehicleHMIController {
public:
    VehicleHMIController() : hmi(std::make_unique<HumanMachineInterface>()) {
        initializeHMI();
        std::cout << "Vehicle HMI Controller initialized" << std::endl;
    }

    void simulateHMIOperation() {
        std::cout << "\n=== HMI System Simulation ===" << std::endl;
        
        // Simulate various interactions
        simulateUserInteractions();
        simulateVehicleDataUpdates();
        simulateVoiceCommands();
        simulateGestureInputs();
        simulateAlerts();
        
        std::cout << "HMI simulation completed successfully" << std::endl;
    }

    void shutdown() {
        if (hmi) {
            hmi->shutdown();
        }
        std::cout << "Vehicle HMI Controller shutdown complete" << std::endl;
    }

private:
    std::unique_ptr<HumanMachineInterface> hmi;

    void initializeHMI() {
        // Initialize with default settings
        hmi->setUserProfile("driver1");
        
        // Set initial vehicle data
        hmi->updateVehicleData(0.0f, 85.0f, 250.0f, "ECO");
        
        std::cout << "HMI initialized with default settings" << std::endl;
    }

    void simulateUserInteractions() {
        std::cout << "\nSimulating touch interactions..." << std::endl;
        
        // Simulate touching battery widget
        hmi->processTouch(425, 75, 1.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Simulate touching navigation area
        hmi->processTouch(400, 500, 1.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        std::cout << "Touch interactions simulated" << std::endl;
    }

    void simulateVehicleDataUpdates() {
        std::cout << "\nSimulating vehicle data updates..." << std::endl;
        
        // Simulate driving scenario
        for (int i = 0; i < 5; i++) {
            float speed = 30.0f + i * 20.0f;
            float battery = 85.0f - i * 5.0f;
            float range = 250.0f - i * 20.0f;
            
            hmi->updateVehicleData(speed, battery, range, "NORMAL");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        
        std::cout << "Vehicle data updates simulated" << std::endl;
    }

    void simulateVoiceCommands() {
        std::cout << "\nSimulating voice commands..." << std::endl;
        
        // Simulate various voice commands
        hmi->processVoiceCommand("navigate to home", 0.95f);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        hmi->processVoiceCommand("set temperature 24", 0.90f);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        hmi->processVoiceCommand("play music", 0.88f);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        hmi->processVoiceCommand("volume up", 0.92f);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "Voice commands simulated" << std::endl;
    }

    void simulateGestureInputs() {
        std::cout << "\nSimulating gesture inputs..." << std::endl;
        
        // Note: Gesture simulation would require more complex touch pattern simulation
        // For demo purposes, we're showing the concept
        
        std::cout << "Gesture inputs simulated" << std::endl;
    }

    void simulateAlerts() {
        std::cout << "\nSimulating system alerts..." << std::endl;
        
        hmi->showAlert("Welcome to your vehicle", HumanMachineInterface::AlertType::INFO);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        hmi->showAlert("Maintenance due in 500 km", HumanMachineInterface::AlertType::MAINTENANCE);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Simulate low battery warning
        hmi->updateVehicleData(60.0f, 15.0f, 40.0f, "ECO+");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        std::cout << "System alerts simulated" << std::endl;
    }
};

#endif // HUMAN_MACHINE_INTERFACE_H
