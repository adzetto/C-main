/**
 * @file cabin_assistant.h
 * @author adzetto
 * @brief Advanced Cabin Assistant and Infotainment System
 * @version 1.0
 * @date 2025-08-31
 *
 * @copyright Copyright (c) 2025
 *
 * @details This module provides a comprehensive framework for an intelligent cabin assistant,
 *          integrating comfort control, infotainment, and user interaction features.
 *          It simulates voice commands, environmental adjustments, and media playback.
 */

#ifndef CABIN_ASSISTANT_H
#define CABIN_ASSISTANT_H

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <chrono>
#include <thread>
#include <atomic>

namespace cabin_assistant {

/**
 * @brief Represents different cabin zones.
 */
enum class CabinZone { DRIVER, PASSENGER_FRONT, PASSENGER_REAR_LEFT, PASSENGER_REAR_RIGHT, ALL };

/**
 * @brief Represents different media types.
 */
enum class MediaType { AUDIO, VIDEO, NAVIGATION, NONE };

/**
 * @brief Manages environmental controls (temperature, fan speed).
 */
class EnvironmentalControl {
public:
    EnvironmentalControl() : temperature_c(22.0), fan_speed_level(3) {}

    void setTemperature(CabinZone zone, double temp) {
        std::cout << "[EnvControl] Setting temperature in " << zoneToString(zone) << " to " << temp << " C\n";
        temperature_c = temp;
    }

    void setFanSpeed(CabinZone zone, int level) {
        std::cout << "[EnvControl] Setting fan speed in " << zoneToString(zone) << " to level " << level << "\n";
        fan_speed_level = level;
    }

    double getTemperature() const { return temperature_c; }
    int getFanSpeed() const { return fan_speed_level; }

private:
    std::string zoneToString(CabinZone zone) {
        switch (zone) {
            case CabinZone::DRIVER: return "Driver Zone";
            case CabinZone::PASSENGER_FRONT: return "Front Passenger Zone";
            case CabinZone::PASSENGER_REAR_LEFT: return "Rear Left Zone";
            case CabinZone::PASSENGER_REAR_RIGHT: return "Rear Right Zone";
            case CabinZone::ALL: return "All Zones";
        }
        return "Unknown Zone";
    }

    double temperature_c;
    int fan_speed_level;
};

/**
 * @brief Manages media playback and infotainment.
 */
class InfotainmentSystem {
public:
    InfotainmentSystem() : current_media_type(MediaType::NONE), volume(50) {}

    void playMedia(MediaType type, const std::string& content_id) {
        std::cout << "[Infotainment] Playing " << mediaTypeToString(type) << ": " << content_id << "\n";
        current_media_type = type;
        current_content = content_id;
    }

    void setVolume(int vol) {
        std::cout << "[Infotainment] Setting volume to " << vol << "\n";
        volume = vol;
    }

    MediaType getCurrentMediaType() const { return current_media_type; }
    std::string getCurrentContent() const { return current_content; }

private:
    std::string mediaTypeToString(MediaType type) {
        switch (type) {
            case MediaType::AUDIO: return "Audio";
            case MediaType::VIDEO: return "Video";
            case MediaType::NAVIGATION: return "Navigation";
            case MediaType::NONE: return "None";
        }
        return "Unknown Media Type";
    }

    MediaType current_media_type;
    std::string current_content;
    int volume;
};

/**
 * @brief The main Cabin Assistant, integrating various cabin functions.
 */
class CabinAssistant {
public:
    CabinAssistant()
        : env_control(std::make_unique<EnvironmentalControl>()),
          infotainment(std::make_unique<InfotainmentSystem>()) {}

    /**
     * @brief Processes a voice command.
     * @param command The voice command string.
     */
    void processVoiceCommand(const std::string& command) {
        std::cout << "\n[CabinAssistant] Processing voice command: \"" << command << "\"\n";
        if (command.find("set temperature to") != std::string::npos) {
            double temp = std::stod(command.substr(command.find("to") + 3));
            env_control->setTemperature(CabinZone::ALL, temp);
        } else if (command.find("play music") != std::string::npos) {
            infotainment->playMedia(MediaType::AUDIO, "Favorite Playlist");
        } else if (command.find("navigate to") != std::string::npos) {
            std::string destination = command.substr(command.find("to") + 3);
            infotainment->playMedia(MediaType::NAVIGATION, destination);
        } else if (command.find("increase fan speed") != std::string::npos) {
            env_control->setFanSpeed(CabinZone::ALL, env_control->getFanSpeed() + 1);
        } else {
            std::cout << "[CabinAssistant] Command not recognized.\n";
        }
    }

    EnvironmentalControl* getEnvironmentalControl() const { return env_control.get(); }
    InfotainmentSystem* getInfotainmentSystem() const { return infotainment.get(); }

private:
    std::unique_ptr<EnvironmentalControl> env_control;
    std::unique_ptr<InfotainmentSystem> infotainment;
};

} // namespace cabin_assistant

#endif // CABIN_ASSISTANT_H