#ifndef BLE_DEVICE_CONFIG_H
#define BLE_DEVICE_CONFIG_H

#include <Arduino.h>

class BLEConfig {
    public:
    typedef struct {
        uint32_t bleFeatures; // mask of BLE::FeatureType
    } ConfigState;

    static void init();
    
    // Handle incoming UART command
    static void handleCommand(const char* input);
    
    static const ConfigState& getState();
    static void setBleFeatures(uint32_t features);

    static bool loadConfig();
    static bool saveConfig();
    
private:
    static ConfigState state;

    static void handleGpsCommand(const char* args);
    static void handleCanCommand(const char* args);
    static void handleBleCommand(const char* args);
    static void handleStatusCommand();
    static void handleVersionCommand();
    static void handleRestartCommand();
    static void handleHelpCommand();
};

#endif // BLE_DEVICE_CONFIG_H

