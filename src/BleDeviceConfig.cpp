#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "BleDeviceConfig.h"
#include "BleDevice.h"
#include "GpsDevice.h"
#include "CanDevice.h"

// Command constants (with trailing space for easier parsing)
static const char CMD_GPS[]     = "gps ";
static const char CMD_CAN[]     = "can ";
static const char CMD_BLE[]     = "ble ";
static const char CMD_STATUS[]  = "status";
static const char CMD_VERSION[] = "version";
static const char CMD_RESTART[] = "restart";
static const char CMD_HELP[]    = "help";

// Argument constants
static const char ARG_ON[]         = "on";
static const char ARG_OFF[]        = "off";
static const char ARG_STATS[]      = "stats";
static const char ARG_FRAME[]      = "frame";

// Persistent storage
#define CONFIG_VERSION 0x1
static const char CONFIG_FILENAME[] = "/config.dat";
typedef struct __attribute__((packed)) {
    uint16_t version;
    BLEConfig::ConfigState config;
    uint16_t checksum;
} StoredConfig;

BLEConfig::ConfigState BLEConfig::state;

extern GPS::FrameData gpsData;
#if CAN_COMPOSED_DATA
extern CAN::FrameData composedCanData1, composedCanData2;
#else
extern CAN::FrameData canData;
#endif


static void gpsResponseCallback(const char* response) {
    BLE::uartPrint("GPS response: "); BLE::uartPrintln(response);
}

static uint16_t calculateChecksum(const BLEConfig::ConfigState& config) {
    uint16_t sum = 0;
    const uint8_t* data = (const uint8_t*)&config;
    for (size_t i = 0; i < sizeof(config); i++) {
        sum += data[i];
    }
    return sum;
}

void BLEConfig::init() {
    InternalFS.begin();
    
    if (!loadConfig()) {
        Serial.println("Config set to default");
        state.bleFeatures = BLE::FeatureType::GPS_COORDS | BLE::FeatureType::GPS_TIME | BLE::FeatureType::CAN | BLE::FeatureType::UART;
        saveConfig();
    } 
    else {
        Serial.println("Config loaded from flash");
    }
}

const BLEConfig::ConfigState& BLEConfig::getState() {
    return state;
}

void BLEConfig::setBleFeatures(uint32_t features) {
    state.bleFeatures = features;
}

bool BLEConfig::loadConfig() {
    using namespace Adafruit_LittleFS_Namespace;
    
    File file(InternalFS);
    
    if (!file.open(CONFIG_FILENAME, FILE_O_READ)) {
        return false;
    }
    
    StoredConfig stored;
    uint32_t bytesRead = file.read(&stored, sizeof(StoredConfig));
    file.close();
    
    if (bytesRead != sizeof(StoredConfig)) {
        Serial.print("ERROR: Invalid config file size: "); Serial.print(bytesRead); Serial.print("b vs. "); Serial.print(sizeof(StoredConfig)); Serial.println("b");
        return false;
    }
    
    if (stored.version != CONFIG_VERSION) {
        Serial.print("ERROR: Invalid config version "); Serial.println(stored.version);
        return false;
    }
    
    uint16_t calculatedChecksum = calculateChecksum(stored.config);
    if (stored.checksum != calculatedChecksum) {
        Serial.println("ERROR: Config checksum mismatch");
        return false;
    }
    
    state = stored.config;    
    return true;
}

bool BLEConfig::saveConfig() {
    using namespace Adafruit_LittleFS_Namespace;
    
    StoredConfig stored;
    stored.version = CONFIG_VERSION;
    stored.config = state;
    stored.checksum = calculateChecksum(state);
    
    // Delete old file if it exists
    InternalFS.remove(CONFIG_FILENAME);
    
    // Create and write new file
    File file(InternalFS);
    if (!file.open(CONFIG_FILENAME, FILE_O_WRITE)) {
        Serial.println("ERROR: Failed to open config file for writing");
        return false;
    }
    
    uint32_t bytesWritten = file.write((uint8_t*)&stored, sizeof(StoredConfig));
    file.close();
    
    if (bytesWritten != sizeof(StoredConfig)) {
        Serial.print("ERROR: Failed to save config: "); Serial.print(bytesWritten); Serial.print("b vs. expected "); Serial.print(sizeof(StoredConfig)); Serial.println("b");
        InternalFS.remove(CONFIG_FILENAME);
        return false;
    }
    
    Serial.println("Config saved to flash");
    return true;
}

void BLEConfig::handleCommand(const char* input) {
    if (input == NULL || *input == '\0') {
        BLE::uartPrint("ERROR: Empty input");
        return;
    }
    
    // Create a copy of current state to detect changes
    ConfigState stateBefore = state;
    
    // Route to appropriate handler based on command prefix
    if (strncmp(input, CMD_GPS, strlen(CMD_GPS)) == 0) {
        handleGpsCommand(input + strlen(CMD_GPS));
    }
    else if (strncmp(input, CMD_CAN, strlen(CMD_CAN)) == 0) {
        handleCanCommand(input + strlen(CMD_CAN));
    }
    else if (strncmp(input, CMD_BLE, strlen(CMD_BLE)) == 0) {
        handleBleCommand(input + strlen(CMD_BLE));
    }
    else if (strncmp(input, CMD_STATUS, strlen(CMD_STATUS)) == 0) {
        handleStatusCommand();
    }
    else if (strncmp(input, CMD_VERSION, strlen(CMD_VERSION)) == 0) {
        handleVersionCommand();
    }
    else if (strncmp(input, CMD_RESTART, strlen(CMD_RESTART)) == 0) {
        handleRestartCommand();
    }
    else if (strncmp(input, CMD_HELP, strlen(CMD_HELP)) == 0) {
        handleHelpCommand();
    }
    else {
        BLE::uartPrint("ERROR: Unknown command "); BLE::uartPrintln(input);
    }
    
    // Save configuration if state was modified
    if (memcmp(&stateBefore, &state, sizeof(ConfigState)) != 0) {
        saveConfig();
    }
}

void BLEConfig::handleGpsCommand(const char* args) {
    if ((strncmp(args, "PQTM", 4) == 0 || strncmp(args, "PAIR", 4) == 0)) {
        // transmit raw command to GPS
        GPS::sendCommand(args, gpsResponseCallback);
        BLE::uartPrintln("GPS command sent");
    }
    else if (strcmp(args, ARG_ON) == 0) {
        state.bleFeatures |= BLE::FeatureType::GPS;
        BLE::uartPrintln("GPS enabled");
    }
    else if (strcmp(args, ARG_OFF) == 0) {
        state.bleFeatures &= ~BLE::FeatureType::GPS;
        BLE::uartPrintln("GPS disabled");
    }
    else if (strcmp(args, ARG_STATS) == 0) {
        if (!(state.bleFeatures & BLE::FeatureType::GPS)) {
            BLE::uartPrintln("GPS off");
        }
        else {
            GPS::Stats gpsStats = GPS::getStats();
            if (gpsStats.timestamp != 0) {
                char statusMsg[128];
                snprintf(statusMsg, sizeof(statusMsg), "GPS Stats (per second, %lus ago): NMEA msgs=%d (RMS=%d, GGA=%d), UART errors: %d", (millis() - gpsStats.timestamp) / 1000, gpsStats.nmeaMessages, gpsStats.rmsMessages, gpsStats.ggaMessages, gpsStats.uartErrors);
                BLE::uartPrintln(statusMsg);
            }
            else {
                BLE::uartPrintln("GPS N/A");
            }
        }
    }
    else if (strcmp(args, ARG_FRAME) == 0) {
        if (!(state.bleFeatures & BLE::FeatureType::GPS)) {
            BLE::uartPrintln("GPS off");
        }
        else if (gpsData.timestamp != 0) {
            char frameMsg[128];
            snprintf(frameMsg, sizeof(frameMsg), "GPS Frame (%lums ago): fix=%d, sats=%d, lat=%d, lon=%d, hdop=%dcm", millis() - gpsData.timestamp, gpsData.fixQuality, gpsData.satellites, gpsData.latitude, gpsData.longitude, gpsData.hdopX100);
            BLE::uartPrintln(frameMsg);
        }
        else {
            BLE::uartPrintln("GPS N/A");
        }
    }
    else {
        BLE::uartPrintln("ERROR: Use 'gps on/off/stats/frame/<PQTM/PAIR command>'");
    }
}

void BLEConfig::handleCanCommand(const char* args) {
    if (strcmp(args, ARG_ON) == 0) {
        state.bleFeatures |= BLE::FeatureType::CAN;
        BLE::uartPrintln("CAN enabled");
    }
    else if (strcmp(args, ARG_OFF) == 0) {
        state.bleFeatures &= ~BLE::FeatureType::CAN;
        BLE::uartPrintln("CAN disabled");
    }
    else if (strcmp(args, ARG_STATS) == 0) {
        if (!(state.bleFeatures & BLE::FeatureType::CAN)) {
            BLE::uartPrintln("CAN off");
        }
        else {
            CAN::Stats canStats = CAN::getStats();
            if (canStats.timestamp != 0) {
                char statusMsg[128];
                snprintf(statusMsg, sizeof(statusMsg), "CAN Stats (per second, %lus ago): messages=%d, ignored=%d, errors=%d, overflows=%d, PIDs allowed=%d%s", (millis() - canStats.timestamp) / 1000, canStats.canMessages, canStats.canMessagesIgnored, canStats.errors, canStats.overflowErrors, canStats.pidsAllowed, canStats.isHwPidsFilters ? " (HW filters)" : "");
                BLE::uartPrintln(statusMsg);
            }
            else {
                BLE::uartPrintln("CAN N/A");
            }
        }
    }
    else if (strcmp(args, ARG_FRAME) == 0) {
        if (!(state.bleFeatures & BLE::FeatureType::CAN)) {
            BLE::uartPrintln("CAN off");
        }
#if CAN_COMPOSED_DATA
        else if (composedCanData1.timestamp != 0 || composedCanData2.timestamp != 0) {
            // Print composedCanData1 frame
            if (composedCanData1.timestamp != 0) {
                char frameMsg[256];
                uint16_t speed = (composedCanData1.data[0]<<8) | composedCanData1.data[1];
                uint16_t rpm = (composedCanData1.data[2]<<8) | composedCanData1.data[3];
                uint8_t engineLoad = composedCanData1.data[4];
                uint8_t accelPedal = composedCanData1.data[5];
                uint16_t braking = (composedCanData1.data[6]<<8) | composedCanData1.data[7];
                
                snprintf(frameMsg, sizeof(frameMsg), "CAN Frame 1 (%lums ago): speed=%d.%02dkm/h, rpm=%d, load=%d%%, accel=%d%%, brake=%d", 
                    millis() - composedCanData1.timestamp, speed/100, speed%100, rpm, engineLoad, accelPedal, braking);
                BLE::uartPrintln(frameMsg);
            }
            else {
                BLE::uartPrintln("CAN Frame 1 N/A");
            }
            
            // Print composedCanData2 frame
            if (composedCanData2.timestamp != 0) {
                char frameMsg[256];
                int16_t latG = (composedCanData2.data[0]<<8) | composedCanData2.data[1];
                int16_t lonG = (composedCanData2.data[2]<<8) | composedCanData2.data[3];
                int16_t steering = (composedCanData2.data[4]<<8) | composedCanData2.data[5];
                uint8_t gear = composedCanData2.data[6];
                uint8_t tractionControl = composedCanData2.data[7];
                
                snprintf(frameMsg, sizeof(frameMsg), "CAN Frame 2 (%lums ago): latG=%d.%02dg, lonG=%d.%02dg, steer=%ddeg, gear=%d, TC=%d", 
                    millis() - composedCanData2.timestamp, latG/100, abs(latG)%100, lonG/100, abs(lonG)%100, steering, gear, tractionControl);
                BLE::uartPrintln(frameMsg);
            }
            else {
                BLE::uartPrintln("CAN Frame 2 N/A");
            }

        }
#else
        else if (canData.timestamp != 0) {
            char frameMsg[256];
            snprintf(frameMsg, sizeof(frameMsg), "CAN Frame (%lums ago): PID=0x%03lX, len=%d, data=", millis() - canData.timestamp, canData.pid, canData.length);
            BLE::uartPrint(frameMsg);
            
            // Print data bytes
            for (int i = 0; i < canData.length; i++) {
                char byteStr[6];
                snprintf(byteStr, sizeof(byteStr), "%02X%s", canData.data[i], (i < canData.length - 1) ? " " : "");
                BLE::uartPrint(byteStr);
            }
            BLE::uartPrintln("");
        }
#endif
        else {
            BLE::uartPrintln("CAN N/A");
        }
    }
    else {
        BLE::uartPrintln("ERROR: Use 'can on/off/stats/frame'");
    }
}

void BLEConfig::handleBleCommand(const char* args) {
    if (strcmp(args, ARG_STATS) == 0) {
        BLE::Stats bleStats = BLE::getStats();
        if (bleStats.timestamp != 0) {
            char statusMsg[128];
            snprintf(statusMsg, sizeof(statusMsg), "BLE Stats (per second, %lus ago): sent=%db, errors=%d", (millis() - bleStats.timestamp) / 1000, bleStats.dataSent, bleStats.dataSentErrors);
            BLE::uartPrintln(statusMsg);
        }
        else {
            BLE::uartPrintln("BLE N/A");
        }
    }
    else {
        BLE::uartPrintln("ERROR: Use 'ble stats'");
    }
}

void BLEConfig::handleStatusCommand() {
    BLE::uartPrint("GPS: "); BLE::uartPrintln((state.bleFeatures & BLE::FeatureType::GPS) ? "ON" : "OFF");    
    BLE::uartPrint("CAN: "); BLE::uartPrintln((state.bleFeatures & BLE::FeatureType::CAN) ? "ON" : "OFF");
}

void BLEConfig::handleVersionCommand() {
    BLE::uartPrintln("GPS Racing Tracker v1.0");
}

void BLEConfig::handleRestartCommand() {
    BLE::uartPrintln("Restart...");
    delay(100);  // Give time for messages to be sent
    NVIC_SystemReset();
}

void BLEConfig::handleHelpCommand() {
    BLE::uartPrintln("Available commands:");
    BLE::uartPrintln("  gps on/off       - Enable/disable GPS");
    BLE::uartPrintln("  gps stats        - Show GPS module stats for the last second");
    BLE::uartPrintln("  gps frame        - Show last GPS frame data");
    BLE::uartPrintln("  gps PQTM/PAIR*   - Send the command to GPS module");
    BLE::uartPrintln("  can on/off       - Enable/disable CAN");
    BLE::uartPrintln("  can stats        - Show CAN stats for the last second");
    BLE::uartPrintln("  can frame        - Show last CAN frame data");
    BLE::uartPrintln("  ble stats        - Show BLE stats for the last second");
    BLE::uartPrintln("  status           - Show general status");
    BLE::uartPrintln("  version          - Show firmware version");
    BLE::uartPrintln("  restart          - Restart the device");
    BLE::uartPrintln("  help             - Show this help");
    BLE::uartPrintln("Examples:");
    BLE::uartPrintln("  gps stats");
    BLE::uartPrintln("  gps PQTMVERNO");
    BLE::uartPrintln("  can stats");
    BLE::uartPrintln("  can frame");
    BLE::uartPrintln("  ble stats");
    BLE::uartPrintln("  status");
}

