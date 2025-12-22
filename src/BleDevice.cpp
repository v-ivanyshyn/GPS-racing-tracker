#include <bluefruit.h>
#include "BleDevice.h"
#include "BleDeviceConfig.h"

// #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
// #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
// #define DEBUG_PRINT_8BYTES(data) { for (int i = 0; i < 8; i++) { if (((const uint8_t *)data)[i] < 0x10) Serial.print("0"); Serial.print(((const uint8_t *)data)[i], HEX); if (i < 7) Serial.print(" "); } }
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINT_8BYTES(data)

struct {
    BLEService mainService = BLEService(0x1ff8);

    BLECharacteristic canBusMainCharacteristic = BLECharacteristic(0x01);
    BLECharacteristic canBusFilterCharacteristic = BLECharacteristic(0x02);
    uint32_t canPidsAllowed[32];
    BLE::CanBusFilterCallback canBusFilterCallback = nullptr;

    BLECharacteristic gpsMainCharacteristic = BLECharacteristic(0x03);
    BLECharacteristic gpsTimeCharacteristic = BLECharacteristic(0x04);
} raceChronoBle;

static BLEUart bleUart;
static BLE::UartCommandCallback uartCommandCallback = nullptr;
static char uartRxBuffer[128];
static size_t uartRxBufferLen = 0;
static unsigned long uartRxLastTime = 0;
static const unsigned long UART_RX_TIMEOUT = 100;  // 100ms

static BLE::Stats bleStats;

static void canBusFilterWriteCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    static const int CAN_BUS_CMD_DENY_ALL = 0;
    static const int CAN_BUS_CMD_ALLOW_ALL = 1;
    static const int CAN_BUS_CMD_ADD_PID = 2;

    if (len == 0) {
        Serial.println("ERROR: Trying to send 0 bytes of data");
        return;
    }
    DEBUG_PRINT("CAN bus filter command: "); DEBUG_PRINT_8BYTES(data); DEBUG_PRINTLN();

    // https://github.com/aollin/racechrono-ble-diy-device?tab=readme-ov-file#can-bus-filter-characteristic-uuid-0x0002
    uint8_t command = data[0];
    switch (command) {
        case CAN_BUS_CMD_DENY_ALL:
            if (len == 1) {
                memset(raceChronoBle.canPidsAllowed, 0xFFFFFFFF, sizeof(raceChronoBle.canPidsAllowed)); // assume that no real PID is 0xFFFFFFFF
                raceChronoBle.canBusFilterCallback(raceChronoBle.canPidsAllowed, 0);
            }
            break;
        case CAN_BUS_CMD_ALLOW_ALL:
            if (len == 3) {
                memset(raceChronoBle.canPidsAllowed, 0, sizeof(raceChronoBle.canPidsAllowed)); // if canPidsAllowed[0] == 0, all PIDs are allowed
                raceChronoBle.canBusFilterCallback(raceChronoBle.canPidsAllowed, 1);
            }
            break;
        case CAN_BUS_CMD_ADD_PID:
            if (len == 7) {
                //uint16_t notifyIntervalMs = data[1] << 8 | data[2];
                uint32_t pid = data[3] << 24 | data[4] << 16 | data[5] << 8 | data[6];
                DEBUG_PRINT("CAN bus filter command: allow pid 0x"); DEBUG_PRINTLN(pid, HEX);
                for (size_t i=0; i<sizeof(raceChronoBle.canPidsAllowed) / sizeof(raceChronoBle.canPidsAllowed[0]); i++) {
                    if (raceChronoBle.canPidsAllowed[i] == pid) {
                        DEBUG_PRINTLN("PID already allowed");
                        break;
                    }
                    if (raceChronoBle.canPidsAllowed[i] == 0xFFFFFFFF) {
                        DEBUG_PRINTLN("Adding PID to allowed list");
                        raceChronoBle.canPidsAllowed[i] = pid;
                        raceChronoBle.canBusFilterCallback(raceChronoBle.canPidsAllowed, i+1);
                        break;
                    }
                }
            }
            break;
        default:
            break;         
    }
}

static void uartRxCallback(uint16_t conn_handle) {
    // Just store received data in buffer, don't process yet
    while (bleUart.available()) {
        char c = (char)bleUart.read();
        uartRxLastTime = millis();

        if (c == '\r' || c == '\n') { // Skip carriage return and newline (they're just terminators)
            continue;
        }

        // if (c == '\b' || c == 127) { // Handle backspace
        //     if (uartRxBufferLen > 0) {
        //         uartRxBufferLen--;
        //     }
        //     continue;
        // }

        if (uartRxBufferLen < sizeof(uartRxBuffer) - 1) { // Add character to buffer
            uartRxBuffer[uartRxBufferLen++] = c;
        }
        else { // Buffer overflow - reset
            Serial.println("ERROR: BLE UART buffer overflow");
            uartRxBufferLen = 0;
        }
    }
}

static void connectCallback(uint16_t conn_handle) {
    BLEConnection* connection = Bluefruit.Connection(conn_handle);
    char central_name[32] = { 0 };  
    connection->getPeerName(central_name, sizeof(central_name));
    Serial.print("BLE connected to ");  Serial.println(central_name);
    
    if (BLEConfig::getState().bleFeatures & BLE::FeatureType::UART) {
        // Send welcome message over UART
        if (bleUart.notifyEnabled()) {
            bleUart.println("GPS Racing Tracker");
            bleUart.println("Type 'help' for commands");
        }
    }
}

static void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
    Serial.print("BLE disconnected, reason = 0x");  Serial.println(reason, HEX);
}

void BLE::setup() {
    //Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.autoConnLed(false);
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.begin();
    //uint8_t mac[6] = { 0, 0, 0, 0, 0, 0 };
    //Bluefruit.getAddr(mac);
    Bluefruit.setName("GPS Racing Tracker");
    
    memset(&bleStats, 0, sizeof(bleStats));

    raceChronoBle.mainService.begin();
    if (BLEConfig::getState().bleFeatures & BLE::FeatureType::CAN) {
        Serial.println("Start BLE CAN characteristic...");
        raceChronoBle.canBusMainCharacteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
        raceChronoBle.canBusMainCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
        raceChronoBle.canBusMainCharacteristic.begin();
        raceChronoBle.canBusFilterCharacteristic.setProperties(CHR_PROPS_WRITE);
        raceChronoBle.canBusFilterCharacteristic.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
        raceChronoBle.canBusFilterCharacteristic.setWriteCallback(*canBusFilterWriteCallback);
        raceChronoBle.canBusFilterCharacteristic.begin();
    }
    if (BLEConfig::getState().bleFeatures & BLE::FeatureType::GPS_COORDS) {
        Serial.println("Start BLE GPS_COORDS characteristic...");
        raceChronoBle.gpsMainCharacteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
        raceChronoBle.gpsMainCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
        raceChronoBle.gpsMainCharacteristic.begin();
    }
    if (BLEConfig::getState().bleFeatures & BLE::FeatureType::GPS_TIME) {
        Serial.println("Start BLE GPS_TIME characteristic...");
        raceChronoBle.gpsTimeCharacteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
        raceChronoBle.gpsTimeCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
        raceChronoBle.gpsTimeCharacteristic.begin();
    }

    // Setup BLE UART (Nordic UART Service)
    if (BLEConfig::getState().bleFeatures & BLE::FeatureType::UART) {
        Serial.println("Start BLE UART...");
        bleUart.begin();
        bleUart.setRxCallback(uartRxCallback);
    }

    Bluefruit.Periph.setConnectCallback(connectCallback);
    Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

    startAdvertising();
}

void BLE::setCanBusFilterCallback(CanBusFilterCallback callback) {
    raceChronoBle.canBusFilterCallback = callback;
}

void BLE::setUartCommandCallback(UartCommandCallback callback) {
    uartCommandCallback = callback;
}

void BLE::uartPrint(const char* text) {
    if (Bluefruit.connected() && bleUart.notifyEnabled()) {
        bleUart.print(text);
    }
}

void BLE::uartPrintln(const char* text) {
    if (Bluefruit.connected() && bleUart.notifyEnabled()) {
        bleUart.println(text);
    }
}

void BLE::processUart() {
    if (uartRxBufferLen > 0) {
        // Check if timeout expired (no new data for UART_RX_TIMEOUT ms)
        if (millis() - uartRxLastTime >= UART_RX_TIMEOUT) {
            // Null-terminate and process command
            uartRxBuffer[uartRxBufferLen] = '\0';
            DEBUG_PRINT("BLE UART RX: "); DEBUG_PRINTLN(uartRxBuffer);
            if (uartCommandCallback != nullptr) {
                uartCommandCallback(uartRxBuffer);
            }
            uartRxBufferLen = 0;
        }
    }
}

void BLE::startAdvertising() {
    Bluefruit.setTxPower(+4);
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(raceChronoBle.mainService);
    if (BLEConfig::getState().bleFeatures & BLE::FeatureType::UART) {
        Bluefruit.Advertising.addService(bleUart);
    }
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(160, 160); // in unit of 0.625 ms, 160=100ms
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);
}

bool BLE::isConnected() {
    return Bluefruit.connected();
}

void BLE::sendData(FeatureType feature, const uint8_t* data, size_t len) {
    // Save stats once per second
    static BLE::Stats currentStats = bleStats;
    if ((currentStats.timestamp / 1000) != (millis() / 1000)) {
        bleStats = currentStats;
        memset(&currentStats, 0, sizeof(currentStats));
        currentStats.timestamp = millis();
    }
    
    bool success = false;
    switch (feature) {
        case BLE::CAN:
            success = raceChronoBle.canBusMainCharacteristic.notify(data, len);
            break;
        case BLE::GPS_COORDS:
            success = raceChronoBle.gpsMainCharacteristic.notify(data, len);
            break;
        case BLE::GPS_TIME:
            success = raceChronoBle.gpsTimeCharacteristic.notify(data, len);
            break;
        default:
            break;
    }
    
    if (success) {
        currentStats.dataSent += len;
    } else {
        currentStats.dataSentErrors++;
    }
}

BLE::Stats BLE::getStats() {
    return bleStats;
}
