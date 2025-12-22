#include <Arduino.h>
#include "BleDevice.h"
#include "BleDeviceConfig.h"
#include "GpsDevice.h"
#include "CanDevice.h"

// #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
// #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
// #define DEBUG_PRINT_8BYTES(data) { for (int i = 0; i < 8; i++) { if (((const uint8_t *)data)[i] < 0x10) Serial.print("0"); Serial.print(((const uint8_t *)data)[i], HEX); if (i < 7) Serial.print(" "); } }
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINT_8BYTES(data)

GPS::FrameData gpsData;
#if CAN_COMPOSED_DATA
CAN::FrameData composedCanData1, composedCanData2;
#else
CAN::FrameData canData;
#endif

void canBusFilterCallback(const uint32_t* pids, int count) {
    if (count == 0) {
        DEBUG_PRINTLN("CAN filters: no PIDs allowed");
        CAN::filterPids(pids, count);
    }
    else if (pids[0] == 0) {
        DEBUG_PRINTLN("CAN filters: all PIDs allowed");
        CAN::filterPids(pids, count);
    }
    else {
        Serial.print("CAN filters: allowed PIDs");
        for (int i = 0; i < count; i++) {
            DEBUG_PRINT(" 0x"); DEBUG_PRINTLN(pids[i], HEX);
        }
        DEBUG_PRINTLN();
        CAN::filterPids(pids, count);
    }
}

void setup() {
    for (int i=0; i<3; i++) {
        digitalWrite(LED_GREEN, LOW);
        delay(50);
        digitalWrite(LED_GREEN, HIGH);
        delay(200);
    }

    // USB serial
    Serial.begin(115200);
    // for (int i=0; !Serial; i++) { // wait for USB serial
    //     digitalWrite(LED_RED, (i%10 == 0) ? LOW : HIGH);
    //     digitalWrite(LED_BLUE, (i%10 == 0) ? LOW : HIGH);
    //     delay(10);
    // }
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_BLUE, HIGH);

    Serial.println("Setup BLE Config...");
    BLEConfig::init();

    Serial.println("Setup GPS...");
    GPS::setup();

    Serial.println("Setup CAN...");
    CAN::setup();

    Serial.println("Setup BLE...");
    BLE::setup();
    BLE::setCanBusFilterCallback(canBusFilterCallback);
    BLE::setUartCommandCallback(BLEConfig::handleCommand);
    BLE::startAdvertising();

    Serial.println("Setup complete");
}

bool gpsLoop() {
    static uint32_t loop_counter = 0;

    static uint8_t gpsSyncBits = 0;
    static int gpsPreviousDateAndHour = 0;

    if (GPS::loop(gpsData)) {
        loop_counter++;

        if (
            gpsData.date == 0 ||
            gpsData.timeMs == 0 ||
            gpsData.latitude == 0 ||
            gpsData.longitude == 0
        ) {
            return false; // Invalid data, skip sending
        }

        // https://github.com/aollin/racechrono-ble-diy-device?tab=readme-ov-file#gps-main-characteristic-uuid-0x0003
        // byte 	description
        // 0-2 	    Sync bits* (3 bits) and time from hour start (21 bits = (minute * 30000) + (seconds * 500) + (milliseconds / 2))
        // 3 	    Fix quality (2 bits), locked satellites (6 bits, invalid value 0x3F)
        // 4-7 	    Latitude in (degrees * 10_000_000), signed 2's complement, invalid value 0x7FFFFFFF
        // 8-11 	Longitude in (degrees * 10_000_000), signed 2's complement, invalid value 0x7FFFFFFF
        // 12-13 	Altitude (((meters + 500) * 10) & 0x7FFF) or (((meters + 500) & 0x7FFF) | 0x8000), invalid value 0xFFFF. **
        // 14-15 	Speed in ((km/h * 100) & 0x7FFF) or (((km/h * 10) & 0x7FFF) | 0x8000), invalid value 0xFFFF. ***
        // 16-17 	Bearing (degrees * 100), invalid value 0xFFFF
        // 18 	    HDOP (dop * 10), invalid value 0xFF
        // 19 	    VDOP (dop * 10), invalid value 0xFF
        uint8_t gpsCoordsData[20];

        int year = gpsData.date / 10000;
        int month = (gpsData.date % 10000) / 100;
        int day = gpsData.date % 100;
        int hour = gpsData.timeMs / 3600 / 1000;
        int dateAndHour = (year - 2000) * 8928 + (month - 1) * 744 + ((day - 1) * 24) + hour;
        if (gpsPreviousDateAndHour != dateAndHour) {
            gpsPreviousDateAndHour = dateAndHour;
            gpsSyncBits++;
        }
        int timeSinceHourStart = (gpsData.timeMs - hour * 3600 * 1000) / 2;
        int altitude = (gpsData.altitudeX100 > 600000) ? ((max(0, gpsData.altitudeX100/100 + 500) & 0x7FFF) | 0x8000) : (max(0, gpsData.altitudeX100/10 + 5000) & 0x7FFF); 
        int speed = (gpsData.speedX100 > 60000) ? ((max(0, gpsData.speedX100 / 10) & 0x7FFF) | 0x8000) : (max(0, gpsData.speedX100) & 0x7FFF); 
        gpsCoordsData[0] = ((gpsSyncBits & 0x7) << 5) | (((timeSinceHourStart) >> 16) & 0x1F);
        gpsCoordsData[1] = timeSinceHourStart >> 8;
        gpsCoordsData[2] = timeSinceHourStart;
        gpsCoordsData[3] = (min(0x3, gpsData.fixQuality & 0x3) << 6) | ((min(0x3F, gpsData.satellites)) & 0x3F);
        gpsCoordsData[4] = gpsData.latitude >> 24;
        gpsCoordsData[5] = gpsData.latitude >> 16;
        gpsCoordsData[6] = gpsData.latitude >> 8;
        gpsCoordsData[7] = gpsData.latitude >> 0;
        gpsCoordsData[8] = gpsData.longitude >> 24;
        gpsCoordsData[9] = gpsData.longitude >> 16;
        gpsCoordsData[10] = gpsData.longitude >> 8;
        gpsCoordsData[11] = gpsData.longitude >> 0;
        gpsCoordsData[12] = altitude >> 8;
        gpsCoordsData[13] = altitude;
        gpsCoordsData[14] = speed >> 8;
        gpsCoordsData[15] = speed;
        gpsCoordsData[16] = gpsData.bearingX100 >> 8;
        gpsCoordsData[17] = gpsData.bearingX100;
        gpsCoordsData[18] = gpsData.hdopX100 / 10;
        gpsCoordsData[19] = 0xFF; // Unimplemented 
        BLE::sendData(BLE::GPS_COORDS, gpsCoordsData, sizeof(gpsCoordsData));

        // https://github.com/aollin/racechrono-ble-diy-device?tab=readme-ov-file#gps-time-characteristic-uuid-0x0004
        // byte 	description
        // 0-2 	    Sync bits* (3 bits) and hour and date (21 bits = (year - 2000) * 8928 + (month - 1) * 744 + (day - 1) * 24 + hour)
        uint8_t gpsTimeData[3];
        gpsTimeData[0] = ((gpsSyncBits & 0x7) << 5) | ((dateAndHour >> 16) & 0x1F);
        gpsTimeData[1] = dateAndHour >> 8;
        gpsTimeData[2] = dateAndHour;
        BLE::sendData(BLE::GPS_TIME, gpsTimeData, sizeof(gpsTimeData));

        return true;
    }
    return false;
}

bool canLoop() {
#if CAN_COMPOSED_DATA
    static CAN::FrameData prevComposedData1;
    static CAN::FrameData prevComposedData2;
    int pidSize = sizeof(uint32_t);
    uint8_t canMsgData[pidSize + 8];
    if (CAN::loop(composedCanData1, composedCanData2)) {
        if (memcmp(&composedCanData1.data, prevComposedData1.data, sizeof(CAN::FrameData::data)) != 0) {
            memcpy(&prevComposedData1, &composedCanData1, sizeof(CAN::FrameData));
            ((uint32_t*)canMsgData)[0] = composedCanData1.pid;
            memcpy(&canMsgData[pidSize], composedCanData1.data, composedCanData1.length);
            BLE::sendData(BLE::CAN, canMsgData, pidSize + composedCanData1.length);
            DEBUG_PRINT("BLE send CAN: 0x"); DEBUG_PRINT(((uint32_t*)canMsgData)[0], HEX); DEBUG_PRINT(" - ");
            DEBUG_PRINT_8BYTES(canMsgData[4]); DEBUG_PRINTLN("");
        }
        if (memcmp(&composedCanData2.data, prevComposedData2.data, sizeof(CAN::FrameData::data)) != 0) {
            memcpy(&prevComposedData2, &composedCanData2, sizeof(CAN::FrameData));
            ((uint32_t*)canMsgData)[0] = composedCanData2.pid;
            memcpy(&canMsgData[pidSize], composedCanData2.data, composedCanData2.length);
            BLE::sendData(BLE::CAN, canMsgData, pidSize + composedCanData2.length);
            DEBUG_PRINT("BLE send CAN: 0x"); DEBUG_PRINT(((uint32_t*)canMsgData)[0], HEX); DEBUG_PRINT(" - ");
            DEBUG_PRINT_8BYTES(canMsgData[4]); DEBUG_PRINTLN("");
        }
    }
#else
    if (CAN::loop(canData)) {
        // https://github.com/aollin/racechrono-ble-diy-device?tab=readme-ov-file#can-bus-main-characteristic-uuid-0x0001
        int pidSize = sizeof(uint32_t);
        uint8_t canMsgData[pidSize + 8];
        ((uint32_t*)canMsgData)[0] = canData.pid;
        memcpy(&canMsgData[pidSize], canData.data, canData.length);
        BLE::sendData(BLE::CAN, canMsgData, pidSize + canData.length);
        return true;
    }
#endif
    return false;
}

void loop() {
    uint32_t time = millis();
    static uint32_t gpsLastMsgTime = 0;
    static uint32_t canLastMsgTime = 0;

    // if (!BLE::isConnected()) {
    //     digitalWrite(LED_BLUE, (time % 1000 < 100) ? LOW : HIGH); // BLUE LED blinks when not connected
    //     return;
    // }
    digitalWrite(LED_BLUE, HIGH);

    BLE::processUart();
    if (BLEConfig::getState().bleFeatures & BLE::FeatureType::GPS) {
        bool isGpsMsg = gpsLoop();
        if (isGpsMsg) {
            gpsLastMsgTime = time;
        }
    }
    if (BLEConfig::getState().bleFeatures & BLE::FeatureType::CAN) {
        bool isCanMsg = canLoop();
        if (isCanMsg) {
            canLastMsgTime = time;
        }
    }

    // Fast blink when a message received during the last second (red - GPS, green - CAN):
    if (time - gpsLastMsgTime < 1000) {
        digitalWrite(LED_RED, (time % 200 < 50) ? LOW : HIGH);
    }
    else {
        digitalWrite(LED_RED, HIGH);
    }
    if (time - canLastMsgTime < 1000) {
        digitalWrite(LED_GREEN, ((time + 50) % 200 < 50) ? LOW : HIGH);
    }
    else {
        digitalWrite(LED_GREEN, HIGH);
    }
}
