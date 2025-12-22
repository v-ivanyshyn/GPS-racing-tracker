#include <mcp2515.h>
#include "CanDevice.h"

// #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
// #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
// #define DEBUG_PRINT_8BYTES(data) { for (int i = 0; i < 8; i++) { if (((const uint8_t *)data)[i] < 0x10) Serial.print("0"); Serial.print(((const uint8_t *)data)[i], HEX); if (i < 7) Serial.print(" "); } }
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINT_8BYTES(data)

static inline uint16_t BYTES16(uint8_t b1, uint8_t b2) {
    return  uint16_t(b1)<<8 | uint16_t(b2);
}

static uint32_t canPidsAllowed[32];
static int canPidsAllowedCount = 0;
struct can_frame canMsg;
static MCP2515 mcp2515(D7, 4000000, &SPI); // Chip Select on pin D7, SPI clock is default from SPISettings
#if CAN_COMPOSED_DATA
CAN::FrameData composedData1;
CAN::FrameData composedData2;
#endif
static CAN::Stats canStats;

void CAN::setup() {
    SPI.begin();
    if (mcp2515.reset() != MCP2515::ERROR_OK) {
        Serial.println("ERROR: CAN BUS reset failed");
    }
    if (mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ) != MCP2515::ERROR_OK) {
        Serial.println("ERROR: CAN BUS set bitrate failed");
    }
    memset(canPidsAllowed, 0, sizeof(canPidsAllowed)); // if canPidsAllowed[0] == 0, all PIDs are allowed
    mcp2515.setListenOnlyMode();

#if CAN_COMPOSED_DATA
    const uint32_t pids[] = { 0x76, 0x77, 0x7D, 0x204, 0x213, 0x230, 0x43E };
    filterPids(pids, sizeof(pids) / sizeof(pids[0]));

    memset(&composedData1, 0, sizeof(composedData1));
    composedData1.pid = 0x101;
    composedData1.length = 8;
    memset(&composedData2, 0, sizeof(composedData2));
    composedData2.pid = 0x102;
    composedData2.length = 8;
#endif
    memset(&canStats, 0, sizeof(canStats));
}

void CAN::filterPids(const uint32_t* pids, int count) {
    memcpy(canPidsAllowed, pids, count * sizeof(pids[0]));
    canPidsAllowedCount = count;

    uint32_t mcp2515FilterMasks[2];
    uint32_t mcp2515FilterPids[6];
    memset(mcp2515FilterPids, 0, sizeof(mcp2515FilterPids));
    
#if CAN_COMPOSED_DATA
    // Optimized for specific 7 PIDs: 0x76, 0x77, 0x7D, 0x204, 0x213, 0x230, 0x43E
    // Use RXB0 (RXF0, RXF1) with MASK0 to catch 0x76 and 0x77 together
    mcp2515FilterMasks[0] = 0x7FE;  // Mask: ignore LSB, match other bits
    mcp2515FilterPids[0] = 0x76;    // Catches both 0x76 and 0x77
    mcp2515FilterPids[1] = 0x7D;    // Exact match for 0x7D
    
    // Use RXB1 (RXF2-5) with MASK1 for exact matches
    mcp2515FilterMasks[1] = 0x7FF;  // Exact match all bits
    mcp2515FilterPids[2] = 0x204;   // Exact match
    mcp2515FilterPids[3] = 0x213;   // Exact match
    mcp2515FilterPids[4] = 0x230;   // Exact match
    mcp2515FilterPids[5] = 0x43E;   // Exact match
#else
    if (count == 0) { // No PIDs allowed
        memset(mcp2515FilterMasks, 0x7FF, sizeof(mcp2515FilterMasks));
    }
    else if (pids[0] == 0) { // All PIDs allowed
        memset(mcp2515FilterMasks, 0, sizeof(mcp2515FilterMasks));
    }
    else if (count <= 6) {
        // Use exact matching for <= 5 PIDs
        memset(mcp2515FilterMasks, 0x7FF, sizeof(mcp2515FilterMasks));
        for (int i = 0; i < count; i++) {
            mcp2515FilterPids[i] = pids[i];
        }
    }
    else { // All PIDs allowed because there's not enough filters in MCP2515
        memset(mcp2515FilterMasks, 0, sizeof(mcp2515FilterMasks));
    }
#endif
    // Filters RXF0 and RXF1 (and filter mask RXM0) are associated with RXB0.
    // Filters RXF2, RXF3, RXF4, RXF5 and mask RXM1 are associated with RXB1
    mcp2515.setConfigMode();
    mcp2515.setFilterMask(MCP2515::MASK0, false, mcp2515FilterMasks[0]);
    mcp2515.setFilter(MCP2515::RXF0, false, mcp2515FilterPids[0]);
    mcp2515.setFilter(MCP2515::RXF1, false, mcp2515FilterPids[1]);
    mcp2515.setFilterMask(MCP2515::MASK1, false, mcp2515FilterMasks[1]);
    mcp2515.setFilter(MCP2515::RXF2, false, mcp2515FilterPids[2]);
    mcp2515.setFilter(MCP2515::RXF3, false, mcp2515FilterPids[3]);
    mcp2515.setFilter(MCP2515::RXF4, false, mcp2515FilterPids[4]);
    mcp2515.setFilter(MCP2515::RXF5, false, mcp2515FilterPids[5]);
    mcp2515.setListenOnlyMode();
}

static CAN::Stats currentStats = canStats;
#if CAN_COMPOSED_DATA
bool CAN::loop(CAN::FrameData &data1, CAN::FrameData &data2) {
    refreshStats();
    while (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        // composed payload (2x 8 bytes): 
        // 0-1: speed (kmph x100)
        // 2-3: RPM
        // 4: engine load (%)
        // 5: acceleration pedal (%)
        // 6-7: braking
        // 0-1: lat. G
        // 2-3: lon. G
        // 4-5: steering angle
        // 6: gear
        // 7: traction control applied

        // Reference for Ford Mustang 2015 GT: https://github.com/v-ivanyshyn/parse_can_logs/blob/master/Ford%20CAN%20IDs%20Summary.md

        switch (canMsg.can_id) {
        case 0x076:
            {   // steering angle:
                int value = 1600 - BYTES16(canMsg.data[0], canMsg.data[1])/10;
                DEBUG_PRINT(" Steering="); DEBUG_PRINT(value);
                composedData2.data[4] = value>>8;
                composedData2.data[5] = value & 0xFF;
                composedData2.timestamp = millis();
            }
            break;
        case 0x077:
            {   // speed:
                uint16_t value = BYTES16(canMsg.data[0], canMsg.data[1]);
                composedData1.data[0] = value>>8;
                composedData1.data[1] = value & 0xFF;
                composedData1.timestamp = millis();
                DEBUG_PRINT(" Speed X10="); DEBUG_PRINT(value);
            }
            {   // lateral G force:
                int value = ((BYTES16(canMsg.data[2], canMsg.data[3]) & 0x1FFF) - 2048) / 2;
                DEBUG_PRINT(" LatG X100="); DEBUG_PRINTLN(value);
                composedData2.data[0] = value>>8;
                composedData2.data[1] = value & 0xFF;
                composedData2.timestamp = millis();
            }
            break;
        case 0x7D:
            {   // braking:
                int value = (BYTES16(canMsg.data[3], canMsg.data[4]) & 0x0FFF) * 4 / 10;
                DEBUG_PRINT(" Braking X100="); DEBUG_PRINT(value);
                composedData1.data[6] = value>>8;
                composedData1.data[7] = value & 0xFF;
                composedData1.timestamp = millis();
            }
            break;
        case 0x204:
            {   // RPM:
                int value = (BYTES16(canMsg.data[3], canMsg.data[4]) & 0x1FFF) * 2;
                DEBUG_PRINT(" RPM="); DEBUG_PRINT(value);
                composedData1.data[2] = value>>8;
                composedData1.data[3] = value & 0xFF;
                composedData1.timestamp = millis();
            }
            {   // accelerator pedal:
                int value = (BYTES16(canMsg.data[0], canMsg.data[1]) & 0x03FF) / 10;
                DEBUG_PRINT(" Accel="); DEBUG_PRINT(value);
                composedData1.data[5] = value;
                composedData1.timestamp = millis();
            }
            break;
        case 0x213:
            {   // longitudinal G force:
                int value = ((BYTES16(canMsg.data[5], canMsg.data[6]) & 0x03FF) - 512) * 364 / 1000;
                DEBUG_PRINT(" LonG X100="); DEBUG_PRINT(value);
                composedData2.data[2] = value>>8;
                composedData2.data[3] = value & 0xFF;
                composedData2.timestamp = millis();
            }
            {   // traction control applied:
                bool stabilityControlApplied = canMsg.data[2] & (1<<7);
                bool tractionControlApplied = canMsg.data[3] & (1<<7);
                DEBUG_PRINT(" ATC="); DEBUG_PRINT((stabilityControlApplied<<1) | (tractionControlApplied<<0));
                composedData2.data[7] = ((stabilityControlApplied<<1) | (tractionControlApplied<<0));
                composedData2.timestamp = millis();
            }
            break;
        case 0x230:
            {   // gear:
                int value = canMsg.data[0]>>4;
                int gear = ((value > 0) && (value <= 10)) ? value : 0;
                DEBUG_PRINT(" Gear="); DEBUG_PRINT(gear);
                composedData2.data[6] = gear;
                composedData2.timestamp = millis();
            }
            break;
        case 0x43E:
            {   // engine load:
                bool isValid = (canMsg.data[5] < 0x43);
                if (isValid) {
                    int value = BYTES16(canMsg.data[5], canMsg.data[6]) / 72 - 136;
                    DEBUG_PRINT(" EngLoad="); DEBUG_PRINT(value);
                    composedData1.data[4] = value;
                    composedData1.timestamp = millis();
                }
                else {
                    DEBUG_PRINT(" EngLoad=N/A");
                }
            }
            break;
        default:
            currentStats.canMessagesIgnored++;
            return false;
        }
        currentStats.canMessages++;
        memcpy(&data1, &composedData1, sizeof(FrameData));
        memcpy(&data2, &composedData2, sizeof(FrameData));
        DEBUG_PRINT(" CAN 0x"); DEBUG_PRINT(canMsg.can_id, HEX); DEBUG_PRINT(" - "); DEBUG_PRINT_8BYTES(canMsg.data); DEBUG_PRINTLN();
        return true;
    }
    return false;
}
#else
bool CAN::loop(CAN::FrameData &data) {
    refreshStats();
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        // Additional check if PID is allowed despite MCP2515 filters in case of more than 6 PIDs:
        bool isAllowed = (canPidsAllowed[0] == 0); // if canPidsAllowed[0] == 0, all PIDs are allowed
        for (int i=0; !isAllowed && i<canPidsAllowedCount; i++) {
            if (canPidsAllowed[i] == canMsg.can_id) {
                isAllowed = true;
            }
        }
        if (isAllowed) {
            data.pid = canMsg.can_id;
            data.length = canMsg.can_dlc;
            memcpy(data.data, canMsg.data, canMsg.can_dlc);
            data.timestamp = millis();
            currentStats.canMessages++;
            return true;
        }
        else {
            currentStats.canMessagesIgnored++;
            return false;
        }
    }
    return false;
}
#endif

CAN::Stats CAN::getStats() {
    return canStats;
}

void CAN::refreshStats() {
    // Save stats once per second:
    if ((currentStats.timestamp / 1000) != (millis() / 1000)) {
        canStats = currentStats;
        memset(&currentStats, 0, sizeof(currentStats));
        currentStats.timestamp = millis();
        currentStats.pidsAllowed = (canPidsAllowed[0] == 0) ? -1 : canPidsAllowedCount;
        currentStats.isHwPidsFilters = (canPidsAllowedCount > 0) && (canPidsAllowedCount <= 6);
    }

    if (mcp2515.checkError()) {
        currentStats.errors++;
        uint8_t errorFlags = mcp2515.getErrorFlags();
        
        if (errorFlags & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR)) { // Check for overflow errors (bits 6 and 7)
            currentStats.overflowErrors++;
            mcp2515.clearRXnOVRFlags();
        }
    }
}