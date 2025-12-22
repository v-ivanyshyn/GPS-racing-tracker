#ifndef BLE_H
#define BLE_H

#include <Arduino.h>

class BLE {
    public:
    typedef void (*CanBusFilterCallback)(const uint32_t* pids, int count);
    typedef void (*UartCommandCallback)(const char* command);

    typedef enum {
        CAN = (1<<0),
        GPS_COORDS = (1<<1),
        GPS_TIME = (1<<2),
        GPS = (1<<1)|(1<<2),
        UART = (1<<3)
    } FeatureType;

    typedef struct {
        int dataSent;
        int dataSentErrors;
        uint32_t timestamp;
    } Stats;

    static void setup();
    static void setCanBusFilterCallback(CanBusFilterCallback callback);
    static void setUartCommandCallback(UartCommandCallback callback);
    static void uartPrint(const char* text);
    static void uartPrintln(const char* text);
    static void processUart();
    static void startAdvertising();
    static bool isConnected();
    static void sendData(FeatureType feature, const uint8_t* data, size_t len);
    static Stats getStats();
};

#endif // BLE_H