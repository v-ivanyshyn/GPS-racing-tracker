#ifndef GPS_H
#define GPS_H

#include <Arduino.h>

class GPS {
    public:
    typedef struct {
        int date; // 4 digits for year, 2 digits for month, 2 digits for day
        int timeMs;
        int latitude;
        int longitude;
        int altitudeX100;
        int speedX100;
        int bearingX100;
        int fixQuality;
        int satellites;
        int hdopX100;
        uint32_t timestamp;
    } FrameData;

    typedef struct {
        int nmeaMessages;
        int rmsMessages;
        int ggaMessages;
        int uartErrors;
        uint32_t timestamp;
    } Stats;

    typedef void (*ResponseCallback)(const char* response);

    static void setup();
    static bool loop(FrameData &data);
    static void sendCommand(const char* command, ResponseCallback callback = NULL);
    static Stats getStats();
private:
    static void processUartLine();
    static void processQueryResponse();
    static void processNmeaResponse();
    
    // UART line buffer and state
    static const int UART_LINE_BUFFER_SIZE = 256;
    static char uartLineBuffer[UART_LINE_BUFFER_SIZE];
    static int uartLineBufferLen;

    static ResponseCallback pendingCallback;
};

#endif // GPS_H
