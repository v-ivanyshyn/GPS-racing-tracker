#ifndef CAN_H
#define CAN_H

#include <Arduino.h>

class CAN {
    public:
    typedef struct {
        uint32_t pid;
        uint8_t data[8];
        uint8_t length;
        uint32_t timestamp;
    } FrameData;

    typedef struct {
        int canMessages;
        int canMessagesIgnored;
        int pidsAllowed;
        int isHwPidsFilters;
        int errors;
        int overflowErrors;
        uint32_t timestamp;
    } Stats;

    static void setup();
    static void filterPids(const uint32_t* pids, int count);
#if CAN_COMPOSED_DATA
    static bool loop(FrameData& data1, FrameData& data2);
#else 
    static bool loop(FrameData &data);
#endif
    static Stats getStats();

private:
    static void refreshStats();
};

#endif // CAN_H