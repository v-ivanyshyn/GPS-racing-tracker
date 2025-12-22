#include <TinyGPS++.h>
#include <Arduino.h>
#include "GpsDevice.h"

// #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
// #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)

// #define DEBUG_NMEA_PRINT(...) Serial.print(__VA_ARGS__)
// #define DEBUG_NMEA_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DEBUG_NMEA_PRINT(...)
#define DEBUG_NMEA_PRINTLN(...)

#define GPS_SERIAL Serial1
#define GPS_BAUD_RATE 460800

// Quectel LC29H GPS commands:
// "PQTMVERNO": firmware version information: $PQTMVERNO* -> $PQTMVERNO,<VerStr>,<BuildDate>,<BuildTime>*
// "PQTMCFGRCVRMODE": Sets/gets the receiver working mode. -> $PQTMCFGRCVRMODE,OK,<Mode>* (0 = Unknown, 1 = Rover, 2 = Station)
// "PQTMCFGFIXRATE": Sets/gets the position fix interval (10 Hz by default on LC29HEA). -> $PQTMCFGFIXRATE,OK,<Interval>*
// "PQTMCFGMSGRATE": Sets/gets the message output rate on the current port ($PQTMCFGMSGRATE,W,<MsgName>,<Rate>[,<MsgVer>]). Supported messages: RMC, GGA, GSV, GSA, VTG, GLL, ZDA, GRS, GST, GNS
// "PQTMCFGNMEADP": Sets/gets the decimal places of NMEA messages. -> $PQTMCFGNMEADP,OK,<UTC_DP>,<POS_DP>,<ALT_DP>,<DOP_DP>,<SPD_DP>,<COG_DP>
// "PQTMGNSSSTART": Starts GNSS engine.
// "PQTMGNSSSTOP": Stops GNSS engine.
// "PQTMSAVEPAR": Saves the configurations into NVM.
// "PQTMRESTOREPAR": Restores the parameters configured by all commands to their default values. This command takes effect after a reboot.

// "PAIR050": Sets position fix interval ($PAIR050,<Time>), time in ms, default value: 1000, affects only RMC and GGA
// "PAIR062": Sets the output rate of standard NMEA messages of each type ($PAIR062,<Type>,<OutputRate>). -1 = Reset all to default values, 0 = NMEA_SEN_GGA, 1 = NMEA_SEN_GLL, 2 = NMEA_SEN_GSA, 3 = NMEA_SEN_GSV, 4 = NMEA_SEN_RMC, 5 = NMEA_SEN_VTG, 6 = NMEA_SEN_ZDA, 7 = NMEA_SEN_GRS, 8 = NMEA_SEN_GST
// "PAIR864": Sets the baud rate of UART interface ($PAIR864,0,0,<Baudrate>). Reboot required for the new baud rate to take effect.
// "PAIR866": Sets UART flow control ($PAIR866,0,0,<FlowControl>). 0 = Flow control disabled, 1 = Software flow control enabled
// "PAIR007": Performs a cold start and clears system and user configurations at the start, i.e., resets the module to its factory settings

// For LC29H (BA, CA, DA, EA), standard NMEA sentences are output in NMEA 0183 V4.10
// NMEA 0183 V4.10 format (RMC): $<TalkerID>RMC,<UTC>,<Status>,<Lat>,<N/S>,<Lon>,<E/W>,<SOG>,<COG>,<Date>,<MagVar>,<MagVarDir>,<ModeInd>,<NavStatus>*<Checksum><CR><LF>
// NMEA 0183 V4.10 format (GGA): $<TalkerID>GGA,<UTC>,<Lat>,<N/S>,<Lon>,<E/W>,<Quality>,<NumSatUsed>,<HDOP>,<Alt>,M,<Sep>,M,<DiffAge>,<DiffStation>*<Checksum><CR><LF>

GPS::Stats gpsStats;

char GPS::uartLineBuffer[UART_LINE_BUFFER_SIZE] = {0};
int GPS::uartLineBufferLen = 0;

GPS::ResponseCallback GPS::pendingCallback = NULL;

TinyGPSPlus gps;

static uint8_t calculateNmeaChecksum(const char* msg) {
    uint8_t checksum = 0;
    for (int i = 0; msg[i] != '\0'; i++) {
        checksum ^= msg[i];
    }
    return checksum;
}

void GPS::setup() {
    GPS_SERIAL.setPins(D0, D1);    
    GPS_SERIAL.begin(GPS_BAUD_RATE, SERIAL_8N1);
    while (!GPS_SERIAL) {
        delay(0);
    }

    memset(&gpsStats, 0, sizeof(gpsStats));
}

bool GPS::loop(FrameData &data) {
    // Save stats once per second:
    static GPS::Stats currentStats = gpsStats;
    if ((currentStats.timestamp / 1000) != (millis() / 1000)) {
        gpsStats = currentStats;
        memset(&currentStats, 0, sizeof(currentStats));
        currentStats.timestamp = millis();
    }

    bool isUartLineReady = false;
    while (GPS_SERIAL.available()) {
        char c = GPS_SERIAL.read();
        DEBUG_NMEA_PRINT(c);
        
        if (c == '$') { // Start of a new NMEA sentence
            uartLineBufferLen = 0;
            uartLineBuffer[uartLineBufferLen++] = c;
        }
        else if (c == '\r') { // Skip carriage return
            continue;
        }
        else if (c == '\n' && uartLineBufferLen < UART_LINE_BUFFER_SIZE) { // Check for line end, assuming that the line fits in the buffer
            if (uartLineBufferLen > 0) {
                uartLineBuffer[uartLineBufferLen] = '\0'; // Null-terminate the string
                isUartLineReady = true;
                break; // Stop receiving the stream until next loop iteration, move to processing it
            }
        }
        else if (uartLineBufferLen >= UART_LINE_BUFFER_SIZE) { // Buffer overflow, skip all further characters until new message ('$')
            Serial.println("ERROR: GPS UART buffer overflow");
            currentStats.uartErrors++;
            while(GPS_SERIAL.available()) {
                char c = GPS_SERIAL.read();
                if (c == '\r' || c == '\n') { // Clean up UART RX queue until the end of the line
                    break;
                }
            }
        } 
        else if (uartLineBuffer[0] == '$') { // Add character to buffer if the buffer already contains a message starting with '$'
            uartLineBuffer[uartLineBufferLen++] = c;
        }
        // delay(1); // Small delay to allow data to accumulate. For UART baud rate 115200bps each byte (10 bits including start/stop) ≈ 87 µs.
    }
    if (isUartLineReady) {
        processUartLine();

        currentStats.nmeaMessages++;
        if (strncmp(uartLineBuffer + 3, "RMC", 3) == 0) {
            currentStats.rmsMessages++;
        }
        if (strncmp(uartLineBuffer + 3, "GGA", 3) == 0) {
            currentStats.ggaMessages++;
        }
    }    

    // Check if TinyGPS++ has parsed a complete sentence with valid location
    if (gps.location.isUpdated()) {
        DEBUG_NMEA_PRINT(F(" Date: "));
        if (gps.date.isValid()) {
            data.date = gps.date.year() * 10000 + gps.date.month() * 100 + gps.date.day();
            DEBUG_NMEA_PRINT(gps.date.year()); DEBUG_NMEA_PRINT(F("-")); DEBUG_NMEA_PRINT(gps.date.month()); DEBUG_NMEA_PRINT(F("-")); DEBUG_NMEA_PRINT(gps.date.day());
        }
        else {
            DEBUG_NMEA_PRINT(F("N/A"));
        }

        DEBUG_NMEA_PRINT(F(" Time: "));
        if (gps.time.isValid()) {
            data.timeMs = (gps.time.hour() * 3600000) + (gps.time.minute() * 60000) + (gps.time.second() * 1000) + (gps.time.centisecond() * 10);

            if (gps.time.hour() < 10) DEBUG_NMEA_PRINT(F("0"));
            DEBUG_NMEA_PRINT(gps.time.hour()); DEBUG_NMEA_PRINT(F(":"));
            if (gps.time.minute() < 10) DEBUG_NMEA_PRINT(F("0"));
            DEBUG_NMEA_PRINT(gps.time.minute()); DEBUG_NMEA_PRINT(F(":"));
            if (gps.time.second() < 10) DEBUG_NMEA_PRINT(F("0"));
            DEBUG_NMEA_PRINT(gps.time.second()); DEBUG_NMEA_PRINT(F("."));
            if (gps.time.centisecond() < 10) DEBUG_NMEA_PRINT(F("0"));
            DEBUG_NMEA_PRINT(gps.time.centisecond());
        }
        else {
            DEBUG_NMEA_PRINT(F("N/A"));
        }

        DEBUG_NMEA_PRINT(F(" Location: ")); 
        if (gps.location.isValid()) {
            data.latitude = (gps.location.rawLat().deg * 10000000 + gps.location.rawLat().billionths / 100) * (gps.location.rawLat().negative ? -1 : 1);
            data.longitude = (gps.location.rawLng().deg * 10000000 + gps.location.rawLng().billionths / 100) * (gps.location.rawLng().negative ? -1 : 1);
            data.fixQuality = (gps.location.FixQuality() - TinyGPSLocation::Invalid); // GPS = '1', DGPS = '2', PPS = '3', RTK = '4', FloatRTK = '5', Estimated = '6', Manual = '7', Simulated = '8'
            DEBUG_NMEA_PRINT(gps.location.lat(), 6); DEBUG_NMEA_PRINT(F(", ")); DEBUG_NMEA_PRINT(gps.location.lng(), 6); DEBUG_NMEA_PRINT(F(" Q")); DEBUG_NMEA_PRINT((char)gps.location.FixQuality());
        }
        else {
            DEBUG_NMEA_PRINT(F("N/A"));
        }

        DEBUG_NMEA_PRINT(F(" Altitude: "));
        if (gps.altitude.isValid()) {
            data.altitudeX100 = gps.altitude.value();
            DEBUG_NMEA_PRINT(data.altitudeX100/100); DEBUG_NMEA_PRINT(F("m"));
        }
        else {
            DEBUG_NMEA_PRINT(F("N/A"));
        }

        DEBUG_NMEA_PRINT(F(" Speed: "));
        if (gps.speed.isValid()) {
            data.speedX100 = gps.speed.kmph() * 100.0;
            DEBUG_NMEA_PRINT(data.speedX100/100); DEBUG_NMEA_PRINT(F("km/h"));
        }
        else {
            DEBUG_NMEA_PRINT(F("N/A"));
        }

        DEBUG_NMEA_PRINT(F(" Bearing: "));
        if (gps.course.isValid()) {
            data.bearingX100 = gps.course.value();
            DEBUG_NMEA_PRINT(data.bearingX100/100); DEBUG_NMEA_PRINT(F("*"));
        }
        else {
            DEBUG_NMEA_PRINT(F("N/A"));
        }

        DEBUG_NMEA_PRINT(F(" Satellites: "));
        if (gps.satellites.isValid()) {
            data.satellites = gps.satellites.value();
            DEBUG_NMEA_PRINT(gps.satellites.value());
        }
        else {
            DEBUG_NMEA_PRINT(F("N/A"));
        }

        DEBUG_NMEA_PRINT(F(" HDOP: "));
        if (gps.hdop.isValid()) {
            data.hdopX100 = gps.hdop.value();
            DEBUG_NMEA_PRINT(data.hdopX100); DEBUG_NMEA_PRINTLN(F("cm"));
        }
        else {
            DEBUG_NMEA_PRINT(F("N/A"));
        }
        DEBUG_NMEA_PRINTLN();
        data.timestamp = millis();
        return true;
    }
    
    return false;
}

static void intToHex2digits(uint8_t value, char* output) {
    const char hexDigits[] = "0123456789ABCDEF";
    output[0] = hexDigits[(value >> 4) & 0x0F];
    output[1] = hexDigits[value & 0x0F];
    output[2] = '\0';
}

GPS::Stats GPS::getStats() {
    return gpsStats;
}

void GPS::sendCommand(const char* command, ResponseCallback callback) {
    // Store the callback for upcoming response
    pendingCallback = callback;
    
    char fullCommand[128];
    memset(fullCommand, 0, sizeof(fullCommand));
    fullCommand[0] = '$';
    strcpy(&fullCommand[1], command);
    
    uint8_t checksum = calculateNmeaChecksum(&fullCommand[1]);
    fullCommand[strlen(fullCommand)] = '*';
    intToHex2digits(checksum, fullCommand + strlen(fullCommand));
    fullCommand[strlen(fullCommand)] = '\r';
    fullCommand[strlen(fullCommand)] = '\n';

    DEBUG_PRINT("Sending: "); DEBUG_PRINT(fullCommand);    
    GPS_SERIAL.print(fullCommand);
}

void GPS::processUartLine() {
    // Check for valid NMEA/command format (starts with $ and has minimum length)
    if (uartLineBufferLen < 6 || uartLineBuffer[0] != '$') {
        Serial.print("ERROR: GPS UART invalid line format: "); Serial.println(uartLineBuffer);
        return;
    }
    
    if (memcmp(uartLineBuffer, "$PQTM", 5) == 0) {
        processQueryResponse();
    } 
    else if (memcmp(uartLineBuffer, "$PAIR", 5) == 0) {
        processQueryResponse();
    } 
    else {
        // Assume a standard NMEA sentence (e.g., GNGGA, GNRMC, GPGGA, etc.)
        // Format: $<TalkerID><SentenceType>,data
        // TalkerID is 2 chars (GN, GP, GL, GA, GB, etc.)
        // SentenceType is 3 chars (GGA, RMC, GSA, GSV, etc.)
        // Minimum valid NMEA: $GPGGA,... (6 chars before comma)
        processNmeaResponse();
    }
}

void GPS::processQueryResponse() {
    DEBUG_PRINTLN(uartLineBuffer);
    
    const char *asteriskPtr = strchr(uartLineBuffer, '*');
    if (asteriskPtr != NULL) {
        char resultStr[256] = {0};
        int len = asteriskPtr - (uartLineBuffer + 1); // Skip initial '$'
        strncpy(resultStr, uartLineBuffer + 1, len);
        resultStr[len] = '\0';

        // char checksum[3] = {0};
        // checksum[0] = *(asteriskPtr + 1);
        // checksum[1] = *(asteriskPtr + 2);
        // checksum[2] = '\0';

        if ((strncmp(uartLineBuffer, "$PAIR001", sizeof("$PAIR001")-1) == 0) && (uartLineBuffer[sizeof("$PAIR001,xxx,")-1] == '0' || uartLineBuffer[sizeof("$PAIR001,xxx,")-1] == '1')) {
            DEBUG_PRINTLN("Skip PAIR001,xxx,0/1 response (Command sent / is being processed)");
            return;
        }
        if (pendingCallback != NULL) {
            pendingCallback(resultStr);
        }
    }
    else {
        Serial.println("ERROR: GPS UART no checksum in response");
        if (pendingCallback != NULL) {
            pendingCallback(uartLineBuffer);
        }
    }
    pendingCallback = NULL;
}

void GPS::processNmeaResponse() {
    // Feed the complete NMEA sentence to TinyGPS++ for parsing
    for (int i = 0; i < uartLineBufferLen; i++) {
        gps.encode(uartLineBuffer[i]);
    }
    gps.encode('\r');
    gps.encode('\n');
}
