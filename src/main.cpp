#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "UBXGps.h"

HardwareSerial GPSSerial(USART1); // PA10(RX), PA9(TX) are default for USART1 on Blackpill
HardwareSerial FCSerial(USART2);  // PA3(RX), PA2(TX) are default for USART2, BUT you want PB7/PB6

// GPS wired to USART1: PA9 = TX (STM32) -> GPS RX, PA10 = RX (STM32) <- GPS TX
UBXGps gps(GPSSerial, 115200);

uint32_t mspCount = 0;

#define MSP2_SENSOR_GPS 0x1F03

// ---------------------------------------------------------------------
// WS2812 status LED strip
// ---------------------------------------------------------------------
#define LED_PIN   PB0
#define NUM_LEDS  6

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

static inline uint32_t colGreen()    { return strip.Color(0, 100, 0); }
static inline uint32_t colBlueDim()  { return strip.Color(5, 5, 15); }
static inline uint32_t colYellow()   { return strip.Color(100, 50, 0); }
static inline uint32_t colRed()      { return strip.Color(100, 0, 0); }
static inline uint32_t colOff()      { return strip.Color(0, 0, 0); }

// Startup: green chase across the strip, once, blocking (only runs at boot)
void ledStartupAnimation() {
    strip.clear();
    strip.show();
 
    // forward: 0 -> NUM_LEDS-1
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        strip.clear();
        strip.setPixelColor(i, colGreen());
        strip.show();
        delay(120);
    }
    // back: NUM_LEDS-2 -> 0 (skip the last pixel, already shown)
    for (int8_t i = NUM_LEDS - 2; i >= 0; i--) {
        strip.clear();
        strip.setPixelColor(i, colGreen());
        strip.show();
        delay(120);
    }

    strip.clear();
    strip.show();
}

// GPS detect: light up one more yellow LED per failed attempt (1-indexed)
void ledGpsSearchStep(uint8_t attempt) {
    if (attempt >= 1 && attempt <= NUM_LEDS) {
        strip.setPixelColor(attempt - 1, colYellow());
        strip.show();
    }
}

// GPS detect failed after all attempts: blink all red forever (blocking - nothing else to do)
void ledGpsFail() {
    while (true) {
        for (uint8_t i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, colRed());
        strip.show();
        delay(300);
        strip.clear();
        strip.show();
        delay(300);
    }
}

// GPS detected successfully: turn all LEDs to dim blue
void ledGpsReady() {
    for (uint8_t i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, colBlueDim());
    strip.show();
}

// GPS has a fix: blink green, 800ms period / 200ms on, number of lit LEDs = numSV / 2
const uint16_t LED_FIX_PERIOD_MS = 800;
const uint16_t LED_FIX_ON_MS     = 200;
bool ledFixOn = false;

void ledUpdateFix(uint8_t numSV) {
    uint32_t phase = millis() % LED_FIX_PERIOD_MS;
    bool shouldBeOn = phase < LED_FIX_ON_MS;

    if (shouldBeOn == ledFixOn) return; // nothing to change yet
    ledFixOn = shouldBeOn;

    if (ledFixOn) {
        uint8_t litCount = numSV / 2;
        if (litCount > NUM_LEDS) litCount = NUM_LEDS;
        for (uint8_t i = 0; i < NUM_LEDS; i++) {
            strip.setPixelColor(i, i < litCount ? colGreen() : colOff());
        }
    } else {
        strip.clear();
    }
    strip.show();
}

// ---------------------------------------------------------------------
// GPS fix state, updated in loop(), consumed by ledUpdateFix() every pass
// ---------------------------------------------------------------------
bool gpsFixValid = false;
uint8_t gpsNumSV = 0;

uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    crc ^= a;

    for (int i = 0; i < 8; i++)
    {
        if (crc & 0x80)
            crc = (crc << 1) ^ 0xD5;
        else
            crc <<= 1;
    }

    return crc;
}

void sendMSP(uint8_t instance,
             uint16_t gpsWeek, uint32_t msTOW,
             uint8_t fixType, uint8_t satellitesInView,
             uint16_t horizontalPosAccuracy, uint16_t verticalPosAccuracy,
             uint16_t horizontalVelAccuracy, uint16_t hdop,
             int32_t longitude, int32_t latitude, int32_t mslAltitude,
             int32_t nedVelNorth, int32_t nedVelEast, int32_t nedVelDown,
             uint16_t groundCourse, uint16_t trueYaw,
             uint16_t year, uint8_t month, uint8_t day,
             uint8_t hour, uint8_t min, uint8_t sec)
{
    uint8_t payload[41];
    uint8_t idx = 0;

    payload[idx++] = instance;
    memcpy(&payload[idx], &gpsWeek, 2); idx += 2;
    memcpy(&payload[idx], &msTOW, 4); idx += 4;
    payload[idx++] = fixType;
    payload[idx++] = satellitesInView;
    memcpy(&payload[idx], &horizontalPosAccuracy, 2); idx += 2;
    memcpy(&payload[idx], &verticalPosAccuracy, 2); idx += 2;
    memcpy(&payload[idx], &horizontalVelAccuracy, 2); idx += 2;
    memcpy(&payload[idx], &hdop, 2); idx += 2;
    memcpy(&payload[idx], &longitude, 4); idx += 4;
    memcpy(&payload[idx], &latitude, 4); idx += 4;
    memcpy(&payload[idx], &mslAltitude, 4); idx += 4;
    memcpy(&payload[idx], &nedVelNorth, 4); idx += 4;
    memcpy(&payload[idx], &nedVelEast, 4); idx += 4;
    memcpy(&payload[idx], &nedVelDown, 4); idx += 4;
    memcpy(&payload[idx], &groundCourse, 2); idx += 2;
    memcpy(&payload[idx], &trueYaw, 2); idx += 2;
    memcpy(&payload[idx], &year, 2); idx += 2;
    payload[idx++] = month;
    payload[idx++] = day;
    payload[idx++] = hour;
    payload[idx++] = min;
    payload[idx++] = sec;

    const uint16_t payloadSize = idx; // should be 41

    FCSerial.write('$');
    FCSerial.write('X');
    FCSerial.write('<');

    uint8_t flags = 0;
    FCSerial.write(flags);
    FCSerial.write((uint8_t)(MSP2_SENSOR_GPS & 0xFF));
    FCSerial.write((uint8_t)(MSP2_SENSOR_GPS >> 8));
    FCSerial.write((uint8_t)(payloadSize & 0xFF));
    FCSerial.write((uint8_t)(payloadSize >> 8));

    uint8_t crc = 0;
    crc = crc8_dvb_s2(crc, flags);
    crc = crc8_dvb_s2(crc, MSP2_SENSOR_GPS & 0xFF);
    crc = crc8_dvb_s2(crc, MSP2_SENSOR_GPS >> 8);
    crc = crc8_dvb_s2(crc, payloadSize & 0xFF);
    crc = crc8_dvb_s2(crc, payloadSize >> 8);

    for (uint16_t i = 0; i < payloadSize; i++) {
        FCSerial.write(payload[i]);
        crc = crc8_dvb_s2(crc, payload[i]);
    }

    FCSerial.write(crc);
    mspCount++;
}

void setup() {
    strip.begin();
    strip.setBrightness(60); // 0-255, adjust to taste / current budget
    strip.show();


    Serial.begin(115200);   // USB CDC, for debug prints
    delay(1000);
    Serial.println("Autodetecting GPS..."); 
    
    ledStartupAnimation();

    bool gpsFound = false;
    for (uint8_t attempt = 1; attempt <= NUM_LEDS; attempt++) {
        ledGpsSearchStep(attempt);
        if (gps.autodetect()) {
            gpsFound = true;
            break;
        }
    }

    if (!gpsFound) {
        Serial.println("No UBX GPS found on any known baud rate.");
        ledGpsFail(); // blocks forever, blinking red
    }

    Serial.println("GPS found, configuring...");

    ledGpsReady();
    gps.configure(140); // NAV-PVT every 140ms (7Hz)

    FCSerial.begin(9600);
}

void loop() {
    if (gps.update()) {
        const ubxNavPvt_t &f = gps.fix();
        if (gps.hasFix()) {
            gpsFixValid = true;
            gpsNumSV = f.numSV;

            Serial.print("Fix: ");
            Serial.print(f.numSV);
            Serial.print(" sats, lat=");
            Serial.print(f.lat / 1e7, 7);
            Serial.print(" lon=");
            Serial.print(f.lon / 1e7, 7);
            Serial.print(" hop=");
            Serial.print(f.pDOP);
            Serial.print(" alt=");
            Serial.print(f.hMSL / 1000.0, 1);
            Serial.println(" m");

            sendMSP(0,                    // instance
                    0xFFFF,               // gpsWeek - not in NAV-PVT, use NAV-TIMEGPS if you need it; 0xFFFF = "not available"
                    f.iTOW,                // msTOW - GPS time of week, ms
                    f.fixType,
                    f.numSV,
                    f.hAcc  / 10,          // horizontalPosAccuracy: hAcc is mm -> cm
                    f.vAcc  / 10,          // verticalPosAccuracy: mm -> cm
                    f.sAcc  / 10,          // horizontalVelAccuracy: speed accuracy mm/s -> cm/s
                    f.pDOP,                 // hdop - NAV-PVT pDOP is already scaled *100, matches INAV's expected scale
                    f.lon,
                    f.lat,
                    f.hMSL  / 10,          // mslAltitude: mm -> cm
                    f.velN  / 10,          // nedVelNorth: mm/s -> cm/s
                    f.velE  / 10,          // nedVelEast
                    f.velD  / 10,          // nedVelDown
                    f.headMot / 1000,      // groundCourse: deg*1e-5 -> deg*100
                    0xFFFF,                 // trueYaw - not available unless you have dual-antenna heading (NAV-RELPOSNED)
                    f.year,
                    f.month,
                    f.day,
                    f.hour,
                    f.minute,
                    f.sec
                    );

        } else {
            gpsFixValid = false;
            Serial.print("No 3D fix yet, fixType=");
            Serial.println(f.fixType);
        }
    }

    // Runs every loop pass (not just on new GPS data) so the blink timing stays smooth
    if (gpsFixValid) {
        ledUpdateFix(gpsNumSV);
    }
}