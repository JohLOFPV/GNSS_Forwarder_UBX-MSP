#include <Arduino.h>
#include "UBXGps.h"

HardwareSerial GPSSerial(USART1); // PA10(RX), PA9(TX) are default for USART1 on Blackpill
HardwareSerial FCSerial(USART2);  // PA3(RX), PA2(TX) are default for USART2, BUT you want PB7/PB6


// GPS wired to USART1: PA9 = TX (STM32) -> GPS RX, PA10 = RX (STM32) <- GPS TX
UBXGps gps(GPSSerial, 115200);


uint32_t mspCount = 0;


#define MSP2_SENSOR_GPS 0x1F03


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
    Serial.begin(115200);   // USB CDC, for debug prints
    delay(2000);
    Serial.println("Autodetecting GPS...");

    if (!gps.autodetect()) {
        Serial.println("No UBX GPS found on any known baud rate.");
        while (true) { delay(1000); }
    }

    Serial.println("GPS found, configuring...");
    gps.configure(140); // NAV-PVT every 200ms (5Hz)

    FCSerial.begin(9600);
}

void loop() {
    if (gps.update()) {
        const ubxNavPvt_t &f = gps.fix();
        if (gps.hasFix()) {
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
            Serial.print("No 3D fix yet, fixType=");
            Serial.println(f.fixType);
        }
    }
}
