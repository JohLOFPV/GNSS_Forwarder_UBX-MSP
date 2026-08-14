#pragma once
#include <Arduino.h>

// ---- UBX framing ----------------------------------------------------------
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

// ---- Classes ----------------------------------------------------------
#define UBX_CLASS_NAV 0x01
#define UBX_CLASS_ACK 0x05
#define UBX_CLASS_CFG 0x06
#define UBX_CLASS_MON 0x0A

// ---- IDs ----------------------------------------------------------
#define UBX_ID_NAV_PVT   0x07
#define UBX_ID_ACK_NAK   0x00
#define UBX_ID_ACK_ACK   0x01
#define UBX_ID_CFG_PRT   0x00
#define UBX_ID_CFG_MSG   0x01
#define UBX_ID_CFG_RATE  0x08
#define UBX_ID_MON_VER   0x04

// Fix types (NAV-PVT.fixType)
enum ubxFixType_e {
    UBX_FIX_NONE = 0,
    UBX_FIX_DEAD_RECKONING = 1,
    UBX_FIX_2D = 2,
    UBX_FIX_3D = 3,
    UBX_FIX_GNSS_DEAD_RECKONING = 4,
    UBX_FIX_TIME_ONLY = 5,
};

#pragma pack(push, 1)
typedef struct {
    uint32_t iTOW;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  sec;
    uint8_t  valid;
    uint32_t tAcc;
    int32_t  nano;
    uint8_t  fixType;
    uint8_t  flags;
    uint8_t  flags2;
    uint8_t  numSV;
    int32_t  lon;     // 1e-7 deg
    int32_t  lat;     // 1e-7 deg
    int32_t  height;  // mm, ellipsoid
    int32_t  hMSL;    // mm, MSL
    uint32_t hAcc;    // mm
    uint32_t vAcc;    // mm
    int32_t  velN;    // mm/s
    int32_t  velE;
    int32_t  velD;
    int32_t  gSpeed;  // mm/s ground speed
    int32_t  headMot; // 1e-5 deg
    uint32_t sAcc;    // mm/s
    uint32_t headAcc; // 1e-5 deg
    uint16_t pDOP;
    uint8_t  reserved1[6];
    int32_t  headVeh;
    int16_t  magDec;
    uint16_t magAcc;
} ubxNavPvt_t;
#pragma pack(pop)

class UBXGps {
public:
    // serialPort: the HardwareSerial wired to the GPS module's TX/RX.
    // targetBaud: baud rate the GPS module gets configured to and the port runs at afterwards.
    explicit UBXGps(HardwareSerial &serialPort, uint32_t targetBaud = 115200);

    // Cycles known baud rates, polls CFG-PRT, waits for any valid UBX frame back.
    // Returns true and leaves 'serial' running at the detected baud if found.
    bool autodetect(uint32_t timeoutPerBaudMs = 300);

    // Reconfigures the module: sets its UART baud to targetBaud, disables NMEA
    // output, enables UBX-only output, enables NAV-PVT at the given rate (ms).
    // Call after autodetect() succeeds.
    bool configure(uint16_t navPvtRateMs = 200);

    // Call every loop() iteration; feeds available bytes into the parser.
    // Returns true exactly on the iteration a fresh NAV-PVT finished parsing.
    bool update();

    bool hasFix() const { return lastFix.fixType >= UBX_FIX_3D; }
    const ubxNavPvt_t &fix() const { return lastFix; }

private:
    enum class State : uint8_t {
        SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CK_A, CK_B
    };

    HardwareSerial &serial;
    uint32_t baud;

    State state = State::SYNC1;
    uint8_t msgClass = 0, msgId = 0;
    uint16_t payloadLen = 0, payloadIdx = 0;
    uint8_t payloadBuf[92]; // NAV-PVT payload is 92 bytes
    uint8_t ckA = 0, ckB = 0, rxCkA = 0, rxCkB = 0;

    // set by parseByte() when *any* complete, checksum-valid UBX frame arrives
    volatile bool anyFrameSeen = false;

    ubxNavPvt_t lastFix{};

    void resetParser();
    bool parseByte(uint8_t b);       // returns true on a validated NAV-PVT frame
    void sendUBX(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);
    static void checksum(const uint8_t *data, uint16_t len, uint8_t &a, uint8_t &b);

    bool waitAnyFrame(uint32_t timeoutMs);
};
