#include "UBXGps.h"

// Baud rates u-blox modules commonly boot at / accept.
// Order matters: try the target rate first (fast path if already configured),
// then fall back through the usual defaults, same idea as INAV's gpsFindNextBaudrate().
static const uint32_t kCandidateBauds[] = {
    115200, 9600, 38400, 57600, 19200, 230400
};

UBXGps::UBXGps(HardwareSerial &serialPort, uint32_t targetBaud)
    : serial(serialPort), baud(targetBaud) {}

void UBXGps::checksum(const uint8_t *data, uint16_t len, uint8_t &a, uint8_t &b) {
    a = 0; b = 0;
    for (uint16_t i = 0; i < len; i++) {
        a += data[i];
        b += a;
    }
}

void UBXGps::sendUBX(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
    uint8_t header[4] = { cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };

    uint8_t a = 0, b = 0;
    // checksum covers class,id,len,payload -- accumulate header then payload
    for (uint16_t i = 0; i < 4; i++) { a += header[i]; b += a; }
    for (uint16_t i = 0; i < len; i++) { a += payload[i]; b += a; }

    serial.write(UBX_SYNC1);
    serial.write(UBX_SYNC2);
    serial.write(header, 4);
    if (len) serial.write(payload, len);
    serial.write(a);
    serial.write(b);
}

void UBXGps::resetParser() {
    state = State::SYNC1;
    payloadIdx = 0;
}

bool UBXGps::parseByte(uint8_t b) {
    switch (state) {
        case State::SYNC1:
            if (b == UBX_SYNC1) state = State::SYNC2;
            break;

        case State::SYNC2:
            state = (b == UBX_SYNC2) ? State::CLASS : State::SYNC1;
            break;

        case State::CLASS:
            msgClass = b;
            state = State::ID;
            break;

        case State::ID:
            msgId = b;
            state = State::LEN1;
            break;

        case State::LEN1:
            payloadLen = b;
            state = State::LEN2;
            break;

        case State::LEN2:
            payloadLen |= (uint16_t)b << 8;
            payloadIdx = 0;
            if (payloadLen == 0) {
                state = State::CK_A;
            } else if (payloadLen > sizeof(payloadBuf)) {
                // frame we don't care about the body of (too big for our buffer,
                // e.g. MON-VER) -- still consume it so the checksum bytes line up
                state = State::PAYLOAD;
            } else {
                state = State::PAYLOAD;
            }
            break;

        case State::PAYLOAD:
            if (payloadIdx < sizeof(payloadBuf)) {
                payloadBuf[payloadIdx] = b;
            }
            payloadIdx++;
            if (payloadIdx >= payloadLen) {
                state = State::CK_A;
            }
            break;

        case State::CK_A:
            rxCkA = b;
            state = State::CK_B;
            break;

        case State::CK_B: {
            rxCkB = b;
            state = State::SYNC1;

            // verify checksum over class+id+len+payload
            uint8_t a = 0, bb = 0;
            uint8_t hdr[4] = { msgClass, msgId,
                                (uint8_t)(payloadLen & 0xFF), (uint8_t)(payloadLen >> 8) };
            for (uint8_t i = 0; i < 4; i++) { a += hdr[i]; bb += a; }
            uint16_t n = payloadLen < sizeof(payloadBuf) ? payloadLen : sizeof(payloadBuf);
            for (uint16_t i = 0; i < n; i++) { a += payloadBuf[i]; bb += a; }

            if (a != rxCkA || bb != rxCkB) {
                return false; // corrupt frame, drop silently
            }

            anyFrameSeen = true; // any well-formed UBX frame counts for autodetect

            if (msgClass == UBX_CLASS_NAV && msgId == UBX_ID_NAV_PVT &&
                payloadLen == sizeof(ubxNavPvt_t)) {
                memcpy(&lastFix, payloadBuf, sizeof(ubxNavPvt_t));
                return true;
            }
            break;
        }
    }
    return false;
}

bool UBXGps::waitAnyFrame(uint32_t timeoutMs) {
    anyFrameSeen = false;
    resetParser();
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        while (serial.available()) {
            parseByte((uint8_t)serial.read());
            if (anyFrameSeen) return true;
        }
    }
    return false;
}

bool UBXGps::autodetect(uint32_t timeoutPerBaudMs) {
    for (uint32_t candidate : kCandidateBauds) {
        serial.end();
        delay(10);
        serial.begin(candidate);
        delay(50); // let the UART settle

        // flush whatever garbage is sitting in the RX buffer
        while (serial.available()) serial.read();

        // Poll CFG-PRT (empty payload = poll request). Any module answers this
        // over UBX regardless of current NMEA/UBX output config, which is what
        // makes it useful for detection rather than a message we've enabled.
        sendUBX(UBX_CLASS_CFG, UBX_ID_CFG_PRT, nullptr, 0);

        if (waitAnyFrame(timeoutPerBaudMs)) {
            baud = candidate;
            return true;
        }
    }
    return false;
}

bool UBXGps::configure(uint16_t navPvtRateMs) {
    // --- 1. Set the module's UART1 baud to targetBaud, UBX in/out only -----
    // CFG-PRT payload for UART port (portID=1):
    // offset: 0 portID,1 reserved,2-3 txReady,4-7 mode,8-11 baudrate,
    //         12-13 inProtoMask,14-15 outProtoMask,16-17 flags,18-19 reserved
    uint8_t prt[20] = {0};
    prt[0] = 0x01;                    // portID = 1 (UART1)
    // mode: 8N1, no parity -> 0x000008D0 (standard u-blox value)
    prt[4] = 0xD0; prt[5] = 0x08; prt[6] = 0x00; prt[7] = 0x00;
    prt[8]  = (uint8_t)(baud & 0xFF);
    prt[9]  = (uint8_t)((baud >> 8) & 0xFF);
    prt[10] = (uint8_t)((baud >> 16) & 0xFF);
    prt[11] = (uint8_t)((baud >> 24) & 0xFF);
    prt[12] = 0x01; prt[13] = 0x00;   // inProtoMask  = UBX only
    prt[14] = 0x01; prt[15] = 0x00;   // outProtoMask = UBX only
    sendUBX(UBX_CLASS_CFG, UBX_ID_CFG_PRT, prt, sizeof(prt));

    // module may switch baud immediately; re-open our side at the same rate
    delay(100);
    serial.end();
    delay(10);
    serial.begin(baud);
    delay(50);
    while (serial.available()) serial.read();

    // --- 2. Set nav/measurement rate ---------------------------------------
    // CFG-RATE: measRate(ms), navRate(cycles), timeRef(0=UTC)
    uint8_t rate[6];
    rate[0] = (uint8_t)(navPvtRateMs & 0xFF);
    rate[1] = (uint8_t)(navPvtRateMs >> 8);
    rate[2] = 1; rate[3] = 0;   // navRate = 1 (every measurement)
    rate[4] = 1; rate[5] = 0;   // timeRef = GPS time
    sendUBX(UBX_CLASS_CFG, UBX_ID_CFG_RATE, rate, sizeof(rate));
    delay(50);

    // --- 3. Enable NAV-PVT on this port, rate = every cycle ---------------
    // CFG-MSG: msgClass, msgId, rate-per-port[6] (we only set UART1 = index 1)
    uint8_t msg[8] = { UBX_CLASS_NAV, UBX_ID_NAV_PVT, 0, 1, 0, 0, 0, 0 };
    sendUBX(UBX_CLASS_CFG, UBX_ID_CFG_MSG, msg, sizeof(msg));
    delay(50);

    resetParser();
    return true;
}

bool UBXGps::update() {
    bool gotFix = false;
    while (serial.available()) {
        if (parseByte((uint8_t)serial.read())) {
            gotFix = true;
        }
    }
    return gotFix;
}
