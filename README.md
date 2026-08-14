# STM32 UBX GPS → MSP Bridge

Firmware for a Blackpill STM32F4 board that autodetects a u-blox (UBX protocol) GPS module, configures it for high-rate NAV-PVT output, and forwards fix data to a flight controller over **MSPv2** (`MSP2_SENSOR_GPS`, 0x1F03). A strip of WS2812 (NeoPixel) LEDs gives a live visual status of the GPS lock process.

Built with [PlatformIO](https://platformio.org/) and the Arduino framework.

### Intended use

The `USART2` output is written as standard MSPv2 for a flight controller, but in this project's actual usage it's fed into **ExpressLRS's AirPort** feature, which tunnels that MSP traffic over the ExpressLRS RF link to a **ground station** instead of an onboard flight controller. Wire `USART2` to whichever device you're targeting — a flight controller directly, or an ExpressLRS receiver/transmitter's AirPort UART.

---

## Hardware

- **Board:** Blackpill STM32F411CE (also has an `blackpill_f401ce` build target)
- **GPS module:** any u-blox module speaking the UBX binary protocol (M8N, M9N, M10, etc.)
- **LEDs:** WS2812 / NeoPixel-compatible addressable RGB strip, 6 LEDs
- **Flight controller:** anything that accepts MSPv2 GPS sensor telemetry (e.g. INAV) — or, as used in this project, an ExpressLRS AirPort link to a ground station instead

### Wiring

| Signal | STM32 Pin | Peripheral | Connects to |
|---|---|---|---|
| GPS TX  → STM32 RX | `PA10` | `USART1` (RX) | GPS module TX |
| STM32 TX → GPS RX  | `PA9`  | `USART1` (TX) | GPS module RX |
| FC TX → STM32 RX   | `PB7`* | `USART2` (RX) | Flight controller TX (or ExpressLRS AirPort UART TX) |
| STM32 TX → FC RX   | `PB6`* | `USART2` (TX) | Flight controller RX (or ExpressLRS AirPort UART RX) |
| LED data           | `PB0`  | —            | WS2812 strip DIN |

\* `USART2`'s default pins on the Blackpill are `PA2`/`PA3`, but this project uses the **remapped** `PB6`/`PB7` pins instead (see the comment in `main.cpp`) — wire the flight controller (or ExpressLRS AirPort UART) to `PB6`/`PB7`, not `PA2`/`PA3`.

Also connect GND between the STM32, GPS module, and flight controller / ExpressLRS device.

### Serial rates

- **GPS (`USART1`):** autodetected, then reconfigured to and run at **115200 baud**, UBX protocol only (NMEA disabled).
- **Flight controller / ExpressLRS AirPort (`USART2`):** fixed at **9600 baud** — set the AirPort UART baud on the ExpressLRS side to match if that's what you're using.
- **USB (debug):** 115200 baud, printed to the Serial Monitor via `Serial.println(...)`.

---

## What it does

### 1. GPS autodetection (`UBXGps::autodetect`)

On boot, the module's current baud rate is unknown, so the firmware cycles through the common u-blox boot rates in order:

```
115200 → 9600 → 38400 → 57600 → 19200 → 230400
```

At each rate it sends a **poll request for `CFG-PRT`** (an empty-payload UBX message that any UBX-capable module answers regardless of its current output configuration) and waits up to 300 ms for *any* well-formed UBX frame in response. The first baud rate that gets a reply is adopted.

If no rate responds after all attempts, the firmware halts and blinks a fail pattern (see LEDs below) — it does not continue into `loop()`.

### 2. GPS configuration (`UBXGps::configure`)

Once detected, the module is reconfigured via `CFG-PRT`, `CFG-RATE`, and `CFG-MSG`:

- UART1 baud set to the target rate (115200), 8N1
- Input/output protocol restricted to **UBX only** (NMEA output disabled)
- Navigation/measurement rate set to `140 ms` (~7 Hz)
- `NAV-PVT` enabled on the UART port at that rate

### 3. UBX parsing (`UBXGps::update` / `parseByte`)

A byte-at-a-time state machine (`SYNC1 → SYNC2 → CLASS → ID → LEN → PAYLOAD → CK_A → CK_B`) parses incoming UBX frames and validates their 8-bit Fletcher checksum. When a checksum-valid `NAV-PVT` (class `0x01`, id `0x07`) frame completes, its payload is copied into a `ubxNavPvt_t` struct and `update()` returns `true` for that `loop()` iteration.

`hasFix()` reports true once `fixType >= 3` (3D fix or better).

### 4. Forwarding to the flight controller (`sendMSP`)

Every time a fresh `NAV-PVT` frame is parsed **and has a 3D fix**, `main.cpp` builds and sends an MSPv2 `MSP2_SENSOR_GPS` (`0x1F03`) packet over `USART2`, containing:

- GPS time of week, fix type, satellite count
- Horizontal/vertical position accuracy, horizontal velocity accuracy, DOP
- Longitude, latitude, MSL altitude
- North/East/Down velocity
- Ground course
- UTC date/time

Units are converted from UBX's native scales (mm, mm/s, 1e-7 deg, etc.) to what INAV-style MSP GPS sensor frames expect (cm, cm/s, etc.) before sending. GPS week and true yaw aren't available from `NAV-PVT` and are sent as `0xFFFF` ("not available").

The MSPv2 frame is wrapped as `$X<`, a flags byte, the 16-bit function ID, a 16-bit payload length, the payload, and a single **CRC-8/DVB-S2** checksum byte covering everything after `$X<`.

In this project's setup, that MSPv2 frame is picked up by an **ExpressLRS AirPort** UART rather than a flight controller directly — AirPort tunnels it over the ExpressLRS RF link so the GPS data shows up at a ground station.

---

## Status LEDs

6 WS2812 LEDs on `PB0` show what the firmware is doing at every stage. All LED logic lives in `main.cpp`.

| Stage | Pattern | Meaning |
|---|---|---|
| **Startup** | A single green pixel chases forward across all 6 LEDs, then back — once, blocking | Firmware has booted and is about to start GPS detection |
| **Searching for GPS** | One additional **yellow** LED lights up per failed baud-rate attempt (LED *n* lights on attempt *n*) | Autodetect is cycling through candidate baud rates |
| **GPS not found** | All 6 LEDs blink **red** together, forever (300 ms on / 300 ms off) | Autodetect failed on every candidate baud rate — firmware halts here, `loop()` never runs |
| **GPS found, configuring** | All 6 LEDs solid **dim blue** | Module was detected and is being configured; steady state while waiting for a fix |
| **3D fix acquired** | LEDs blink **green** as a group, 800 ms period (200 ms on / 600 ms off); the **number of LEDs lit** = `numSV / 2` (capped at 6) | Each blink shows how many satellites are currently in the fix — more LEDs lit ≈ more satellites |
| **Fix lost** | Reverts to whatever `ledUpdateFix`/no-fix state applies (no dedicated "lost fix" pattern beyond simply not blinking green again) | `gpsFixValid` is cleared and a "No 3D fix yet" message is printed over USB serial |

Brightness for all LEDs is set once at boot via `strip.setBrightness(60)`.

---

## Dependencies

- [Adafruit NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel) `1.15.5`
- [ststm32](https://github.com/platformio/platform-ststm32) PlatformIO platform, Arduino framework

## Building

```bash
pio run -e blackpill_f411ce -t upload
```

Open the serial monitor (115200 baud) to see debug output including detected fix status, satellite count, and coordinates.

---

## Notes / gotchas

- If the GPS module has previously been configured to output NMEA in addition to UBX, autodetection may still find it (it only needs *any* valid UBX frame back), but `configure()` disables NMEA output afterward.
- `USART2` uses the `PB6`/`PB7` remap rather than the Blackpill's default `PA2`/`PA3` — double-check this if you're adapting the wiring table for a different board.
- The GPS-not-found state is terminal: the firmware blocks forever blinking red rather than retrying, so a power cycle is needed if the GPS is connected after boot.
