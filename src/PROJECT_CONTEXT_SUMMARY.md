# Equilibot ESP-IDF Project Summary

## 1) High-level overview

This project is an **ESP-IDF (v5.5.2) C++23 firmware** targeting **ESP32-S3** (`CONFIG_IDF_TARGET="esp32s3"`).

At runtime, it currently does three main things:

1. Initializes shared SPI bus and attached peripherals (IMU + two motor-driver devices)
2. Starts a Wi-Fi SoftAP + HTTP/WebSocket telemetry server
3. Runs an infinite loop reading IMU data at ~400 Hz, low-pass filtering it, and publishing telemetry every 50 ms (~20 Hz)

The codebase is modular and already split into reusable components (`spi_bus`, `icm42670_spi`, `web_server`, `led_strip`, `tmc5240`, filters).

---

## 2) Build/system configuration

- **Top-level CMake:** sets `CMAKE_CXX_STANDARD 23`
- **Main component sources:**
  - `main.cpp`
  - `led_strip.cpp`
  - `spi_bus.cpp`
  - `icm42670_spi.cpp`
  - `web_server.cpp`
  - `tmc5240.cpp`
- **Embedded asset:** `main/index.html` (served from firmware image)
- **Managed component dependency:** `espressif/led_strip` `3.0.3`
- **IDF/toolchain lock:** `idf 5.5.2`
- **Flash config (sdkconfig):** 16MB, dual OTA large partition scheme
- **HTTP/websocket related config:** `CONFIG_LWIP_MAX_SOCKETS=10`, `CONFIG_HTTPD_MAX_REQ_HDR_LEN=1024`

---

## 3) Main runtime flow (`main/main.cpp`)

### Peripherals and objects

- **SPI bus (`SPI2_HOST`)**
  - MISO = GPIO13
  - MOSI = GPIO14
  - SCLK = GPIO21
- **IMU (`ICM42670Spi`)**
  - CS = GPIO41
  - Config:
    - Accel FS ±2g, ODR 400Hz
    - Gyro FS ±250 dps, ODR 400Hz
    - Both in low-noise power mode
    - LPF register setting currently bypass
- **LED strip (`EquilibotLedStrip`)**
  - GPIO38
  - 31 LEDs partitioned into tilt/battery/motor/connection segments
- **Motor driver wrappers (`TMC5240`)**
  - Motor 1 CS = GPIO12
  - Motor 2 CS = GPIO10

### Loop behavior

- Uses `esp_timer_get_time()` to estimate `dt_seconds`
- Reads raw IMU sample (`acc`, `gyro`)
- Applies `LowPassIIR<3>`:
  - accel cutoff = 8 Hz
  - gyro cutoff = 10 Hz
- Every 50 ms, sends JSON telemetry over websocket with both raw and filtered vectors

### Notably present but not yet integrated

- Motor driver objects are instantiated but not commanded in `main.cpp`
- LED strip object is instantiated but UI update methods are not called in the loop
- No balancing/control loop yet (currently telemetry-focused firmware)

---

## 4) Module-by-module summary

## `spi_bus.*`

A thread-safe SPI bus manager:

- Initializes one bus instance
- Registers multiple devices by integer ID
- Provides synchronous `transfer()` call
- Uses FreeRTOS mutex (`StaticMutex` + RAII `LockGuard`)

This is the foundation for sharing SPI cleanly across IMU and motor devices.

## `icm42670_spi.*`

Custom SPI driver wrapper for ICM42670:

- Adds IMU as SPI device in mode CPOL1/CPHA1 @ 1 MHz
- Validates sensor identity (`WHO_AM_I == 0x67`)
- Configures gyro/accel range + ODR
- Configures LPF bits for gyro + accel
- Powers accel/gyro according to provided config
- Converts raw register readings to physical units:
  - accel in g
  - gyro in dps

Read API is clean: `ICM42670Sample read_sample()`.

## `filter.hpp`

Generic filter interface and a simple first-order low-pass IIR implementation:

- Templated on vector size (`LowPassIIR<3>` used for xyz)
- Adaptive alpha based on runtime `dt_seconds`
- Initializes state from first sample to avoid startup transient

## `web_server.*`

Hosts on-device dashboard and websocket telemetry endpoint:

- Initializes NVS + netif + event loop + Wi-Fi SoftAP
- AP credentials hardcoded:
  - SSID: `equilibot`
  - Password: `equilibot123`
- Serves `/` from embedded `index.html`
- WebSocket endpoint at `/ws`
- Broadcasts telemetry JSON to all connected websocket clients

Telemetry payload shape:

```json
{
  "t_ms": 12345,
  "acc":   {"x":...,"y":...,"z":...},
  "gyro":  {"x":...,"y":...,"z":...},
  "f_acc": {"x":...,"y":...,"z":...},
  "f_gyro":{"x":...,"y":...,"z":...}
}
```

## `index.html`

Single-page dashboard with:

- Live websocket connection status
- Numeric readouts for accel/gyro axes
- Two live canvas charts:
  - Acceleration (raw dashed + filtered solid)
  - Gyroscope (raw dashed + filtered solid)
- Auto reconnect logic
- Dynamic y-axis scaling

## `led_strip.*`

WS2812 LED abstraction with logical segments:

- Config validation (bounds + overlap checks)
- Segment-oriented APIs:
  - `set_tilt(float)`
  - `set_battery_level(int)`
  - `set_motor_speed(float, float)`
  - `set_connection_status(bool)`
- Thread-safe updates + push to strip

Currently ready, but not wired to runtime signals in `main.cpp`.

## `tmc5240.*`

Thin SPI read/write wrapper around TMC5240 register transactions:

- Adds each motor device to SPI bus
- Supports 5-byte write and two-phase read operation
- Returns SPI status byte from response

A base layer exists, but no high-level motor-control API yet.

---

## 5) Current architecture snapshot

- **Implemented well:** sensor IO, filtering, transport, live UI visualization, SPI sharing infrastructure
- **Partially implemented:** motor driver and LED status integration
- **Not implemented yet:** balancing controller/state estimator/motor command loop

So this firmware currently behaves like a **telemetry bring-up platform** for Equilibot hardware.

---

## 6) Technical notes / risks / improvements

1. **Security:** SoftAP password is hardcoded and simple (`equilibot123`). Good for lab bring-up, weak for real deployment.
2. **High log volume:** telemetry is logged (`ESP_LOGI`) on every publish; at 20 Hz this can become noisy and impact performance.
3. **Error handling strategy:** many init failures log and continue; consider explicit fail-fast behavior depending on required subsystems.
4. **Unused objects in main loop:** `led_strip`, `mot1`, `mot2` are currently instantiated but unused after construction.
5. **Control loop gap:** no closed-loop stabilization logic yet (likely next major milestone).
6. **Potential config confusion:** `sdkconfig` includes `CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"` while active filename is `partitions_two_ota_large.csv`; worth validating intended final partition setup.

---

## 7) Practical “where to start next” suggestions

For fastest progress toward a balancing robot:

1. Add a **state estimator** (pitch/roll from accel+gyro fusion, e.g., complementary filter)
2. Add a **motor control abstraction** on top of `tmc5240` (velocity/torque command APIs)
3. Implement a deterministic **control task** (fixed rate, e.g., 200–500 Hz)
4. Wire LED segments to meaningful state (arming, connection, fault, battery)
5. Keep websocket telemetry as debug/observability channel

---

## 8) Important file map

- Build/config:
  - `CMakeLists.txt`
  - `main/CMakeLists.txt`
  - `sdkconfig`
  - `dependencies.lock`
- Core runtime:
  - `main/main.cpp`
- Sensor path:
  - `main/icm42670_spi.hpp`
  - `main/icm42670_spi.cpp`
  - `main/filter.hpp`
- Shared bus and sync:
  - `main/spi_bus.hpp`
  - `main/spi_bus.cpp`
  - `main/lock_guard.hpp`
- Web telemetry:
  - `main/web_server.hpp`
  - `main/web_server.cpp`
  - `main/index.html`
- Outputs/indicators/actuation:
  - `main/led_strip.hpp`
  - `main/led_strip.cpp`
  - `main/tmc5240.hpp`
  - `main/tmc5240.cpp`

---

If useful, the next step can be a second document that proposes a concrete control architecture (tasks, rates, data ownership, and APIs) tailored to this exact codebase.