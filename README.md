# SmartToy Board Bring-Up & Logger Firmware

Bring-up, diagnostic, and high-precision sensor logging firmware for the **SmartToy** hardware platform (`SmartToy_CharlesEdits_09_14_26.kicad_sch`), targeting the **ESP32-C3-MINI-1-N4**.

The firmware serves three concurrent roles:

1. **Automated Self-Test & Diagnostic Shell:** Executes self-tests at boot, drives a visual state indicator, and provides an interactive ASCII serial terminal.
2. **Synchronized 9-DoF + Environmental Logger:** Records calibrated sensor data at 50 Hz to an onboard 2 Gb SPI NAND flash chip with wear-conscious directory checkpointing and power-fail recovery.
3. **Dual-Transport Binary Host Interface:** Supports microsecond-level clock synchronization, file transfers, and remote control over native USB CDC and low-overhead BLE.

---

## 1. Hardware Architecture & Pin Map

### Microcontroller

* **MCU:** ESP32-C3-MINI-1-N4 (Single-core RISC-V @ 160 MHz, 4 MB embedded flash)
* **Native USB CDC:** Active on GPIO18 (`D-`) and GPIO19 (`D+`)

### Pin Assignments

| Pin / Net | Function | Notes / External Circuitry |
| --- | --- | --- |
| **IO0** | `PIN_FFC_IO0` | Auxiliary FFC connector line 0 |
| **IO1** | `PIN_FFC_IO1` | Auxiliary FFC connector line 1 |
| **IO2** | `PIN_NAND_CS` | SPI NAND Chip Select (Strapping pin; pulled high via R9) |
| **IO3** | `PIN_NAND_SCK` | SPI NAND Serial Clock (10 MHz, SPI Mode 0) |
| **IO4** | `PIN_VBATT_DIV` | Battery monitoring (ADC1_CH4 via 1 MΩ / 1 MΩ divider, 2× scale) |
| **IO5** | `PIN_SDA` | I2C Data (400 kHz Fast-Mode; shared bus) |
| **IO6** | `PIN_SCL` | I2C Clock (400 kHz Fast-Mode; shared bus) |
| **IO7** | `PIN_KEEP_ON` | Power latch (D1 → R1 → Q1 base → holds Q2 on) |
| **IO8** | `PIN_DLED` | WS2812C DIN (Strapping pin; R11 pull-up; driven by ESP32 RMT peripheral) |
| **IO9** | `PIN_BOOT` / `PIN_NAND_MOSI` | Test point `TP1` (Strapping pin) shared with SPI NAND MOSI |
| **IO10** | `PIN_NAND_MISO` | SPI NAND MISO |
| **IO20** | `PIN_INT_LSM` | LSM6DS3 Data-Ready interrupt (`U0RXD`) |
| **IO21** | `PIN_INT_BMP` | BMP581 Data-Ready interrupt (`U0TXD`) |

### Onboard Peripherals & I2C Addresses

* **U2 (LSM6DS3 / LSM6DS3TR-C):** 6-DoF IMU (`0x6A` or `0x6B`). Configured for ±2 g accel, ±2000 dps gyro @ 104 Hz.
* **U5 (BMP581):** High-precision barometric pressure & temperature (`0x46`). Configured for 50 Hz normal mode (`OSR_P=2x`, `OSR_T=2x`).
* **U7 (MMC5633NJL):** 3-axis AMR Magnetometer (`0x30`). Configured for 50 Hz continuous mode (`Cmm_en` with automatic SET/RESET).
* **U8 (Micron MT29F2G01ABAGD):** 2 Gb (256 MB) 3.3V SPI NAND Flash. 2048 blocks, 64 pages/block, 2048 bytes/page.
* **LED Chain (5× WS2812C):**
* Index `0` (D2, Front): Boot pass (dim green) / fail (dim red) indicator.
* Indices `1..4` (Back ring): Real-time IMU visualizer (Tilt / Spin / Jolt).



---

## 2. Build & Flash Configuration

### Toolchain Requirements

* **Framework:** Arduino IDE or CLI with `arduino-esp32` core **v3.0.0 or higher** (mandatory for the core 3.x RMT driver API).
* **External Libraries:** **None.** Relies exclusively on built-in core runtime APIs (`esp_timer`, `Wire`, `SPI`, `freertos`, `BLEDevice`).

### Arduino IDE Board Settings

* **Board:** `ESP32C3 Dev Module`
* **Flash Size:** `4MB (32Mb)`
* **Partition Scheme:** `Default 4MB with spiffs`
* **USB CDC On Boot:** `Enabled` *(Critical: routes `Serial` over native USB on IO18/IO19)*
* **CPU Frequency:** `160MHz`
* **Upload Speed:** `921600` (or `115200` if flashing over a UART bridge)

---

## 3. Storage Layout & Data Logging Format

### NAND Flash Memory Map

The 2048 blocks of the MT29F2G01 are segmented to ensure data resilience:

* **Blocks 0 – 2044:** Data Blocks (append-only allocation per recording session).
* **Blocks 2045 & 2046:** Directory Blocks A and B. Directory copies alternate between blocks on wrap. Power-fail safe; boot recovers using sequence counters and CRC-32 verification.
* **Block 2047:** Reserved scratch block for destructive self-tests (`F` menu command).

### Sample Record (`SampleRecord`) — 28 Bytes

Logged at **50 Hz** (`LOG_RATE_HZ = 50`). Barometer and magnetometer readings update every `MAG_BARO_REFRESH_DIV = 2` cycles (25 Hz) and are held forward on alternating ticks.

```c
#pragma pack(push, 1)
struct SampleRecord {
  uint32_t t_ms;        // Milliseconds since recording started (t_ms == 0 corresponds to startEpochUs)
  int16_t  ax_mg;       // Accelerometer X (milli-g, ±2g scale)
  int16_t  ay_mg;       // Accelerometer Y (milli-g, ±2g scale)
  int16_t  az_mg;       // Accelerometer Z (milli-g, ±2g scale)
  int16_t  gx_dps10;    // Gyroscope X (deci-dps: dps = value / 10.0)
  int16_t  gy_dps10;    // Gyroscope Y (deci-dps: dps = value / 10.0)
  int16_t  gz_dps10;    // Gyroscope Z (deci-dps: dps = value / 10.0)
  int32_t  press_pa;    // Barometric pressure (Pascals)
  int16_t  temp_cdeg;   // Temperature (centi-Celsius: °C = value / 100.0)
  int16_t  mx_dt;       // Magnetometer X (deci-µT: µT = value / 10.0)
  int16_t  my_dt;       // Magnetometer Y (deci-µT: µT = value / 10.0)
  int16_t  mz_dt;       // Magnetometer Z (deci-µT: µT = value / 10.0)
};
#pragma pack(pop)

```

* **Page Layout:** Exactly 72 records are buffered in RAM and written as a 2016-byte payload per 2048-byte NAND page, leaving headroom for transport framing.
* **Interrupted Files:** On boot, unfinalized files flagged with `recording = 1` are recovered by scanning pages forward to salvage all committed data up to the last written page.

---

## 4. Host Binary Protocol

Communication shares the USB-C CDC interface alongside the ASCII menu, and runs over BLE GATT (Nordic UART Service: Service `6e400001-...`, RX `6e400002-...`, TX `6e400003-...`).

Binary frames start with `0xAA` to avoid collision with standard ASCII terminal input.

### Frame Construction

```
[0xAA] [LEN_LO] [LEN_HI] [CMD] [PAYLOAD (LEN bytes)] [CRC8]

```

* **SOF:** `0xAA` (1 byte)
* **Payload Length:** 16-bit little-endian integer (`LEN_LO | (LEN_HI << 8)`)
* **CMD:** 1 byte
* **CRC8:** 1 byte covering `CMD + PAYLOAD`:
* **Polynomial:** `0x31` ($x^8 + x^5 + x^4 + 1$)
* **Init:** `0x00`
* **Reflect In / Out:** False
* **Xor Out:** `0x00`



### Command Reference

| CMD | Name | Request Payload | Response Payload | Description |
| --- | --- | --- | --- | --- |
| `0x00` | `PING` | *None* | `version (u8)` | Echoes protocol version (`0x02`). |
| `0x01` | `SET_TIME` | `epoch_sec (u32 LE)` | `err (u8)` | Legacy time synchronization (1-second resolution). |
| `0x02` | `REC_START` | *None* | `err (u8), file_id (u8)` | Initiates a new recording file slot on NAND. |
| `0x03` | `REC_STOP` | *None* | `err (u8), file_id (u8)` | Flushes partial page buffer and finalizes file entry. |
| `0x04` | `LIST_FILES` | *None* | `count (u8)`, then 14 bytes per file: `[id(u8), startEpochUs(u64 LE), recordCount(u32 LE), recording(u8)]` | Lists all valid files recorded on the system. |
| `0x05` | `GET_PAGE` | `file_id (u8), page_idx (u32 LE)` | `err (u8), file_id (u8), page_idx (u32 LE), valid_bytes (u16 LE), data (N bytes)` | Reads a 2048-byte NAND page. **USB CDC only**; BLE returns `ERR_TOO_BIG_FOR_TRANSPORT`. |
| `0x06` | `DELETE` | `file_id (u8)` | `err (u8)` | Marks a recording file entry as invalid. |
| `0x07` | `STATUS` | *None* | `15 bytes status structure` (see below) | Returns run-time state, battery voltage, and free space. |
| `0x08` | `POWER_OFF` | *None* | `err (u8)` | Flushes, confirms, and de-asserts `PIN_KEEP_ON`. |
| `0x09` | `TIME_PING` | `seq (u8)` | `seq (u8), t2_us (u64 LE), t3_minus_t2_us (u32 LE)` | Round-trip microsecond clock offset calculation. Fits single BLE packet. |
| `0x0A` | `SET_TIME_US` | `offset_us (int64 LE)` | `err (u8)` | Applies calculated microsecond offset relative to `esp_timer_get_time()`. |

#### Status Packet (`0x07`) Payload Layout (15 Bytes)

* `[0]` `is_recording` (`u8`): `0` or `1`
* `[1]` `active_file_id` (`u8`): Active index, or `0xFF` if idle
* `[2..5]` `record_count` (`u32 LE`): Active file record counter
* `[6..7]` `free_blocks` (`u16 LE`): Remaining unallocated NAND blocks
* `[8..9]` `vbatt_mv` (`u16 LE`): Battery level in millivolts
* `[10]` `is_time_synced` (`u8`): `1` if synced via `TIME_PING`/`SET_TIME`
* `[11..14]` `current_epoch_sec` (`u32 LE`): Board wall-clock time in seconds

---

## 5. Microsecond Time Synchronization Flow

To align sample logs across multiple sensors and external video captures without onboard RTC drift:

```
HOST                                              SMARTTOY
  |                                                  |
  |--- 0x09 TIME_PING [seq] ------------------------>|  Captures t2_us = nowUs()
  |                                                  |  Computes dt = nowUs() - t2_us
  |<-- 0x09 Reply [seq, t2_us, dt] ------------------|  Transmits immediately
  |                                                  |
  | Compute round-trip delay:                        |
  |   RTT = (t4 - t1) - dt                           |
  | Compute board offset relative to host epoch:     |
  |   offset_us = host_epoch_us - (t2_us + RTT / 2)  |
  |                                                  |
  |--- 0x0A SET_TIME_US [offset_us] ---------------->|  Board applies gOffsetUs
  |<-- 0x0A Reply [ERR_OK] --------------------------|

```

Every sample's absolute wall-clock timestamp is calculated as:


$$\text{Epoch}_{\mu\text{s}} = \text{startEpochUs} + (t\_ms \times 1000)$$

---

## 6. Interactive Serial Diagnostic Menu

Connect via any serial terminal at **115200 baud, 8-N-1** (with line endings set to CR/LF or None):

* `a` — **Run all non-destructive tests:** Executes full hardware checkout (I2C scan, WHO_AM_I, basic data reads, interrupt edge verification, NAND ID, battery voltage, LED cycles).
* `i` — **I2C bus scan:** Probes `0x01` to `0x7F` for connected peripherals.
* `s` — **Live sensor stream:** Emits calibrated 9-DoF data, pressure, temperature, and battery voltage at ~10 Hz (press any key to stop).
* `l` — **LED chain checkout:** Sequences RGBW patterns and runs a single-pixel walk across all 5 WS2812 LEDs.
* `f` — **SPI NAND check:** Queries device ID, reads configuration registers, checks ECC status, and reads Block 0 / Page 0.
* `F` — **Destructive SPI NAND test:** Erases, writes a test pattern to, and verifies block 2047 (requires interactive `'Y'` confirmation within 10 seconds).
* `v` — **Battery voltage:** Reports raw voltage divider reading and computed battery pack level.
* `n` — **Interrupt lines test:** Enables pulsed DRDY on LSM6DS3 (IO20) and BMP581 (IO21) and checks edge counts over 1 second.
* `g` — **FFC toggle test:** Drives IO0 and IO1 in antiphase at 2 Hz for 5 seconds for oscilloscope probing.
* `b` — **BOOT test point:** Arming sequence to short `TP1` (IO9) to ground within 10 seconds.
* `k` — **Release KEEP_ON:** Drives IO7 low to test auto-shutdown circuitry when the external power magnet is removed.
* `m` — **Back LED visualizer mode:** Cycles through `off`, `tilt` (gravity vector level indicator), and `spin` (gyro Z-axis yaw comet).
* `c` — **Gyroscope calibration:** Collects 50 samples at rest to calculate zero-rate bias offsets.
* `p` — **Status overview:** Dumps time sync parameters, free storage, and directory metadata.
* `x` — **MMC5633 continuous check:** Compares automatic SET/RESET continuous mode against manual SET/RESET differential readouts.
* `h` — **Help:** Displays the command menu.