/*
   SmartToy board bring-up test firmware
   Target: ESP32-C3-MINI-1-N4  (SmartToy_CharlesEdits_09_14_26.kicad_sch)

   Arduino IDE settings:
     Board:            ESP32C3 Dev Module   (arduino-esp32 core 3.x -- uses its RMT API)
     USB CDC On Boot:  Enabled              (Serial = native USB on IO18/IO19)
     Flash Size:       4MB

   No external libraries beyond the arduino-esp32 core: WS2812 via its RMT
   API, BLE via its bundled BLEDevice library (Bluedroid host, BLE-only mode
   -- no separate install, so no repeat of the RMT/NeoPixel conflict).

   On boot it latches KEEP_ON, runs all non-destructive tests, prints a report,
   and leaves the front LED (D2) dim GREEN (all pass) or dim RED (any fail) so
   you can also run it untethered on battery. The four back LEDs then show a
   live IMU visualisation (see VIZ section).

   Host link (USB-C CDC serial, and a control-only subset over BLE): a small
   binary protocol shares the same USB serial connection as the interactive
   menu below -- frames start with 0xAA, so they never collide with the
   single-letter menu commands. Frame: SOF(0xAA) LEN_LO LEN_HI CMD PAYLOAD[LEN]
   CRC8. CRC8 is poly 0x31, init 0x00, MSB-first, no reflection (the same
   construction as Maxim/Dallas 1-Wire CRC8, just a different polynomial) --
   implement this exactly in the companion app or frames will be dropped as
   corrupt. Commands: 0x00 PING, 0x01 SET_TIME (payload: uint32 LE epoch
   seconds -- legacy, 1 s resolution), 0x02 REC_START, 0x03 REC_STOP,
   0x04 LIST_FILES, 0x05 GET_PAGE (payload: fileId, uint32 LE pageIndex -- one
   2048-byte NAND page per call; USB only, BLE replies ERR_TOO_BIG_FOR_TRANSPORT),
   0x06 DELETE (payload: fileId), 0x07 STATUS, 0x08 POWER_OFF, 0x09 TIME_PING,
   0x0A SET_TIME_US. See the NAND FILE LOG and HOST PROTOCOL sections for
   exact reply payloads. Data is logged at LOG_RATE_HZ into 28-byte
   SampleRecord entries; see that struct for field scaling.

   Time sync (microsecond class): every board timestamp comes from esp_timer
   (nowUs() = microseconds since boot; Arduino's millis() is the same timer).
   The host runs an NTP-style exchange with 0x09 TIME_PING
     request:  seq(1)
     reply:    seq(1) t2_us(u64 LE) t3_minus_t2_us(u32 LE)
   where t2 = board clock when the request arrived and t3 = board clock when the
   reply was handed to the transport (so t3 - t2 is the board's processing time; the
   reply is 13 bytes so the whole frame fits one 20-byte BLE notification).
   The host then sends 0x0A SET_TIME_US
     request:  offset_us(int64 LE)      epoch_us = nowUs() + offset_us
     reply:    err(1)
   A recording's start time is stored as epoch MICROseconds -- the instant at
   which sample t_ms == 0 -- and LIST_FILES reports it that way:
     reply: count(1) then per file: id(1) startEpochUs(u64 LE) recordCount(u32 LE) recording(1)
            (14 bytes per file; startEpochUs = 0 if the board had never been synced)
   Wall-clock time of a sample = startEpochUs + t_ms * 1000.
   Sample timestamps are taken with esp_timer just before the IMU is read and
   rounded to the nearest ms. They do not include the IMU's own internal
   latency (the 104 Hz output is polled at 50 Hz, so a value can be up to ~10 ms old).
   Baro and mag are refreshed every MAG_BARO_REFRESH_DIV-th sample (25 Hz at the default
   of 2). The BMP581 free-runs at 50 Hz and, while recording, the MMC5633 runs in continuous
   mode at 50 Hz with automatic SET/RESET, so each refresh picks up a result that is at most
   ~20-30 ms old and the loop never blocks on a magnetometer measurement.

   BLE: advertises as "SmartToy-XXXX" (XXXX = last 2 MAC bytes), Nordic-UART-
   style GATT service, Just Works bonding (no PIN). Meant for commands and
   status only -- bulk file transfer stays on USB-C. Commands received over
   BLE are queued and executed from loop() so they never touch the NAND / I2C /
   directory concurrently with the logger.

   Serial menu (115200, any line ending):
     a  run all non-destructive tests
     i  I2C scan
     s  stream sensor data (any key stops)
     l  LED colour cycle
     f  SPI NAND reset / ID / page read (non-destructive)
     F  SPI NAND erase/program/verify on block 2047 (destructive to that block)
     v  battery divider voltage
     n  interrupt line test (INT_LSM, INT_BMP)
     g  toggle FFC IO0/IO1 in antiphase for 5 s (scope them)
     b  BOOT test point (short TP1 to GND)
     k  release KEEP_ON (board should power off once the magnet is removed)
     m  cycle back-LED visualisation mode (off / tilt / spin)
     c  re-calibrate gyro bias (hold the board still)
     p  print log / time-sync / BLE status
     x  MMC5633 continuous-mode check vs manual SET/RESET (hold the board still)
     h  this menu
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLESecurity.h>

// Types used in function signatures must be declared before the first function,
// otherwise the Arduino IDE's auto-generated prototypes reference unknown types.
enum Transport : uint8_t { TR_SERIAL, TR_BLE };
#pragma pack(push, 1)
struct SampleRecord {
  uint32_t t_ms;                     // ms since this recording started (t_ms == 0 <-> file startEpochUs)
  int16_t  ax_mg, ay_mg, az_mg;      // milli-g
  int16_t  gx_dps10, gy_dps10, gz_dps10;  // deci-dps (actual dps = value / 10)
  int32_t  press_pa;                 // Pa, refreshed every MAG_BARO_REFRESH_DIV samples, carried forward otherwise
  int16_t  temp_cdeg;                // BMP temp x100 C, carried forward
  int16_t  mx_dt, my_dt, mz_dt;      // deci-uT, carried forward
};
#pragma pack(pop)

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
#error "Needs arduino-esp32 core 3.x (new RMT API)"
#endif

// ---------------------------------------------------------------- Pin map
static const int PIN_FFC_IO0   = 0;
static const int PIN_FFC_IO1   = 1;
static const int PIN_NAND_CS   = 2;   // strapping pin, R9 pull-up
static const int PIN_NAND_SCK  = 3;
static const int PIN_VBATT_DIV = 4;   // R5/R6 1M/1M divider -> ADC1_CH4
static const int PIN_SDA       = 5;
static const int PIN_SCL       = 6;
static const int PIN_KEEP_ON   = 7;   // D1 -> R1 -> Q1 base -> holds Q2 on
static const int PIN_DLED      = 8;   // WS2812C DIN, strapping pin, R11 pull-up
static const int PIN_BOOT      = 9;   // TP1 / strapping pin
static const int PIN_NAND_MOSI = 9;   // MOSI label reads as sharing IO9 with BOOT -- change if not
static const int PIN_NAND_MISO = 10;
static const int PIN_INT_LSM   = 20;  // U0RXD
static const int PIN_INT_BMP   = 21;  // U0TXD

static const uint8_t ADDR_MMC = 0x30;
static const uint8_t ADDR_BMP = 0x46;

// ---------------------------------------------------------------- Time base
// All board timestamps come from the same free-running 64-bit microsecond timer.
static inline uint64_t nowUs() {
  return (uint64_t)esp_timer_get_time();
}

// ---------------------------------------------------------------- LED chain
// D2 (front) DOUT -> LEDO -> four back-side WS2812s.
static const int     NUM_LEDS  = 5;
static const int     FRONT_LED = 0;                     // D2: test status
static const uint8_t BACK_IDX[4] = {1, 2, 3, 4};        // chain index of each back LED
// Where each back LED sits around the board, as an angle in the IMU's X/Y frame
// (0 = +X, 90 = +Y). Placeholder -- set these to match the layout.
static const float   BACK_ANGLE_DEG[4] = {0, 90, 180, 270};
static const uint8_t VIZ_MAX_BRIGHT = 255;               // 0-255 cap for the back LEDs

// Minimal WS2812 driver on the core's RMT API (same method names as Adafruit_NeoPixel
// so the rest of the sketch is unchanged). 10 MHz tick: 1 = 800/400 ns, 0 = 400/800 ns.
class LedStrip {
  public:
    LedStrip(int pin) : pin_(pin) {}
    bool begin() {
      ok_ = rmtInit(pin_, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000);
      return ok_;
    }
    static uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
      return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    }
    void setPixelColor(int i, uint32_t c) {
      if (i >= 0 && i < NUM_LEDS) px_[i] = c;
    }
    void fill(uint32_t c) {
      for (int i = 0; i < NUM_LEDS; i++) px_[i] = c;
    }
    void clear() {
      fill(0);
    }
    void show() {
      if (!ok_) return;
      size_t k = 0;
      for (int i = 0; i < NUM_LEDS; i++) {
        uint32_t c = px_[i];
        const uint8_t grb[3] = {(uint8_t)(c >> 8), (uint8_t)(c >> 16), (uint8_t)c};  // wire order G,R,B
        for (uint8_t byte : grb) {
          for (int b = 7; b >= 0; b--) {
            bool one = byte & (1 << b);
            sym_[k].level0 = 1; sym_[k].duration0 = one ? 8 : 4;
            sym_[k].level1 = 0; sym_[k].duration1 = one ? 4 : 8;
            k++;
          }
        }
      }
      rmtWrite(pin_, sym_, k, RMT_WAIT_FOR_EVER);
      delayMicroseconds(300);                         // latch (WS2812C-2020 needs >280 us)
    }
    // hue 0..65535 (0 = red, 21845 = green, 43690 = blue), like Adafruit's ColorHSV
    static uint32_t ColorHSV(uint16_t hue, uint8_t sat, uint8_t val) {
      uint32_t h = (uint32_t)hue * 6;
      uint8_t region = (uint8_t)(h >> 16);
      uint32_t f = h & 0xFFFF;
      uint8_t p = (uint8_t)(val * (255 - sat) / 255);
      uint8_t q = (uint8_t)(val * (255 - sat * f / 65535) / 255);
      uint8_t t = (uint8_t)(val * (255 - sat * (65535 - f) / 65535) / 255);
      switch (region) {
        case 0:  return Color(val, t, p);
        case 1:  return Color(q, val, p);
        case 2:  return Color(p, val, t);
        case 3:  return Color(p, q, val);
        case 4:  return Color(t, p, val);
        default: return Color(val, p, q);
      }
    }
    static uint32_t gamma32(uint32_t c) {
      auto g = [](uint8_t x) {
        return (uint8_t)(powf(x / 255.0f, 2.6f) * 255.0f + 0.5f);
      };
      return Color(g(c >> 16), g(c >> 8), g(c));
    }

  private:
    int pin_;
    bool ok_ = false;
    uint32_t px_[NUM_LEDS] = {0};
    rmt_data_t sym_[NUM_LEDS * 24];
};

static LedStrip strip(PIN_DLED);

// ---------------------------------------------------------------- Reporting
static int gPass = 0, gFail = 0;

static String fmt(const char *f, ...) {
  char b[192];
  va_list ap;
  va_start(ap, f);
  vsnprintf(b, sizeof b, f, ap);
  va_end(ap);
  return String(b);
}

static void result(bool ok, const char *name, const String &detail = "") {
  if (ok) gPass++; else gFail++;
  Serial.printf("  [%s] %-16s %s\n", ok ? "PASS" : "FAIL", name, detail.c_str());
}

static void info(const char *name, const String &detail) {
  Serial.printf("  [INFO] %-16s %s\n", name, detail.c_str());
}

// ---------------------------------------------------------------- LED
// Front (status) LED only; leaves the back LEDs as they are
static void ledSet(uint8_t r, uint8_t g, uint8_t b) {
  strip.setPixelColor(FRONT_LED, strip.Color(r, g, b));
  strip.show();
}

static void testLED() {
  Serial.println("\n-- WS2812 chain -- all 5 LEDs: red, green, blue, white, then walk the chain");
  const uint8_t seq[4][3] = {{40, 0, 0}, {0, 40, 0}, {0, 0, 40}, {30, 30, 30}};
  for (auto &c : seq) {
    strip.fill(strip.Color(c[0], c[1], c[2]));
    strip.show();
    delay(500);
  }
  for (int i = 0; i < NUM_LEDS; i++) {             // one at a time, in chain order
    strip.clear();
    strip.setPixelColor(i, strip.Color(30, 30, 30));
    strip.show();
    delay(400);
  }
  strip.clear();
  strip.show();
  info("LED", "visual check; chain walk shows index order for BACK_ANGLE_DEG");
}

// ---------------------------------------------------------------- I2C helpers
static bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint16_t)addr, (size_t)n, true) != n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static int i2cRead8(uint8_t addr, uint8_t reg) {
  uint8_t v;
  return i2cRead(addr, reg, &v, 1) ? v : -1;
}

static void testI2CScan() {
  Serial.println("\n-- I2C scan (SDA=IO5, SCL=IO6) --");
  bool seenMMC = false, seenBMP = false, seenLSM = false;
  String found;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      found += fmt("0x%02X ", a);
      if (a == ADDR_MMC) seenMMC = true;
      if (a == ADDR_BMP) seenBMP = true;
      if (a == 0x6A || a == 0x6B) seenLSM = true;
    }
  }
  info("found", found.length() ? found : String("nothing (check R12/R13 pull-ups, 3V3 rail)"));
  result(seenLSM, "LSM6DS3 ACK", "expect 0x6A (SA0=0) or 0x6B");
  result(seenBMP, "BMP581 ACK", "expect 0x46");
  result(seenMMC, "MMC5633 ACK", "expect 0x30");
}

// ---------------------------------------------------------------- LSM6DS3 (U2)
static uint8_t lsmAddr = 0, lsmWho = 0;

static bool lsmInit() {
  lsmAddr = 0;
  const uint8_t addrs[] = {0x6A, 0x6B};
  for (uint8_t a : addrs) {
    int w = i2cRead8(a, 0x0F);                    // WHO_AM_I
    if (w >= 0) {
      lsmAddr = a;
      lsmWho = (uint8_t)w;
      break;
    }
  }
  if (!lsmAddr) return false;
  i2cWrite(lsmAddr, 0x12, 0x01); delay(20);       // CTRL3_C: SW_RESET
  i2cWrite(lsmAddr, 0x12, 0x44);                  // CTRL3_C: BDU | IF_INC
  i2cWrite(lsmAddr, 0x10, 0x40);                  // CTRL1_XL: 104 Hz, +-2 g
  i2cWrite(lsmAddr, 0x11, 0x4C);                  // CTRL2_G: 104 Hz, 2000 dps
  delay(100);
  return true;
}

static bool lsmRead(float a[3], float g[3], float &t) {
  uint8_t b[14];
  if (!lsmAddr || !i2cRead(lsmAddr, 0x20, b, 14)) return false;  // OUT_TEMP_L..OUTZ_H_XL
  int16_t r[7];
  for (int i = 0; i < 7; i++) r[i] = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8));
  t = 25.0f + r[0] / (lsmWho == 0x69 ? 16.0f : 256.0f);
  for (int i = 0; i < 3; i++) {
    g[i] = r[1 + i] * 0.070f;      // 70 mdps/LSB @ 2000 dps
    a[i] = r[4 + i] * 0.000061f;   // 0.061 mg/LSB @ 2 g
  }
  return true;
}

static void testLSM() {
  Serial.println("\n-- LSM6DS3 (U2) --");
  if (!lsmInit()) {
    result(false, "LSM present", "no ACK at 0x6A/0x6B");
    return;
  }
  bool whoOk = lsmWho == 0x69 || lsmWho == 0x6A || lsmWho == 0x6B || lsmWho == 0x6C;
  result(whoOk, "LSM WHO_AM_I",
         fmt("0x%02X @0x%02X (69=DS3, 6A=DS3TR-C, 6C=DSO)", lsmWho, lsmAddr));
  float a[3], g[3], t;
  if (!lsmRead(a, g, t)) {
    result(false, "LSM read", "I2C read failed");
    return;
  }
  float am = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  result(am > 0.8f && am < 1.2f, "LSM accel",
         fmt("%.3f %.3f %.3f g  |a|=%.3f (board at rest)", a[0], a[1], a[2], am));
  float gm = fmaxf(fabsf(g[0]), fmaxf(fabsf(g[1]), fabsf(g[2])));
  result(gm < 10.0f, "LSM gyro", fmt("%.2f %.2f %.2f dps (board at rest)", g[0], g[1], g[2]));
  info("LSM temp", fmt("%.1f C", t));
}

// ---------------------------------------------------------------- BMP581 (U5)
// ODR_CONFIG code: 0x0F = 50 Hz, 0x14 = 25 Hz, 0x15 = 20 Hz, 0x17 = 10 Hz. The sensor runs
// faster than the 25 Hz logging reads so every read finds a new result (no repeats). The
// 'n' interrupt test counts the data-ready edges and should report ~50 per second.
static const uint8_t BMP_ODR_CODE = 0x0F;

static bool bmpInit(uint8_t &id) {
  i2cWrite(ADDR_BMP, 0x7E, 0xB6);                 // CMD: soft reset
  delay(5);
  int r = i2cRead8(ADDR_BMP, 0x01);               // CHIP_ID
  id = r < 0 ? 0 : (uint8_t)r;
  if (id != 0x50 && id != 0x51) return false;
  int st = i2cRead8(ADDR_BMP, 0x28);              // STATUS
  i2cRead8(ADDR_BMP, 0x27);                       // INT_STATUS (clears POR flag)
  if (st < 0 || !(st & 0x02) || (st & 0x04)) return false;  // nvm_rdy && !nvm_err
  i2cWrite(ADDR_BMP, 0x37, 0x80);                 // ODR_CONFIG: standby, deep_dis
  delay(5);
  i2cWrite(ADDR_BMP, 0x36, 0x49);                 // OSR_CONFIG: press_en, osr_p=2x, osr_t=2x
  i2cWrite(ADDR_BMP, 0x37, 0x80 | (BMP_ODR_CODE << 2) | 0x01);  // BMP_ODR_CODE (50 Hz), normal mode
  delay(150);
  return true;
}

static bool bmpRead(float &tC, float &pPa) {
  uint8_t b[6];
  if (!i2cRead(ADDR_BMP, 0x1D, b, 6)) return false;  // TEMP_XLSB..PRESS_MSB
  int32_t tr = (int32_t)(((uint32_t)b[2] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[0] << 8)) >> 8;
  uint32_t pr = ((uint32_t)b[5] << 16) | ((uint32_t)b[4] << 8) | b[3];
  tC = tr / 65536.0f;
  pPa = pr / 64.0f;
  return true;
}

static bool bmpOk = false;

static void testBMP() {
  Serial.println("\n-- BMP581 (U5) --");
  uint8_t id;
  bmpOk = bmpInit(id);
  result(bmpOk, "BMP CHIP_ID/NVM", fmt("id 0x%02X (expect 0x50)", id));
  if (!bmpOk) return;
  float t, p;
  if (!bmpRead(t, p)) {
    result(false, "BMP read", "I2C read failed");
    bmpOk = false;
    return;
  }
  result(p > 80000 && p < 110000, "BMP pressure", fmt("%.2f hPa", p / 100.0f));
  result(t > -10 && t < 60, "BMP temp", fmt("%.2f C", t));
}

// ---------------------------------------------------------------- MMC5633NJL (U7)
static bool mmcContOn = false;                     // true while the chip is in continuous mode (see mmcContStart)

static bool mmcMeasure(int32_t out[3]) {
  i2cWrite(ADDR_MMC, 0x1B, 0x01);                 // Internal Control 0: Take_meas_M
  uint32_t t0 = millis();
  for (;;) {
    int s = i2cRead8(ADDR_MMC, 0x18);             // Status1
    if (s >= 0 && (s & 0x40)) break;              // Meas_M_done
    if (millis() - t0 > 20) return false;
    delay(1);
  }
  uint8_t b[9];
  if (!i2cRead(ADDR_MMC, 0x00, b, 9)) return false;
  for (int i = 0; i < 3; i++) {
    uint32_t raw = ((uint32_t)b[2 * i] << 12) | ((uint32_t)b[2 * i + 1] << 4) | (b[6 + i] >> 4);
    out[i] = (int32_t)raw - 524288;               // 20-bit, null at 2^19
  }
  return true;
}

// SET/RESET-differenced reading in microtesla (cancels bridge offset)
static bool mmcReadUT(float B[3]) {
  if (mmcContOn) return false;                    // manual SET/RESET would fight continuous mode
  int32_t s[3], r[3];
  i2cWrite(ADDR_MMC, 0x1B, 0x08); delay(1);       // Do_Set
  if (!mmcMeasure(s)) return false;
  i2cWrite(ADDR_MMC, 0x1B, 0x10); delay(1);       // Do_Reset
  if (!mmcMeasure(r)) return false;
  for (int i = 0; i < 3; i++) B[i] = (s[i] - r[i]) / 2.0f / 16384.0f * 100.0f;  // counts -> G -> uT
  return true;
}

static bool mmcOk = false;

// Continuous mode, used while recording. Per the datasheet: write ODR, set Cmm_freq_en (lets
// the chip compute its period), then set Cmm_en. Auto_SR_en makes the chip do the SET/RESET
// pair itself for every output (two 6.6 ms measurements at BW=00), so the output rate can be
// up to 75 Hz; 50 Hz leaves headroom. Recording reads every MAG_BARO_REFRESH_DIV-th sample
// (25 Hz), so there is always a new result waiting and the loop never blocks on the sensor.
// Supply current is proportional to measurements/s (~3.4 mA typ. at 100/s, BW=00), which is
// why it only runs while recording.
static const uint8_t MAG_ODR_HZ = 50;

static void mmcContStart() {
  if (!mmcOk || mmcContOn) return;
  i2cWrite(ADDR_MMC, 0x1C, 0x00);                 // Internal Control 1: BW=00 (6.6 ms), no axis inhibits
  i2cWrite(ADDR_MMC, 0x1A, MAG_ODR_HZ);           // ODR
  i2cWrite(ADDR_MMC, 0x1B, 0xA0);                 // Control 0: Cmm_freq_en | Auto_SR_en
  delay(1);                                       // Cmm_freq_en self-clears once the period is computed
  i2cWrite(ADDR_MMC, 0x1D, 0x10);                 // Control 2: Cmm_en -> continuous mode starts
  mmcContOn = true;
}

static void mmcContStop() {
  if (!mmcContOn) return;
  i2cWrite(ADDR_MMC, 0x1D, 0x00);                 // Cmm_en = 0
  mmcContOn = false;
}

// Latest continuous-mode field in microtesla. Returns false if no NEW result has completed
// since the last read (Status1 Meas_m_done is cleared by reading the data registers).
static bool mmcContRead(float B[3]) {
  int s = i2cRead8(ADDR_MMC, 0x18);               // Status1
  if (s < 0 || !(s & 0x40)) return false;
  uint8_t b[9];
  if (!i2cRead(ADDR_MMC, 0x00, b, 9)) return false;
  for (int i = 0; i < 3; i++) {
    uint32_t raw = ((uint32_t)b[2 * i] << 12) | ((uint32_t)b[2 * i + 1] << 4) | (b[6 + i] >> 4);
    B[i] = ((int32_t)raw - 524288) / 16384.0f * 100.0f;   // 20-bit, null at 2^19; counts -> G -> uT
  }
  return true;
}

static void testMMC() {
  Serial.println("\n-- MMC5633NJL (U7) --");
  i2cWrite(ADDR_MMC, 0x1C, 0x80);                 // Internal Control 1: SW_reset
  mmcContOn = false;                              // a reset also ends continuous mode
  delay(20);
  int id = i2cRead8(ADDR_MMC, 0x39);              // Product ID
  mmcOk = (id == 0x10);
  result(mmcOk, "MMC product ID", fmt("0x%02X (expect 0x10)", id < 0 ? 0 : id));
  if (!mmcOk) return;
  float B[3];
  if (!mmcReadUT(B)) {
    result(false, "MMC measure", "timeout / I2C error");
    mmcOk = false;
    return;
  }
  float m = sqrtf(B[0] * B[0] + B[1] * B[1] + B[2] * B[2]);
  result(m > 10 && m < 1000, "MMC field",
         fmt("%.1f %.1f %.1f uT |B|=%.1f (earth ~25-65; hall magnet nearby will inflate)", B[0], B[1], B[2], m));
  i2cWrite(ADDR_MMC, 0x1B, 0x02);                 // Take_meas_T
  delay(10);
  int tr = i2cRead8(ADDR_MMC, 0x09);
  if (tr >= 0) info("MMC temp", fmt("%.1f C", -75.0f + tr * 0.8f));
}

// ---------------------------------------------------------------- SPI NAND MT29F2G01ABAGD (U8)
static SPISettings nandSPI(10000000, MSBFIRST, SPI_MODE0);

static inline void nSel() {
  SPI.beginTransaction(nandSPI);
  digitalWrite(PIN_NAND_CS, LOW);
}
static inline void nDes() {
  digitalWrite(PIN_NAND_CS, HIGH);
  SPI.endTransaction();
}

static void nCmd(uint8_t cmd) {
  nSel();
  SPI.transfer(cmd);
  nDes();
}

static void nCmdRow(uint8_t cmd, uint32_t row) {
  nSel();
  SPI.transfer(cmd);
  SPI.transfer((uint8_t)(row >> 16));
  SPI.transfer((uint8_t)(row >> 8));
  SPI.transfer((uint8_t)row);
  nDes();
}

static uint8_t nGetF(uint8_t a) {
  nSel(); SPI.transfer(0x0F); SPI.transfer(a); uint8_t v = SPI.transfer(0); nDes();
  return v;
}

static void nSetF(uint8_t a, uint8_t v) {
  nSel(); SPI.transfer(0x1F); SPI.transfer(a); SPI.transfer(v); nDes();
}

static bool nWait(uint32_t ms, uint8_t &st) {
  uint32_t t0 = millis();
  do {
    st = nGetF(0xC0);                             // status register
    if (!(st & 0x01)) return true;                // OIP clear
  } while (millis() - t0 < ms);
  return false;
}

// 2 planes: plane select = block LSB (row bit 6) -> column bit 12
static uint16_t nCol(uint32_t row, uint16_t col) {
  return col | (uint16_t)(((row >> 6) & 1) << 12);
}

static bool nReadPage(uint32_t row, uint16_t col, uint8_t *buf, size_t len, uint8_t &st) {
  nCmdRow(0x13, row);                             // PAGE READ to cache
  if (!nWait(5, st)) return false;
  uint16_t c = nCol(row, col);
  nSel();
  SPI.transfer(0x03);                             // READ FROM CACHE
  SPI.transfer((uint8_t)(c >> 8));
  SPI.transfer((uint8_t)c);
  SPI.transfer(0x00);                             // dummy
  for (size_t i = 0; i < len; i++) buf[i] = SPI.transfer(0xFF);
  nDes();
  return true;
}

static bool nProgramPage(uint32_t row, const uint8_t *buf, size_t len, uint8_t &st) {
  nCmd(0x06);                                     // WRITE ENABLE
  uint16_t c = nCol(row, 0);
  nSel();
  SPI.transfer(0x02);                             // PROGRAM LOAD
  SPI.transfer((uint8_t)(c >> 8));
  SPI.transfer((uint8_t)c);
  for (size_t i = 0; i < len; i++) SPI.transfer(buf[i]);
  nDes();
  nCmdRow(0x10, row);                             // PROGRAM EXECUTE
  if (!nWait(5, st)) return false;
  return !(st & 0x08);                            // P_FAIL
}

static bool nErase(uint32_t row, uint8_t &st) {
  nCmd(0x06);
  nCmdRow(0xD8, row);                             // BLOCK ERASE
  if (!nWait(20, st)) return false;
  return !(st & 0x04);                            // E_FAIL
}

static void testNAND() {
  Serial.println("\n-- SPI NAND MT29F2G01 (U8) --");
  uint8_t st = 0;
  nCmd(0xFF);                                     // RESET
  bool rdy = nWait(10, st);
  result(rdy, "NAND reset", fmt("status 0x%02X", st));
  nSel();
  SPI.transfer(0x9F); SPI.transfer(0x00);         // READ ID + dummy
  uint8_t mid = SPI.transfer(0), did = SPI.transfer(0);
  nDes();
  result(mid == 0x2C && did == 0x24, "NAND ID", fmt("MID 0x%02X DID 0x%02X (expect 2C 24)", mid, did));
  uint8_t lock = nGetF(0xA0), cfg = nGetF(0xB0);
  info("NAND features", fmt("lock=0x%02X cfg=0x%02X (ECC %s)", lock, cfg, (cfg & 0x10) ? "on" : "off"));
  uint8_t b[8];
  bool ok = nReadPage(0, 0, b, sizeof b, st);
  result(ok && ((st >> 4) & 7) != 2, "NAND page read",
         fmt("blk0/pg0 status 0x%02X, data %02X %02X %02X %02X...", st, b[0], b[1], b[2], b[3]));
}

static uint8_t pageBuf[2048], readBuf[2048];

static void testNANDDestructive() {
  Serial.println("\n-- SPI NAND erase/program/verify, block 2047 --");
  const uint32_t blk = 2047, row = blk << 6;
  uint8_t st = 0, bbm = 0;
  nSetF(0xA0, 0x00);                              // unlock all blocks (re-locks on power cycle)
  if (!nReadPage(row, 2048, &bbm, 1, st)) {
    result(false, "NAND BBM read", "timeout");
    return;
  }
  if (bbm != 0xFF) {
    info("NAND", fmt("block %u factory-marked bad (0x%02X), skipping", (unsigned)blk, bbm));
    return;
  }

  bool ok = nErase(row, st);
  result(ok, "NAND erase", fmt("status 0x%02X", st));
  if (!ok) return;

  nReadPage(row, 0, readBuf, sizeof readBuf, st);
  bool blank = true;
  for (size_t i = 0; i < sizeof readBuf; i++) if (readBuf[i] != 0xFF) {
      blank = false;
      break;
    }
  result(blank, "NAND blank check");

  for (size_t i = 0; i < sizeof pageBuf; i++) pageBuf[i] = (uint8_t)(i * 31 + 7) ^ (uint8_t)(i >> 8);
  ok = nProgramPage(row, pageBuf, sizeof pageBuf, st);
  result(ok, "NAND program", fmt("status 0x%02X", st));
  if (!ok) return;

  ok = nReadPage(row, 0, readBuf, sizeof readBuf, st);
  int bad = 0;
  for (size_t i = 0; i < sizeof readBuf; i++) if (readBuf[i] != pageBuf[i]) bad++;
  result(ok && bad == 0, "NAND verify", fmt("%d mismatched bytes, ECC status %u", bad, (st >> 4) & 7));

  nErase(row, st);                                // leave the block clean
}

// ---------------------------------------------------------------- NAND file log
// Simple append-only per-file block log on the SPI NAND: no wear levelling
// and no relocation of a block once a file has claimed it (a block found bad
// at allocation time is just skipped). Fine for a bring-up/prototype tool --
// not a substitute for a real flash filesystem in a shipping product.
// The directory lives in blocks 2045/2046 as a power-fail-safe log of CRC'd copies (see
// dirSave); data blocks run 0..2044 and each is erased when a file claims it.
static const uint32_t PAGE_SIZE        = 2048;
static const uint32_t PAGES_PER_BLOCK  = 64;
static const uint32_t DATA_START_BLOCK = 0;
static const uint32_t DATA_END_BLOCK   = 2044;   // inclusive; 2045/2046 = directory copies A/B, 2047 = the 'F' self-test block
static const uint32_t DIR_BLOCK_A      = 2045;   // the directory alternates between these two blocks
static const uint32_t DIR_BLOCK_B      = 2046;   // (B was the single directory block in older firmware)
static const uint32_t DIR_BLOCKS[2]    = {DIR_BLOCK_A, DIR_BLOCK_B};
static const uint8_t  MAX_FILES         = 8;
static const uint8_t  MAX_BLOCKS_PER_FILE = 32;  // ~4 MB/file cap at 2048 B pages

#pragma pack(push, 1)
struct FileEntry {
  uint8_t  valid;
  uint8_t  recording;
  uint32_t startEpoch;                // whole seconds (kept for older tools); 0 if never time-synced
  uint32_t recordCount;
  uint8_t  numBlocks;
  uint32_t blocks[MAX_BLOCKS_PER_FILE];
};
struct DirImage {
  uint32_t magic;
  uint32_t nextFreeBlock;
  FileEntry files[MAX_FILES];
  // Appended AFTER the original layout so a directory written by older firmware still
  // loads: those bytes were never programmed and read back as 0xFF.
  // 0xFFFFFFFFFFFFFFFF = file made by old firmware (use startEpoch seconds), 0 = never synced.
  uint64_t startEpochUs[MAX_FILES];   // epoch microseconds at sample t_ms == 0
};
// Each directory page on NAND is a DirHeader followed by a DirImage (see dirSave).
struct DirHeader {
  uint32_t magic;                     // DIR_PAGE_MAGIC
  uint32_t seq;                       // increases with every save; the highest valid one wins
  uint32_t crc;                       // CRC-32 of the DirImage that follows
};
#pragma pack(pop)

static_assert(sizeof(SampleRecord) == 28, "record size drifted");
static_assert(sizeof(DirHeader) + sizeof(DirImage) <= PAGE_SIZE, "directory no longer fits one page");
// 72 records = 2016 bytes per NAND page. The 8-byte GET_PAGE reply header plus this must
// stay <= 2048 (the app's max frame payload), and a page always holds whole records.
static const uint32_t RECORDS_PER_PAGE = (PAGE_SIZE - 8) / sizeof(SampleRecord);
static const uint32_t DIR_MAGIC = 0x53544C47;    // 'STLG' (inside the DirImage; also the whole-page magic of older firmware)
static const uint32_t DIR_PAGE_MAGIC = 0x53544432;   // 'STD2' (DirHeader of a CRC'd directory page)

static DirImage dir;
static int8_t   activeFile = -1;
static uint8_t  logPageBuf[PAGE_SIZE];

static uint32_t rowOf(uint32_t block, uint32_t page) {
  return (block << 6) | page;
}

static bool blockBad(uint32_t block) {
  uint8_t bbm = 0xFF, st;
  nReadPage(rowOf(block, 0), PAGE_SIZE, &bbm, 1, st);   // spare-area bad-block marker
  return bbm != 0xFF;
}

// ---- directory persistence: power-fail-safe and wear-friendly ----
// The directory is a log of CRC'd copies spread over two blocks (A and B). Every save
// programs the NEXT blank page; a block is erased only when the log wraps onto it, and at
// that moment the newest copy is safe in the other block. A power cut at any point
// therefore leaves at least one complete, CRC-valid copy, and boot picks the highest
// sequence number whose CRC checks out. As a side effect the directory is erased once per
// 64 saves instead of on every checkpoint (every ~6 s while recording).
// pageBuf / readBuf are shared scratch (only the 'F' self-test also uses them).
static uint32_t dirCrc32(const uint8_t *p, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  while (n--) {
    c ^= *p++;
    for (int k = 0; k < 8; k++) c = (c & 1) ? (c >> 1) ^ 0xEDB88320u : (c >> 1);
  }
  return ~c;
}

static uint32_t dirSeq = 0;      // sequence number of the newest saved copy (0 = none)
static int      dirBlock = 0;    // index into DIR_BLOCKS of that copy
static int      dirPage = -1;    // page of that copy (-1 = none yet)

static bool pageBlank(uint32_t row, size_t len) {
  uint8_t st;
  if (!nReadPage(row, 0, readBuf, len, st)) return false;
  for (size_t i = 0; i < len; i++) if (readBuf[i] != 0xFF) return false;
  return true;
}

static void dirSave() {
  const size_t len = sizeof(DirHeader) + sizeof(DirImage);
  DirHeader hdr;
  hdr.magic = DIR_PAGE_MAGIC;
  hdr.seq = dirSeq + 1;
  hdr.crc = dirCrc32((const uint8_t *)&dir, sizeof(DirImage));
  memcpy(pageBuf, &hdr, sizeof(hdr));
  memcpy(pageBuf + sizeof(hdr), &dir, sizeof(DirImage));

  uint8_t st;
  int blk = dirBlock, pg = dirPage;
  bool switched = false;
  for (int attempt = 0; attempt < (int)(PAGES_PER_BLOCK * 2); attempt++) {
    pg++;
    if (pg >= (int)PAGES_PER_BLOCK) {
      if (switched) break;                             // never erase the block that holds the newest copy
      switched = true;
      blk ^= 1;
      pg = 0;
    }
    if (pg == 0) nErase(rowOf(DIR_BLOCKS[blk], 0), st);   // safe: newest copy is in the other block (or none exists yet)
    uint32_t row = rowOf(DIR_BLOCKS[blk], pg);
    if (!pageBlank(row, len)) continue;                // torn leftover or failed erase: skip this page
    if (!nProgramPage(row, pageBuf, len, st)) continue;
    if (!nReadPage(row, 0, readBuf, len, st) || ((st >> 4) & 7) == 2 ||
        memcmp(readBuf, pageBuf, len) != 0) continue;   // read back and verify
    dirBlock = blk; dirPage = pg; dirSeq = hdr.seq;
    return;
  }
  Serial.println("NAND directory save FAILED -- previous good copy left in place");
}

// True if the page holds programmed data: its first record's t_ms is not the erased 0xFFFFFFFF
// and the NAND's ECC didn't flag it uncorrectable (a torn write from a power loss).
static bool pageProgrammed(uint32_t row) {
  uint8_t b[4], st;
  if (!nReadPage(row, 0, b, sizeof b, st)) return false;
  if (((st >> 4) & 7) == 2) return false;           // ECC status 010 = uncorrectable
  return !(b[0] == 0xFF && b[1] == 0xFF && b[2] == 0xFF && b[3] == 0xFF);
}

// A file still flagged "recording" at boot means power was lost mid-recording. It must NOT be
// resumed: the RAM page buffer is gone and t_ms (relative to its start) can't be continued.
// Instead close it. The directory is only checkpointed every 4 pages, so fully programmed
// pages (and possibly one newly claimed block) can exist beyond what it records -- find
// them first so they aren't lost. Only the partly filled page that was still in RAM is
// lost, at most RECORDS_PER_PAGE - 1 samples (~1.4 s at 50 Hz).
static void recoverInterruptedFiles() {
  bool changed = false;
  for (int i = 0; i < MAX_FILES; i++) {
    FileEntry &f = dir.files[i];
    if (!f.valid || !f.recording) continue;
    uint32_t before = f.recordCount;
    for (;;) {
      uint32_t page = f.recordCount / RECORDS_PER_PAGE;        // next page that could hold data
      uint32_t blockSlot = page / PAGES_PER_BLOCK, pageInBlock = page % PAGES_PER_BLOCK;
      bool adopted = false;
      uint32_t savedNext = dir.nextFreeBlock;
      if (blockSlot >= f.numBlocks) {                          // file may have claimed a block after the checkpoint
        if (f.numBlocks >= MAX_BLOCKS_PER_FILE) break;
        uint32_t nb = dir.nextFreeBlock;
        while (nb <= DATA_END_BLOCK && blockBad(nb)) nb++;
        if (nb > DATA_END_BLOCK) break;
        f.blocks[f.numBlocks++] = nb;
        dir.nextFreeBlock = nb + 1;
        adopted = true;
      }
      if (!pageProgrammed(rowOf(f.blocks[blockSlot], pageInBlock))) {
        if (adopted) {                                         // nothing in it: leave the block free
          f.numBlocks--;
          dir.nextFreeBlock = savedNext;
        }
        break;
      }
      f.recordCount += RECORDS_PER_PAGE;
    }
    f.recording = 0;
    changed = true;
    Serial.printf("Recovered interrupted recording: file %d, %lu -> %lu records\n",
                  i, (unsigned long)before, (unsigned long)f.recordCount);
  }
  if (changed) dirSave();
}

// Scans both directory blocks for the newest copy whose CRC verifies. On success `dir` holds
// it and dirSeq/dirBlock/dirPage point at it, so the next save continues after it.
static bool dirFindNewest() {
  const size_t len = sizeof(DirHeader) + sizeof(DirImage);
  uint32_t seqs[2][PAGES_PER_BLOCK];                // 0 = erased / not a directory page
  uint8_t st, hdr[8];
  for (int b = 0; b < 2; b++) {
    for (uint32_t p = 0; p < PAGES_PER_BLOCK; p++) {
      seqs[b][p] = 0;
      if (!nReadPage(rowOf(DIR_BLOCKS[b], p), 0, hdr, sizeof hdr, st) || ((st >> 4) & 7) == 2) continue;
      uint32_t magic, seq;
      memcpy(&magic, hdr, 4);
      memcpy(&seq, hdr + 4, 4);
      if (magic == DIR_PAGE_MAGIC) seqs[b][p] = seq;
    }
  }
  for (;;) {
    uint32_t best = 0, bp = 0;
    int bb = -1;
    for (int b = 0; b < 2; b++)
      for (uint32_t p = 0; p < PAGES_PER_BLOCK; p++)
        if (seqs[b][p] > best) {
          best = seqs[b][p];
          bb = b;
          bp = p;
        }
    if (bb < 0) return false;                       // no valid copy anywhere
    seqs[bb][bp] = 0;                               // consumed either way
    if (!nReadPage(rowOf(DIR_BLOCKS[bb], bp), 0, readBuf, len, st) || ((st >> 4) & 7) == 2) continue;
    DirHeader h;
    memcpy(&h, readBuf, sizeof h);
    if (h.magic != DIR_PAGE_MAGIC) continue;
    if (h.crc != dirCrc32(readBuf + sizeof(DirHeader), sizeof(DirImage))) continue;   // torn write: try the next newest
    memcpy(&dir, readBuf + sizeof(DirHeader), sizeof(DirImage));
    if (dir.magic != DIR_MAGIC) continue;
    dirSeq = h.seq; dirBlock = bb; dirPage = (int)bp;
    return true;
  }
}

static void dirLoad() {
  uint8_t st;
  nSetF(0xA0, 0x00);                                // unlock all blocks (re-locks every power cycle)

  if (dirFindNewest()) {
    recoverInterruptedFiles();                      // activeFile stays -1: recording only starts on REC_START
    return;
  }

  // No CRC'd copy. Older firmware kept a single un-CRC'd directory in block B page 0.
  dirBlock = 0; dirPage = -1; dirSeq = 0;
  nReadPage(rowOf(DIR_BLOCK_B, 0), 0, (uint8_t *)&dir, sizeof(dir), st);
  if (dir.magic == DIR_MAGIC && ((st >> 4) & 7) != 2) {
    if (dir.nextFreeBlock > DATA_END_BLOCK + 1) {
      Serial.println("WARNING: old firmware had data in block 2045, which is now a directory block");
      dir.nextFreeBlock = DATA_END_BLOCK + 1;
    }
    dirSave();                                      // first CRC'd copy goes to block A; the old copy in B is left alone
    Serial.println("NAND directory migrated from older firmware");
    recoverInterruptedFiles();
    return;
  }

  // Blank NAND (first boot) or both copies unreadable: format.
  uint8_t e;
  nErase(rowOf(DIR_BLOCK_A, 0), e);
  nErase(rowOf(DIR_BLOCK_B, 0), e);
  memset(&dir, 0, sizeof(dir));
  dir.magic = DIR_MAGIC;
  dir.nextFreeBlock = DATA_START_BLOCK;
  dirSave();
  Serial.println("NAND directory formatted (blank or corrupt)");
}

static bool allocBlock(uint32_t &blockOut) {
  while (dir.nextFreeBlock <= DATA_END_BLOCK) {
    uint32_t b = dir.nextFreeBlock++;
    if (blockBad(b)) {
      Serial.printf("NAND block %u marked bad, skipping\n", (unsigned)b);
      continue;
    }
    // Always hand out an erased block. After a power loss a free block can hold a torn,
    // half-programmed page, and NAND pages can't be programmed twice without an erase.
    // (Costs a few ms once per block, i.e. about every 90 s of recording.)
    uint8_t st;
    if (nErase(rowOf(b, 0), st)) {
      blockOut = b;
      return true;
    }
    Serial.printf("NAND block %u failed to erase, skipping\n", (unsigned)b);
  }
  return false;
}

// ---- host time sync ----
// epoch_us = nowUs() + gOffsetUs. Set by 0x0A SET_TIME_US (microsecond class) or the
// legacy 0x01 SET_TIME (whole seconds).
static int64_t gOffsetUs = 0;
static bool    gTimeSynced = false;

static uint64_t epochUsNow() {
  return gTimeSynced ? (uint64_t)((int64_t)nowUs() + gOffsetUs) : 0;
}
static uint32_t currentEpoch() {                   // whole seconds, for the STATUS reply
  return (uint32_t)(epochUsNow() / 1000000ULL);
}

static uint32_t logLastMs = 0, logTickCount = 0;
static uint64_t logStartUs = 0;                    // nowUs() at sample t_ms == 0
static float    lastPressPa = 0, lastTempC = 0, lastMagUT[3] = {0, 0, 0};
static const uint32_t LOG_RATE_HZ = 50;
static const uint32_t LOG_PERIOD_MS = 1000 / LOG_RATE_HZ;
static const uint8_t  MAG_BARO_REFRESH_DIV = 2;    // refresh mag/baro every Nth sample (25 Hz at 50 Hz logging)

// Start time (epoch us) of a file, falling back to whole seconds for files that were
// created by older firmware.
static uint64_t fileStartEpochUs(uint8_t i) {
  uint64_t v = dir.startEpochUs[i];
  if (v == UINT64_MAX) return (uint64_t)dir.files[i].startEpoch * 1000000ULL;
  return v;
}

// (Re)computes the active file's start time from logStartUs and the current offset.
// Called when recording starts and whenever the host syncs the clock mid-recording
// (the board clock is continuous, so the latest offset is the best estimate for the
// whole file). The directory is written to NAND at the next checkpoint / stop.
static void updateActiveStartTime() {
  if (activeFile < 0) return;
  uint64_t sUs = gTimeSynced ? (uint64_t)((int64_t)logStartUs + gOffsetUs) : 0;
  dir.startEpochUs[activeFile] = sUs;
  dir.files[activeFile].startEpoch = (uint32_t)(sUs / 1000000ULL);
}

static int logStart() {
  if (activeFile >= 0) return -1;                  // already recording
  int slot = -1;
  for (int i = 0; i < MAX_FILES; i++) if (!dir.files[i].valid) {
      slot = i;
      break;
    }
  if (slot < 0) return -2;                         // directory full
  FileEntry &f = dir.files[slot];
  memset(&f, 0, sizeof(f));
  uint32_t b0;
  if (!allocBlock(b0)) return -3;                  // NAND full
  f.valid = 1; f.recording = 1;
  f.numBlocks = 1; f.blocks[0] = b0;
  mmcContStart();                                  // its first result needs ~20 ms; the directory write below overlaps that
  logStartUs = nowUs();                            // t_ms == 0. Taken BEFORE the slow directory write, so the
  activeFile = slot;                               // start time that write saves is exact; the first sample
  updateActiveStartTime();                         // then lands a few ms after t_ms == 0.
  dirSave();
  if (mmcContOn) {                                 // prime the mag value so the first samples don't repeat the
    uint32_t t0 = millis();                        // previous recording's last reading
    float B[3];
    while (millis() - t0 < 40) {
      if (mmcContRead(B)) {
        memcpy(lastMagUT, B, sizeof(B));
        break;
      }
      delay(1);
    }
  }
  logLastMs = millis() - LOG_PERIOD_MS;            // fire on the first loop() tick
  logTickCount = 0;
  return slot;
}

static void logFlushPartial() {
  if (activeFile < 0) return;
  FileEntry &f = dir.files[activeFile];
  if (f.recordCount == 0 || f.recordCount % RECORDS_PER_PAGE == 0) return;  // nothing buffered
  uint32_t page = (f.recordCount - 1) / RECORDS_PER_PAGE;
  uint32_t blockSlot = page / PAGES_PER_BLOCK, pageInBlock = page % PAGES_PER_BLOCK;
  uint8_t st;
  nProgramPage(rowOf(f.blocks[blockSlot], pageInBlock), logPageBuf, PAGE_SIZE, st);
}

static int logStop() {
  if (activeFile < 0) return -1;                    // not recording
  logFlushPartial();
  mmcContStop();
  dir.files[activeFile].recording = 0;
  dirSave();
  int id = activeFile;
  activeFile = -1;
  return id;
}

static bool logAppend(const SampleRecord &rec) {
  if (activeFile < 0) return false;
  FileEntry &f = dir.files[activeFile];
  uint32_t idx = f.recordCount;
  uint32_t page = idx / RECORDS_PER_PAGE;
  uint32_t blockSlot = page / PAGES_PER_BLOCK, pageInBlock = page % PAGES_PER_BLOCK;

  if (blockSlot >= f.numBlocks) {
    if (f.numBlocks >= MAX_BLOCKS_PER_FILE) {
      f.recording = 0;
      mmcContStop();
      dirSave();
      activeFile = -1;
      return false;
    }
    uint32_t nb;
    if (!allocBlock(nb)) {
      f.recording = 0;  // NAND full
      mmcContStop();
      dirSave();
      activeFile = -1;
      return false;
    }
    f.blocks[f.numBlocks++] = nb;
  }

  uint32_t offInPage = (idx % RECORDS_PER_PAGE) * sizeof(SampleRecord);
  if (idx % RECORDS_PER_PAGE == 0) memset(logPageBuf, 0xFF, PAGE_SIZE);
  // NAND pages can only be programmed once per erase, so the page is built up
  // in RAM and only PROGRAMmed once it's full (or the recording stops).
  memcpy(logPageBuf + offInPage, &rec, sizeof(rec));
  f.recordCount++;

  if (f.recordCount % RECORDS_PER_PAGE == 0) {
    uint8_t st;
    nProgramPage(rowOf(f.blocks[blockSlot], pageInBlock), logPageBuf, PAGE_SIZE, st);
  }
  if (f.recordCount % (RECORDS_PER_PAGE * 4) == 0) dirSave();   // periodic checkpoint
  return true;
}

// Serves the requested file's page: from NAND if it's already been programmed,
// or straight out of the in-RAM buffer if it's the current (unflushed) page of
// an active recording.
static bool logReadPage(uint8_t fileId, uint32_t pageIndex, uint8_t *out, uint16_t &validBytes) {
  if (fileId >= MAX_FILES || !dir.files[fileId].valid) return false;
  FileEntry &f = dir.files[fileId];
  uint32_t firstRec = pageIndex * RECORDS_PER_PAGE;
  if (firstRec > f.recordCount) return false;
  validBytes = (uint16_t)(min((uint32_t)RECORDS_PER_PAGE, f.recordCount - firstRec) * sizeof(SampleRecord));
  if (validBytes == 0) return true;                 // one past the last page: empty reply = end of file

  if (fileId == activeFile && pageIndex == f.recordCount / RECORDS_PER_PAGE) {
    memcpy(out, logPageBuf, PAGE_SIZE);
    return true;
  }
  uint32_t blockSlot = pageIndex / PAGES_PER_BLOCK, pageInBlock = pageIndex % PAGES_PER_BLOCK;
  if (blockSlot >= f.numBlocks) return false;
  uint8_t st;
  return nReadPage(rowOf(f.blocks[blockSlot], pageInBlock), 0, out, PAGE_SIZE, st);
}

static void logTick() {
  if (activeFile < 0) return;
  uint32_t now = millis();
  if (now - logLastMs < LOG_PERIOD_MS) return;
  logLastMs += LOG_PERIOD_MS;
  if (now - logLastMs > LOG_PERIOD_MS) logLastMs = now;   // resync if we fell far behind (e.g. after a NAND erase)

  uint64_t sampleUs = nowUs();                      // stamp immediately before the IMU read
  float a[3], g[3], tC;
  if (!lsmAddr || !lsmRead(a, g, tC)) return;

  if (logTickCount % MAG_BARO_REFRESH_DIV == 0) {
    float p, bt, B[3];
    if (bmpOk && bmpRead(bt, p)) {
      lastPressPa = p;
      lastTempC = bt;
    }
    if (mmcContOn && mmcContRead(B)) memcpy(lastMagUT, B, sizeof(B));   // ~0.5 ms; no new result -> carry forward
  }
  logTickCount++;

  SampleRecord rec;
  rec.t_ms = (uint32_t)((sampleUs - logStartUs + 500) / 1000);   // rounded to nearest ms
  rec.ax_mg = (int16_t)lroundf(a[0] * 1000.0f);
  rec.ay_mg = (int16_t)lroundf(a[1] * 1000.0f);
  rec.az_mg = (int16_t)lroundf(a[2] * 1000.0f);
  rec.gx_dps10 = (int16_t)lroundf(g[0] * 10.0f);
  rec.gy_dps10 = (int16_t)lroundf(g[1] * 10.0f);
  rec.gz_dps10 = (int16_t)lroundf(g[2] * 10.0f);
  rec.press_pa = (int32_t)lroundf(lastPressPa);
  rec.temp_cdeg = (int16_t)lroundf(lastTempC * 100.0f);
  rec.mx_dt = (int16_t)lroundf(lastMagUT[0] * 10.0f);
  rec.my_dt = (int16_t)lroundf(lastMagUT[1] * 10.0f);
  rec.mz_dt = (int16_t)lroundf(lastMagUT[2] * 10.0f);
  logAppend(rec);
}

static void printLogStatus() {
  Serial.println("\n-- Log / time-sync / BLE status --");
  Serial.printf("  Time sync: %s", gTimeSynced ? "yes" : "no");
  if (gTimeSynced) Serial.printf(" (epoch now %llu us, offset %lld us)",
                                   (unsigned long long)epochUsNow(), (long long)gOffsetUs);
  Serial.println();
  uint16_t freeBlocks = (dir.nextFreeBlock <= DATA_END_BLOCK) ? (uint16_t)(DATA_END_BLOCK - dir.nextFreeBlock + 1) : 0;
  Serial.printf("  NAND free blocks: %u / %u\n", freeBlocks, (unsigned)(DATA_END_BLOCK - DATA_START_BLOCK + 1));
  for (uint8_t i = 0; i < MAX_FILES; i++) {
    if (!dir.files[i].valid) continue;
    Serial.printf("  file %u: %s, %lu records, %u blocks, startEpochUs=%llu\n",
                  i, dir.files[i].recording ? "RECORDING" : "stopped",
                  (unsigned long)dir.files[i].recordCount, dir.files[i].numBlocks,
                  (unsigned long long)fileStartEpochUs(i));
  }
}

// ---------------------------------------------------------------- Host protocol
// Frame: SOF(0xAA) LEN_LO LEN_HI CMD PAYLOAD[LEN] CRC8. CRC8 covers CMD+PAYLOAD,
// poly 0x31, init 0x00, MSB-first, no reflection.
static const uint8_t  PROTO_SOF = 0xAA;
static const uint16_t PROTO_MAX_PAYLOAD = PAGE_SIZE;   // one NAND page keeps GET_PAGE simple
static const uint8_t  PROTO_VERSION = 2;               // 2 = TIME_PING / SET_TIME_US / 14-byte LIST_FILES entries

enum : uint8_t {
  CMD_PING = 0x00, CMD_SET_TIME = 0x01, CMD_REC_START = 0x02, CMD_REC_STOP = 0x03,
  CMD_LIST_FILES = 0x04, CMD_GET_PAGE = 0x05, CMD_DELETE = 0x06, CMD_STATUS = 0x07,
  CMD_POWER_OFF = 0x08, CMD_TIME_PING = 0x09, CMD_SET_TIME_US = 0x0A,
};
enum : uint8_t {
  ERR_OK = 0, ERR_ALREADY_RECORDING = 1, ERR_TABLE_FULL = 2, ERR_NO_SPACE = 3,
  ERR_NOT_RECORDING = 4, ERR_BAD_FILE = 5, ERR_BAD_PAGE = 6, ERR_TOO_BIG_FOR_TRANSPORT = 7,
};

static uint8_t crc8_update(uint8_t crc, uint8_t b) {
  crc ^= b;
  for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  return crc;
}

static BLECharacteristic *bleTx = nullptr;
static volatile bool bleConnected = false;
static float readVbatt(int n = 32);

static void serialSendFrame(uint8_t cmd, const uint8_t *payload, uint16_t len) {
  // Build the whole frame and write it in chunks. Per-byte Serial.write() on the
  // native USB CDC is very slow/lossy for a 2 KB page reply.
  static uint8_t fb[5 + 8 + PAGE_SIZE];
  if ((size_t)len + 5 > sizeof(fb)) return;
  uint8_t crc = crc8_update(0, cmd);
  fb[0] = PROTO_SOF; fb[1] = (uint8_t)(len & 0xFF); fb[2] = (uint8_t)(len >> 8); fb[3] = cmd;
  for (uint16_t i = 0; i < len; i++) {
    fb[4 + i] = payload[i];
    crc = crc8_update(crc, payload[i]);
  }
  fb[4 + len] = crc;
  size_t total = 5 + (size_t)len, off = 0;
  uint32_t t0 = millis();
  while (off < total && millis() - t0 < 2000) {
    size_t n = min((size_t)64, total - off);
    size_t w = Serial.write(fb + off, n);
    off += w;
    if (w == 0) delay(1);
  }
}

// Replies are sent in notifications of at most 20 bytes (default ATT MTU 23 - 3 header),
// which works with any central whatever MTU it negotiated. Frames up to 20 bytes (PING,
// STATUS, TIME_PING, one-file LIST_FILES) go out as a single notification; longer ones
// are fragmented and reassembled by the app's streaming parser.
static void bleSendFrame(uint8_t cmd, const uint8_t *payload, uint16_t len) {
  if (!bleConnected || !bleTx) return;
  static uint8_t buf[512];
  if (5 + len > sizeof(buf)) return;
  uint8_t crc = 0;
  buf[0] = PROTO_SOF; buf[1] = (uint8_t)(len & 0xFF); buf[2] = (uint8_t)(len >> 8); buf[3] = cmd;
  crc = crc8_update(crc, cmd);
  for (uint16_t i = 0; i < len; i++) {
    buf[4 + i] = payload[i];
    crc = crc8_update(crc, payload[i]);
  }
  buf[4 + len] = crc;
  size_t total = 5 + (size_t)len, off = 0;
  const size_t chunk = 20;
  while (off < total) {
    size_t n = min(chunk, total - off);
    bleTx->setValue(buf + off, n);
    bleTx->notify();
    off += n;
  }
}

static uint8_t buildStatus(uint8_t *rpl) {
  rpl[0] = (activeFile >= 0) ? 1 : 0;
  rpl[1] = (activeFile >= 0) ? (uint8_t)activeFile : 0xFF;
  uint32_t sc = (activeFile >= 0) ? dir.files[activeFile].recordCount : 0;
  memcpy(rpl + 2, &sc, 4);
  uint16_t freeBlocks = (dir.nextFreeBlock <= DATA_END_BLOCK) ? (uint16_t)(DATA_END_BLOCK - dir.nextFreeBlock + 1) : 0;
  memcpy(rpl + 6, &freeBlocks, 2);
  uint16_t vmv = (uint16_t)(readVbatt(8) * 1000.0f);
  memcpy(rpl + 8, &vmv, 2);
  rpl[10] = gTimeSynced ? 1 : 0;
  uint32_t ep = currentEpoch();
  memcpy(rpl + 11, &ep, 4);
  return 15;
}

// rxUs = nowUs() captured when the request frame arrived (used by TIME_PING).
static void handleCommand(Transport tr, uint8_t cmd, const uint8_t *pl, uint16_t len, uint64_t rxUs) {
  auto reply = [&](uint8_t rcmd, const uint8_t *rpl, uint16_t rlen) {
    if (tr == TR_SERIAL) serialSendFrame(rcmd, rpl, rlen); else bleSendFrame(rcmd, rpl, rlen);
  };
  switch (cmd) {
    case CMD_PING: {
        uint8_t v[1] = { PROTO_VERSION };
        reply(CMD_PING, v, 1);
        break;
      }
    case CMD_SET_TIME: {                              // legacy: whole seconds
        if (len < 4) return;
        uint32_t epoch; memcpy(&epoch, pl, 4);
        gOffsetUs = (int64_t)epoch * 1000000LL - (int64_t)nowUs();
        gTimeSynced = true;
        updateActiveStartTime();
        uint8_t ok[1] = { ERR_OK }; reply(CMD_SET_TIME, ok, 1);
        break;
      }
    case CMD_TIME_PING: {                             // req: seq(1)  reply: seq(1) t2_us(8) (t3-t2)_us(4)
        if (len < 1) return;
        uint8_t rpl[13];                              // 13 B payload -> 18 B frame: one BLE notification at any MTU
        rpl[0] = pl[0];
        uint64_t t2 = rxUs;
        memcpy(rpl + 1, &t2, 8);
        uint32_t dt = (uint32_t)(nowUs() - t2);       // t3 - t2, taken as late as possible (right before sending)
        memcpy(rpl + 9, &dt, 4);
        reply(CMD_TIME_PING, rpl, 13);
        break;
      }
    case CMD_SET_TIME_US: {                           // req: offset_us(int64 LE)  reply: err(1)
        if (len < 8) return;
        int64_t off; memcpy(&off, pl, 8);
        gOffsetUs = off;
        gTimeSynced = true;
        updateActiveStartTime();                      // if recording, refine the file's start time
        uint8_t ok[1] = { ERR_OK }; reply(CMD_SET_TIME_US, ok, 1);
        break;
      }
    case CMD_REC_START: {
        int r = logStart();
        uint8_t rpl[2] = { (uint8_t)(r >= 0 ? ERR_OK : (uint8_t)(-r)), (uint8_t)(r >= 0 ? r : 0xFF) };
        reply(CMD_REC_START, rpl, 2);
        break;
      }
    case CMD_REC_STOP: {
        int r = logStop();
        uint8_t rpl[2] = { (uint8_t)(r >= 0 ? ERR_OK : ERR_NOT_RECORDING), (uint8_t)(r >= 0 ? r : 0xFF) };
        reply(CMD_REC_STOP, rpl, 2);
        break;
      }
    case CMD_LIST_FILES: {                            // count(1) then id(1) startEpochUs(8) recordCount(4) recording(1)
        uint8_t rpl[1 + MAX_FILES * 14];
        uint8_t n = 0, *p = rpl + 1;
        for (uint8_t i = 0; i < MAX_FILES; i++) {
          if (!dir.files[i].valid) continue;
          uint64_t su = fileStartEpochUs(i);
          p[0] = i;
          memcpy(p + 1, &su, 8);
          memcpy(p + 9, &dir.files[i].recordCount, 4);
          p[13] = dir.files[i].recording;
          p += 14; n++;
        }
        rpl[0] = n;
        reply(CMD_LIST_FILES, rpl, 1 + n * 14);
        break;
      }
    case CMD_GET_PAGE: {
        if (tr == TR_BLE) {
          uint8_t e[1] = { ERR_TOO_BIG_FOR_TRANSPORT };
          reply(CMD_GET_PAGE, e, 1);
          break;
        }
        if (len < 5) return;
        uint8_t fileId = pl[0];
        uint32_t pageIndex; memcpy(&pageIndex, pl + 1, 4);
        static uint8_t pagebuf[PAGE_SIZE];
        uint16_t validBytes = 0;
        if (!logReadPage(fileId, pageIndex, pagebuf, validBytes)) {
          uint8_t e[1] = { ERR_BAD_PAGE }; reply(CMD_GET_PAGE, e, 1); break;
        }
        static uint8_t rpl[8 + PAGE_SIZE];
        rpl[0] = ERR_OK; rpl[1] = fileId;
        memcpy(rpl + 2, &pageIndex, 4);
        memcpy(rpl + 6, &validBytes, 2);
        memcpy(rpl + 8, pagebuf, validBytes);
        reply(CMD_GET_PAGE, rpl, 8 + validBytes);
        break;
      }
    case CMD_DELETE: {
        if (len < 1) return;
        uint8_t id = pl[0], e = ERR_BAD_FILE;
        if (id < MAX_FILES && dir.files[id].valid) {
          e = dir.files[id].recording ? ERR_ALREADY_RECORDING : ERR_OK;
          if (e == ERR_OK) {
            dir.files[id].valid = 0;
            dirSave();
          }
        }
        uint8_t rpl[1] = { e }; reply(CMD_DELETE, rpl, 1);
        break;
      }
    case CMD_POWER_OFF: {
        uint8_t ok[1] = { ERR_OK };
        reply(CMD_POWER_OFF, ok, 1);
        delay(500);                     // Allow time for the reply frame to send
        digitalWrite(PIN_KEEP_ON, LOW); // Release the power latch
        break;
      }
    case CMD_STATUS: {
        uint8_t rpl[15];
        buildStatus(rpl);
        reply(CMD_STATUS, rpl, 15);
        break;
      }
    default: break;
  }
}

// Unsolicited STATUS notifications over BLE while recording (same 0x07 frame as a STATUS reply).
static const uint32_t BLE_STATUS_PERIOD_MS = 1000;
static uint32_t bleStatusLastMs = 0;

static void bleStatusTick() {
  if (!bleConnected || activeFile < 0) return;
  uint32_t now = millis();
  if (now - bleStatusLastMs < BLE_STATUS_PERIOD_MS) return;
  // Only send right after a sample, so the ADC read and notify land in the
  // slack before the next one rather than delaying it.
  if (now - logLastMs > LOG_PERIOD_MS - 6) return;
  bleStatusLastMs = now;
  uint8_t rpl[15];
  buildStatus(rpl);
  bleSendFrame(CMD_STATUS, rpl, 15);
}

// Non-blocking byte-at-a-time frame reader shared with the interactive menu:
// a leading 0xAA is claimed by this parser, anything else falls through to
// the single-letter menu switch in loop(). Returns true if it consumed a byte.
enum SerialFrameState : uint8_t { SF_SOF, SF_LEN0, SF_LEN1, SF_CMD, SF_PAYLOAD, SF_CRC };
static SerialFrameState sfState = SF_SOF;
static uint16_t sfLen = 0, sfIdx = 0;
static uint8_t  sfCmd = 0, sfCrc = 0;
static uint8_t  sfBuf[PROTO_MAX_PAYLOAD];
static uint32_t sfLastByteMs = 0;
static uint64_t sfRxUs = 0;                         // nowUs() when the current frame's SOF byte was read

static bool serialFrameStep() {
  if (!Serial.available()) return false;
  if (sfState != SF_SOF && millis() - sfLastByteMs > 250) sfState = SF_SOF;  // stalled frame, resync

  int c = Serial.peek();
  if (sfState == SF_SOF && c != PROTO_SOF) return false;   // not a frame byte; let the menu handle it
  Serial.read();
  sfLastByteMs = millis();
  if (sfState == SF_SOF) sfRxUs = nowUs();

  switch (sfState) {
    case SF_SOF:  sfState = SF_LEN0; break;
    case SF_LEN0: sfLen = (uint16_t)c; sfState = SF_LEN1; break;
    case SF_LEN1:
      sfLen |= (uint16_t)(c << 8); sfIdx = 0; sfCrc = 0;
      sfState = (sfLen > sizeof(sfBuf)) ? SF_SOF : SF_CMD;   // oversized: drop and resync
      break;
    case SF_CMD:
      sfCmd = (uint8_t)c; sfCrc = crc8_update(sfCrc, sfCmd);
      sfState = sfLen ? SF_PAYLOAD : SF_CRC;
      break;
    case SF_PAYLOAD:
      sfBuf[sfIdx++] = (uint8_t)c; sfCrc = crc8_update(sfCrc, (uint8_t)c);
      if (sfIdx >= sfLen) sfState = SF_CRC;
      break;
    case SF_CRC:
      if ((uint8_t)c == sfCrc) handleCommand(TR_SERIAL, sfCmd, sfBuf, sfLen, sfRxUs);
      else Serial.println("frame CRC mismatch, dropped");
      sfState = SF_SOF;
      break;
  }
  return true;
}

// ---------------------------------------------------------------- BLE
// Nordic-UART-style service so existing generic BLE terminal apps can at
// least talk to it, even though the payloads are this board's own framing.
static const char *BLE_SVC_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char *BLE_RX_UUID  = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";  // write:  host -> board
static const char *BLE_TX_UUID  = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";  // notify: board -> host

// BLE callbacks run on the Bluetooth stack's task, not the Arduino loop() task. To keep NAND,
// I2C and the directory single-threaded, RxCB only validates the frame and queues it;
// bleProcessQueue() (called first thing in loop()) executes it. The request's arrival time is
// captured in the callback so TIME_PING still reports an honest t2.
static const uint16_t BLE_MAX_CMD_PAYLOAD = 64;
struct BleCmd {
  uint8_t  cmd;
  uint16_t len;
  uint64_t rxUs;
  uint8_t  pl[BLE_MAX_CMD_PAYLOAD];
};
static QueueHandle_t bleCmdQueue = nullptr;

class ServerCB : public BLEServerCallbacks {
    void onConnect(BLEServer *) override {
      bleConnected = true;
      Serial.println("BLE: central connected");
    }
    void onDisconnect(BLEServer *s) override {
      bleConnected = false;
      Serial.println("BLE: disconnected, advertising again");
      s->getAdvertising()->start();
    }
};

class RxCB : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *c) override {
      uint64_t rxUs = nowUs();                        // first thing: arrival time for TIME_PING
      const uint8_t *b = c->getData();
      size_t n = c->getLength();
      if (!b) return;
      // Each BLE write is treated as one complete frame -- no reassembly across
      // writes, so the companion app must keep a command within one ATT write.
      if (n < 5 || b[0] != PROTO_SOF) return;
      uint16_t plen = (uint16_t)(b[1] | (b[2] << 8));
      if (n != (size_t)(4 + plen + 1)) return;
      uint8_t crc = crc8_update(0, b[3]);
      for (uint16_t i = 0; i < plen; i++) crc = crc8_update(crc, b[4 + i]);
      if (crc != b[4 + plen]) {
        Serial.println("BLE frame CRC mismatch");
        return;
      }
      if (plen > BLE_MAX_CMD_PAYLOAD || !bleCmdQueue) return;
      BleCmd q;
      q.cmd = b[3]; q.len = plen; q.rxUs = rxUs;
      memcpy(q.pl, b + 4, plen);
      xQueueSend(bleCmdQueue, &q, 0);                 // drop if the queue is full
    }
};

static void bleProcessQueue() {
  if (!bleCmdQueue) return;
  BleCmd q;
  while (xQueueReceive(bleCmdQueue, &q, 0) == pdTRUE) {
    handleCommand(TR_BLE, q.cmd, q.pl, q.len, q.rxUs);
  }
}

static void bleSetup() {
  char name[24];
  uint64_t mac = ESP.getEfuseMac();
  snprintf(name, sizeof(name), "SmartToy-%04X", (unsigned)(mac & 0xFFFF));
  BLEDevice::init(name);
  BLEDevice::setMTU(247);                            // allow a larger ATT MTU if the central asks (replies still go out in 20 B pieces)
  bleCmdQueue = xQueueCreate(8, sizeof(BleCmd));
  BLESecurity *sec = new BLESecurity();
  sec->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
  sec->setCapability(ESP_IO_CAP_NONE);               // Just Works bonding, no PIN entry
  sec->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  BLEService *svc = server->createService(BLE_SVC_UUID);

  BLECharacteristic *rx = svc->createCharacteristic(
                            BLE_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCB());

  bleTx = svc->createCharacteristic(BLE_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  bleTx->addDescriptor(new BLE2902());

  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SVC_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.printf("BLE advertising as \"%s\"\n", name);
}

// ---------------------------------------------------------------- Interrupt lines
static volatile uint32_t gLsmEdges = 0, gBmpEdges = 0;
static void IRAM_ATTR isrLsm() {
  gLsmEdges++;
}
static void IRAM_ATTR isrBmp() {
  gBmpEdges++;
}

static void testInterrupts() {
  Serial.println("\n-- Interrupt lines --");
  if (lsmAddr) {
    i2cWrite(lsmAddr, 0x0B, 0x80);                // pulsed data-ready
    i2cWrite(lsmAddr, 0x0D, 0x01);                // INT1_CTRL: accel DRDY
    gLsmEdges = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_INT_LSM), isrLsm, RISING);
    delay(1000);
    detachInterrupt(digitalPinToInterrupt(PIN_INT_LSM));
    i2cWrite(lsmAddr, 0x0D, 0x00);
    unsigned n = gLsmEdges;
    result(n > 80 && n < 130, "INT_LSM (IO20)", fmt("%u edges in 1 s, expect ~104", n));
  } else {
    result(false, "INT_LSM (IO20)", "LSM not initialised");
  }

  if (bmpOk) {
    i2cWrite(ADDR_BMP, 0x15, 0x01);               // INT_SOURCE: data ready
    i2cWrite(ADDR_BMP, 0x14, 0x0A);               // INT_CONFIG: enable, active-high, push-pull, pulsed
    gBmpEdges = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_INT_BMP), isrBmp, RISING);
    delay(1000);
    detachInterrupt(digitalPinToInterrupt(PIN_INT_BMP));
    i2cWrite(ADDR_BMP, 0x14, 0x02);               // disable
    unsigned n = gBmpEdges;
    result(n >= 40 && n <= 60, "INT_BMP (IO21)", fmt("%u edges in 1 s, expect ~50", n));
  } else {
    result(false, "INT_BMP (IO21)", "BMP not initialised");
  }
}

// ---------------------------------------------------------------- Battery divider
static float readVbatt(int n) {
  uint32_t s = 0;
  for (int i = 0; i < n; i++) s += analogReadMilliVolts(PIN_VBATT_DIV);
  return s / (float)n * 2.0f / 1000.0f;
}

static void testVbatt() {
  Serial.println("\n-- Battery divider (IO4) --");
  float v = readVbatt();
  result(v > 3.0f && v < 5.6f, "VBATTDIV", fmt("%.3f V at +BATT (~5 V with the VIN jumper)", v));
}

// ---------------------------------------------------------------- FFC GPIO
static void testFFCStatic() {
  Serial.println("\n-- FFC IO0/IO1 --");
  pinMode(PIN_FFC_IO0, INPUT_PULLUP); pinMode(PIN_FFC_IO1, INPUT_PULLUP); delay(2);
  int u0 = digitalRead(PIN_FFC_IO0), u1 = digitalRead(PIN_FFC_IO1);
  pinMode(PIN_FFC_IO0, INPUT_PULLDOWN); pinMode(PIN_FFC_IO1, INPUT_PULLDOWN); delay(2);
  int d0 = digitalRead(PIN_FFC_IO0), d1 = digitalRead(PIN_FFC_IO1);
  info("IO0", (u0 && !d0) ? String("follows pulls (undriven)") : fmt("stuck %s", u0 ? "HIGH" : "LOW"));
  info("IO1", (u1 && !d1) ? String("follows pulls (undriven)") : fmt("stuck %s", u1 ? "HIGH" : "LOW"));

  // bridge check: drive IO0, watch IO1 (pulled down)
  pinMode(PIN_FFC_IO0, OUTPUT);
  digitalWrite(PIN_FFC_IO0, HIGH); delay(2);
  bool bridged = digitalRead(PIN_FFC_IO1) == HIGH;
  digitalWrite(PIN_FFC_IO0, LOW);
  pinMode(PIN_FFC_IO0, INPUT);
  pinMode(PIN_FFC_IO1, INPUT);
  result(!bridged, "IO0/IO1 bridge", bridged ? "IO1 follows IO0 - solder bridge?" : "");
}

static void testFFCToggle() {
  Serial.println("\nToggling IO0/IO1 in antiphase at 2 Hz for 5 s...");
  pinMode(PIN_FFC_IO0, OUTPUT); pinMode(PIN_FFC_IO1, OUTPUT);
  for (int i = 0; i < 20; i++) {
    digitalWrite(PIN_FFC_IO0, i & 1); digitalWrite(PIN_FFC_IO1, !(i & 1));
    delay(250);
  }
  pinMode(PIN_FFC_IO0, INPUT); pinMode(PIN_FFC_IO1, INPUT);
  Serial.println("Done.");
}

// ---------------------------------------------------------------- BOOT / KEEP_ON
static void testBoot() {
  Serial.println("\nShort TP1 (BOOT) to GND within 10 s...");
  SPI.end();                                      // IO9 is shared with NAND MOSI
  pinMode(PIN_BOOT, INPUT_PULLUP);
  uint32_t t0 = millis();
  bool pressed = false;
  while (millis() - t0 < 10000) {
    if (digitalRead(PIN_BOOT) == LOW) {
      pressed = true;
      break;
    }
    delay(5);
  }
  if (pressed) while (digitalRead(PIN_BOOT) == LOW) delay(5);
  result(pressed, "BOOT (IO9)", pressed ? "low + release seen" : "never went low");
  SPI.begin(PIN_NAND_SCK, PIN_NAND_MISO, PIN_NAND_MOSI, -1);
}

static void testKeepOn() {
  Serial.println("\nReleasing KEEP_ON in 3 s -- remove the magnet now.");
  Serial.println("Expected: board powers off (USB serial drops).");
  delay(3000);
  Serial.flush();
  digitalWrite(PIN_KEEP_ON, LOW);
  delay(3000);
  digitalWrite(PIN_KEEP_ON, HIGH);
  Serial.println("Still alive -> magnet still near U1, Q2 bypassed, or Q1/Q2 not releasing. KEEP_ON re-asserted.");
}

// ---------------------------------------------------------------- VIZ: IMU on the back LEDs
// TILT: light pools on the downhill side like liquid in a level.
// SPIN: a comet circles the ring, following integrated yaw (gyro Z).
// Both: colour tracks rotation rate (blue = still -> red = ~720 dps),
//       and a jolt (|a| far from 1 g) flashes the ring white.
enum VizMode : uint8_t { VIZ_OFF, VIZ_TILT, VIZ_SPIN, VIZ_COUNT };
static const char *VIZ_NAMES[VIZ_COUNT] = {"off", "tilt", "spin"};
static uint8_t  vizMode = VIZ_TILT;
static float    gyroBias[3] = {0, 0, 0};
static float    lpAcc[3] = {0, 0, 1};
static float    yawDeg = 0, jolt = 0;
static uint32_t vizLastMs = 0;

static void vizCalibrate() {
  if (!lsmAddr) return;
  Serial.println("Calibrating gyro bias, hold still...");
  float sum[3] = {0, 0, 0}, a[3], g[3], t;
  int n = 0;
  for (int i = 0; i < 50; i++) {
    if (lsmRead(a, g, t)) {
      for (int k = 0; k < 3; k++) sum[k] += g[k];
      n++;
    }
    delay(10);
  }
  if (n) for (int k = 0; k < 3; k++) gyroBias[k] = sum[k] / n;
  yawDeg = 0;
  Serial.printf("Gyro bias %.2f %.2f %.2f dps\n", gyroBias[0], gyroBias[1], gyroBias[2]);
}

static void vizClear() {
  for (uint8_t idx : BACK_IDX) strip.setPixelColor(idx, 0);
  strip.show();
}

static void vizUpdate() {
  if (vizMode == VIZ_OFF || !lsmAddr) return;
  uint32_t now = millis();
  if (now - vizLastMs < 20) return;                   // ~50 Hz
  float dt = fminf((now - vizLastMs) / 1000.0f, 0.1f);
  vizLastMs = now;

  float a[3], g[3], t;
  if (!lsmRead(a, g, t)) return;
  for (int k = 0; k < 3; k++) {
    g[k] -= gyroBias[k];
    lpAcc[k] += 0.15f * (a[k] - lpAcc[k]);            // smooth gravity estimate
  }
  float rate = sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
  float amag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  if (fabsf(amag - 1.0f) > 0.6f) jolt = 1.0f;         // bump / drop / shake
  jolt *= 0.9f;
  yawDeg = fmodf(yawDeg + g[2] * dt + 360.0f, 360.0f);

  float spin = fminf(rate / 720.0f, 1.0f);
  uint16_t hue = (uint16_t)(43690.0f * (1.0f - spin));   // 43690 = blue, 0 = red
  uint8_t sat = (uint8_t)(255.0f * (1.0f - 0.85f * jolt));

  for (int k = 0; k < 4; k++) {
    float th = BACK_ANGLE_DEG[k];
    float lvl;
    if (vizMode == VIZ_TILT) {
      // accel reads +1 g "up"; the side tilted down sees a negative projection
      float r = radians(th);
      float down = -(lpAcc[0] * cosf(r) + lpAcc[1] * sinf(r));
      lvl = 0.06f + 0.94f * constrain(down / 0.5f, 0.0f, 1.0f);   // faint ring when level
    } else {
      float d = fabsf(fmodf(th - yawDeg + 540.0f, 360.0f) - 180.0f);  // 0..180 from comet head
      lvl = fmaxf(0.0f, 1.0f - d / 110.0f);
      lvl = 0.04f + 0.96f * lvl * lvl;
    }
    lvl = fmaxf(lvl, jolt);
    uint8_t val = (uint8_t)(lvl * VIZ_MAX_BRIGHT);
    strip.setPixelColor(BACK_IDX[k], strip.gamma32(strip.ColorHSV(hue, sat, val)));
  }
  strip.show();
}

static void vizNextMode() {
  vizMode = (vizMode + 1) % VIZ_COUNT;
  Serial.printf("Back-LED mode: %s\n", VIZ_NAMES[vizMode]);
  if (vizMode == VIZ_OFF) vizClear();
  if (vizMode == VIZ_SPIN) vizCalibrate();
}

// ---------------------------------------------------------------- Streaming
static void streamSensors() {
  Serial.println("\nStreaming at ~10 Hz, send any key to stop.");
  Serial.println("ax ay az (g) | gx gy gz (dps) | P (hPa) T (C) | Bx By Bz (uT) | Vbatt");
  while (Serial.available()) Serial.read();
  while (!Serial.available()) {
    float a[3] = {0}, g[3] = {0}, lt = 0, bt = 0, p = 0, B[3] = {0};
    if (lsmAddr) lsmRead(a, g, lt);
    if (bmpOk) bmpRead(bt, p);
    if (mmcOk) mmcReadUT(B);
    Serial.printf("%6.3f %6.3f %6.3f | %7.2f %7.2f %7.2f | %8.2f %5.2f | %7.1f %7.1f %7.1f | %.2f\n",
                  a[0], a[1], a[2], g[0], g[1], g[2], p / 100.0f, bt, B[0], B[1], B[2], readVbatt());
    for (int i = 0; i < 6; i++) {
      vizUpdate();
      delay(10);
    }
  }
  while (Serial.available()) Serial.read();
}

// Bench check for the continuous-mode setup (board still, not recording): compares the field
// from continuous mode (chip-side Auto SET/RESET) with the manual SET/RESET-differenced
// reading, and counts the fresh results arriving in one second (should be ~MAG_ODR_HZ).
// The datasheet doesn't say whether the Auto_SR output has the bridge offset removed. If
// "difference" is large and constant, treat it as an offset to calibrate out in post-processing.
static void testMagContinuous() {
  Serial.println("\n-- MMC5633 continuous mode vs manual SET/RESET (hold the board still) --");
  if (!mmcOk) {
    Serial.println("  MMC5633 not present");
    return;
  }
  if (activeFile >= 0) {
    Serial.println("  stop recording first");
    return;
  }
  float man[3] = {0, 0, 0}, con[3] = {0, 0, 0}, B[3];
  int nm = 0, nc = 0;
  for (int i = 0; i < 20; i++) {
    if (mmcReadUT(B)) {
      for (int k = 0; k < 3; k++) man[k] += B[k];
      nm++;
    }
  }
  mmcContStart();
  delay(60);                                      // let the first result land
  uint32_t t0 = millis();
  while (millis() - t0 < 1000) {                  // count fresh results for 1 s
    if (mmcContRead(B)) {
      for (int k = 0; k < 3; k++) con[k] += B[k];
      nc++;
    }
    delay(1);
  }
  mmcContStop();
  if (!nm || !nc) {
    result(false, "MAG cont check", fmt("no data (manual %d reads, continuous %d)", nm, nc));
    return;
  }
  for (int k = 0; k < 3; k++) {
    man[k] /= nm;
    con[k] /= nc;
  }
  info("manual", fmt("%.1f %.1f %.1f uT", man[0], man[1], man[2]));
  info("continuous", fmt("%.1f %.1f %.1f uT", con[0], con[1], con[2]));
  info("difference", fmt("%.1f %.1f %.1f uT (continuous - manual)",
                         con[0] - man[0], con[1] - man[1], con[2] - man[2]));
  result(nc >= MAG_ODR_HZ * 8 / 10 && nc <= MAG_ODR_HZ * 12 / 10, "MAG cont rate",
         fmt("%d fresh results in 1 s (expect ~%d)", nc, (int)MAG_ODR_HZ));
}

// ---------------------------------------------------------------- Top level
static void printMenu() {
  Serial.println(
    "\nCommands: a=all  i=I2C scan  s=stream  l=LED  f=NAND info  F=NAND write test\n"
    "          v=Vbatt  n=interrupts  g=toggle FFC IO  b=BOOT  k=release KEEP_ON\n"
    "          m=back-LED mode  c=gyro calibrate  p=log/BLE status  x=mag check  h=help");
}

static void runAll() {
  gPass = gFail = 0;
  Serial.println("\n========== SmartToy bring-up ==========");
  testI2CScan();
  testLSM();
  testBMP();
  testMMC();
  testInterrupts();
  testNAND();
  testVbatt();
  testFFCStatic();
  testLED();
  Serial.printf("\n========== %d passed, %d failed ==========\n", gPass, gFail);
  if (gFail) ledSet(255, 0, 0); else ledSet(0, 255, 0);
}

void setup() {
  pinMode(PIN_KEEP_ON, OUTPUT);
  digitalWrite(PIN_KEEP_ON, HIGH);                // latch power first thing
  pinMode(PIN_NAND_CS, OUTPUT);
  digitalWrite(PIN_NAND_CS, HIGH);
  pinMode(PIN_INT_LSM, INPUT);
  pinMode(PIN_INT_BMP, INPUT);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(200);

  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  SPI.begin(PIN_NAND_SCK, PIN_NAND_MISO, PIN_NAND_MOSI, -1);
  analogReadResolution(12);
  if (!strip.begin()) Serial.println("RMT init on IO8 failed -- LEDs disabled");
  strip.clear();
  strip.show();

  runAll();
  vizCalibrate();
  dirLoad();
  bleSetup();
  printMenu();
}

void loop() {
  bleProcessQueue();                              // BLE commands first: keeps TIME_PING latency low
  vizUpdate();
  logTick();
  bleStatusTick();
  bool gotFrameByte = false;
  for (int i = 0; i < 256 && serialFrameStep(); i++) gotFrameByte = true;   // drain a whole frame per pass
  if (gotFrameByte) return;                       // bytes consumed by the binary protocol
  if (!Serial.available()) {
    delay(1);
    return;
  }
  char c = Serial.read();
  switch (c) {
    case 'a': runAll(); break;
    case 'i': testI2CScan(); break;
    case 's': streamSensors(); break;
    case 'l': testLED(); break;
    case 'f': testNAND(); break;
    case 'F': {
        Serial.println("\nThis erases NAND block 2047. Send 'Y' within 10 s to continue.");
        delay(50);                                   // let the rest of the "F\r\n" line arrive
        while (Serial.available()) Serial.read();    // drain it
        int k = -1;
        uint32_t t0 = millis();
        while (millis() - t0 < 10000) {
          if (Serial.available()) {
            k = Serial.read();
            if (k == '\r' || k == '\n' || k == ' ') {
              k = -1;  // ignore line endings
              continue;
            }
            break;
          }
          delay(10);
        }
        while (Serial.available()) Serial.read();    // drain the confirmation's line ending
        if (k == 'Y' || k == 'y') testNANDDestructive();
        else Serial.println("Cancelled.");
        break;
      }
    case 'v': testVbatt(); break;
    case 'n': testInterrupts(); break;
    case 'g': testFFCToggle(); break;
    case 'b': testBoot(); break;
    case 'k': testKeepOn(); break;
    case 'm': vizNextMode(); break;
    case 'c': vizCalibrate(); break;
    case 'p': printLogStatus(); break;
    case 'x': testMagContinuous(); break;
    case 'h': case '?': printMenu(); break;
    case '\r': case '\n': case ' ': return;
    default: Serial.printf("Unknown command '%c'\n", c); printMenu(); break;
  }
}
