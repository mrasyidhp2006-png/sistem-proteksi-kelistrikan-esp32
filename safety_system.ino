#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RTClib.h"
#include <SPI.h>
#include <SD.h>
#include <Preferences.h>
#include <esp_task_wdt.h>

// ===================================================================
// KONFIGURASI HARDWARE & PIN
// ===================================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define WDT_TIMEOUT_SECONDS 5

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
RTC_DS1307 rtc;
Preferences prefs;

// MicroSD (VSPI: SCK=18, MISO=19, MOSI=23)
const int SD_CS = 5;

// Multiplexer CD74HC4067
// CATATAN: GPIO 12 (strapping pin) DIPINDAHKAN. S0 -> GPIO16, S1 -> GPIO17,
// S2 -> GPIO13, S3 -> GPIO14. GPIO 15 juga dihindari (strapping pin).
const int MUX_SIG = 34;
const int MUX_S0  = 16;
const int MUX_S1  = 17;
const int MUX_S2  = 13;
const int MUX_S3  = 14;

// Aktuator (Active-HIGH)
const int RELAY_GAS     = 25; // HIGH = Valve gas terbuka
const int RELAY_LISTRIK = 26; // HIGH = Listrik ON
const int RELAY_FAN     = 27; // HIGH = Fan ON
const int BUZZER_PIN    = 33; // HIGH = Sirine ON

// Tombol reset manual (Active-LOW, internal pull-up)
const int RESET_BTN_PIN = 32;

// ===================================================================
// PARAMETER THRESHOLD, HYSTERESIS, FILTER
// ===================================================================
// Gas (Ch 0-7). WARN harus LEBIH RENDAH dari TRIP.
// KALIBRASI per sensor di lapangan! Nilai ini hanya titik awal.
const int GAS_WARN_THRESH  = 2300;
const int GAS_TRIP_THRESH  = 3800;

// Suhu (Ch 8-15)
const int TEMP_WARN_THRESH = 2200;
const int TEMP_TRIP_THRESH = 3200;

const int HYSTERESIS = 150;

// Rentang valid sensor (di luar ini = kandidat fault)
const int SENSOR_MIN_VALID = 100;
const int SENSOR_MAX_VALID = 4000;

// Fault baru sah jika di luar rentang selama N scan berturut-turut
// (100 ms x 20 = 2 detik)
const uint8_t FAULT_CONFIRM_SCANS = 20;

// Channel yang benar-benar terpasang sensor. Set false untuk channel kosong.
bool channelEnabled[16] = {
  true, true, true, true, true, true, true, true,
  true, true, true, true, true, true, true, true
};

// Warm-up. Sensor MQ asli butuh beberapa menit; sesuaikan.
// Selama warm-up: hanya TRIP yang dipantau, WARNING & FAULT diabaikan.
const unsigned long WARMUP_DURATION_MS = 60000;

const float FILTER_ALPHA = 0.2;

// Tombol reset harus ditahan selama ini
const unsigned long RESET_HOLD_MS = 2000;

// ===================================================================
// STATE
// ===================================================================
enum SystemState {
  STATE_WARMUP,
  STATE_NORMAL,
  STATE_WARNING,
  STATE_TRIP_LATCHED
};

// Nilai angka = prioritas (makin besar makin berbahaya)
enum TripReason : uint8_t {
  REASON_NONE         = 0,
  REASON_SENSOR_FAULT = 1,
  REASON_TEMP_TRIP    = 2,
  REASON_GAS_TRIP     = 3
};

SystemState currentState = STATE_WARMUP;
uint8_t latchedReason = REASON_NONE;

float filteredValues[16];
uint8_t faultCount[16];

int lastHazardChannel = -1;
const char* lastHazardType = "NORMAL";
uint8_t lastWarnType = 0; // 0 none, 1 gas, 2 suhu

unsigned long systemStartTime = 0;
unsigned long lastScanTime    = 0;
unsigned long lastSerialTime  = 0;
unsigned long lastDisplayTime = 0;
unsigned long lastBuzzerTime  = 0;
unsigned long lastRtcTime     = 0;
bool buzzerToggleState = false;
bool sdActive = false;

DateTime cachedNow(2000, 1, 1, 0, 0, 0);

// Tombol reset
bool btnDown = false;
bool resetHandled = false;
unsigned long btnStart = 0;
unsigned long resetRefusedUntil = 0;

// ===================================================================
// ANTREAN LOG (log ditulis SETELAH aktuator dieksekusi)
// ===================================================================
struct LogEntry {
  DateTime ts;
  char status[16];
  int ch;
  char type[24];
  int val;
};

const int LOG_QUEUE_SIZE = 6;
LogEntry logQueue[LOG_QUEUE_SIZE];
int logHead = 0;
int logTail = 0;

void enqueueLog(const char* status, int ch, const char* type, int val) {
  int next = (logHead + 1) % LOG_QUEUE_SIZE;
  if (next == logTail) return; // penuh, buang entri baru
  LogEntry &e = logQueue[logHead];
  e.ts = cachedNow;
  strncpy(e.status, status, sizeof(e.status) - 1);
  e.status[sizeof(e.status) - 1] = '\0';
  e.ch = ch;
  strncpy(e.type, type, sizeof(e.type) - 1);
  e.type[sizeof(e.type) - 1] = '\0';
  e.val = val;
  logHead = next;
}

void writeLogToSD(const LogEntry &e) {
  if (!sdActive) return;

  File logFile = SD.open("/riwayat.csv", FILE_WRITE);
  if (logFile) {
    char timeBuff[25];
    snprintf(timeBuff, sizeof(timeBuff), "%04d-%02d-%02d %02d:%02d:%02d",
             e.ts.year(), e.ts.month(), e.ts.day(),
             e.ts.hour(), e.ts.minute(), e.ts.second());

    logFile.print(timeBuff); logFile.print(",");
    logFile.print(e.status); logFile.print(",");
    logFile.print("Ch_"); logFile.print(e.ch); logFile.print(",");
    logFile.print(e.type); logFile.print(",");
    logFile.println(e.val);
    logFile.close();
    Serial.println(F(" -> SD CARD LOG SAVED"));
  }
}

// Tulis maksimal 1 entri per pemanggilan agar loop tetap responsif
void flushLog() {
  if (logTail == logHead) return;
  writeLogToSD(logQueue[logTail]);
  logTail = (logTail + 1) % LOG_QUEUE_SIZE;
}

// ===================================================================
// UTILITAS
// ===================================================================
const char* reasonText(uint8_t r) {
  switch (r) {
    case REASON_GAS_TRIP:     return "GAS BOCOR";
    case REASON_TEMP_TRIP:    return "SUHU OVERHEAT";
    case REASON_SENSOR_FAULT: return "SENSOR FAULT";
    default:                  return "NORMAL";
  }
}

void formatTime(char* buf, size_t len, const DateTime &dt) {
  snprintf(buf, len, "%02d:%02d:%02d  %02d/%02d/%04d",
           dt.hour(), dt.minute(), dt.second(),
           dt.day(), dt.month(), dt.year());
}

void saveLatchToNVS() {
  prefs.putUChar("reason", latchedReason);
  prefs.putChar("ch", (int8_t)lastHazardChannel);
}

// ===================================================================
// MEMBACA MUX & FILTER
// ===================================================================
int readMuxRaw(int channel) {
  digitalWrite(MUX_S0, bitRead(channel, 0));
  digitalWrite(MUX_S1, bitRead(channel, 1));
  digitalWrite(MUX_S2, bitRead(channel, 2));
  digitalWrite(MUX_S3, bitRead(channel, 3));
  delayMicroseconds(50);
  analogRead(MUX_SIG);              // dummy read: buang sisa muatan channel sebelumnya
  int a = analogRead(MUX_SIG);
  int b = analogRead(MUX_SIG);
  return (a + b) / 2;
}

void scanAndFilterSensors() {
  for (int i = 0; i < 16; i++) {
    int rawVal = readMuxRaw(i);

    if (rawVal < SENSOR_MIN_VALID || rawVal > SENSOR_MAX_VALID) {
      if (faultCount[i] < 255) faultCount[i]++;
    } else {
      faultCount[i] = 0;
    }

    filteredValues[i] = (FILTER_ALPHA * rawVal) + ((1.0f - FILTER_ALPHA) * filteredValues[i]);
  }
}

// ===================================================================
// KONTROL AKTUATOR
// ===================================================================
void applyActuators() {
  switch (currentState) {
    case STATE_WARMUP:
    case STATE_NORMAL:
      digitalWrite(RELAY_GAS, HIGH);
      digitalWrite(RELAY_LISTRIK, HIGH);
      digitalWrite(RELAY_FAN, LOW);
      digitalWrite(BUZZER_PIN, LOW);
      break;

    case STATE_WARNING:
      digitalWrite(RELAY_GAS, HIGH);
      digitalWrite(RELAY_LISTRIK, HIGH);
      digitalWrite(RELAY_FAN, HIGH);
      // Buzzer diatur oleh task buzzer intermiten
      break;

    case STATE_TRIP_LATCHED:
      if (latchedReason == REASON_GAS_TRIP) {
        digitalWrite(RELAY_GAS, LOW);      // Valve menutup
        digitalWrite(RELAY_LISTRIK, LOW);  // Listrik trip
        digitalWrite(RELAY_FAN, HIGH);     // Ventilasi
        digitalWrite(BUZZER_PIN, HIGH);
      } else {
        // Suhu overheat & sensor fault: gas tutup, listrik tetap, fan ON
        digitalWrite(RELAY_GAS, LOW);
        digitalWrite(RELAY_LISTRIK, HIGH);
        digitalWrite(RELAY_FAN, HIGH);
        digitalWrite(BUZZER_PIN, HIGH);
      }
      break;
  }
}

// ===================================================================
// LOGIKA STATE MACHINE
// ===================================================================
bool allChannelsBelowWarn() {
  for (int i = 0; i < 16; i++) {
    if (!channelEnabled[i]) continue;
    int limit = (i < 8) ? (GAS_WARN_THRESH - HYSTERESIS) : (TEMP_WARN_THRESH - HYSTERESIS);
    if (filteredValues[i] > limit) return false;
  }
  return true;
}

bool anyFaultActive() {
  for (int i = 0; i < 16; i++) {
    if (channelEnabled[i] && faultCount[i] > 0) return true;
  }
  return false;
}

void tripTo(uint8_t reason, int ch) {
  currentState = STATE_TRIP_LATCHED;
  latchedReason = reason;
  lastHazardChannel = ch;
  lastHazardType = reasonText(reason);
  saveLatchToNVS();

  Serial.print(F("\n>>> EMERGENCY TRIP LATCHED! Tipe: "));
  Serial.println(lastHazardType);
  enqueueLog("TRIP", ch, lastHazardType, (int)filteredValues[ch]);
}

void escalateTrip(uint8_t reason, int ch) {
  latchedReason = reason;
  lastHazardChannel = ch;
  lastHazardType = reasonText(reason);
  saveLatchToNVS();

  Serial.print(F("\n>>> ESKALASI BAHAYA! Tipe: "));
  Serial.println(lastHazardType);
  enqueueLog("ESKALASI", ch, lastHazardType, (int)filteredValues[ch]);
}

void tryReset() {
  if (currentState != STATE_TRIP_LATCHED) return;

  // Semua channel harus aman (di bawah ambang WARN - hysteresis) dan tanpa fault
  if (!allChannelsBelowWarn() || anyFaultActive()) {
    Serial.println(F("\n>>> RESET DITOLAK! SENSOR MASIH BAHAYA / FAULT <<<"));
    resetRefusedUntil = millis() + 3000;
    return;
  }

  currentState = STATE_NORMAL;
  latchedReason = REASON_NONE;
  lastHazardChannel = -1;
  lastHazardType = "NORMAL";
  lastWarnType = 0;
  saveLatchToNVS();
  applyActuators();

  Serial.println(F("\n>>> RESET MANUAL BERHASIL. KONDISI NORMAL KEMBALI <<<"));
  enqueueLog("RESET_MANUAL", -1, "NORMAL", 0);
}

void handleResetButton(unsigned long ms) {
  bool pressed = (digitalRead(RESET_BTN_PIN) == LOW);
  if (pressed) {
    if (!btnDown) {
      btnDown = true;
      btnStart = ms;
      resetHandled = false;
    } else if (!resetHandled && (ms - btnStart >= RESET_HOLD_MS)) {
      resetHandled = true;
      tryReset();
    }
  } else {
    btnDown = false;
  }
}

void evaluateAndUpdateState(unsigned long ms) {
  bool inWarmup = (currentState == STATE_WARMUP);

  int gasTripCh = -1, tempTripCh = -1, faultCh = -1;
  int gasWarnCh = -1, tempWarnCh = -1;

  for (int i = 0; i < 16; i++) {
    if (!channelEnabled[i]) continue;
    float v = filteredValues[i];

    if (!inWarmup && faultCount[i] >= FAULT_CONFIRM_SCANS && faultCh == -1) {
      faultCh = i;
    }

    if (i < 8) {
      if (v > GAS_TRIP_THRESH) {
        if (gasTripCh == -1 || v > filteredValues[gasTripCh]) gasTripCh = i;
      } else if (v > GAS_WARN_THRESH) {
        if (gasWarnCh == -1 || v > filteredValues[gasWarnCh]) gasWarnCh = i;
      }
    } else {
      if (v > TEMP_TRIP_THRESH) {
        if (tempTripCh == -1 || v > filteredValues[tempTripCh]) tempTripCh = i;
      } else if (v > TEMP_WARN_THRESH) {
        if (tempWarnCh == -1 || v > filteredValues[tempWarnCh]) tempWarnCh = i;
      }
    }
  }

  // Tentukan bahaya tertinggi (prioritas: GAS > SUHU > FAULT)
  uint8_t newReason = REASON_NONE;
  int newCh = -1;
  if (gasTripCh >= 0)       { newReason = REASON_GAS_TRIP;     newCh = gasTripCh; }
  else if (tempTripCh >= 0) { newReason = REASON_TEMP_TRIP;    newCh = tempTripCh; }
  else if (faultCh >= 0)    { newReason = REASON_SENSOR_FAULT; newCh = faultCh; }

  if (currentState == STATE_TRIP_LATCHED) {
    // Sudah latched: hanya bisa dieskalasi ke bahaya yang lebih tinggi
    if (newReason > latchedReason) {
      escalateTrip(newReason, newCh);
    }
    return;
  }

  if (newReason != REASON_NONE) {
    tripTo(newReason, newCh);
    return;
  }

  if (inWarmup) {
    if (ms - systemStartTime >= WARMUP_DURATION_MS) {
      currentState = STATE_NORMAL;
      Serial.println(F("\n>>> WARM-UP SELESAI. SISTEM SIAGA <<<"));
    }
    return;
  }

  // Logika WARNING
  if (gasWarnCh >= 0 || tempWarnCh >= 0) {
    uint8_t wt = (gasWarnCh >= 0) ? 1 : 2;
    int ch = (wt == 1) ? gasWarnCh : tempWarnCh;

    if (currentState != STATE_WARNING || wt != lastWarnType) {
      enqueueLog("WARNING", ch, (wt == 1) ? "GAS_WARN" : "TEMP_WARN", (int)filteredValues[ch]);
    }
    currentState = STATE_WARNING;
    lastWarnType = wt;
    lastHazardChannel = ch;
    lastHazardType = (wt == 1) ? "GAS WARN" : "SUHU WARN";
  }
  else if (currentState == STATE_WARNING) {
    if (allChannelsBelowWarn()) {
      currentState = STATE_NORMAL;
      lastWarnType = 0;
      lastHazardChannel = -1;
      lastHazardType = "NORMAL";
      enqueueLog("RESTORE_AMAN", -1, "NORMAL", 0);
    }
  }
}

// ===================================================================
// DISPLAY OLED (maks 21 karakter per baris)
// ===================================================================
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  char line[28];
  display.setCursor(0, 0);
  formatTime(line, sizeof(line), cachedNow);
  display.print(line);
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  switch (currentState) {
    case STATE_WARMUP: {
      unsigned long elapsed = millis() - systemStartTime;
      unsigned long remaining = (elapsed >= WARMUP_DURATION_MS) ? 0 : (WARMUP_DURATION_MS - elapsed) / 1000;
      display.setCursor(0, 15);
      display.print(F("STATUS: WARM-UP"));
      display.setCursor(0, 30);
      display.print(F("Stabilisasi sensor"));
      display.setCursor(0, 45);
      display.print(F("Sisa: ")); display.print(remaining); display.print(F("s"));
      break;
    }

    case STATE_NORMAL:
      display.setCursor(0, 15);
      display.print(F("STATUS: NORMAL"));
      display.setCursor(0, 30);
      display.print(F("Gas&Suhu: NORMAL"));
      display.setCursor(0, 45);
      display.print(F("SD: "));
      display.print(sdActive ? F("READY") : F("OFFLINE"));
      break;

    case STATE_WARNING:
      display.setCursor(0, 15);
      display.print(F("STATUS: WARNING!"));
      display.setCursor(0, 28);
      display.print(F("Ch:")); display.print(lastHazardChannel);
      display.print(F(" ")); display.print(lastHazardType);
      display.setCursor(0, 40);
      display.print(F("ADC: "));
      if (lastHazardChannel >= 0) display.print((int)filteredValues[lastHazardChannel]);
      display.setCursor(0, 52);
      display.print(F("FAN:ON PWR:ON GAS:ON"));
      break;

    case STATE_TRIP_LATCHED:
      display.setCursor(0, 15);
      display.print(F("STATUS: TRIP LATCHED"));
      display.setCursor(0, 27);
      display.print(F("Tipe: ")); display.print(lastHazardType);
      display.setCursor(0, 39);
      display.print(F("Ch:")); display.print(lastHazardChannel);
      if (lastHazardChannel >= 0) {
        display.print(F(" ADC:")); display.print((int)filteredValues[lastHazardChannel]);
      }
      display.setCursor(0, 52);
      if (millis() < resetRefusedUntil) {
        display.print(F("RESET DITOLAK!"));
      } else {
        display.print(F("TAHAN RESET 2 DETIK"));
      }
      break;
  }

  display.display();
}

// ===================================================================
// SERIAL MONITOR
// ===================================================================
void printSerialMonitoring() {
  Serial.println(F("--------------------------------------------------"));
  char timeBuff[28];
  formatTime(timeBuff, sizeof(timeBuff), cachedNow);
  Serial.print(F("Waktu: ")); Serial.println(timeBuff);

  for (int i = 0; i < 16; i++) {
    Serial.print(F("[Ch")); Serial.print(i); Serial.print(F(":"));
    if (!channelEnabled[i]) Serial.print(F("OFF"));
    else Serial.print((int)filteredValues[i]);
    Serial.print(F("] "));
    if ((i + 1) % 4 == 0) Serial.println();
  }

  Serial.print(F(">>> STATUS: "));
  switch (currentState) {
    case STATE_WARMUP:       Serial.println(F("WARM-UP <<<")); break;
    case STATE_NORMAL:       Serial.println(F("NORMAL <<<")); break;
    case STATE_WARNING:      Serial.print(F("WARNING (")); Serial.print(lastHazardType); Serial.println(F(") <<<")); break;
    case STATE_TRIP_LATCHED: Serial.print(F("TRIP LATCHED (")); Serial.print(lastHazardType); Serial.println(F(") <<<")); break;
  }
}

// ===================================================================
// SETUP
// ===================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println(F("=================================================="));
  Serial.println(F(" INDUSTRIAL SAFETY SYSTEM v3 (LATCH + NVS + WDT)  "));
  Serial.println(F("=================================================="));

  // 1. Baca status latch dari NVS SEBELUM menyentuh relay
  prefs.begin("safety", false);
  uint8_t savedReason = prefs.getUChar("reason", REASON_NONE);
  int savedCh = prefs.getChar("ch", -1);

  systemStartTime = millis();

  // 2. Setup pin output, langsung ke kondisi yang benar
  pinMode(RELAY_GAS, OUTPUT);
  pinMode(RELAY_LISTRIK, OUTPUT);
  pinMode(RELAY_FAN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  if (savedReason != REASON_NONE) {
    currentState = STATE_TRIP_LATCHED;
    latchedReason = savedReason;
    lastHazardChannel = savedCh;
    lastHazardType = reasonText(savedReason);
    Serial.print(F("[!] TRIP LATCH DIPULIHKAN DARI NVS: "));
    Serial.println(lastHazardType);
  } else {
    currentState = STATE_WARMUP;
  }
  applyActuators();

  // 3. Pin MUX & tombol
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(RESET_BTN_PIN, INPUT_PULLUP);
  analogReadResolution(12);

  // 4. I2C
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeOut(50);

  // 5. Pre-fill filter EMA
  for (int i = 0; i < 16; i++) {
    faultCount[i] = 0;
    filteredValues[i] = readMuxRaw(i);
  }

  // 6. OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("[ERROR] OLED SSD1306 tidak terhubung!"));
  } else {
    Serial.println(F("[OK] OLED SSD1306 siap."));
  }

  // 7. RTC
  if (!rtc.begin()) {
    Serial.println(F("[ERROR] RTC DS1307 tidak terdeteksi!"));
  } else {
    Serial.println(F("[OK] RTC DS1307 siap."));
    if (!rtc.isrunning()) {
      // Waktu compile. Sebaiknya set waktu manual / sinkron NTP.
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    cachedNow = rtc.now();
  }

  // 8. SD Card
  if (!SD.begin(SD_CS)) {
    Serial.println(F("[WARNING] MicroSD gagal / tidak ditemukan!"));
    sdActive = false;
  } else {
    Serial.println(F("[OK] MicroSD siap."));
    sdActive = true;
    if (!SD.exists("/riwayat.csv")) {
      File logFile = SD.open("/riwayat.csv", FILE_WRITE);
      if (logFile) {
        logFile.println("TIMESTAMP,STATUS,LOKASI_CH,TIPE_BAHAYA,NILAI_ADC");
        logFile.close();
      }
    }
  }

  if (savedReason != REASON_NONE) {
    enqueueLog("BOOT_LATCHED", savedCh, lastHazardType, 0);
  }

  // 9. Watchdog (diaktifkan terakhir, setelah init yang lambat selesai)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
    .idle_core_mask = 0,          // hanya awasi task loop kita
    .trigger_panic = true
  };
  esp_err_t wdtErr = esp_task_wdt_reconfigure(&wdt_config);
  if (wdtErr != ESP_OK) wdtErr = esp_task_wdt_init(&wdt_config);
#else
  esp_err_t wdtErr = esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
#endif
  esp_task_wdt_add(NULL);

  if (wdtErr == ESP_OK) Serial.println(F("[OK] Watchdog aktif."));
  else Serial.println(F("[WARNING] Watchdog gagal diinisialisasi!"));
}

// ===================================================================
// LOOP (NON-BLOCKING)
// ===================================================================
void loop() {
  esp_task_wdt_reset();
  unsigned long ms = millis();

  // Task 1: Scan sensor + state machine + aktuator (tiap 100 ms)
  // PRIORITAS TERTINGGI: aktuator dieksekusi sebelum logging/display.
  if (ms - lastScanTime >= 100) {
    lastScanTime = ms;
    scanAndFilterSensors();
    evaluateAndUpdateState(ms);
    applyActuators();
  }

  // Tombol reset (dicek tiap iterasi)
  handleResetButton(ms);

  // Task 2: Baca RTC (tiap 500 ms, bukan tiap iterasi)
  if (ms - lastRtcTime >= 500) {
    lastRtcTime = ms;
    cachedNow = rtc.now();
  }

  // Task 3: Buzzer intermiten saat WARNING (tiap 500 ms)
  if (currentState == STATE_WARNING) {
    if (ms - lastBuzzerTime >= 500) {
      lastBuzzerTime = ms;
      buzzerToggleState = !buzzerToggleState;
      digitalWrite(BUZZER_PIN, buzzerToggleState ? HIGH : LOW);
    }
  }

  // Task 4: Serial monitor (tiap 1 detik)
  if (ms - lastSerialTime >= 1000) {
    lastSerialTime = ms;
    printSerialMonitoring();
  }

  // Task 5: OLED (tiap 500 ms)
  if (ms - lastDisplayTime >= 500) {
    lastDisplayTime = ms;
    updateDisplay();
  }

  // Task 6: Tulis log SD (paling akhir, 1 entri per iterasi)
  flushLog();

  delay(1); // beri waktu ke task sistem
}
