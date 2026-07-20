#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266httpUpdate.h>
#include <HX711.h>
#include <WiFiClientSecure.h>
#include "version.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

// Linker HX711
// DOUT: D1 / GPIO5
// SCK : D2 / GPIO4
constexpr uint8_t HX711_LEFT_DOUT_PIN = D1;
constexpr uint8_t HX711_LEFT_SCK_PIN = D2;

// Rechter HX711
// DOUT: D5 / GPIO14
// SCK : D6 / GPIO12
constexpr uint8_t HX711_RIGHT_DOUT_PIN = D5;
constexpr uint8_t HX711_RIGHT_SCK_PIN = D6;

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr size_t EEPROM_SIZE = 512;

constexpr uint32_t CONFIG_MAGIC = 0x48583232UL;  // "HX22"
constexpr uint16_t CONFIG_VERSION = 2;

constexpr uint8_t DEFAULT_SAMPLES = 10;
constexpr uint8_t DEFAULT_STREAM_SAMPLES = 5;
constexpr uint32_t DEFAULT_STREAM_INTERVAL_MS = 500;

constexpr uint8_t MIN_SAMPLES = 1;
constexpr uint8_t MAX_SAMPLES = 100;
constexpr uint32_t MIN_STREAM_INTERVAL_MS = 50;
constexpr uint32_t MAX_STREAM_INTERVAL_MS = 60000;

constexpr size_t MAX_SERIAL_LINE = 160;
constexpr size_t MAX_CAL_POINTS = 16;
constexpr size_t MAX_POINT_NAME_LEN = 20;

constexpr uint32_t HX711_TIMEOUT_MS = 120;
constexpr float MIN_DETERMINANT = 1e6f;
constexpr float EPSILON = 1e-9f;

constexpr float WARNING_LARGE_JUMP_G = 80.0f;
constexpr float WARNING_OUTSIDE_RANGE_MARGIN = 0.35f;
constexpr float WARNING_TINY_SHARE_THRESHOLD = 0.02f;
constexpr float WARNING_BIG_RESIDUAL_G = 30.0f;
constexpr float MAX_PLAUSIBLE_ERROR_G = 2000.0f;

constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 12000;
constexpr char DEVICE_HOSTNAME[] = "Slotcar-Magnet-Scale";
constexpr char FALLBACK_AP_SSID[] = "Slotcar-Magnet-Scale";
constexpr char FALLBACK_AP_PASSWORD[] = "slotcar123";

struct StoredConfig {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;

  int32_t offsetLeft;
  int32_t offsetRight;
  float factorLeft;
  float factorRight;

  uint8_t hasTare;
  uint8_t hasCalibration;
  uint8_t wifiConfigured;
  uint8_t reserved3;

  char wifiSsid[33];
  char wifiPassword[65];

  uint32_t checksum;
};

struct CalibrationPoint {
  char name[MAX_POINT_NAME_LEN + 1];
  float netLeft;
  float netRight;
  float knownWeight;
};

struct Measurement {
  bool leftOk;
  bool rightOk;
  double rawLeft;
  double rawRight;
  double netLeft;
  double netRight;
};

struct ShareInfo {
  bool valid;
  float leftPercent;
  float rightPercent;
};

HX711 scaleLeft;
HX711 scaleRight;
ESP8266WebServer server(80);

int32_t offsetLeft = 0;
int32_t offsetRight = 0;
float factorLeft = 1.0f;
float factorRight = 1.0f;
bool hasTare = false;
bool hasCalibration = false;

CalibrationPoint calPoints[MAX_CAL_POINTS];
size_t calPointCount = 0;
bool calSessionActive = false;
float calReferenceWeight = 0.0f;

bool streamEnabled = false;
uint32_t streamIntervalMs = DEFAULT_STREAM_INTERVAL_MS;
uint8_t streamSamples = DEFAULT_STREAM_SAMPLES;
uint32_t lastStreamAtMs = 0;

char serialBuffer[MAX_SERIAL_LINE + 1];
size_t serialLen = 0;
bool serialOverflow = false;

bool hasLastWeight = false;
float lastWeight = 0.0f;

float calibratedMinLeftShare = 0.0f;
float calibratedMaxLeftShare = 0.0f;
bool hasCalibratedShareRange = false;

String storedWifiSsid;
String storedWifiPassword;
bool wifiConfigured = false;
bool stationConnected = false;
uint32_t lastWifiRetryMs = 0;

void handleCalibrationPage();

uint32_t fnv1a(const uint8_t *data, size_t len) {
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < len; ++i) {
    hash ^= data[i];
    hash *= 16777619UL;
  }
  return hash;
}

uint32_t calcChecksum(const StoredConfig &config) {
  const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&config);
  return fnv1a(ptr, offsetof(StoredConfig, checksum));
}

void printHxError(bool leftReady, bool rightReady) {
  if (!leftReady && !rightReady) {
    Serial.println(F("FEHLER: Beide HX711 antworten nicht."));
  } else if (!leftReady) {
    Serial.println(F("FEHLER: Linker HX711 antwortet nicht."));
  } else if (!rightReady) {
    Serial.println(F("FEHLER: Rechter HX711 antwortet nicht."));
  }
}

bool waitBothReady(uint32_t timeoutMs, bool &leftReady, bool &rightReady) {
  leftReady = scaleLeft.wait_ready_timeout(timeoutMs);
  rightReady = scaleRight.wait_ready_timeout(timeoutMs);
  return leftReady && rightReady;
}

bool readBothAverage(uint8_t samples, Measurement &result) {
  result = {};

  if (samples < MIN_SAMPLES || samples > MAX_SAMPLES) {
    Serial.printf("FEHLER: Ungueltige Anzahl an Messungen (%u). Erlaubt: %u..%u\n", samples,
                  MIN_SAMPLES, MAX_SAMPLES);
    return false;
  }

  int64_t sumLeft = 0;
  int64_t sumRight = 0;

  for (uint8_t i = 0; i < samples; ++i) {
    bool leftReady = false;
    bool rightReady = false;
    if (!waitBothReady(HX711_TIMEOUT_MS, leftReady, rightReady)) {
      printHxError(leftReady, rightReady);
      return false;
    }

    const long rawL = scaleLeft.read();
    const long rawR = scaleRight.read();

    sumLeft += static_cast<int64_t>(rawL);
    sumRight += static_cast<int64_t>(rawR);

    yield();
  }

  result.leftOk = true;
  result.rightOk = true;
  result.rawLeft = static_cast<double>(sumLeft) / static_cast<double>(samples);
  result.rawRight = static_cast<double>(sumRight) / static_cast<double>(samples);
  result.netLeft = result.rawLeft - static_cast<double>(offsetLeft);
  result.netRight = result.rawRight - static_cast<double>(offsetRight);
  return true;
}

bool parseFloatLoose(String text, float &value) {
  text.trim();
  text.replace(',', '.');

  if (text.length() == 0) {
    return false;
  }

  char *end = nullptr;
  value = strtof(text.c_str(), &end);
  return end != text.c_str() && *end == '\0' && isfinite(value);
}

bool parseUIntLoose(String text, uint32_t &value) {
  text.trim();
  if (text.length() == 0) {
    return false;
  }

  char *end = nullptr;
  const unsigned long parsed = strtoul(text.c_str(), &end, 10);
  if (end == text.c_str() || *end != '\0') {
    return false;
  }

  value = static_cast<uint32_t>(parsed);
  return true;
}

uint8_t parseSamples(const String &text, uint8_t defaultValue, bool &ok) {
  ok = true;
  if (text.length() == 0) {
    return defaultValue;
  }

  uint32_t parsed = 0;
  if (!parseUIntLoose(text, parsed) || parsed < MIN_SAMPLES || parsed > MAX_SAMPLES) {
    Serial.printf("FEHLER: Ungueltige Anzahl an Messungen. Erlaubt: %u..%u\n", MIN_SAMPLES, MAX_SAMPLES);
    ok = false;
    return 0;
  }

  return static_cast<uint8_t>(parsed);
}

void writeConfigToEeprom() {
  StoredConfig cfg{};
  cfg.magic = CONFIG_MAGIC;
  cfg.version = CONFIG_VERSION;
  cfg.offsetLeft = offsetLeft;
  cfg.offsetRight = offsetRight;
  cfg.factorLeft = factorLeft;
  cfg.factorRight = factorRight;
  cfg.hasTare = hasTare ? 1 : 0;
  cfg.hasCalibration = hasCalibration ? 1 : 0;
  cfg.wifiConfigured = wifiConfigured ? 1 : 0;

  storedWifiSsid.trim();
  if (storedWifiSsid.length() > 32) {
    storedWifiSsid = storedWifiSsid.substring(0, 32);
  }

  if (storedWifiPassword.length() > 64) {
    storedWifiPassword = storedWifiPassword.substring(0, 64);
  }

  strncpy(cfg.wifiSsid, storedWifiSsid.c_str(), sizeof(cfg.wifiSsid) - 1);
  strncpy(cfg.wifiPassword, storedWifiPassword.c_str(), sizeof(cfg.wifiPassword) - 1);

  cfg.checksum = calcChecksum(cfg);

  EEPROM.put(0, cfg);
  if (EEPROM.commit()) {
    Serial.println(F("OK: Einstellungen gespeichert."));
  } else {
    Serial.println(F("FEHLER: EEPROM commit fehlgeschlagen."));
  }
}

void loadDefaults() {
  offsetLeft = 0;
  offsetRight = 0;
  factorLeft = 1.0f;
  factorRight = 1.0f;
  hasTare = false;
  hasCalibration = false;
  hasCalibratedShareRange = false;
  storedWifiSsid = "";
  storedWifiPassword = "";
  wifiConfigured = false;
  stationConnected = false;
}

bool loadConfigFromEeprom() {
  StoredConfig cfg{};
  EEPROM.get(0, cfg);

  const bool valid =
      cfg.magic == CONFIG_MAGIC && cfg.version == CONFIG_VERSION &&
      cfg.checksum == calcChecksum(cfg) && isfinite(cfg.factorLeft) && isfinite(cfg.factorRight);

  if (!valid) {
    loadDefaults();
    return false;
  }

  offsetLeft = cfg.offsetLeft;
  offsetRight = cfg.offsetRight;
  factorLeft = cfg.factorLeft;
  factorRight = cfg.factorRight;
  hasTare = cfg.hasTare != 0;
  hasCalibration = cfg.hasCalibration != 0;
  wifiConfigured = cfg.wifiConfigured != 0;
  storedWifiSsid = String(cfg.wifiSsid);
  storedWifiPassword = String(cfg.wifiPassword);

  storedWifiSsid.trim();
  if (storedWifiSsid.length() == 0) {
    wifiConfigured = false;
  }

  return true;
}

bool connectToConfiguredWifi() {
  stationConnected = false;

  if (!wifiConfigured || storedWifiSsid.length() == 0) {
    Serial.println(F("INFO: Keine WLAN-Zugangsdaten gespeichert."));
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.hostname(DEVICE_HOSTNAME);
  WiFi.begin(storedWifiSsid.c_str(), storedWifiPassword.c_str());

  Serial.printf("WLAN: Verbinde mit \"%s\" ...\n", storedWifiSsid.c_str());

  const uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < WIFI_CONNECT_TIMEOUT_MS) {
    delay(200);
    Serial.print('.');
    yield();
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    stationConnected = true;
    Serial.printf("OK: WLAN verbunden. IP: %s\n", WiFi.localIP().toString().c_str());
    return true;
  }

  Serial.println(F("WARNUNG: WLAN-Verbindung fehlgeschlagen."));
  return false;
}

void ensureFallbackAccessPoint() {
  WiFi.mode(wifiConfigured ? WIFI_AP_STA : WIFI_AP);
  const bool apOk = WiFi.softAP(FALLBACK_AP_SSID, FALLBACK_AP_PASSWORD);

  if (!apOk) {
    Serial.println(F("FEHLER: AP konnte nicht gestartet werden."));
    return;
  }

  Serial.printf("OK: AP aktiv: %s\n", FALLBACK_AP_SSID);
  Serial.printf("AP Passwort: %s\n", FALLBACK_AP_PASSWORD);
  Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());

  if (wifiConfigured && storedWifiSsid.length() > 0) {
    WiFi.begin(storedWifiSsid.c_str(), storedWifiPassword.c_str());
    Serial.println(F("INFO: STA-Reconnect im Hintergrund aktiv."));
  }
}

void serviceWifi() {
  const wl_status_t status = WiFi.status();
  stationConnected = status == WL_CONNECTED;

  if (stationConnected) {
    return;
  }

  if (!wifiConfigured || storedWifiSsid.length() == 0) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastWifiRetryMs < WIFI_RETRY_INTERVAL_MS) {
    return;
  }

  lastWifiRetryMs = now;
  Serial.println(F("INFO: WLAN-Reconnect wird versucht..."));
  WiFi.begin(storedWifiSsid.c_str(), storedWifiPassword.c_str());
}

bool computeShare(float netLeft, float netRight, ShareInfo &share) {
  const float absL = fabsf(netLeft);
  const float absR = fabsf(netRight);
  const float denom = absL + absR;

  if (denom < EPSILON) {
    share = {false, 0.0f, 0.0f};
    return false;
  }

  share.valid = true;
  share.leftPercent = 100.0f * (absL / denom);
  share.rightPercent = 100.0f * (absR / denom);
  return true;
}

bool computeWeightFromNet(float netLeft, float netRight, float &weight) {
  const double value = static_cast<double>(factorLeft) * static_cast<double>(netLeft) +
                       static_cast<double>(factorRight) * static_cast<double>(netRight);
  if (!isfinite(value)) {
    return false;
  }
  weight = static_cast<float>(value);
  return isfinite(weight);
}

void printHelp() {
  Serial.println();
  Serial.println(F("===== BEFEHLE ====="));
  Serial.println(F("HILFE"));
  Serial.println(F("STATUS"));
  Serial.println(F("TARA [messungen]"));
  Serial.println(F("KAL_START <gewicht_g>"));
  Serial.println(F("KAL_PUNKT <name> [messungen]"));
  Serial.println(F("KAL_LISTE"));
  Serial.println(F("KAL_LOESCHEN"));
  Serial.println(F("KAL_BERECHNEN"));
  Serial.println(F("KAL_ABBRUCH"));
  Serial.println(F("MESSEN [messungen]"));
  Serial.println(F("ROH [messungen]"));
  Serial.println(F("START [intervall_ms] [messungen]"));
  Serial.println(F("STOP"));
  Serial.println(F("INTERVALL <ms>"));
  Serial.println(F("FAKTOREN <faktor_links> <faktor_rechts>"));
  Serial.println(F("WLAN <ssid> <passwort>"));
  Serial.println(F("WLAN_LOESCHEN"));
  Serial.println(F("SPEICHERN"));
  Serial.println(F("RESET"));
  Serial.println(F("==================="));
  Serial.println();
}

void printStatus() {
  Serial.println();
  Serial.println(F("===== STATUS ====="));
  Serial.printf("LINKS  DOUT/SCK: D5(GPIO14)/D6(GPIO12) | bereit: %s\n", scaleLeft.is_ready() ? "ja" : "nein");
  Serial.printf("RECHTS DOUT/SCK: D1(GPIO5)/D2(GPIO4)   | bereit: %s\n", scaleRight.is_ready() ? "ja" : "nein");
  Serial.printf("Tara vorhanden: %s\n", hasTare ? "ja" : "nein");
  Serial.printf("Kalibrierung vorhanden: %s\n", hasCalibration ? "ja" : "nein");
  Serial.printf("Offset links : %ld\n", static_cast<long>(offsetLeft));
  Serial.printf("Offset rechts: %ld\n", static_cast<long>(offsetRight));
  Serial.printf("Faktor links : %.9f\n", factorLeft);
  Serial.printf("Faktor rechts: %.9f\n", factorRight);
  Serial.printf("WLAN konfiguriert: %s\n", wifiConfigured ? "ja" : "nein");
  if (wifiConfigured) {
    Serial.printf("WLAN SSID: %s\n", storedWifiSsid.c_str());
  }

  const WiFiMode_t mode = WiFi.getMode();
  const char *modeText = "OFF";
  if (mode == WIFI_STA) {
    modeText = "STA";
  } else if (mode == WIFI_AP) {
    modeText = "AP";
  } else if (mode == WIFI_AP_STA) {
    modeText = "AP+STA";
  }
  Serial.printf("WLAN Modus: %s\n", modeText);
  Serial.printf("WLAN verbunden: %s\n", stationConnected ? "ja" : "nein");
  if (stationConnected) {
    Serial.printf("STA IP: %s\n", WiFi.localIP().toString().c_str());
  }
  if (mode == WIFI_AP || mode == WIFI_AP_STA) {
    Serial.printf("AP SSID: %s\n", FALLBACK_AP_SSID);
    Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
  }

  Serial.printf("Kalibriermodus: %s\n", calSessionActive ? "aktiv" : "inaktiv");
  if (calSessionActive) {
    Serial.printf("Refgewicht: %.2f g | Punkte: %u/%u\n", calReferenceWeight,
                  static_cast<unsigned>(calPointCount), static_cast<unsigned>(MAX_CAL_POINTS));
  }
  Serial.printf("Stream: %s | Intervall: %lu ms | Messungen: %u\n", streamEnabled ? "AN" : "AUS",
                static_cast<unsigned long>(streamIntervalMs), streamSamples);
  Serial.println(F("=================="));
  Serial.println();
}

void warnForMeasurement(const Measurement &m, float weight) {
  if (weight < -1.0f) {
    Serial.println(F("WARNUNG: Gesamtgewicht ist negativ."));
  }

  const float netL = static_cast<float>(m.netLeft);
  const float netR = static_cast<float>(m.netRight);

  const float absL = fabsf(netL);
  const float absR = fabsf(netR);
  const float sumAbs = absL + absR;

  if (sumAbs > EPSILON) {
    const float shareL = absL / sumAbs;
    const float shareR = absR / sumAbs;
    if (shareL < WARNING_TINY_SHARE_THRESHOLD) {
      Serial.println(F("WARNUNG: Linke Zelle liefert fast keinen Signalanteil."));
    }
    if (shareR < WARNING_TINY_SHARE_THRESHOLD) {
      Serial.println(F("WARNUNG: Rechte Zelle liefert fast keinen Signalanteil."));
    }
  }

  if ((netL > 0.0f && netR < 0.0f) || (netL < 0.0f && netR > 0.0f)) {
    Serial.println(F("WARNUNG: Sensoren liefern stark gegenlaeufige Vorzeichen."));
  }

  if (fabs(m.rawLeft) > 8300000.0 || fabs(m.rawRight) > 8300000.0) {
    Serial.println(F("WARNUNG: Rohwert moeglicherweise im Saettigungsbereich."));
  }

  if (hasLastWeight) {
    const float delta = fabsf(weight - lastWeight);
    if (delta > WARNING_LARGE_JUMP_G) {
      Serial.printf("WARNUNG: Ungewoehnlich grosse Differenz zur letzten Messung: %.2f g\n", delta);
    }
  }

  if (hasCalibratedShareRange) {
    ShareInfo share{};
    if (computeShare(netL, netR, share)) {
      const float minShare = calibratedMinLeftShare - WARNING_OUTSIDE_RANGE_MARGIN * 100.0f;
      const float maxShare = calibratedMaxLeftShare + WARNING_OUTSIDE_RANGE_MARGIN * 100.0f;
      if (share.leftPercent < minShare || share.leftPercent > maxShare) {
        Serial.println(F("WARNUNG: Lastverteilung liegt deutlich ausserhalb des kalibrierten Bereichs."));
      }
    }
  }
}

bool measureCombined(uint8_t samples, Measurement &m, float &weight, bool verboseErrors) {
  if (!hasTare) {
    if (verboseErrors) {
      Serial.println(F("FEHLER: Keine Tara vorhanden. Zuerst TARA ausfuehren."));
    }
    return false;
  }

  if (!hasCalibration) {
    if (verboseErrors) {
      Serial.println(F("FEHLER: Keine gueltige Kalibrierung vorhanden."));
    }
    return false;
  }

  if (!readBothAverage(samples, m)) {
    return false;
  }

  if (!computeWeightFromNet(static_cast<float>(m.netLeft), static_cast<float>(m.netRight), weight)) {
    if (verboseErrors) {
      Serial.println(F("FEHLER: Gewicht konnte nicht berechnet werden."));
    }
    return false;
  }

  warnForMeasurement(m, weight);

  hasLastWeight = true;
  lastWeight = weight;
  return true;
}

bool doTare(uint8_t samples) {
  streamEnabled = false;

  Measurement m{};
  if (!readBothAverage(samples, m)) {
    return false;
  }

  offsetLeft = static_cast<int32_t>(lround(m.rawLeft));
  offsetRight = static_cast<int32_t>(lround(m.rawRight));
  hasTare = true;

  Serial.printf("OK: Tara gesetzt. Offset links=%ld, rechts=%ld\n", static_cast<long>(offsetLeft),
                static_cast<long>(offsetRight));
  return true;
}

void clearTempCalibrationPoints() {
  calPointCount = 0;
  for (size_t i = 0; i < MAX_CAL_POINTS; ++i) {
    calPoints[i].name[0] = '\0';
    calPoints[i].netLeft = 0.0f;
    calPoints[i].netRight = 0.0f;
    calPoints[i].knownWeight = 0.0f;
  }
}

void startCalibrationSession(float knownWeight) {
  calSessionActive = true;
  calReferenceWeight = knownWeight;
  clearTempCalibrationPoints();
  streamEnabled = false;

  Serial.printf("OK: Kalibriermodus gestartet. Referenzgewicht: %.2f g\n", calReferenceWeight);
  Serial.println(F("Hinweis: Gewicht an mehreren Positionen auf derselben Platte platzieren."));
}

bool copyPointName(const String &name, char *dst, size_t dstSize) {
  if (name.length() == 0 || name.length() > MAX_POINT_NAME_LEN) {
    return false;
  }

  for (size_t i = 0; i < name.length(); ++i) {
    const char c = name[i];
    if (c < 32 || c > 126) {
      return false;
    }
  }

  strncpy(dst, name.c_str(), dstSize - 1);
  dst[dstSize - 1] = '\0';
  return true;
}

bool addCalibrationPoint(const String &name, uint8_t samples) {
  if (!calSessionActive) {
    Serial.println(F("FEHLER: Kein Kalibriermodus aktiv. Zuerst KAL_START ausfuehren."));
    return false;
  }

  if (!hasTare) {
    Serial.println(F("FEHLER: Keine Tara vorhanden. Zuerst TARA ausfuehren."));
    return false;
  }

  if (calPointCount >= MAX_CAL_POINTS) {
    Serial.printf("FEHLER: Maximal %u Kalibrierpunkte erreicht.\n", static_cast<unsigned>(MAX_CAL_POINTS));
    return false;
  }

  CalibrationPoint point{};
  if (!copyPointName(name, point.name, sizeof(point.name))) {
    Serial.printf("FEHLER: Ungueltiger Name. 1..%u ASCII-Zeichen erlaubt.\n",
                  static_cast<unsigned>(MAX_POINT_NAME_LEN));
    return false;
  }

  Measurement m{};
  if (!readBothAverage(samples, m)) {
    return false;
  }

  point.netLeft = static_cast<float>(m.netLeft);
  point.netRight = static_cast<float>(m.netRight);
  point.knownWeight = calReferenceWeight;

  calPoints[calPointCount++] = point;

  Serial.printf("OK: Kalibrierpunkt gespeichert: %s | links=%.0f | rechts=%.0f | soll=%.2f g\n",
                point.name, point.netLeft, point.netRight, point.knownWeight);
  return true;
}

void listCalibrationPoints() {
  if (!calSessionActive && calPointCount == 0) {
    Serial.println(F("INFO: Keine temporaeren Kalibrierpunkte vorhanden."));
    return;
  }

  Serial.println(F("Temporare Kalibrierpunkte:"));
  for (size_t i = 0; i < calPointCount; ++i) {
    Serial.printf("%u) %s | NET_L=%.0f | NET_R=%.0f | SOLL=%.2f g\n", static_cast<unsigned>(i + 1),
                  calPoints[i].name, calPoints[i].netLeft, calPoints[i].netRight,
                  calPoints[i].knownWeight);
  }

  if (calPointCount == 0) {
    Serial.println(F("(leer)"));
  }
}

void clearCalibrationPoints() {
  clearTempCalibrationPoints();
  Serial.println(F("OK: Temporare Kalibrierpunkte geloescht."));
}

void abortCalibrationSession() {
  clearTempCalibrationPoints();
  calSessionActive = false;
  calReferenceWeight = 0.0f;
  Serial.println(F("OK: Kalibriermodus beendet. Temporare Punkte verworfen."));
}

bool calculateLeastSquaresFactors(float &newFactorLeft, float &newFactorRight, float &det) {
  if (calPointCount < 2) {
    Serial.println(F("FEHLER: Zu wenige Kalibrierpunkte. Mindestens 2 erforderlich."));
    return false;
  }

  double sLL = 0.0;
  double sRR = 0.0;
  double sLR = 0.0;
  double sLW = 0.0;
  double sRW = 0.0;

  // Lineare Least-Squares-Berechnung ueber beide Sensoren gemeinsam.
  for (size_t i = 0; i < calPointCount; ++i) {
    const double l = static_cast<double>(calPoints[i].netLeft);
    const double r = static_cast<double>(calPoints[i].netRight);
    const double w = static_cast<double>(calPoints[i].knownWeight);

    sLL += l * l;
    sRR += r * r;
    sLR += l * r;
    sLW += l * w;
    sRW += r * w;
  }

  const double detDouble = sLL * sRR - sLR * sLR;
  det = static_cast<float>(detDouble);

  if (!isfinite(detDouble) || fabs(detDouble) < MIN_DETERMINANT) {
    Serial.println(F("FEHLER: Die Lastverteilung der Kalibrierpunkte ist zu aehnlich."));
    Serial.println(F("Das Referenzgewicht weiter links beziehungsweise rechts positionieren."));
    return false;
  }

  const double left = (sLW * sRR - sRW * sLR) / detDouble;
  const double right = (sRW * sLL - sLW * sLR) / detDouble;

  if (!isfinite(left) || !isfinite(right)) {
    Serial.println(F("FEHLER: Ungueltige oder unendliche Faktoren berechnet."));
    return false;
  }

  newFactorLeft = static_cast<float>(left);
  newFactorRight = static_cast<float>(right);

  if (!isfinite(newFactorLeft) || !isfinite(newFactorRight)) {
    Serial.println(F("FEHLER: Ungueltige oder unendliche Faktoren berechnet."));
    return false;
  }

  return true;
}

bool analyzeCalibrationFit(float testFactorLeft, float testFactorRight, float &avgAbsErr, float &maxAbsErr,
                           float &minLeftShare, float &maxLeftShare) {
  avgAbsErr = 0.0f;
  maxAbsErr = 0.0f;
  minLeftShare = 100.0f;
  maxLeftShare = 0.0f;

  if (calPointCount == 0) {
    return false;
  }

  Serial.println();
  Serial.println(F("Kalibrierpunkt-Auswertung:"));

  for (size_t i = 0; i < calPointCount; ++i) {
    const CalibrationPoint &p = calPoints[i];

    const double calc = static_cast<double>(testFactorLeft) * static_cast<double>(p.netLeft) +
                        static_cast<double>(testFactorRight) * static_cast<double>(p.netRight);
    if (!isfinite(calc)) {
      return false;
    }

    const float calcWeight = static_cast<float>(calc);
    const float dev = calcWeight - p.knownWeight;
    const float absDev = fabsf(dev);

    avgAbsErr += absDev;
    if (absDev > maxAbsErr) {
      maxAbsErr = absDev;
    }

    ShareInfo share{};
    if (computeShare(p.netLeft, p.netRight, share)) {
      if (share.leftPercent < minLeftShare) {
        minLeftShare = share.leftPercent;
      }
      if (share.leftPercent > maxLeftShare) {
        maxLeftShare = share.leftPercent;
      }
    }

    Serial.printf("Position: %s\n", p.name);
    Serial.printf("Soll: %.2f g\n", p.knownWeight);
    Serial.printf("Berechnet: %.2f g\n", calcWeight);
    Serial.printf("Abweichung: %.2f g\n", dev);
  }

  avgAbsErr /= static_cast<float>(calPointCount);
  return true;
}

bool calculateCalibration() {
  float newLeft = 0.0f;
  float newRight = 0.0f;
  float det = 0.0f;

  if (!calculateLeastSquaresFactors(newLeft, newRight, det)) {
    return false;
  }

  float avgAbsErr = 0.0f;
  float maxAbsErr = 0.0f;
  float minLeftShare = 0.0f;
  float maxLeftShare = 0.0f;

  if (!analyzeCalibrationFit(newLeft, newRight, avgAbsErr, maxAbsErr, minLeftShare, maxLeftShare)) {
    Serial.println(F("FEHLER: Fehleranalyse der Kalibrierung fehlgeschlagen."));
    return false;
  }

  Serial.println();
  Serial.printf("Faktor links: %.9f\n", newLeft);
  Serial.printf("Faktor rechts: %.9f\n", newRight);
  Serial.printf("Determinante: %.3f\n", det);
  Serial.printf("Durchschnittlicher absoluter Fehler: %.3f g\n", avgAbsErr);
  Serial.printf("Groesster absoluter Fehler: %.3f g\n", maxAbsErr);
  Serial.printf("Anzahl Kalibrierpunkte: %u\n", static_cast<unsigned>(calPointCount));
  Serial.printf("Lastverteilung links min/max: %.2f %% / %.2f %%\n", minLeftShare, maxLeftShare);

  if (!isfinite(newLeft) || !isfinite(newRight)) {
    Serial.println(F("FEHLER: Ungueltige oder unendliche Faktoren."));
    return false;
  }

  if (maxAbsErr > MAX_PLAUSIBLE_ERROR_G) {
    Serial.println(F("FEHLER: Groesster Fehler ist offensichtlich unplausibel. Kalibrierung verworfen."));
    return false;
  }

  if (maxAbsErr > WARNING_BIG_RESIDUAL_G) {
    Serial.println(F("WARNUNG: Hoher Kalibrierfehler. Mechanik, Ausrichtung und Referenzgewicht pruefen."));
  }

  factorLeft = newLeft;
  factorRight = newRight;
  hasCalibration = true;

  hasCalibratedShareRange = true;
  calibratedMinLeftShare = minLeftShare;
  calibratedMaxLeftShare = maxLeftShare;

  writeConfigToEeprom();
  Serial.println(F("OK: Neue Kalibrierfaktoren aktiviert und gespeichert."));

  calSessionActive = false;
  clearTempCalibrationPoints();
  return true;
}

void printRawAndNet(uint8_t samples) {
  Measurement m{};
  if (!readBothAverage(samples, m)) {
    return;
  }

  Serial.printf("LINKS_ROH: %.0f\n", m.rawLeft);
  Serial.printf("RECHTS_ROH: %.0f\n", m.rawRight);
  Serial.printf("LINKS_NETTO: %.0f\n", m.netLeft);
  Serial.printf("RECHTS_NETTO: %.0f\n", m.netRight);
}

void printMeasurement(uint8_t samples) {
  Measurement m{};
  float weight = 0.0f;
  if (!measureCombined(samples, m, weight, true)) {
    return;
  }

  Serial.printf("LINKS_ROH: %.0f\n", m.rawLeft);
  Serial.printf("RECHTS_ROH: %.0f\n", m.rawRight);
  Serial.printf("LINKS_NETTO: %.0f\n", m.netLeft);
  Serial.printf("RECHTS_NETTO: %.0f\n", m.netRight);

  ShareInfo share{};
  if (computeShare(static_cast<float>(m.netLeft), static_cast<float>(m.netRight), share)) {
    Serial.printf("LINKS_ANTEIL: %.2f %%\n", share.leftPercent);
    Serial.printf("RECHTS_ANTEIL: %.2f %%\n", share.rightPercent);
  }

  Serial.printf("GEWICHT: %.2f g\n", weight);
}

void printStreamLine(uint8_t samples) {
  Measurement m{};
  float weight = 0.0f;
  if (!measureCombined(samples, m, weight, false)) {
    return;
  }

  ShareInfo share{};
  if (computeShare(static_cast<float>(m.netLeft), static_cast<float>(m.netRight), share)) {
    Serial.printf("GEWICHT: %.2f g | LINKS: %.1f %% | RECHTS: %.1f %%\n", weight, share.leftPercent,
                  share.rightPercent);
  } else {
    Serial.printf("GEWICHT: %.2f g\n", weight);
  }
}

void doReset() {
  streamEnabled = false;
  loadDefaults();
  calSessionActive = false;
  clearTempCalibrationPoints();
  writeConfigToEeprom();
  Serial.println(F("OK: Tara und Kalibrierung zurueckgesetzt."));
}

void splitFirstToken(const String &text, String &first, String &rest) {
  String s = text;
  s.trim();

  if (s.length() == 0) {
    first = "";
    rest = "";
    return;
  }

  const int idx = s.indexOf(' ');
  if (idx < 0) {
    first = s;
    rest = "";
    return;
  }

  first = s.substring(0, idx);
  rest = s.substring(idx + 1);
  rest.trim();
}

bool parseNameAndOptionalSamples(const String &args, String &name, uint8_t &samples) {
  String first;
  String rest;
  splitFirstToken(args, first, rest);

  if (first.length() == 0) {
    return false;
  }

  name = first;

  bool ok = true;
  samples = parseSamples(rest, DEFAULT_SAMPLES, ok);
  return ok;
}

String jsonEscape(const String &text) {
  String escaped;
  escaped.reserve(text.length() + 8);

  for (size_t i = 0; i < text.length(); ++i) {
    const char c = text[i];
    if (c == '"' || c == '\\') {
      escaped += '\\';
      escaped += c;
    } else if (c == '\n') {
      escaped += "\\n";
    } else if (c == '\r') {
      escaped += "\\r";
    } else {
      escaped += c;
    }
  }

  return escaped;
}

void sendJson(int statusCode, const String &body) {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.send(statusCode, "application/json", body);
}

bool parseHttpSamples(const String &argName, uint8_t defaultValue, uint8_t &samples, String &error) {
  if (!server.hasArg(argName)) {
    samples = defaultValue;
    return true;
  }

  uint32_t parsed = 0;
  if (!parseUIntLoose(server.arg(argName), parsed) || parsed < MIN_SAMPLES || parsed > MAX_SAMPLES) {
    error = "samples muss zwischen 1 und 100 liegen";
    return false;
  }

  samples = static_cast<uint8_t>(parsed);
  return true;
}

const char *wifiModeText() {
  const WiFiMode_t mode = WiFi.getMode();
  if (mode == WIFI_STA) {
    return "STA";
  }
  if (mode == WIFI_AP) {
    return "AP";
  }
  if (mode == WIFI_AP_STA) {
    return "AP+STA";
  }
  return "OFF";
}

void handleApiStatus() {
  String json = "{";
  json += "\"ok\":true";
  json += ",\"hasTare\":" + String(hasTare ? "true" : "false");
  json += ",\"hasCalibration\":" + String(hasCalibration ? "true" : "false");
  json += ",\"offsetLeft\":" + String(offsetLeft);
  json += ",\"offsetRight\":" + String(offsetRight);
  json += ",\"factorLeft\":" + String(factorLeft, 9);
  json += ",\"factorRight\":" + String(factorRight, 9);
  json += ",\"calSessionActive\":" + String(calSessionActive ? "true" : "false");
  json += ",\"calReferenceWeight\":" + String(calReferenceWeight, 2);
  json += ",\"calPointCount\":" + String(static_cast<unsigned>(calPointCount));
  json += ",\"streamEnabled\":" + String(streamEnabled ? "true" : "false");
  json += ",\"streamIntervalMs\":" + String(streamIntervalMs);
  json += ",\"streamSamples\":" + String(streamSamples);
  json += ",\"wifiConfigured\":" + String(wifiConfigured ? "true" : "false");
  json += ",\"wifiSsid\":\"" + jsonEscape(storedWifiSsid) + "\"";
  json += ",\"stationConnected\":" + String(stationConnected ? "true" : "false");
  json += ",\"wifiMode\":\"" + String(wifiModeText()) + "\"";
  json += ",\"staIp\":\"" + jsonEscape(stationConnected ? WiFi.localIP().toString() : "") + "\"";
  json += ",\"apSsid\":\"" + String(FALLBACK_AP_SSID) + "\"";
  json += ",\"apIp\":\"" + jsonEscape(WiFi.softAPIP().toString()) + "\"";
  json += "}";
  sendJson(200, json);
}

void handleApiRead() {
  String error;
  uint8_t samples = DEFAULT_SAMPLES;
  if (!parseHttpSamples("samples", DEFAULT_SAMPLES, samples, error)) {
    sendJson(400, "{\"ok\":false,\"error\":\"" + jsonEscape(error) + "\"}");
    return;
  }

  Measurement m{};
  float weight = 0.0f;
  if (!measureCombined(samples, m, weight, false)) {
    sendJson(400, "{\"ok\":false,\"error\":\"Messung fehlgeschlagen oder ungueltiger Zustand\"}");
    return;
  }

  String json = "{";
  json += "\"ok\":true";
  json += ",\"rawLeft\":" + String(m.rawLeft, 0);
  json += ",\"rawRight\":" + String(m.rawRight, 0);
  json += ",\"netLeft\":" + String(m.netLeft, 0);
  json += ",\"netRight\":" + String(m.netRight, 0);
  ShareInfo share{};
  if (computeShare(static_cast<float>(m.netLeft), static_cast<float>(m.netRight), share)) {
    json += ",\"leftPercent\":" + String(share.leftPercent, 2);
    json += ",\"rightPercent\":" + String(share.rightPercent, 2);
  }
  json += ",\"weight\":" + String(weight, 2);
  json += "}";
  sendJson(200, json);
}

void handleApiRaw() {
  String error;
  uint8_t samples = DEFAULT_SAMPLES;
  if (!parseHttpSamples("samples", DEFAULT_SAMPLES, samples, error)) {
    sendJson(400, "{\"ok\":false,\"error\":\"" + jsonEscape(error) + "\"}");
    return;
  }

  Measurement m{};
  if (!readBothAverage(samples, m)) {
    sendJson(400, "{\"ok\":false,\"error\":\"Rohmessung fehlgeschlagen\"}");
    return;
  }

  sendJson(200, "{\"ok\":true,\"rawLeft\":" + String(m.rawLeft, 0) +
                    ",\"rawRight\":" + String(m.rawRight, 0) + ",\"netLeft\":" +
                    String(m.netLeft, 0) + ",\"netRight\":" + String(m.netRight, 0) + "}");
}

void handleApiTare() {
  String error;
  uint8_t samples = DEFAULT_SAMPLES;
  if (!parseHttpSamples("samples", DEFAULT_SAMPLES, samples, error)) {
    sendJson(400, "{\"ok\":false,\"error\":\"" + jsonEscape(error) + "\"}");
    return;
  }

  if (!doTare(samples)) {
    sendJson(400, "{\"ok\":false,\"error\":\"Tara fehlgeschlagen\"}");
    return;
  }

  sendJson(200, "{\"ok\":true,\"offsetLeft\":" + String(offsetLeft) +
                    ",\"offsetRight\":" + String(offsetRight) + "}");
}

void handleApiCalStart() {
  if (!server.hasArg("weight")) {
    sendJson(400, "{\"ok\":false,\"error\":\"weight fehlt\"}");
    return;
  }

  float known = 0.0f;
  if (!parseFloatLoose(server.arg("weight"), known) || known <= 0.0f) {
    sendJson(400, "{\"ok\":false,\"error\":\"weight muss > 0 sein\"}");
    return;
  }

  startCalibrationSession(known);
  sendJson(200, "{\"ok\":true}");
}

void handleApiCalPoint() {
  if (!server.hasArg("name")) {
    sendJson(400, "{\"ok\":false,\"error\":\"name fehlt\"}");
    return;
  }

  String error;
  uint8_t samples = DEFAULT_SAMPLES;
  if (!parseHttpSamples("samples", DEFAULT_SAMPLES, samples, error)) {
    sendJson(400, "{\"ok\":false,\"error\":\"" + jsonEscape(error) + "\"}");
    return;
  }

  if (!addCalibrationPoint(server.arg("name"), samples)) {
    sendJson(400, "{\"ok\":false,\"error\":\"Kalibrierpunkt konnte nicht gespeichert werden\"}");
    return;
  }

  sendJson(200, "{\"ok\":true,\"count\":" + String(static_cast<unsigned>(calPointCount)) + "}");
}

void handleApiCalList() {
  String json = "{\"ok\":true,\"points\":[";
  for (size_t i = 0; i < calPointCount; ++i) {
    if (i > 0) {
      json += ",";
    }
    json += "{";
    json += "\"name\":\"" + jsonEscape(String(calPoints[i].name)) + "\"";
    json += ",\"netLeft\":" + String(calPoints[i].netLeft, 0);
    json += ",\"netRight\":" + String(calPoints[i].netRight, 0);
    json += ",\"knownWeight\":" + String(calPoints[i].knownWeight, 2);
    json += "}";
  }
  json += "]}";
  sendJson(200, json);
}

void handleApiCalReport() {
  String json = "{\"ok\":true";
  json += ",\"factorLeft\":" + String(factorLeft, 9);
  json += ",\"factorRight\":" + String(factorRight, 9);
  json += ",\"hasCalibration\":" + String(hasCalibration ? "true" : "false");
  json += ",\"calSessionActive\":" + String(calSessionActive ? "true" : "false");
  json += ",\"points\":[";

  float sumAbsDev = 0.0f;
  float maxAbsDev = 0.0f;

  for (size_t i = 0; i < calPointCount; ++i) {
    if (i > 0) {
      json += ",";
    }

    const CalibrationPoint &p = calPoints[i];
    const float calcWeight = factorLeft * p.netLeft + factorRight * p.netRight;
    const float deviation = calcWeight - p.knownWeight;
    const float absDev = fabsf(deviation);
    sumAbsDev += absDev;
    if (absDev > maxAbsDev) {
      maxAbsDev = absDev;
    }

    json += "{";
    json += "\"name\":\"" + jsonEscape(String(p.name)) + "\"";
    json += ",\"netLeft\":" + String(p.netLeft, 0);
    json += ",\"netRight\":" + String(p.netRight, 0);
    json += ",\"knownWeight\":" + String(p.knownWeight, 2);
    json += ",\"calcWeight\":" + String(calcWeight, 2);
    json += ",\"deviation\":" + String(deviation, 2);
    ShareInfo share{};
    if (computeShare(p.netLeft, p.netRight, share)) {
      json += ",\"leftPercent\":" + String(share.leftPercent, 2);
      json += ",\"rightPercent\":" + String(share.rightPercent, 2);
    }
    json += "}";
  }

  json += "]";
  json += ",\"count\":" + String(static_cast<unsigned>(calPointCount));
  if (calPointCount > 0) {
    json += ",\"avgAbsDeviation\":" + String(sumAbsDev / static_cast<float>(calPointCount), 3);
    json += ",\"maxAbsDeviation\":" + String(maxAbsDev, 3);
  } else {
    json += ",\"avgAbsDeviation\":0.0";
    json += ",\"maxAbsDeviation\":0.0";
  }
  json += "}";

  sendJson(200, json);
}

void handleApiCalClear() {
  clearCalibrationPoints();
  sendJson(200, "{\"ok\":true}");
}

void handleApiCalAbort() {
  abortCalibrationSession();
  sendJson(200, "{\"ok\":true}");
}

void handleApiCalCalculate() {
  if (!calculateCalibration()) {
    sendJson(400, "{\"ok\":false,\"error\":\"Kalibrierberechnung fehlgeschlagen\"}");
    return;
  }
  sendJson(200, "{\"ok\":true,\"factorLeft\":" + String(factorLeft, 9) +
                    ",\"factorRight\":" + String(factorRight, 9) + "}");
}

void handleApiStreamStart() {
  uint32_t interval = streamIntervalMs;
  if (server.hasArg("interval")) {
    if (!parseUIntLoose(server.arg("interval"), interval) || interval < MIN_STREAM_INTERVAL_MS ||
        interval > MAX_STREAM_INTERVAL_MS) {
      sendJson(400, "{\"ok\":false,\"error\":\"interval ungueltig\"}");
      return;
    }
  }

  String error;
  uint8_t samples = streamSamples;
  if (!parseHttpSamples("samples", streamSamples, samples, error)) {
    sendJson(400, "{\"ok\":false,\"error\":\"" + jsonEscape(error) + "\"}");
    return;
  }

  if (!hasTare || !hasCalibration) {
    sendJson(400, "{\"ok\":false,\"error\":\"Vorher TARA und Kalibrierung erforderlich\"}");
    return;
  }

  streamIntervalMs = interval;
  streamSamples = samples;
  streamEnabled = true;
  lastStreamAtMs = millis() - streamIntervalMs;
  sendJson(200, "{\"ok\":true}");
}

void handleApiStreamStop() {
  streamEnabled = false;
  sendJson(200, "{\"ok\":true}");
}

void handleApiSetFactors() {
  if (!server.hasArg("left") || !server.hasArg("right")) {
    sendJson(400, "{\"ok\":false,\"error\":\"left/right fehlen\"}");
    return;
  }

  float left = 0.0f;
  float right = 0.0f;
  if (!parseFloatLoose(server.arg("left"), left) || !parseFloatLoose(server.arg("right"), right)) {
    sendJson(400, "{\"ok\":false,\"error\":\"Faktoren ungueltig\"}");
    return;
  }

  factorLeft = left;
  factorRight = right;
  hasCalibration = true;
  writeConfigToEeprom();
  sendJson(200, "{\"ok\":true}");
}

void handleApiWifiSave() {
  if (!server.hasArg("ssid")) {
    sendJson(400, "{\"ok\":false,\"error\":\"ssid fehlt\"}");
    return;
  }

  String ssid = server.arg("ssid");
  String password = server.hasArg("password") ? server.arg("password") : "";
  ssid.trim();

  if (ssid.length() == 0 || ssid.length() > 32) {
    sendJson(400, "{\"ok\":false,\"error\":\"ssid muss 1..32 Zeichen sein\"}");
    return;
  }

  if (password.length() > 64) {
    sendJson(400, "{\"ok\":false,\"error\":\"password darf max 64 Zeichen haben\"}");
    return;
  }

  storedWifiSsid = ssid;
  storedWifiPassword = password;
  wifiConfigured = true;
  writeConfigToEeprom();

  const bool connected = connectToConfiguredWifi();
  if (!connected) {
    ensureFallbackAccessPoint();
  }

  sendJson(200, "{\"ok\":true,\"connected\":" + String(stationConnected ? "true" : "false") + "}");
}

void handleApiWifiClear() {
  storedWifiSsid = "";
  storedWifiPassword = "";
  wifiConfigured = false;
  stationConnected = false;
  writeConfigToEeprom();

  WiFi.disconnect();
  ensureFallbackAccessPoint();
  sendJson(200, "{\"ok\":true}");
}

void handleApiWifiScan() {
  WiFi.scanDelete();
  const int count = WiFi.scanNetworks(false, true);
  if (count < 0) {
    sendJson(500, "{\"ok\":false,\"error\":\"WLAN-Scan fehlgeschlagen\"}");
    return;
  }

  String json = "{\"ok\":true,\"networks\":[";
  for (int i = 0; i < count; ++i) {
    if (i > 0) {
      json += ",";
    }

    String ssid = WiFi.SSID(i);
    ssid.replace("\\", "\\\\");
    ssid.replace("\"", "\\\"");

    json += "{";
    json += "\"ssid\":\"" + ssid + "\"";
    json += ",\"rssi\":" + String(WiFi.RSSI(i));
    json += ",\"encrypted\":" + String(WiFi.encryptionType(i) == ENC_TYPE_NONE ? "false" : "true");
    json += "}";
  }
  json += "]}";
  sendJson(200, json);
}

void handleApiReset() {
  doReset();
  sendJson(200, "{\"ok\":true}");
}

void handleApiVersion() {
  String json = "{\"ok\":true,\"version\":\"";
  json += jsonEscape(String(FIRMWARE_VERSION));
  json += "\"}";
  sendJson(200, json);
}

void handleApiOtaFlash() {
  if (!stationConnected) {
    sendJson(400, "{\"ok\":false,\"error\":\"Kein WLAN verbunden\"}");
    return;
  }
  String url = server.arg("url");
  if (url.length() == 0) {
    sendJson(400, "{\"ok\":false,\"error\":\"url Parameter fehlt\"}");
    return;
  }

  sendJson(200, "{\"ok\":true,\"message\":\"OTA Update gestartet\"}");
  server.client().flush();
  delay(300);

  WiFiClientSecure tlsClient;
  tlsClient.setInsecure();  // GitHub-Zertifikat nicht pruefen (ESP8266 hat kein CA-Buendel)

  ESPhttpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  ESPhttpUpdate.rebootOnUpdate(true);

  const t_httpUpdate_return ret = ESPhttpUpdate.update(tlsClient, url);

  if (ret == HTTP_UPDATE_FAILED) {
    Serial.printf("FEHLER OTA: [%d] %s\n",
      ESPhttpUpdate.getLastError(),
      ESPhttpUpdate.getLastErrorString().c_str());
  }
}

static const uint8_t FAVICON_PNG[] PROGMEM = {
  0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d, 0x49, 0x48, 0x44, 0x52,
  0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x20, 0x08, 0x02, 0x00, 0x00, 0x00, 0xfc, 0x18, 0xed,
  0xa3, 0x00, 0x00, 0x08, 0x5e, 0x49, 0x44, 0x41, 0x54, 0x78, 0xda, 0xad, 0x96, 0x5b, 0x8c, 0x5d,
  0x65, 0x15, 0xc7, 0xff, 0xeb, 0xdb, 0xdf, 0xde, 0xfb, 0xcc, 0x39, 0x73, 0xe6, 0xcc, 0xa5, 0x9d,
  0xe9, 0x4c, 0x2f, 0x0c, 0x33, 0xd3, 0xe9, 0x1d, 0x68, 0x81, 0x5e, 0x28, 0x94, 0xab, 0x1a, 0x28,
  0x08, 0x84, 0x18, 0x84, 0x68, 0xa2, 0xc6, 0x4b, 0x8c, 0xc6, 0x47, 0x1f, 0x44, 0xe2, 0x83, 0x44,
  0x8d, 0x17, 0xaa, 0xc4, 0x78, 0x89, 0x18, 0x7d, 0x12, 0x0d, 0x1a, 0x2c, 0x18, 0xb4, 0x88, 0x90,
  0x62, 0x51, 0x99, 0x42, 0xb9, 0x65, 0xc0, 0x99, 0x42, 0x4b, 0x3b, 0x33, 0xcc, 0xb4, 0x73, 0x3f,
  0xb7, 0x7d, 0xfb, 0xd6, 0x5a, 0x3e, 0x9c, 0x33, 0x05, 0xe2, 0x9b, 0x71, 0x3d, 0xad, 0xbd, 0xbe,
  0xbd, 0xd7, 0xda, 0x7b, 0xed, 0xef, 0xfb, 0xfd, 0x17, 0xe1, 0x7d, 0x16, 0x04, 0x7e, 0x6b, 0xa1,
  0x2b, 0x97, 0x2b, 0x58, 0x6b, 0x55, 0x95, 0x88, 0x9a, 0x0b, 0xaa, 0x20, 0x52, 0x55, 0x40, 0x01,
  0x02, 0xa0, 0xaa, 0x2b, 0x2b, 0x0a, 0x00, 0x0a, 0x00, 0x99, 0x73, 0x49, 0x5a, 0x8b, 0xa2, 0xc5,
  0x34, 0x75, 0x17, 0x72, 0x36, 0x53, 0x90, 0xc1, 0xfa, 0xb5, 0x17, 0xf7, 0x5f, 0x34, 0xb0, 0x71,
  0x4b, 0xae, 0xbd, 0x93, 0x3d, 0xaf, 0x11, 0x57, 0x80, 0x54, 0xf5, 0xbd, 0x74, 0x50, 0xa8, 0xca,
  0x4a, 0xa4, 0xe1, 0xae, 0x94, 0x51, 0x66, 0xcc, 0x9e, 0xc3, 0xd8, 0x68, 0x3c, 0x3d, 0x73, 0x76,
  0x6e, 0x6e, 0xa6, 0xf1, 0x10, 0x11, 0x11, 0x91, 0x6e, 0x1c, 0xda, 0xbe, 0xeb, 0x8a, 0xfe, 0x9d,
  0x7b, 0x2b, 0x33, 0xd3, 0xb5, 0xe9, 0x49, 0x49, 0x13, 0x6a, 0xbc, 0xfe, 0x85, 0xcc, 0x22, 0x4a,
  0x04, 0x11, 0x05, 0x94, 0x45, 0x9a, 0x29, 0x55, 0x54, 0x54, 0xa1, 0x00, 0x44, 0x38, 0x0c, 0x75,
  0x5d, 0x3f, 0x95, 0x3a, 0x82, 0x63, 0x4f, 0xe5, 0xc6, 0xc6, 0xa6, 0x67, 0x67, 0xa7, 0x00, 0xb2,
  0xaa, 0xda, 0xbd, 0xba, 0xef, 0x92, 0xcb, 0xd6, 0x5d, 0xba, 0x67, 0xe6, 0xcf, 0x7f, 0x4c, 0x67,
  0xa6, 0x7c, 0xeb, 0x1b, 0x22, 0x32, 0x06, 0x44, 0x2a, 0xd2, 0xec, 0x90, 0xaa, 0x01, 0x14, 0x50,
  0x11, 0xa8, 0x12, 0x40, 0xaa, 0xc2, 0x0c, 0x80, 0x98, 0x95, 0x59, 0x55, 0xa1, 0xd0, 0x91, 0x63,
  0x7c, 0xf1, 0xa6, 0xf8, 0xba, 0x8f, 0x94, 0xab, 0xb5, 0x52, 0xb5, 0xba, 0x5c, 0xaf, 0x57, 0xad,
  0xe7, 0x99, 0x9e, 0x9e, 0xbe, 0x1d, 0x57, 0xce, 0x3f, 0xf9, 0x58, 0x74, 0xee, 0xdd, 0xa0, 0xd8,
  0xc6, 0x22, 0x30, 0x86, 0xa2, 0xba, 0x24, 0x09, 0xc2, 0xb0, 0xd9, 0xc3, 0x20, 0x94, 0x34, 0x95,
  0xa8, 0x86, 0x30, 0xa4, 0x2c, 0x13, 0xc7, 0x02, 0xa5, 0x20, 0x44, 0x1c, 0x4b, 0x10, 0x72, 0x60,
  0x95, 0x59, 0x2a, 0x65, 0x0d, 0x43, 0xbc, 0xf9, 0xaa, 0x15, 0x49, 0xf7, 0x5d, 0x5f, 0x3e, 0xfb,
  0x4e, 0x7b, 0x14, 0x55, 0xad, 0x6f, 0x5b, 0x07, 0x86, 0xbd, 0x99, 0xe9, 0xca, 0xe4, 0x19, 0xaf,
  0xd4, 0xee, 0x92, 0x04, 0xc6, 0xa0, 0xbc, 0xac, 0xc3, 0x5b, 0xbd, 0x3b, 0xee, 0x35, 0xeb, 0xfa,
  0xd5, 0x0f, 0x00, 0xc8, 0xfd, 0x5f, 0xa9, 0x3b, 0x17, 0x7e, 0xf9, 0x3e, 0xda, 0xb1, 0xd3, 0x9d,
  0x7e, 0x5b, 0x8f, 0x3c, 0x66, 0x3e, 0x7c, 0xa7, 0x5c, 0x34, 0xe0, 0x46, 0x5f, 0xa1, 0x5f, 0xfc,
  0x40, 0xa3, 0x88, 0x55, 0xf1, 0xb1, 0x4f, 0x61, 0xdf, 0xf5, 0xe9, 0xda, 0x7e, 0xfe, 0xd5, 0x43,
  0x98, 0x3d, 0xaf, 0xfd, 0x83, 0xe1, 0xc2, 0x71, 0xdf, 0x1a, 0x13, 0x14, 0x4b, 0xc9, 0xd4, 0x59,
  0x21, 0x22, 0xe7, 0x00, 0xa0, 0x5a, 0xd1, 0x4b, 0xae, 0x30, 0xdf, 0x7d, 0x38, 0x2d, 0x16, 0x5d,
  0xe2, 0x98, 0xc8, 0x8d, 0x1c, 0x4b, 0xcf, 0x9c, 0x0a, 0x7f, 0xfc, 0x1b, 0xef, 0xda, 0xeb, 0xe2,
  0xb9, 0xaa, 0x1b, 0xd8, 0x61, 0x6e, 0xbc, 0x9d, 0x44, 0x33, 0x66, 0xdd, 0xbc, 0x4b, 0x8f, 0x1c,
  0xb6, 0xc7, 0x9f, 0x0b, 0xee, 0x3f, 0xc4, 0x77, 0x7d, 0xba, 0x56, 0x29, 0xa7, 0x8e, 0xd3, 0xf1,
  0x37, 0x5a, 0x02, 0x3f, 0x6c, 0x5f, 0xc5, 0x0a, 0x6b, 0x00, 0x15, 0x71, 0x49, 0x2c, 0xaa, 0xc2,
  0xcc, 0x20, 0xce, 0x32, 0xb9, 0xfd, 0xde, 0x28, 0xc8, 0x57, 0xcb, 0xb5, 0xe4, 0xc8, 0xe1, 0xe4,
  0xc0, 0x70, 0xf5, 0x4b, 0x77, 0xeb, 0xde, 0x03, 0xf6, 0xaa, 0xeb, 0xea, 0xcf, 0x3d, 0x5f, 0xff,
  0xe1, 0x37, 0x52, 0x4f, 0xf8, 0x95, 0xe3, 0xd5, 0xef, 0x7d, 0x2d, 0xf5, 0x48, 0xc6, 0x47, 0x93,
  0xf1, 0x51, 0xde, 0x7c, 0x89, 0xde, 0x74, 0x7b, 0x75, 0x61, 0x3e, 0x7d, 0xe3, 0x55, 0xb9, 0xe3,
  0xca, 0x70, 0xf4, 0xa5, 0xd0, 0x78, 0xca, 0x2c, 0x00, 0x0c, 0x14, 0x22, 0x2a, 0xc2, 0x22, 0x22,
  0xc2, 0x95, 0xb2, 0x2c, 0xce, 0xc9, 0xf1, 0x7f, 0xc4, 0x99, 0xcb, 0xe2, 0x38, 0xee, 0x1f, 0x92,
  0x30, 0x28, 0xb9, 0xb4, 0xf5, 0xa6, 0x8f, 0x66, 0x1e, 0xe2, 0xbf, 0xfd, 0x89, 0x37, 0x0c, 0xd8,
  0x42, 0xae, 0xfe, 0xf4, 0x13, 0xb2, 0x61, 0x20, 0x28, 0xe4, 0xe3, 0xa3, 0x47, 0x78, 0xe2, 0xb4,
  0x9d, 0x78, 0x27, 0x3d, 0x73, 0x2a, 0x13, 0x91, 0xd5, 0x6b, 0xb8, 0xd4, 0xde, 0xa2, 0x6a, 0x01,
  0x15, 0x11, 0x00, 0x56, 0x55, 0x45, 0x98, 0x45, 0x54, 0x25, 0x8e, 0xe4, 0xb2, 0xdd, 0xd4, 0xde,
  0x49, 0x03, 0xc3, 0xa6, 0x5a, 0x8e, 0x82, 0x50, 0x83, 0x7c, 0x94, 0x24, 0xb4, 0xae, 0x3f, 0xb7,
  0xeb, 0xaa, 0xe8, 0xad, 0xb3, 0xf1, 0xa9, 0xb1, 0xf0, 0xe3, 0x9f, 0x4d, 0xc7, 0xce, 0xa4, 0xa7,
  0xc7, 0xf2, 0x77, 0x7f, 0x26, 0x99, 0x9e, 0x4d, 0xff, 0xf2, 0x87, 0x96, 0x3d, 0x07, 0xbc, 0x4d,
  0xdb, 0xd5, 0x39, 0x90, 0xe7, 0x58, 0x5d, 0x92, 0x64, 0x44, 0x9e, 0x6a, 0x73, 0x27, 0x5b, 0x55,
  0x65, 0x61, 0xa8, 0x24, 0xb1, 0xac, 0x1f, 0xc0, 0xb7, 0x1e, 0xae, 0x74, 0xb5, 0xa5, 0xb1, 0x4a,
  0xb5, 0x2a, 0x53, 0x13, 0xee, 0x3b, 0x5f, 0xf5, 0x66, 0x26, 0xed, 0x3d, 0x9f, 0x4b, 0xb6, 0xf4,
  0xd7, 0x7e, 0xfe, 0x4b, 0xdd, 0xb2, 0x23, 0x77, 0xe9, 0x60, 0xed, 0x27, 0x0f, 0x63, 0x78, 0x3b,
  0x6d, 0xdf, 0x54, 0xff, 0xfd, 0x63, 0x32, 0x33, 0x91, 0x7b, 0xf4, 0xf9, 0xa5, 0xde, 0x8b, 0xa2,
  0xa5, 0x45, 0x9d, 0x3b, 0xcf, 0x0f, 0xde, 0x17, 0x4e, 0x4f, 0xf8, 0x7e, 0x20, 0xd2, 0x3c, 0x82,
  0xb0, 0xaa, 0x90, 0xc6, 0xa5, 0x9a, 0xd6, 0x36, 0x7e, 0xe4, 0xe1, 0x2c, 0x89, 0x95, 0x19, 0x53,
  0x67, 0xcc, 0xc8, 0xd1, 0xd6, 0x5a, 0xb9, 0xb0, 0x66, 0xad, 0xf7, 0xee, 0x44, 0xf4, 0xe0, 0x21,
  0x7a, 0xfa, 0xf1, 0x96, 0xfe, 0x21, 0xf9, 0xfe, 0x21, 0xf3, 0xd7, 0xc3, 0xb9, 0x0d, 0x83, 0xf2,
  0xa3, 0x43, 0xde, 0x33, 0x4f, 0xb4, 0x94, 0x3a, 0xcc, 0xe1, 0x47, 0x5c, 0x1c, 0xd9, 0xf9, 0x59,
  0x8c, 0x1c, 0x2d, 0x9c, 0x9b, 0x68, 0x29, 0x75, 0x98, 0xf2, 0x92, 0xa8, 0x8a, 0xaa, 0x01, 0x40,
  0x81, 0xdf, 0x7e, 0xcb, 0x9d, 0x1d, 0x4b, 0x8b, 0xfc, 0xc2, 0xdf, 0xcb, 0x51, 0xbc, 0x84, 0x0f,
  0x98, 0xb1, 0x16, 0x22, 0x68, 0x74, 0x13, 0x30, 0x4d, 0xe8, 0x80, 0x56, 0x1c, 0x03, 0xc8, 0x8a,
  0x0f, 0x80, 0x3c, 0x0b, 0x76, 0xea, 0xdb, 0xb6, 0xbd, 0xd7, 0x06, 0xce, 0xc9, 0x3f, 0x8f, 0xd6,
  0x2c, 0x00, 0x63, 0xa8, 0x56, 0xab, 0xed, 0xd9, 0xb3, 0xf7, 0x13, 0x9f, 0xbc, 0x2b, 0x8e, 0x6b,
  0xaa, 0x10, 0x15, 0x76, 0x2e, 0x89, 0x33, 0xe7, 0xd8, 0x39, 0x67, 0x8c, 0xb2, 0x70, 0x96, 0x3a,
  0x55, 0x51, 0x08, 0x67, 0x02, 0x82, 0x31, 0x60, 0x56, 0x11, 0x25, 0x88, 0xf1, 0x8c, 0x8a, 0x66,
  0x8e, 0x99, 0x35, 0x0c, 0xfd, 0x27, 0x1e, 0x7f, 0x36, 0x4d, 0xcf, 0x58, 0x1b, 0x00, 0xb0, 0x50,
  0x55, 0x85, 0x73, 0xae, 0xad, 0xb3, 0xb0, 0x6d, 0xf3, 0xd0, 0xfc, 0xfc, 0x62, 0x1c, 0xc5, 0x51,
  0x1c, 0x67, 0x59, 0xe6, 0x5a, 0x9c, 0xaa, 0x66, 0xce, 0xb1, 0xcb, 0x44, 0x44, 0x58, 0x00, 0xb0,
  0x70, 0x13, 0xa9, 0xda, 0xa4, 0x69, 0x83, 0x80, 0x64, 0x0c, 0x54, 0x45, 0x24, 0x9f, 0x0f, 0x5b,
  0x72, 0xa1, 0x63, 0xf6, 0x3c, 0x05, 0x00, 0xdf, 0x96, 0x6e, 0xbb, 0x6b, 0xc3, 0x81, 0x1b, 0xd6,
  0x5e, 0x20, 0xeb, 0xff, 0xc3, 0xfc, 0xab, 0x6f, 0xe8, 0xda, 0x73, 0x4d, 0x07, 0x10, 0xda, 0x46,
  0x20, 0x8a, 0xe3, 0x8d, 0x43, 0xdb, 0xaf, 0xda, 0xbf, 0x33, 0x4d, 0x5d, 0xb9, 0x52, 0x2e, 0xe4,
  0x5b, 0x7c, 0xdf, 0x9f, 0x9b, 0x9b, 0x2f, 0x14, 0xf2, 0xcc, 0xc2, 0xec, 0x9a, 0x8c, 0xd6, 0xc6,
  0xf6, 0x6b, 0x42, 0x1a, 0x20, 0x15, 0x49, 0x92, 0xa4, 0x54, 0x2a, 0x55, 0x2a, 0x15, 0x85, 0xfa,
  0xd6, 0x27, 0x83, 0x57, 0x5f, 0x3e, 0xe9, 0xdc, 0x32, 0x91, 0x07, 0xa0, 0x59, 0xa0, 0x5e, 0xab,
  0xef, 0xd9, 0x7d, 0xc5, 0xb7, 0x1f, 0xf8, 0xfa, 0xd4, 0xd4, 0xb4, 0xcb, 0x5c, 0x14, 0x27, 0x69,
  0x96, 0xba, 0x2c, 0x73, 0x59, 0x26, 0x22, 0xce, 0xb1, 0x08, 0x03, 0x20, 0x02, 0x40, 0x8a, 0x06,
  0xa1, 0x41, 0x44, 0x80, 0x52, 0x43, 0x33, 0xa0, 0x44, 0x06, 0xd0, 0x42, 0x21, 0xff, 0xcd, 0x07,
  0x1e, 0x4a, 0x92, 0x17, 0x73, 0xb9, 0x3c, 0xa0, 0x64, 0xbd, 0xb6, 0x0f, 0xdd, 0x5a, 0x58, 0x5c,
  0xe0, 0xd1, 0x13, 0x5c, 0x28, 0x90, 0xb0, 0x1a, 0x03, 0x01, 0xa0, 0x60, 0x51, 0x15, 0x58, 0x4b,
  0x0a, 0x6d, 0x24, 0x77, 0xac, 0x86, 0x00, 0x52, 0x00, 0x2a, 0xaa, 0x00, 0x33, 0x3c, 0xaf, 0xf1,
  0x47, 0x54, 0x15, 0x22, 0xf0, 0x3c, 0xd4, 0xeb, 0xb4, 0x6b, 0x9f, 0x71, 0x4e, 0x46, 0x8e, 0x45,
  0xf6, 0x82, 0xaa, 0xf8, 0x7e, 0x4b, 0x5b, 0xa7, 0x3a, 0xa7, 0xd5, 0x8a, 0x42, 0xc9, 0x58, 0xcd,
  0x85, 0x08, 0x02, 0x94, 0x97, 0x15, 0x02, 0x22, 0x80, 0xb4, 0xab, 0x8b, 0x6a, 0x55, 0x75, 0x99,
  0xba, 0x0c, 0xd6, 0x07, 0x19, 0x74, 0x14, 0x51, 0x5e, 0x52, 0x55, 0x08, 0xc3, 0x78, 0xda, 0xde,
  0x45, 0x69, 0x22, 0x49, 0xe2, 0x89, 0xa6, 0x8d, 0x36, 0x9a, 0xa6, 0x9e, 0x40, 0x92, 0x44, 0x07,
  0x36, 0x69, 0xe7, 0x2a, 0x27, 0x8e, 0x86, 0xb7, 0x4a, 0x65, 0x59, 0x57, 0xf7, 0x60, 0xe3, 0x96,
  0x6c, 0x69, 0x1e, 0xfd, 0x43, 0xd4, 0xd3, 0xa7, 0xd5, 0x65, 0xb3, 0xe3, 0x72, 0x69, 0xc9, 0x6b,
  0x9a, 0x98, 0xa1, 0x2d, 0xa4, 0x42, 0xd6, 0x9a, 0xcb, 0xf6, 0x70, 0x1c, 0x51, 0xbe, 0x95, 0x2e,
  0x1e, 0x46, 0x79, 0xd1, 0x6c, 0xda, 0x81, 0xd5, 0x6b, 0x88, 0xb3, 0xf7, 0xe4, 0xba, 0xb1, 0x4d,
  0x15, 0x0a, 0x22, 0xaa, 0xd7, 0x92, 0x52, 0x27, 0xb6, 0xed, 0x44, 0xa9, 0x23, 0x1b, 0xde, 0xea,
  0x07, 0xa1, 0xc4, 0x89, 0x0e, 0x6f, 0x43, 0xa9, 0x93, 0x5b, 0x9d, 0xaa, 0x52, 0xb5, 0xc2, 0xc5,
  0x92, 0x29, 0x75, 0xa0, 0xab, 0x27, 0x65, 0xb6, 0x59, 0x46, 0xcb, 0x8b, 0xba, 0x61, 0x50, 0x5b,
  0x8b, 0x58, 0xdd, 0xeb, 0xd2, 0xc4, 0x8a, 0x70, 0xe6, 0x04, 0x08, 0x2e, 0xa8, 0xad, 0x67, 0x4c,
  0xae, 0x7f, 0xa3, 0x1f, 0xd5, 0x65, 0xee, 0x1c, 0xb3, 0x0b, 0xd3, 0x94, 0x73, 0xf9, 0x34, 0x8e,
  0x4c, 0x57, 0x8f, 0x8a, 0x13, 0xeb, 0x53, 0x5b, 0x07, 0x67, 0x19, 0xac, 0xaf, 0xc5, 0x92, 0xb0,
  0x53, 0x32, 0xd4, 0xb5, 0x5a, 0x6a, 0x35, 0x29, 0x75, 0xc0, 0xb3, 0x4a, 0xd0, 0x62, 0x49, 0xc8,
  0x68, 0x9a, 0xc0, 0xb9, 0xf8, 0xdd, 0xb3, 0xbe, 0x4b, 0xf3, 0x95, 0x4a, 0xda, 0xb3, 0x16, 0xc2,
  0x98, 0x9e, 0x74, 0x56, 0x55, 0x55, 0xc5, 0x31, 0x17, 0x8b, 0xa5, 0x2f, 0x7c, 0xfe, 0x8b, 0x2f,
  0xfc, 0xeb, 0xc5, 0x67, 0x9f, 0x7d, 0x86, 0x3c, 0x67, 0x6d, 0x56, 0x68, 0x0d, 0xf2, 0x05, 0xcf,
  0x0f, 0xe0, 0x59, 0xf5, 0xac, 0x00, 0xc6, 0xf3, 0x34, 0xae, 0x2b, 0xa0, 0x59, 0x8a, 0x5a, 0x05,
  0x71, 0x04, 0x55, 0x2a, 0x2f, 0x49, 0x14, 0xa5, 0xe2, 0x3c, 0xcf, 0x14, 0xb7, 0x6d, 0xdb, 0xbc,
  0x73, 0xd7, 0xa5, 0x3f, 0xfb, 0xe9, 0xaf, 0x57, 0x68, 0x0a, 0xab, 0xd2, 0x98, 0x18, 0xc4, 0x18,
  0x6f, 0x7c, 0xfc, 0xdf, 0xab, 0xbb, 0x3b, 0x6e, 0xbd, 0xed, 0x96, 0x30, 0x0c, 0x4f, 0x9c, 0x78,
  0x65, 0x66, 0x66, 0xe6, 0xfc, 0x74, 0x59, 0xa1, 0xc6, 0x50, 0x18, 0x06, 0xcc, 0xce, 0x78, 0x06,
  0x2a, 0xcc, 0xac, 0x4a, 0xce, 0x65, 0x44, 0xa4, 0xa2, 0x85, 0xd6, 0x42, 0xff, 0x86, 0x8d, 0x6b,
  0xfb, 0x7a, 0xf7, 0xef, 0xdf, 0x37, 0x35, 0x35, 0x35, 0x35, 0x35, 0x11, 0x04, 0x21, 0x51, 0xa4,
  0x4a, 0x0d, 0x6c, 0xe5, 0x76, 0x5f, 0x5d, 0x2c, 0x75, 0x66, 0xc7, 0x9e, 0xb2, 0x7e, 0x18, 0xf7,
  0xae, 0xe9, 0xdb, 0xbf, 0x7f, 0x7f, 0x1c, 0xc7, 0xdd, 0xdd, 0xdd, 0xb5, 0x5a, 0x6d, 0x61, 0x61,
  0x61, 0x61, 0x61, 0xa1, 0x52, 0xa9, 0x9e, 0x3a, 0x7d, 0xaa, 0xbd, 0xd4, 0x1e, 0x45, 0x11, 0x11,
  0x15, 0x0a, 0x05, 0x85, 0xb6, 0x97, 0x4a, 0x7d, 0x7d, 0x7d, 0x71, 0x1c, 0xaf, 0x5a, 0xd5, 0xb5,
  0x6b, 0xd7, 0xe5, 0xa7, 0x4f, 0x9f, 0x9e, 0x3d, 0x3f, 0xfb, 0xda, 0xeb, 0xaf, 0x4f, 0x4c, 0x9e,
  0xe1, 0x2c, 0x77, 0xed, 0xcd, 0x7c, 0x6a, 0x0c, 0x27, 0xdf, 0xac, 0x13, 0x91, 0xe9, 0x59, 0xb3,
  0xea, 0xc6, 0x3b, 0xea, 0x47, 0x1e, 0xcd, 0x7b, 0x1e, 0xa5, 0x69, 0xe2, 0x1c, 0x07, 0x41, 0x30,
  0x38, 0x38, 0xb8, 0x7e, 0xfd, 0xba, 0xc1, 0xc1, 0xc1, 0x55, 0x5d, 0xab, 0xc6, 0x4f, 0x8e, 0xd7,
  0xeb, 0xf5, 0x83, 0x07, 0x0f, 0x9e, 0x38, 0x71, 0xa2, 0xad, 0xd8, 0x16, 0x27, 0x71, 0x6f, 0x6f,
  0x2f, 0x80, 0x91, 0x91, 0x91, 0xc9, 0xc9, 0xc9, 0x7a, 0x3d, 0x2a, 0xb6, 0xb6, 0x9e, 0x7c, 0xeb,
  0xad, 0xf3, 0xe7, 0xcf, 0x07, 0xa1, 0x6f, 0xbd, 0x30, 0x08, 0xf5, 0x9a, 0x9b, 0x93, 0x27, 0x7f,
  0x67, 0xea, 0xf5, 0x0a, 0x11, 0xc1, 0xb7, 0x6d, 0xbb, 0x0f, 0xf8, 0xdd, 0x6b, 0xe3, 0x27, 0x7f,
  0x9b, 0x6f, 0x29, 0x20, 0x08, 0x20, 0x22, 0x69, 0x96, 0x11, 0xc8, 0x78, 0xa6, 0xa3, 0xa3, 0xa3,
  0x52, 0x2e, 0xf7, 0xf6, 0xf5, 0xf5, 0xae, 0x59, 0xc3, 0xcc, 0xf9, 0x7c, 0x3e, 0x4d, 0xd3, 0xb9,
  0xb9, 0x39, 0x11, 0x9d, 0x9b, 0x9b, 0x2b, 0x97, 0xcb, 0xcc, 0x4e, 0x44, 0xac, 0xb5, 0x41, 0x10,
  0xc6, 0xb1, 0x64, 0x09, 0x1d, 0xbc, 0x27, 0x1d, 0x7b, 0x0d, 0xa3, 0x2f, 0x67, 0x8a, 0x88, 0x00,
  0x78, 0xd6, 0xe4, 0x82, 0xce, 0x7d, 0x37, 0xa6, 0x5d, 0xdd, 0x7a, 0xfc, 0x68, 0xb8, 0x34, 0x4f,
  0x22, 0x20, 0x22, 0x55, 0xa8, 0x8a, 0x73, 0xce, 0xf3, 0xbc, 0x2c, 0xcb, 0x9c, 0x73, 0x44, 0x24,
  0x22, 0x64, 0x8c, 0x31, 0x06, 0x0a, 0x6b, 0x3d, 0x6b, 0x2d, 0x11, 0x35, 0x66, 0x34, 0x63, 0xd0,
  0xd5, 0x8d, 0xcb, 0x0f, 0xb8, 0xb3, 0x6f, 0xf3, 0x8b, 0xcf, 0x41, 0x50, 0x65, 0xd6, 0x26, 0x41,
  0x7d, 0xdf, 0xb3, 0xa6, 0x34, 0xbc, 0x03, 0x43, 0x5b, 0x99, 0xd9, 0xa4, 0xb1, 0x01, 0x54, 0x95,
  0x40, 0x30, 0x44, 0xaa, 0x20, 0x6a, 0x94, 0xbc, 0xa0, 0x2d, 0x58, 0x19, 0x4a, 0xdf, 0x8b, 0x04,
  0x39, 0x21, 0x23, 0xa3, 0x2f, 0xe1, 0x9d, 0x71, 0x55, 0xaa, 0x39, 0x27, 0x78, 0x3f, 0xa2, 0x8d,
  0x07, 0xd2, 0x16, 0xdf, 0xfa, 0xad, 0x25, 0xf8, 0x3e, 0x74, 0x05, 0xf9, 0x17, 0xee, 0x78, 0x9f,
  0x0a, 0x34, 0x0f, 0xea, 0xca, 0xfc, 0xaa, 0x04, 0x52, 0x68, 0x9a, 0x68, 0x65, 0x59, 0x99, 0x1d,
  0x99, 0xa4, 0xa9, 0x81, 0xff, 0xad, 0x01, 0x44, 0x8d, 0xd1, 0xf3, 0x7f, 0xb4, 0x0f, 0x7e, 0x24,
  0x00, 0xfc, 0x07, 0xed, 0x4b, 0x7e, 0xd5, 0x91, 0x04, 0x4c, 0x8c, 0x00, 0x00, 0x00, 0x00, 0x49,
  0x45, 0x4e, 0x44, 0xae, 0x42, 0x60, 0x82
};
static const size_t FAVICON_PNG_LEN = 2199;

void handleFavicon() {
  server.sendHeader("Cache-Control", "public, max-age=86400");
  server.send_P(200, PSTR("image/png"), reinterpret_cast<const char*>(FAVICON_PNG), FAVICON_PNG_LEN);
}

void handleRootPage() {
  static const char PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html lang="de">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Slotcar Magnet Scale</title>
  <link rel=icon type=image/png href="/favicon.png" />
  <style>
    :root {
      --bg-0: #050505;
      --bg-1: #0d1308;
      --bg-2: #131313;
      --panel: rgba(14, 16, 14, 0.9);
      --card: rgba(20, 24, 20, 0.95);
      --line: rgba(193, 255, 78, 0.26);
      --text: #edf6df;
      --muted: #abbb9c;
      --accent: #c7ff35;
      --accent-2: #7fd923;
      --btn-text: #101507;
      --danger: #ff6055;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Segoe UI", Tahoma, sans-serif;
      color: var(--text);
      background:
        radial-gradient(circle at 12% 10%, rgba(199, 255, 53, 0.15), transparent 35%),
        radial-gradient(circle at 88% -10%, rgba(127, 217, 35, 0.20), transparent 45%),
        linear-gradient(150deg, var(--bg-0) 0%, var(--bg-1) 45%, var(--bg-2) 100%);
      min-height: 100vh;
    }
    .app { display: flex; min-height: 100vh; }
    .sidebar {
      width: 260px;
      background: rgba(10, 12, 10, 0.94);
      border-right: 1px solid var(--line);
      padding: 14px;
      transition: transform .25s ease;
    }
    .brand { font-weight: 800; color: var(--accent); letter-spacing: .2px; margin-bottom: 12px; }
    .menu { display: flex; flex-direction: column; gap: 8px; }
    .menu button {
      text-align: left;
      width: 100%;
      background: linear-gradient(135deg, rgba(199,255,53,.18), rgba(127,217,35,.10));
      color: #e9f7d3;
      border: 1px solid rgba(199,255,53,.22);
      border-radius: 10px;
      padding: 10px;
      font-weight: 700;
      cursor: pointer;
    }
    .menu button.active {
      background: linear-gradient(135deg, var(--accent), var(--accent-2));
      color: var(--btn-text);
      border-color: rgba(199,255,53,.65);
    }
    .content { flex: 1; padding: 16px; }
    .topbar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      margin-bottom: 12px;
    }
    .menu-toggle {
      display: none;
      border: 1px solid var(--line);
      background: linear-gradient(135deg, rgba(199,255,53,.22), rgba(127,217,35,.12));
      color: #eff8df;
      padding: 8px 10px;
      border-radius: 8px;
      font-weight: 700;
      cursor: pointer;
    }
    .panel {
      background: var(--panel);
      border: 1px solid rgba(199,255,53,.18);
      border-radius: 16px;
      padding: 14px;
      box-shadow: 0 24px 50px rgba(0,0,0,.55), inset 0 0 0 1px rgba(255,255,255,.03);
    }
    .section { display: none; }
    .section.active { display: block; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit,minmax(280px,1fr)); gap: 10px; }
    .card {
      background: linear-gradient(160deg, rgba(27, 31, 27, 0.98), rgba(17, 20, 17, 0.98));
      border: 1px solid var(--line);
      border-radius: 12px;
      padding: 10px;
    }
    h1 { margin: 0; color: var(--accent); font-size: 1.25rem; text-shadow: 0 0 18px rgba(199,255,53,.18); }
    h2 { margin: 0 0 8px; color: var(--accent); font-size: 1.05rem; }
    .muted { font-size: .9rem; color: var(--muted); }
    .big { font-size: 2rem; font-weight: 800; color: #f5ffe4; }
    .row { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; }
    .stack { display: flex; flex-direction: column; gap: 8px; }
    input {
      padding: 8px;
      border: 1px solid rgba(199,255,53,.25);
      border-radius: 8px;
      background: #0f130e;
      color: var(--text);
      min-width: 120px;
    }
    input::placeholder { color: #93a189; }
    button {
      border: 0;
      border-radius: 8px;
      background: linear-gradient(135deg, var(--accent) 0%, var(--accent-2) 100%);
      color: var(--btn-text);
      padding: 8px 10px;
      font-weight: 700;
      cursor: pointer;
    }
    button.alt {
      background: linear-gradient(135deg, #7ca329 0%, #4c6e1b 100%);
      color: #ecf6d8;
    }
    button.danger {
      background: linear-gradient(135deg, #ff8d82 0%, var(--danger) 100%);
      color: #2a0907;
    }
    button:hover { filter: brightness(1.08); }
    .pill {
      display: inline-block;
      border: 1px solid var(--line);
      border-radius: 999px;
      padding: 2px 9px;
      color: #ebf9d7;
      background: rgba(199,255,53,.08);
      margin-right: 6px;
      margin-top: 4px;
      font-size: .85rem;
    }
    .wifi-list {
      margin-top: 8px;
      max-height: 230px;
      overflow: auto;
      border: 1px solid rgba(199,255,53,.22);
      border-radius: 10px;
      background: #0d120c;
    }
    .wifi-item {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
      padding: 8px;
      border-bottom: 1px solid rgba(199,255,53,.12);
    }
    .wifi-item:last-child { border-bottom: 0; }
    table {
      width: 100%;
      border-collapse: collapse;
      margin-top: 8px;
      background: var(--card);
      border: 1px solid var(--line);
      border-radius: 10px;
      overflow: hidden;
      font-size: .92rem;
    }
    th, td { padding: 7px; border-bottom: 1px solid rgba(193,255,78,.16); text-align: right; color: #edf5df; }
    th { background: rgba(199,255,53,.08); color: #dff0c5; }
    th:first-child, td:first-child { text-align: left; }
    pre {
      margin: 8px 0 0;
      max-height: 190px;
      overflow: auto;
      background: #0e120d;
      color: #d8e6ca;
      border: 1px solid rgba(199,255,53,.22);
      padding: 8px;
      border-radius: 8px;
    }
    .wizard-backdrop {
      position: fixed;
      inset: 0;
      background: rgba(0, 0, 0, .72);
      display: none;
      align-items: center;
      justify-content: center;
      z-index: 50;
      padding: 14px;
    }
    .wizard-backdrop.show { display: flex; }
    .wizard {
      width: min(720px, 100%);
      background: linear-gradient(160deg, rgba(24, 28, 24, 0.98), rgba(14, 17, 14, 0.98));
      border: 1px solid rgba(199,255,53,.35);
      border-radius: 14px;
      padding: 14px;
      box-shadow: 0 26px 56px rgba(0,0,0,.65);
    }
    .wizard h3 {
      margin: 0 0 8px;
      color: var(--accent);
      font-size: 1.08rem;
    }
    .wizard-steps {
      margin: 8px 0 10px;
      font-size: .92rem;
      color: #ddefc6;
      line-height: 1.45;
    }
    .wizard-status {
      font-size: .9rem;
      color: var(--muted);
      margin-top: 6px;
    }
    @media (max-width: 900px) {
      .menu-toggle { display: inline-block; }
      .sidebar {
        position: fixed;
        z-index: 20;
        left: 0;
        top: 0;
        height: 100vh;
        transform: translateX(-100%);
      }
      .sidebar.open { transform: translateX(0); }
      .content { width: 100%; }
    }
  </style>
</head>
<body>
  <div class="app">
    <aside id="sidebar" class="sidebar">
      <div class="brand">Slotcar Magnet Scale</div>
      <div class="menu">
        <button data-tab="dashboard" class="active">Dashboard</button>
        <button data-tab="measurement">Messung</button>
        <button data-tab="calibration">Kalibrierung</button>
        <button data-tab="wifi">WLAN</button>
        <button data-tab="system">System</button>
      </div>
      <div class="muted" style="margin-top:12px">AP + STA</div>
    </aside>

    <main class="content">
      <div class="topbar">
        <button class="menu-toggle" onclick="toggleMenu()">Menue</button>
        <h1>Slotcar Magnet Scale</h1>
      </div>

      <section id="tab-dashboard" class="section active panel">
        <div class="grid">
          <div class="card">
            <div class="muted">Aktuelles Gewicht</div>
            <div class="big" id="weight">--.- g</div>
            <div id="shares" class="muted">LINKS -- % | RECHTS -- %</div>
            <div class="row" style="margin-top:8px">
              <button onclick="readNow()">MESSEN</button>
              <button class="alt" onclick="rawNow()">ROH</button>
              <button class="alt" onclick="taraNow()">TARA</button>
            </div>
          </div>
          <div class="card">
            <h2>Verbindung</h2>
            <div id="connectionInfo" class="muted">Lade...</div>
          </div>
        </div>
      </section>

      <section id="tab-measurement" class="section panel">
        <h2>Messung und Stream</h2>
        <div class="row">
          <input id="measureSamples" placeholder="Messungen (z. B. 8)" value="8" />
          <button onclick="readNowWithSamples()">MESSEN</button>
          <button class="alt" onclick="rawNowWithSamples()" title="Rohmessung: zeigt Roh-/Netto-Werte ohne Umrechnung in Gramm.">ROH</button>
        </div>
        <div class="row" style="margin-top:8px">
          <input id="streamInterval" placeholder="Intervall ms" value="500" />
          <input id="streamSamples" placeholder="Messungen" value="5" />
          <button onclick="startStream()">START Stream</button>
          <button class="alt" onclick="stopStream()">STOP Stream</button>
        </div>
        <pre id="measureLog"></pre>
      </section>

      <section id="tab-calibration" class="section panel">
        <h2>Kalibrierung</h2>
        <div class="row" style="margin-bottom:8px">
          <button onclick="startWizard(false)">Kalibrier-Wizard starten</button>
        </div>
        <div class="muted" style="margin-bottom:10px">
          Die Kalibrierung erfolgt ausschliesslich ueber den Wizard.
        </div>
        <input id="ref" type="hidden" value="1000" />
        <input id="pname" type="hidden" value="" />
        <div class="card" style="margin-top:10px">
          <div id="calSummary" class="muted">Lade Kalibrierreport...</div>
          <table>
            <thead>
              <tr>
                <th>Position</th>
                <th>Netto links</th>
                <th>Netto rechts</th>
                <th>Soll (g)</th>
                <th>Berechnet (g)</th>
                <th>Abweichung (g)</th>
                <th>Links %</th>
                <th>Rechts %</th>
              </tr>
            </thead>
            <tbody id="calRows"></tbody>
          </table>
        </div>
      </section>

      <section id="tab-wifi" class="section panel">
        <h2>WLAN Verwaltung</h2>
        <div class="card">
          <div class="stack">
            <input id="ssid" placeholder="SSID" />
            <input id="pass" placeholder="Passwort" type="password" />
            <div class="row">
              <button onclick="saveWifi()">WLAN speichern</button>
              <button class="alt" onclick="clearWifi()">WLAN loeschen</button>
              <button class="alt" onclick="scanWifi()">WLAN suchen</button>
            </div>
          </div>
        </div>
        <div class="card" style="margin-top:10px">
          <h2>Verbindungsinformationen</h2>
          <div id="wifiDetails" class="muted">Lade...</div>
          <div id="wifiList" class="wifi-list"></div>
        </div>
      </section>

      <section id="tab-system" class="section panel">
        <h2>System</h2>
        <div class="row">
          <button class="alt" onclick="refreshAll()">STATUS aktualisieren</button>
          <button class="danger" onclick="doReset()">RESET</button>
        </div>
        <pre id="statusDump"></pre>
        <h2 style="margin-top:12px">Firmware-Update (OTA)</h2>
        <p id="fwCurrent" style="margin:4px 0;color:var(--muted)">Version: …</p>
        <div class="row" style="margin-top:6px">
          <button class="alt" onclick="checkUpdate()">Nach Updates suchen</button>
        </div>
        <div id="updateBox" style="display:none;margin-top:10px" class="card">
          <p id="updateText" style="margin:4px 0"></p>
          <div id="updateActions" style="display:none;margin-top:8px">
            <button id="flashBtn" onclick="doOtaFlash()">Jetzt flashen</button>
          </div>
        </div>
      </section>

      <pre id="log"></pre>
    </main>
  </div>

  <div id="wizardBackdrop" class="wizard-backdrop">
    <div class="wizard">
      <h3>Kalibrier-Wizard</h3>
      <div class="wizard-steps" id="wizardText">
        Vorbereitung...
      </div>
      <div class="row" id="wizardControls"></div>
      <div class="wizard-status" id="wizardStatus"></div>
    </div>
  </div>

<script>
const q = s => document.querySelector(s);
const qa = s => Array.from(document.querySelectorAll(s));
let latestStatus = null;
let wizardRequired = false;
let wizardRecordedPoints = { LINKS: false, MITTE: false, RECHTS: false };

const log = (m) => {
  const el = q('#log');
  el.textContent = `[${new Date().toLocaleTimeString()}] ${m}\n` + el.textContent;
};

async function api(path) {
  const r = await fetch(path);
  const j = await r.json();
  if (!j.ok) {
    throw new Error(j.error || 'Fehler');
  }
  return j;
}

function toggleMenu() {
  q('#sidebar').classList.toggle('open');
}

function setTab(tab) {
  qa('.menu button').forEach(b => b.classList.toggle('active', b.dataset.tab === tab));
  qa('.section').forEach(s => s.classList.toggle('active', s.id === `tab-${tab}`));
  q('#sidebar').classList.remove('open');
  if (location.hash !== `#${tab}`) {
    history.replaceState(null, '', `#${tab}`);
  }
}

qa('.menu button').forEach(btn => btn.addEventListener('click', () => setTab(btn.dataset.tab)));

function applyHashTab() {
  const tab = (location.hash || '#dashboard').replace('#', '');
  const allowed = ['dashboard', 'measurement', 'calibration', 'wifi', 'system'];
  setTab(allowed.includes(tab) ? tab : 'dashboard');
}

function renderStatus(s) {
  latestStatus = s;
  q('#connectionInfo').innerHTML =
    `<span class="pill">Modus: ${s.wifiMode}</span>` +
    `<span class="pill">STA: ${s.stationConnected ? 'verbunden' : 'nicht verbunden'}</span>` +
    `<span class="pill">AP: ${s.apSsid}</span><br>` +
    `STA IP: ${s.staIp || '-'}<br>` +
    `AP IP: ${s.apIp || '-'}<br>` +
    `WLAN konfiguriert: ${s.wifiConfigured ? 'ja' : 'nein'} ${s.wifiSsid ? '(' + s.wifiSsid + ')' : ''}`;

  q('#wifiDetails').innerHTML =
    `Verbindungsmodus: <b>${s.wifiMode}</b><br>` +
    `Mit Router verbunden: <b>${s.stationConnected ? 'ja' : 'nein'}</b><br>` +
    `Router-SSID (konfiguriert): <b>${s.wifiSsid || '-'}</b><br>` +
    `STA IP: <b>${s.staIp || '-'}</b><br>` +
    `Fallback-AP SSID: <b>${s.apSsid}</b><br>` +
    `Fallback-AP IP: <b>${s.apIp || '-'}</b>`;

  q('#statusDump').textContent = JSON.stringify(s, null, 2);

  if (!s.hasCalibration && !wizardRequired) {
    wizardRequired = true;
    startWizard(true);
  }
  if (s.hasCalibration) {
    wizardRequired = false;
  }
}

function setWizardText(stepHtml, statusText) {
  q('#wizardText').innerHTML = stepHtml;
  q('#wizardStatus').textContent = statusText || '';
}

function setWizardControls(buttons) {
  const controls = q('#wizardControls');
  controls.innerHTML = '';
  buttons.forEach(btn => {
    const b = document.createElement('button');
    b.textContent = btn.label;
    if (btn.alt) b.classList.add('alt');
    if (btn.danger) b.classList.add('danger');
    if (btn.disabled) b.disabled = true;
    b.addEventListener('click', btn.onClick);
    controls.appendChild(b);
  });
}

function updateWizardRecordedPoints(report) {
  wizardRecordedPoints = { LINKS: false, MITTE: false, RECHTS: false };
  if (!report || !Array.isArray(report.points)) {
    return;
  }
  for (const p of report.points) {
    if (!p || p.name === undefined || p.name === null) {
      continue;
    }
    const name = String(p.name).trim().toUpperCase();
    if (name === 'LINKS' || name === 'MITTE' || name === 'RECHTS') {
      wizardRecordedPoints[name] = true;
    }
  }
}

function closeWizard() {
  q('#wizardBackdrop').classList.remove('show');
}

function openWizard() {
  q('#wizardBackdrop').classList.add('show');
}

async function wizardStepIntro(autoStart) {
  const s = latestStatus;
  if (!s) {
    setWizardText('Status wird geladen...', 'Bitte kurz warten.');
    return;
  }

  if (s.hasCalibration && autoStart) {
    closeWizard();
    return;
  }

  setWizardText(
    'Dieser Wizard fuehrt dich Schritt fuer Schritt durch die gemeinsame Kalibrierung beider Waegezellen.<br>' +
    'Empfohlen: 1x LINKS, 1x MITTE, 1x RECHTS mit identischem Referenzgewicht.',
    s.hasCalibration ? 'Es ist bereits eine Kalibrierung vorhanden. Du kannst neu kalibrieren.' : 'Keine gueltige Kalibrierung vorhanden.'
  );

  setWizardControls([
    { label: 'Weiter', onClick: () => wizardStepTare() },
    { label: 'Abbrechen', alt: true, onClick: () => closeWizard() }
  ]);
}

async function wizardStepTare() {
  setWizardText(
    'Schritt 1: Tara setzen<br>Platte komplett entlasten und dann Tara ausfuehren.',
    'Die Tara speichert fuer links und rechts jeweils einen eigenen Offset.'
  );
  setWizardControls([
    { label: 'TARA ausfuehren (20 Messungen)', onClick: async () => {
      try {
        await api('/api/tare?samples=20');
        log('Wizard: Tara gesetzt');
        await refreshStatus();
        wizardStepReference();
      } catch (e) {
        log(e.message);
      }
    } },
    { label: 'Zurueck', alt: true, onClick: () => wizardStepIntro(false) },
    { label: 'Abbrechen', alt: true, onClick: () => closeWizard() }
  ]);
}

function wizardStepReference() {
  const current = q('#ref').value || '1000';
  setWizardText(
    'Schritt 2: Referenzgewicht starten<br>Trage dein exaktes Referenzgewicht in Gramm ein und starte den Kalibriermodus.' +
    '<div class="row" style="margin-top:8px">' +
    '<input id="wizardRefInput" type="number" min="1" step="0.1" placeholder="Referenzgewicht g (z. B. 1000)" value="' + current + '">' +
    '</div>',
    'Beispiel: 1000'
  );

  setWizardControls([
    { label: 'KAL_START ausfuehren', onClick: async () => {
      try {
        const wizardRefInput = q('#wizardRefInput');
        const value = wizardRefInput ? wizardRefInput.value : current;
        q('#ref').value = value;
        const w = encodeURIComponent(value || '0');
        await api('/api/cal/start?weight=' + w);
        log('Wizard: KAL_START ausgefuehrt');
        await refreshCal();
        wizardStepPoints();
      } catch (e) {
        log(e.message);
      }
    } },
    { label: 'Zurueck', alt: true, onClick: () => wizardStepTare() },
    { label: 'Abbrechen', alt: true, onClick: () => closeWizard() }
  ]);
}

function wizardStepPoints() {
  const leftDone = wizardRecordedPoints.LINKS;
  const midDone = wizardRecordedPoints.MITTE;
  const rightDone = wizardRecordedPoints.RECHTS;

  setWizardText(
    'Schritt 3: Kalibrierpunkte aufnehmen<br>Lege dasselbe Referenzgewicht nacheinander LINKS, MITTE und RECHTS auf.\n' +
    'Nach jeder Position den passenden Button druecken.',
    'Status: LINKS ' + (leftDone ? 'OK' : 'offen') + ', MITTE ' + (midDone ? 'OK' : 'offen') + ', RECHTS ' + (rightDone ? 'OK' : 'offen') +
      '. Mindestens 2 Punkte sind erforderlich. 3 Punkte werden empfohlen.'
  );

  setWizardControls([
    { label: leftDone ? 'Punkt LINKS (gesetzt)' : 'Punkt LINKS', disabled: leftDone, onClick: () => wizardAddPoint('LINKS') },
    { label: midDone ? 'Punkt MITTE (gesetzt)' : 'Punkt MITTE', disabled: midDone, onClick: () => wizardAddPoint('MITTE') },
    { label: rightDone ? 'Punkt RECHTS (gesetzt)' : 'Punkt RECHTS', disabled: rightDone, onClick: () => wizardAddPoint('RECHTS') },
    { label: 'Berechnung weiter', alt: true, onClick: () => wizardStepCalculate() },
    { label: 'Abbrechen', alt: true, onClick: async () => {
      try { await api('/api/cal/abort'); await refreshCal(); closeWizard(); } catch (e) { log(e.message); }
    } }
  ]);
}

async function wizardAddPoint(name) {
  try {
    await api('/api/cal/point?name=' + encodeURIComponent(name) + '&samples=20');
    log('Wizard: Kalibrierpunkt ' + name + ' gespeichert');
    await refreshCal();
    wizardStepPoints();
  } catch (e) {
    log(e.message);
  }
}

function wizardStepCalculate() {
  setWizardText(
    'Schritt 4: Faktoren berechnen<br>Jetzt werden aus allen Punkten die Faktoren links/rechts mit Least Squares berechnet.',
    'Bei Erfolg werden die Faktoren automatisch gespeichert.'
  );
  setWizardControls([
    { label: 'KAL_BERECHNEN ausfuehren', onClick: async () => {
      try {
        await api('/api/cal/calculate');
        log('Wizard: Kalibrierung erfolgreich gespeichert');
        await refreshAll();
        setWizardText('Kalibrierung abgeschlossen.', 'Du kannst jetzt normal messen.');
        setWizardControls([{ label: 'Fertig', onClick: () => closeWizard() }]);
      } catch (e) {
        log(e.message);
      }
    } },
    { label: 'Zurueck', alt: true, onClick: () => wizardStepPoints() },
    { label: 'Abbrechen', alt: true, onClick: () => closeWizard() }
  ]);
}

function startWizard(autoStart) {
  openWizard();
  wizardStepIntro(autoStart);
}

function renderCalReport(report) {
  q('#calSummary').innerHTML =
    `Faktor links: <b>${Number(report.factorLeft).toFixed(9)}</b> | ` +
    `Faktor rechts: <b>${Number(report.factorRight).toFixed(9)}</b> | ` +
    `Punkte: <b>${report.count}</b> | ` +
    `Durchschnittl. abs. Fehler: <b>${Number(report.avgAbsDeviation).toFixed(3)} g</b> | ` +
    `Max. abs. Fehler: <b>${Number(report.maxAbsDeviation).toFixed(3)} g</b>`;

  const rows = q('#calRows');
  rows.innerHTML = '';
  for (const p of report.points) {
    const tr = document.createElement('tr');
    tr.innerHTML =
      `<td>${p.name}</td>` +
      `<td>${Number(p.netLeft).toFixed(0)}</td>` +
      `<td>${Number(p.netRight).toFixed(0)}</td>` +
      `<td>${Number(p.knownWeight).toFixed(2)}</td>` +
      `<td>${Number(p.calcWeight).toFixed(2)}</td>` +
      `<td>${Number(p.deviation).toFixed(2)}</td>` +
      `<td>${p.leftPercent !== undefined ? Number(p.leftPercent).toFixed(2) : '-'}</td>` +
      `<td>${p.rightPercent !== undefined ? Number(p.rightPercent).toFixed(2) : '-'}</td>`;
    rows.appendChild(tr);
  }
}

function updateWeightDisplay(m) {
  q('#weight').textContent = `${Number(m.weight).toFixed(2)} g`;
  if (m.leftPercent !== undefined) {
    q('#shares').textContent = `LINKS ${Number(m.leftPercent).toFixed(1)} % | RECHTS ${Number(m.rightPercent).toFixed(1)} %`;
  }
}

async function refreshStatus() {
  const s = await api('/api/status');
  renderStatus(s);
}

async function refreshCal() {
  const report = await api('/api/cal/report');
  updateWizardRecordedPoints(report);
  renderCalReport(report);
}

async function refreshAll() {
  try {
    await refreshStatus();
    await refreshCal();
  } catch (e) {
    log(e.message);
  }
}

async function readNow() {
  try {
    const m = await api('/api/read?samples=8');
    updateWeightDisplay(m);
  } catch (e) {
    log(e.message);
  }
}

async function readNowWithSamples() {
  try {
    const samples = encodeURIComponent(q('#measureSamples').value || '8');
    const m = await api('/api/read?samples=' + samples);
    updateWeightDisplay(m);
    q('#measureLog').textContent = `Gewicht: ${Number(m.weight).toFixed(2)} g\nLINKS: ${m.leftPercent ?? '-'}\nRECHTS: ${m.rightPercent ?? '-'}`;
  } catch (e) {
    log(e.message);
  }
}

async function rawNow() {
  try {
    const r = await api('/api/raw?samples=8');
    log(`ROH L=${r.rawLeft} R=${r.rawRight} | NET L=${r.netLeft} R=${r.netRight}`);
  } catch (e) {
    log(e.message);
  }
}

async function rawNowWithSamples() {
  try {
    const samples = encodeURIComponent(q('#measureSamples').value || '8');
    const r = await api('/api/raw?samples=' + samples);
    q('#measureLog').textContent = `RAW LEFT: ${r.rawLeft}\nRAW RIGHT: ${r.rawRight}\nNET LEFT: ${r.netLeft}\nNET RIGHT: ${r.netRight}`;
  } catch (e) {
    log(e.message);
  }
}

async function taraNow() {
  try {
    await api('/api/tare?samples=20');
    log('Tara gesetzt');
    await refreshStatus();
  } catch (e) {
    log(e.message);
  }
}

async function calStart() {
  try {
    const w = encodeURIComponent(q('#ref').value || '0');
    await api('/api/cal/start?weight=' + w);
    log('Kalibrierung gestartet');
    await refreshCal();
  } catch (e) {
    log(e.message);
  }
}

async function calPoint() {
  try {
    const n = encodeURIComponent(q('#pname').value || '');
    await api('/api/cal/point?name=' + n + '&samples=20');
    log('Kalibrierpunkt gespeichert');
    await refreshCal();
  } catch (e) {
    log(e.message);
  }
}

async function calCalc() {
  try {
    await api('/api/cal/calculate');
    log('Kalibrierung berechnet und gespeichert');
    await refreshAll();
  } catch (e) {
    log(e.message);
  }
}

async function calClear() {
  try {
    await api('/api/cal/clear');
    log('Temporare Kalibrierpunkte geloescht');
    await refreshCal();
  } catch (e) {
    log(e.message);
  }
}

async function calAbort() {
  try {
    await api('/api/cal/abort');
    log('Kalibrierung abgebrochen');
    await refreshCal();
  } catch (e) {
    log(e.message);
  }
}

async function saveWifi() {
  try {
    const ssid = encodeURIComponent(q('#ssid').value || '');
    const password = encodeURIComponent(q('#pass').value || '');
    const resp = await api('/api/wifi/save?ssid=' + ssid + '&password=' + password);
    log(`WLAN gespeichert. Verbunden: ${resp.connected ? 'ja' : 'nein'}`);
    await refreshStatus();
  } catch (e) {
    log(e.message);
  }
}

async function clearWifi() {
  try {
    await api('/api/wifi/clear');
    log('WLAN geloescht, AP aktiv');
    await refreshStatus();
  } catch (e) {
    log(e.message);
  }
}

async function scanWifi() {
  try {
    const resp = await api('/api/wifi/scan');
    const box = q('#wifiList');
    box.innerHTML = '';

    if (!resp.networks || resp.networks.length === 0) {
      box.innerHTML = '<div class="wifi-item muted">Keine Netzwerke gefunden.</div>';
      return;
    }

    resp.networks.forEach(n => {
      const row = document.createElement('div');
      row.className = 'wifi-item';
      const meta = `${n.encrypted ? 'verschluesselt' : 'offen'} | RSSI ${n.rssi}`;
      row.innerHTML =
        `<div><div>${n.ssid || '(versteckt)'}</div><div class="muted">${meta}</div></div>` +
        `<button class="alt" type="button">Auswaehlen</button>`;
      row.querySelector('button').addEventListener('click', () => {
        q('#ssid').value = n.ssid || '';
        setTab('wifi');
      });
      box.appendChild(row);
    });

    log(`${resp.networks.length} WLAN-Netze gefunden`);
  } catch (e) {
    log(e.message);
  }
}

async function startStream() {
  try {
    const i = encodeURIComponent(q('#streamInterval').value || '500');
    const s = encodeURIComponent(q('#streamSamples').value || '5');
    await api('/api/stream/start?interval=' + i + '&samples=' + s);
    log('Stream gestartet');
    await refreshStatus();
  } catch (e) {
    log(e.message);
  }
}

async function stopStream() {
  try {
    await api('/api/stream/stop');
    log('Stream gestoppt');
    await refreshStatus();
  } catch (e) {
    log(e.message);
  }
}

async function doReset() {
  try {
    await api('/api/reset');
    log('Reset ausgefuehrt');
    await refreshAll();
  } catch (e) {
    log(e.message);
  }
}

let _otaFlashUrl = null;

async function checkUpdate() {
  log('Suche nach Updates...');
  try {
    const vRes = await api('/api/version');
    const cur = vRes.version || 'unbekannt';
    document.getElementById('fwCurrent').textContent = 'Installiert: ' + cur;
    const ghRes = await fetch('https://api.github.com/repos/Peschi90/Slotcar-Magnet-Scale/releases/latest');
    if (!ghRes.ok) throw new Error('GitHub nicht erreichbar (HTTP ' + ghRes.status + ')');
    const rel = await ghRes.json();
    const latest = rel.tag_name || '';
    const asset = (rel.assets || []).find(a => a.name === 'firmware.bin');
    const box = document.getElementById('updateBox');
    const txt = document.getElementById('updateText');
    const actions = document.getElementById('updateActions');
    box.style.display = '';
    if (!asset) {
      txt.textContent = 'Kein firmware.bin im Release ' + latest + ' gefunden.';
      actions.style.display = 'none';
      return;
    }
    _otaFlashUrl = asset.browser_download_url;
    if (latest === cur) {
      txt.textContent = 'Firmware ist aktuell (' + cur + ').';
      actions.style.display = 'none';
    } else {
      txt.innerHTML = 'Neue Version verfuegbar: <strong>' + latest + '</strong> (aktuell: ' + cur + ')';
      actions.style.display = '';
      document.getElementById('flashBtn').disabled = false;
    }
    log('Neueste Version: ' + latest + ' | Aktuell: ' + cur);
  } catch (e) {
    log('Update-Pruefung fehlgeschlagen: ' + e.message);
  }
}

async function doOtaFlash() {
  if (!_otaFlashUrl) return;
  if (!confirm('Firmware flashen? Das Geraet startet nach dem Update neu.')) return;
  const btn = document.getElementById('flashBtn');
  btn.disabled = true;
  log('Starte OTA-Update...');
  try {
    const r = await fetch('/api/ota/flash?url=' + encodeURIComponent(_otaFlashUrl));
    const d = await r.json();
    if (d.ok) {
      document.getElementById('updateText').textContent = 'Update laeuft... Bitte ca. 30 s warten, dann Seite neu laden.';
      log('OTA gestartet – Geraet wird neu gestartet.');
    } else {
      log('OTA Fehler: ' + (d.error || 'unbekannt'));
      btn.disabled = false;
    }
  } catch (e) {
    log('OTA Fehler: ' + e.message);
    btn.disabled = false;
  }
}

window.addEventListener('hashchange', applyHashTab);
applyHashTab();
refreshAll();
scanWifi();
api('/api/version').then(v => {
  document.getElementById('fwCurrent').textContent = 'Installiert: ' + (v.version || 'unbekannt');
}).catch(() => {});
setInterval(readNow, 1800);
setInterval(refreshStatus, 5000);
</script>
</body>
</html>
)HTML";

  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.send_P(200, PSTR("text/html"), PAGE);
}

void handleCalibrationPage() {
  server.sendHeader("Location", "/#calibration");
  server.send(302, "text/plain", "Redirect");
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRootPage);
  server.on("/favicon.png", HTTP_GET, handleFavicon);
  server.on("/kalibrierung", HTTP_GET, handleCalibrationPage);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/read", HTTP_GET, handleApiRead);
  server.on("/api/raw", HTTP_GET, handleApiRaw);
  server.on("/api/tare", HTTP_GET, handleApiTare);
  server.on("/api/cal/start", HTTP_GET, handleApiCalStart);
  server.on("/api/cal/point", HTTP_GET, handleApiCalPoint);
  server.on("/api/cal/list", HTTP_GET, handleApiCalList);
  server.on("/api/cal/report", HTTP_GET, handleApiCalReport);
  server.on("/api/cal/clear", HTTP_GET, handleApiCalClear);
  server.on("/api/cal/abort", HTTP_GET, handleApiCalAbort);
  server.on("/api/cal/calculate", HTTP_GET, handleApiCalCalculate);
  server.on("/api/stream/start", HTTP_GET, handleApiStreamStart);
  server.on("/api/stream/stop", HTTP_GET, handleApiStreamStop);
  server.on("/api/factors", HTTP_GET, handleApiSetFactors);
  server.on("/api/wifi/save", HTTP_GET, handleApiWifiSave);
  server.on("/api/wifi/clear", HTTP_GET, handleApiWifiClear);
  server.on("/api/wifi/scan", HTTP_GET, handleApiWifiScan);
  server.on("/api/reset", HTTP_GET, handleApiReset);
  server.on("/api/version", HTTP_GET, handleApiVersion);
  server.on("/api/ota/flash", HTTP_GET, handleApiOtaFlash);
  server.onNotFound([]() { sendJson(404, "{\"ok\":false,\"error\":\"Not found\"}"); });
  server.begin();
  Serial.println(F("OK: Webserver gestartet."));
}

void processCommand(const String &line) {
  String cmdLine = line;
  cmdLine.trim();

  if (cmdLine.length() == 0) {
    return;
  }

  String command;
  String args;
  splitFirstToken(cmdLine, command, args);
  command.toUpperCase();

  if (command == "HILFE") {
    printHelp();
    return;
  }

  if (command == "STATUS") {
    printStatus();
    return;
  }

  if (command == "TARA") {
    bool ok = true;
    const uint8_t samples = parseSamples(args, DEFAULT_SAMPLES, ok);
    if (!ok) {
      return;
    }
    doTare(samples);
    return;
  }

  if (command == "KAL_START") {
    float known = 0.0f;
    if (!parseFloatLoose(args, known) || known <= 0.0f) {
      Serial.println(F("FEHLER: Ungueltiges Referenzgewicht. Beispiel: KAL_START 1000"));
      return;
    }
    startCalibrationSession(known);
    return;
  }

  if (command == "KAL_PUNKT") {
    String name;
    uint8_t samples = DEFAULT_SAMPLES;
    if (!parseNameAndOptionalSamples(args, name, samples)) {
      Serial.println(F("FEHLER: Syntax: KAL_PUNKT <name> [messungen]"));
      return;
    }
    addCalibrationPoint(name, samples);
    return;
  }

  if (command == "KAL_LISTE") {
    listCalibrationPoints();
    return;
  }

  if (command == "KAL_LOESCHEN") {
    clearCalibrationPoints();
    return;
  }

  if (command == "KAL_BERECHNEN") {
    calculateCalibration();
    return;
  }

  if (command == "KAL_ABBRUCH") {
    abortCalibrationSession();
    return;
  }

  if (command == "MESSEN") {
    bool ok = true;
    const uint8_t samples = parseSamples(args, DEFAULT_SAMPLES, ok);
    if (!ok) {
      return;
    }
    printMeasurement(samples);
    return;
  }

  if (command == "ROH") {
    bool ok = true;
    const uint8_t samples = parseSamples(args, DEFAULT_SAMPLES, ok);
    if (!ok) {
      return;
    }
    printRawAndNet(samples);
    return;
  }

  if (command == "START") {
    String arg1;
    String rest;
    splitFirstToken(args, arg1, rest);

    uint32_t interval = streamIntervalMs;
    uint8_t samples = streamSamples;

    if (arg1.length() > 0) {
      uint32_t parsedInterval = 0;
      if (!parseUIntLoose(arg1, parsedInterval) || parsedInterval < MIN_STREAM_INTERVAL_MS ||
          parsedInterval > MAX_STREAM_INTERVAL_MS) {
        Serial.printf("FEHLER: Intervall muss zwischen %lu und %lu ms liegen.\n",
                      static_cast<unsigned long>(MIN_STREAM_INTERVAL_MS),
                      static_cast<unsigned long>(MAX_STREAM_INTERVAL_MS));
        return;
      }
      interval = parsedInterval;
    }

    if (rest.length() > 0) {
      bool ok = true;
      samples = parseSamples(rest, streamSamples, ok);
      if (!ok) {
        return;
      }
    }

    if (!hasTare) {
      Serial.println(F("FEHLER: Keine Tara vorhanden."));
      return;
    }
    if (!hasCalibration) {
      Serial.println(F("FEHLER: Keine gueltige Kalibrierung vorhanden."));
      return;
    }

    streamIntervalMs = interval;
    streamSamples = samples;
    streamEnabled = true;
    lastStreamAtMs = millis() - streamIntervalMs;

    Serial.printf("OK: Stream gestartet (%lu ms, %u Messungen).\n",
                  static_cast<unsigned long>(streamIntervalMs), streamSamples);
    return;
  }

  if (command == "STOP") {
    streamEnabled = false;
    Serial.println(F("OK: Stream gestoppt."));
    return;
  }

  if (command == "INTERVALL") {
    uint32_t interval = 0;
    if (!parseUIntLoose(args, interval) || interval < MIN_STREAM_INTERVAL_MS ||
        interval > MAX_STREAM_INTERVAL_MS) {
      Serial.printf("FEHLER: Intervall muss zwischen %lu und %lu ms liegen.\n",
                    static_cast<unsigned long>(MIN_STREAM_INTERVAL_MS),
                    static_cast<unsigned long>(MAX_STREAM_INTERVAL_MS));
      return;
    }

    streamIntervalMs = interval;
    Serial.printf("OK: Intervall gesetzt auf %lu ms.\n", static_cast<unsigned long>(streamIntervalMs));
    return;
  }

  if (command == "FAKTOREN") {
    String leftText;
    String rightText;
    splitFirstToken(args, leftText, rightText);

    float left = 0.0f;
    float right = 0.0f;
    if (!parseFloatLoose(leftText, left) || !parseFloatLoose(rightText, right) || !isfinite(left) ||
        !isfinite(right)) {
      Serial.println(F("FEHLER: Syntax: FAKTOREN <faktor_links> <faktor_rechts>"));
      return;
    }

    factorLeft = left;
    factorRight = right;
    hasCalibration = true;

    Serial.printf("OK: Faktoren gesetzt. links=%.9f rechts=%.9f\n", factorLeft, factorRight);
    return;
  }

  if (command == "WLAN") {
    String ssid;
    String password;
    splitFirstToken(args, ssid, password);
    ssid.trim();

    if (ssid.length() == 0 || ssid.length() > 32) {
      Serial.println(F("FEHLER: WLAN-Syntax: WLAN <ssid> <passwort> (SSID: 1..32 Zeichen)."));
      return;
    }

    if (password.length() > 64) {
      Serial.println(F("FEHLER: Passwort darf maximal 64 Zeichen haben."));
      return;
    }

    storedWifiSsid = ssid;
    storedWifiPassword = password;
    wifiConfigured = true;
    writeConfigToEeprom();

    const bool connected = connectToConfiguredWifi();
    if (!connected) {
      ensureFallbackAccessPoint();
    }
    printStatus();
    return;
  }

  if (command == "WLAN_LOESCHEN") {
    storedWifiSsid = "";
    storedWifiPassword = "";
    wifiConfigured = false;
    stationConnected = false;
    writeConfigToEeprom();

    WiFi.disconnect();
    ensureFallbackAccessPoint();
    Serial.println(F("OK: WLAN-Zugangsdaten geloescht. AP-Backup ist aktiv."));
    return;
  }

  if (command == "SPEICHERN") {
    writeConfigToEeprom();
    return;
  }

  if (command == "RESET") {
    doReset();
    return;
  }

  Serial.printf("FEHLER: Unbekannter Befehl: %s\n", command.c_str());
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());

    if (ch == '\r' || ch == '\n') {
      if (serialOverflow) {
        Serial.println(F("FEHLER: Serielle Eingabe zu lang. Zeile verworfen."));
        serialOverflow = false;
        serialLen = 0;
      } else if (serialLen > 0) {
        serialBuffer[serialLen] = '\0';
        processCommand(String(serialBuffer));
        serialLen = 0;
      }
      continue;
    }

    if (serialOverflow) {
      continue;
    }

    if (serialLen >= MAX_SERIAL_LINE) {
      serialOverflow = true;
      continue;
    }

    serialBuffer[serialLen++] = ch;
  }
}

void handleStreaming() {
  if (!streamEnabled) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastStreamAtMs < streamIntervalMs) {
    return;
  }

  lastStreamAtMs = now;
  printStreamLine(streamSamples);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(40);

  Serial.println();
  Serial.println(F("Slotcar Magnet Scale - Dual HX711"));

  EEPROM.begin(EEPROM_SIZE);
  if (loadConfigFromEeprom()) {
    Serial.println(F("OK: Konfiguration aus EEPROM geladen."));
  } else {
    Serial.println(F("INFO: Keine gueltige Konfiguration gefunden. Standardwerte aktiv."));
  }

  scaleLeft.begin(HX711_LEFT_DOUT_PIN, HX711_LEFT_SCK_PIN);
  scaleRight.begin(HX711_RIGHT_DOUT_PIN, HX711_RIGHT_SCK_PIN);

  Serial.println(F("Sensoren initialisiert."));

  const bool wifiConnected = connectToConfiguredWifi();
  if (!wifiConnected) {
    ensureFallbackAccessPoint();
  }

  setupWebServer();

  printHelp();
  printStatus();
}

void loop() {
  pollSerialCommands();
  server.handleClient();
  serviceWifi();
  handleStreaming();
}
