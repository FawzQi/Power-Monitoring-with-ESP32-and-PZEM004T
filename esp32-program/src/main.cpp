#include <Arduino.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <PZEM004Tv30.h>
#include <WiFi.h>

// ======================
// Konfigurasi WiFi
// ======================
const char* ssid = "b401_wiFi";
const char* password = "b401juara1";

// ======================
// OTA Configuration
// ======================
const char* baseUrl = "https://raw.githubusercontent.com/FawzQi/Power-Monitoring-with-ESP32-and-PZEM004T/main/firmware/";
String currentVersion = "1.0.1";  // versi lokal saat ini

// ======================
// Konfigurasi PZEM
// ======================
#if defined(ESP32)
PZEM004Tv30 pzem(Serial2, 16, 17);  // RX=16, TX=17
#else
PZEM004Tv30 pzem(Serial2);
#endif

// ======================
// Fungsi OTA Check
// ======================
void checkForOTAUpdate() {
    Serial.println("Checking for OTA updates...");

    String versionUrl = String(baseUrl) + "version.txt";
    HTTPClient http;
    http.begin(versionUrl);
    int httpCode = http.GET();

    if (httpCode == 200) {
        String latestVersion = http.getString();
        latestVersion.trim();

        Serial.println("Current version : " + currentVersion);
        Serial.println("Latest version  : " + latestVersion);

        if (latestVersion != currentVersion) {
            Serial.println("⚙️  New version available! Starting OTA update...");
            String firmwareUrl = String(baseUrl) + "firmware_v" + latestVersion + ".bin";

            WiFiClient client;
            t_httpUpdate_return ret = httpUpdate.update(client, firmwareUrl);

            if (ret == HTTP_UPDATE_OK) {
                Serial.println("✅ OTA Update successful!");
            } else {
                Serial.printf("❌ OTA Update failed! Error (%d): %s\n",
                              httpUpdate.getLastError(),
                              httpUpdate.getLastErrorString().c_str());
            }
        } else {
            Serial.println("Firmware is up to date.");
        }
    } else {
        Serial.println("⚠️ Failed to fetch version.txt (HTTP code: " + String(httpCode) + ")");
    }

    http.end();
    Serial.println();
}

// ======================
// Setup
// ======================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("==============================");
    Serial.println("Starting ESP32 with OTA + PZEM");
    Serial.println("==============================");

    // Koneksi WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi ");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println("\n✅ WiFi Connected!");

    // Cek OTA update sebelum mulai loop utama
    checkForOTAUpdate();
}

// ======================
// Loop utama: baca data PZEM
// ======================
void loop() {
    float voltage = pzem.voltage();
    float current = pzem.current();
    float power = pzem.power();
    float energy = pzem.energy();
    float frequency = pzem.frequency();
    float pf = pzem.pf();

    if (isnan(voltage) || isnan(current) || isnan(power) || isnan(energy) || isnan(frequency) || isnan(pf)) {
        Serial.println("⚠️  Error reading from PZEM sensor!");
    } else {
        Serial.println("📊  PZEM Measurements:");
        Serial.printf("Voltage: %.2f V\n", voltage);
        Serial.printf("Current: %.3f A\n", current);
        Serial.printf("Power: %.2f W\n", power);
        Serial.printf("Energy: %.3f kWh\n", energy);
        Serial.printf("Frequency: %.1f Hz\n", frequency);
        Serial.printf("Power Factor: %.2f\n", pf);
        Serial.println();
    }

    delay(2000);
}
