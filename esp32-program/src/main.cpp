#include <Arduino.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <PZEM004Tv30.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ======================
// Konfigurasi WiFi
// ======================
const char* ssid = "b401_wifi";
const char* password = "b401juara1";

// ======================
// OTA Configuration
// ======================
const char* baseUrl = "https://raw.githubusercontent.com/FawzQi/Power-Monitoring-with-ESP32-and-PZEM004T/main/firmware/";
String currentVersion = "1.0.2";  // versi lokal saat ini

// ======================
// Konfigurasi PZEM
// ======================
#if defined(ESP32)
PZEM004Tv30 pzem(Serial2, 16, 17);  // RX=16, TX=17
#else
PZEM004Tv30 pzem(Serial2);
#endif

// === TAMBAHAN: Konfigurasi MQTT ===
const char* mqtt_server = "192.168.200.245"; // ⚠️ GANTI DENGAN IP SERVER LAB ANDA
const int mqtt_port = 1883;
const char* mqtt_topic = "lab/pzem/data"; // Topic yang akan di-subscribe Node-RED

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// === TAMBAHAN: Timer untuk non-blocking ===
unsigned long lastRead = 0;
const int readInterval = 2000; // Interval 2 detik (sama seperti delay Anda)

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

            WiFiClientSecure client;
            client.setInsecure();

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

// === TAMBAHAN: Fungsi untuk reconnect MQTT ===
void reconnect_mqtt() {
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Coba koneksi
        if (mqttClient.connect("esp32-pzem-client")) { // ID Klien (bisa apa saja)
            Serial.println("connected!");
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

// ======================
// Setup
// ======================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("==============================");
    Serial.println("Starting ESP32 with OTA Update");
    Serial.println("==============================");

    // Koneksi WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi ");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println("\n✅ WiFi Connected!");
    Serial.println("IP Address: " + WiFi.localIP().toString());


    // Cek OTA update
    checkForOTAUpdate();

    // === TAMBAHAN: Set server MQTT ===
    mqttClient.setServer(mqtt_server, mqtt_port);
    Serial.println("Connecting to MQTT Broker...");
    reconnect_mqtt();
}

// ======================
// Loop utama: baca data PZEM & Publish MQTT
// ======================
void loop() {
    // === TAMBAHAN: Pastikan koneksi MQTT selalu terjaga ===
    if (!mqttClient.connected()) {
        reconnect_mqtt();
    }
    mqttClient.loop(); // Wajib dipanggil di loop untuk proses MQTT

    // === TAMBAHAN: Mengganti delay(2000) dengan timer non-blocking ===
    unsigned long now = millis();
    if (now - lastRead > readInterval) {
        lastRead = now;

        float voltage = pzem.voltage();
        float current = pzem.current();
        float power = pzem.power();
        float energy = pzem.energy();
        float frequency = pzem.frequency();
        float pf = pzem.pf();

        if (isnan(voltage) || isnan(current) || isnan(power) || isnan(energy) || isnan(frequency) || isnan(pf)) {
            Serial.println("⚠️  Error reading from PZEM sensor!");
        } else {
            // Tampilkan di Serial (untuk debugging)
            Serial.println("📊  PZEM Measurements:");
            Serial.printf("Voltage: %.2f V\n", voltage);
            Serial.printf("Current: %.3f A\n", current);
            Serial.printf("Power: %.2f W\n", power);
            Serial.printf("Energy: %.3f kWh\n", energy);
            Serial.printf("Frequency: %.1f Hz\n", frequency);
            Serial.printf("Power Factor: %.2f\n", pf);

            // === TAMBAHAN: Buat JSON payload ===
            JsonDocument doc;
            doc["voltage"] = voltage;
            doc["current"] = current;
            doc["power"] = power;
            doc["energy"] = energy;
            doc["frequency"] = frequency;
            doc["power_factor"] = pf;

            char json_payload[256];
            serializeJson(doc, json_payload);

            // === TAMBAHAN: Publish ke MQTT Broker ===
            if (mqttClient.publish(mqtt_topic, json_payload)) {
                Serial.println("✅ MQTT Data Published!");
            } else {
                Serial.println("❌ MQTT Publish Failed!");
            }
            Serial.println();
        }
    }
    // Hapus delay(2000) dari sini
}