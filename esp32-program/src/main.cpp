#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <HardwareSerial.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#define CURRENT_VERSION "1.2.2"

// ======================
// Konfigurasi WiFi
// ======================
const char* ssid = "realme10";
const char* password = "paansih7";

// ======================
// OTA Configuration
// ======================
const char* baseUrl = "https://github.com/FawzQi/Power-Monitoring-with-ESP32-and-PZEM004T/releases/latest/download/";

// === TAMBAHAN: Konfigurasi MQTT ===
const char* mqtt_server = "192.168.200.245";  // GANTI DENGAN IP SERVER LAB ANDA
const int mqtt_port = 1883;
const char* mqtt_topic = "lab/pzem/data";  // Topic yang akan di-subscribe Node-RED

WiFiClient espClient;
PubSubClient mqttClient(espClient);

unsigned long lastRead = 0;
const int readInterval = 2000;  // Interval 2000 ms

HardwareSerial pzemSerial(2);  // UART2

typedef struct {
    float voltage;
    float current;
    float power;
    float energy;
    float frequency;
    float pf;
} PZEMData;

void checkForOTAUpdate() {
    Serial.print("Current: ");
    Serial.println(CURRENT_VERSION);
    Serial.println("Checking for OTA updates...");
    String versionUrl = String(baseUrl) + "version.json";
    HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.begin(versionUrl);
    int httpCode = http.GET();
    if (httpCode == 200) {
        String payload = http.getString();
        StaticJsonDocument<128> doc;
        deserializeJson(doc, payload);
        String latestVersion = doc["version"].as<String>();

        Serial.print("Latest: ");
        Serial.println(latestVersion);

        if (latestVersion != CURRENT_VERSION) {
            Serial.println("⚙️  New version available! Starting OTA update...");
            String firmwareUrl = String(baseUrl) + "firmware.bin";

            WiFiClientSecure client;
            client.setInsecure();

            httpUpdate.rebootOnUpdate(true);
            httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
            t_httpUpdate_return ret = httpUpdate.update(client, firmwareUrl);

            switch (ret) {
                case HTTP_UPDATE_FAILED:
                    Serial.printf("OTA Update failed! Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
                    break;

                case HTTP_UPDATE_NO_UPDATES:
                    Serial.println("No updates available.");
                    break;

                case HTTP_UPDATE_OK:
                    Serial.println("OTA Update successful!");
                    break;
            }
        } else {
            Serial.println("Firmware is up to date.");
        }
    } else {
        Serial.println("Failed to fetch version.txt (HTTP code: " + String(httpCode) + ")");
    }

    http.end();
    Serial.println();
}

// === TAMBAHAN: Fungsi untuk reconnect MQTT ===
void reconnect_mqtt() {
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Coba koneksi
        if (mqttClient.connect("esp32-pzem-client")) {  // ID Klien (bisa apa saja)
            Serial.println("connected!");
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

// ================= CRC =================
uint16_t crc16(uint8_t* buffer, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < length; pos++) {
        crc ^= buffer[pos];
        for (int i = 0; i < 8; i++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

// =============== READ ALL (6 registers) ===============
bool readAllPZEM(uint16_t* out) {
    uint8_t req[8];
    req[0] = 0x01;  // slave
    req[1] = 0x04;  // function code Read Input Registers
    req[2] = 0x00;  // start high
    req[3] = 0x00;  // start low (0x0000)
    req[4] = 0x00;  // length high
    req[5] = 0x09;  // read 9 registers !!!

    uint16_t crc = crc16(req, 6);
    req[6] = crc & 0xFF;  // CRC low
    req[7] = crc >> 8;    // CRC high

    pzemSerial.write(req, 8);
    delay(30);

    uint8_t resp[32];
    int expected = 5 + 9 * 2;  // 5 header + 18 bytes data = 23 bytes

    int len = pzemSerial.readBytes(resp, expected);
    if (len < expected) return false;

    if (resp[0] != 0x01 || resp[1] != 0x04) return false;
    if (resp[2] != 18) return false;  // byte count = 18

    // decode 9 register (18 bytes)
    for (int i = 0; i < 9; i++) {
        out[i] = (resp[3 + i * 2] << 8) | resp[4 + i * 2];
    }

    return true;
}

QueueHandle_t pzemQueue = xQueueCreate(1, sizeof(PZEMData));

void taskPZEM(void* pvParameters) {
    PZEMData pzemData;
    uint16_t d[12];  // cukup besar

    while (true) {
        if (readAllPZEM(d)) {
            // Voltage (16-bit)
            pzemData.voltage = d[0] / 10.0;

            // Current 32-bit
            uint32_t current_raw = ((uint32_t)d[2] << 16) | d[1];
            pzemData.current = current_raw / 1000.0;

            // Power 32-bit
            uint32_t power_raw = ((uint32_t)d[4] << 16) | d[3];
            pzemData.power = power_raw / 10.0;

            // Energy 32-bit
            uint32_t energy_raw = ((uint32_t)d[6] << 16) | d[5];
            pzemData.energy = energy_raw;

            // Frequency (16-bit)
            pzemData.frequency = d[7] / 10.0;

            // Power Factor (16-bit)
            pzemData.pf = d[8] / 100.0;

            xQueueOverwrite(pzemQueue, &pzemData);

            // Serial.println("PZEM Measurements:");
            // Serial.printf("Time        : %lu ms\n", millis());
            // Serial.printf("Voltage     : %.1f V\n", pzemData.voltage);
            // Serial.printf("Current     : %.3f A\n", pzemData.current);
            // Serial.printf("Power       : %.1f W\n", pzemData.power);
            // Serial.printf("Energy      : %.0f Wh\n", pzemData.energy);
            // Serial.printf("Frequency   : %.1f Hz\n", pzemData.frequency);
            // Serial.printf("PowerFactor : %.2f\n", pzemData.pf);
            // Serial.println("---------------------------");
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void taskPrintPZEM(void* pvParameters) {
    PZEMData pzemData;

    while (true) {
        if (xQueueReceive(pzemQueue, &pzemData, portMAX_DELAY) == pdTRUE) {
            Serial.println("PZEM Measurements:");
            Serial.printf("Time        : %lu ms\n", millis());
            Serial.printf("Voltage     : %.1f V\n", pzemData.voltage);
            Serial.printf("Current     : %.3f A\n", pzemData.current);
            Serial.printf("Power       : %.0f W\n", pzemData.power);
            Serial.printf("Energy      : %.0f Wh\n", pzemData.energy);
            Serial.printf("Frequency   : %.1f Hz\n", pzemData.frequency);
            Serial.printf("PowerFactor : %.2f\n", pzemData.pf);

            Serial.println("----------------------");
        }
    }
}

void setup() {
    Serial.begin(115200);
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
    Serial.println();
    Serial.println("WiFi connected!");

    // Cek OTA update
    checkForOTAUpdate();

    // Set server MQTT
    mqttClient.setServer(mqtt_server, mqtt_port);
    Serial.println("Connecting to MQTT Broker...");
    reconnect_mqtt();

    pzemSerial.begin(9600, SERIAL_8N1, 26, 27);  // RX=26, TX=27
    xTaskCreatePinnedToCore(taskPZEM, "PZEM Task", 4096, NULL, 1, NULL, 1);
}

void loop() {
    // === TAMBAHAN: Pastikan koneksi MQTT selalu terjaga ===
    if (!mqttClient.connected()) {
        reconnect_mqtt();
    }
    mqttClient.loop();  // Wajib dipanggil di loop untuk proses MQTT

    if (millis() - lastRead > readInterval) {
        lastRead = millis();

        PZEMData pzemData;
        if (xQueueReceive(pzemQueue, &pzemData, 0) == pdTRUE) {
            float voltage = pzemData.voltage;
            float current = pzemData.current;
            float power = pzemData.power;
            float energy = pzemData.energy;
            float frequency = pzemData.frequency;
            float pf = pzemData.pf;
            if (isnan(voltage) || isnan(current) || isnan(power) || isnan(energy) || isnan(frequency) || isnan(pf)) {
                Serial.println("Error reading from PZEM sensor!");
            } else {
                // Tampilkan di Serial (untuk debugging)
                Serial.println("PZEM Measurements:");
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
                    Serial.println("MQTT Data Published!");
                } else {
                    Serial.println("MQTT Publish Failed!");
                }
                Serial.println();
            }
        }
    }
}
