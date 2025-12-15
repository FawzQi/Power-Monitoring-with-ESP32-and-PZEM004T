#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <HardwareSerial.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#define CURRENT_VERSION "1.3.0"
#define RELAY_PIN 13

int state_LED = 0;

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
const char* mqtt_server1 = "10.220.4.77";   // GANTI DENGAN IP SERVER LAB ANDA
const char* mqtt_server2 = "10.220.4.209";  // Public Broker (sebagai cadangan)
const int mqtt_port = 1883;
const char* mqtt_topic = "lab/pzem/data";  // Topic yang akan di-subscribe Node-RED
const char* mqtt_topic_sub = "lab/pzem/control/relay";

WiFiClient espClient1;
PubSubClient mqttClient(espClient1);

WiFiClient espClient2;
PubSubClient mqttClient2(espClient2);

unsigned long lastRead = 0;
const int readInterval = 200;  // Interval 200 ms

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
    while (!mqttClient.connected() && !mqttClient2.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (!mqttClient.connected()) {                      // Coba koneksi
            if (mqttClient.connect("esp32-pzem-client")) {  // ID Klien (bisa apa saja)
                Serial.println("connected!");
            } else {
                Serial.print("failed, rc=");
                Serial.print(mqttClient.state());
                Serial.println(" try again in 5 seconds");
            }
            mqttClient.subscribe(mqtt_topic_sub);
        }

        if (!mqttClient2.connected()) {
            if (mqttClient2.connect("esp32-pzem-client-2")) {  // ID Klien (bisa apa saja)
                Serial.println("connected to second broker!");
            } else {
                Serial.print("failed to connect to second broker, rc=");
                Serial.print(mqttClient2.state());
                Serial.println(" try again in 5 seconds");
            }
            mqttClient2.subscribe(mqtt_topic_sub);
        }
        delay(5000);
    }
    state_LED = 3;  // LED menyala menandakan koneksi MQTT berhasil
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
    uint16_t d[12];                              // cukup besar
    pzemSerial.begin(9600, SERIAL_8N1, 26, 27);  // RX=26, TX=27
    Serial.println("PZEM Serial started.");
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

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void HandleSendDataPZEM() {
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
            // // Tampilkan di Serial (untuk debugging)
            // Serial.println("PZEM Measurements:");
            // Serial.printf("Voltage: %.2f V\n", voltage);
            // Serial.printf("Current: %.3f A\n", current);
            // Serial.printf("Power: %.2f W\n", power);
            // Serial.printf("Energy: %.3f kWh\n", energy);
            // Serial.printf("Frequency: %.1f Hz\n", frequency);
            // Serial.printf("Power Factor: %.2f\n", pf);
            // Serial.println("---------------------------");

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
            if (mqttClient.connected()) {
                if (mqttClient.publish(mqtt_topic, json_payload)) {
                    // Serial.println("MQTT Data Published!");
                } else {
                    // Serial.println("MQTT Publish Failed!");
                }
            }

            if (mqttClient2.connected()) {
                if (mqttClient2.publish(mqtt_topic, json_payload)) {
                    // Serial.println("MQTT Data Published to second broker!");
                } else {
                    // Serial.println("MQTT Publish to second broker Failed!");
                }
            }
            Serial.println();
        }
    }
}

// Pastikan library ArduinoJson sudah di-include di paling atas
// #include <ArduinoJson.h>

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("DARURAT! Pesan masuk dari topik: ");
    Serial.println(topic);

    // Cek apakah ini topik kontrol relay
    // (Sesuaikan dengan konstanta mqtt_topic_sub Anda)
    if (strcmp(topic, "lab/pzem/control/relay") == 0) {
        // 1. Siapkan container JSON (Ukuran kecil cukup untuk command)
        StaticJsonDocument<256> doc;

        // 2. Deserialize (Ubah byte payload langsung jadi JSON)
        DeserializationError error = deserializeJson(doc, payload, length);

        if (error) {
            Serial.print(F("Gagal baca JSON perintah: "));
            Serial.println(error.f_str());

            // FALLBACK: Cek manual jika Node-RED mengirim raw string "0" atau "1"
            // Jaga-jaga kalau Anda mengubah node-red jadi simple string
            if ((char)payload[0] == '0') {
                digitalWrite(RELAY_PIN, LOW);
                Serial.println(">> RELAY OFF (Manual Fallback)");
            }
            return;
        }

        // 3. Ambil data dari JSON
        // Kita pakai default -1 jika field 'command' tidak ada
        int cmd = doc["command"] | -1;
        const char* alertMsg = doc["alert"];  // Ambil pesan alert juga buat log

        if (alertMsg) {
            Serial.print("Pesan Alert: ");
            Serial.println(alertMsg);
        }

        // 4. Eksekusi Perintah
        if (cmd == 0) {
            digitalWrite(RELAY_PIN, LOW);  // MATIKAN RELAY (Self Destruct prevention)
            Serial.println(">> VOLTAGE ANOMALY DETECTED -> RELAY DIMATIKAN!");
        } else if (cmd == 1) {
            digitalWrite(RELAY_PIN, HIGH);
            Serial.println(">> System Normal -> Relay ON");
        }
    }
}

void TaskMQTT(void* pvParameters) {
    // Set server MQTT
    mqttClient.setServer(mqtt_server1, mqtt_port);
    mqttClient2.setServer(mqtt_server2, mqtt_port);
    Serial.println("Connecting to MQTT Broker...");
    reconnect_mqtt();
    Serial.println("MQTT connected!");
    state_LED = 3;  // LED menyala menandakan semua siap
    mqttClient.setCallback(callback);
    mqttClient2.setCallback(callback);

    while (true) {
        if (!mqttClient.connected() || !mqttClient2.connected()) {
            state_LED = 2;  // LED berkedip menandakan koneksi MQTT sedang dicoba
            reconnect_mqtt();
            state_LED = 3;  // LED menyala menandakan koneksi MQTT berhasil
        }
        mqttClient.loop();  // Wajib dipanggil di loop untuk proses MQTT
        mqttClient2.loop();

        HandleSendDataPZEM();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void taskLED(void* pvParameters) {
    pinMode(2, OUTPUT);
    while (true) {
        if (state_LED == 0) {
            digitalWrite(2, LOW);
        } else if (state_LED == 1) {
            digitalWrite(2, HIGH);
            vTaskDelay(pdMS_TO_TICKS(500));
            digitalWrite(2, LOW);
            vTaskDelay(pdMS_TO_TICKS(500));
        } else if (state_LED == 2) {
            digitalWrite(2, HIGH);
            vTaskDelay(pdMS_TO_TICKS(100));
            digitalWrite(2, LOW);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else if (state_LED == 3) {
            digitalWrite(2, HIGH);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("==============================");
    Serial.println("Starting ESP32 with OTA Update");
    Serial.println("==============================");

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);
    Serial.println("RELAY initialized to ON state.");

    xTaskCreate(taskLED, "LED Task", 4096, NULL, 1, NULL);

    // Koneksi WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi ");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println();
    Serial.println("WiFi connected!");
    state_LED = 1;  // LED berkedip menandakan koneksi WiFi berhasil

    // Cek OTA update
    checkForOTAUpdate();
    state_LED = 2;  // Matikan LED setelah cek OTA
    xTaskCreatePinnedToCore(TaskMQTT, "MQTT Task", 8192, NULL, 1, NULL, 0);

    xTaskCreatePinnedToCore(taskPZEM, "PZEM Task", 4096, NULL, 1, NULL, 1);
}

void loop() {
}
