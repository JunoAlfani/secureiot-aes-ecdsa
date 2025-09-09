#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <SHA256.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

extern "C" {
  #include "uECC.h"
}

// === Konfigurasi Sensor & Jaringan ===
#define DHTPIN 4
#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);

const char* ssid = "VLAN30_Access";
const char* password = "vlan3012345";
const char* esp32_host = "192.168.30.247";
const uint16_t esp32_port = 8888;

#define EEPROM_SIZE 512

char device_id[32];
uint8_t privateKey[21];
uint8_t publicKey[40];
uint8_t signature[40];

int analogInPin = A0;
float kalibrasi = 0.36;

// Timer 3 menit
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 3 * 60 * 1000;

// NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // GMT+7

String bytesToHexString(uint8_t* bytes, int len) {
  String hex = "";
  for (int i = 0; i < len; i++) {
    if (bytes[i] < 16) hex += "0";
    hex += String(bytes[i], HEX);
  }
  return hex;
}

void loadFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  int addr = 0;
  for (int i = 0; i < sizeof(device_id); i++) {
    device_id[i] = EEPROM.read(addr++);
    if (device_id[i] == '\0') break;
  }
  for (int i = 0; i < sizeof(privateKey); i++) privateKey[i] = EEPROM.read(addr++);
  for (int i = 0; i < sizeof(publicKey); i++) publicKey[i] = EEPROM.read(addr++);
  for (int i = 0; i < sizeof(signature); i++) signature[i] = EEPROM.read(addr++);
  EEPROM.end();

  Serial.printf("[EEPROM] Device ID: %s\n", device_id);
  Serial.println("[EEPROM] PublicKey: " + bytesToHexString(publicKey, sizeof(publicKey)));
  Serial.println("[EEPROM] Signature: " + bytesToHexString(signature, sizeof(signature)));
}

void setup() {
  Serial.begin(115200);
  dht.begin();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi terhubung ke VLAN WiFi");

  loadFromEEPROM();

  timeClient.begin();
  while (!timeClient.update()) {
    timeClient.forceUpdate();
  }
}

void loop() {
  if (millis() - lastSendTime >= sendInterval) {
    lastSendTime = millis();

    float temperature = round(dht.readTemperature() * 100) / 100.0;
    float humidity = round(dht.readHumidity() * 100) / 100.0;

    int sensorValue = analogRead(analogInPin);
    float tegangan = (((sensorValue * 3.3) / 1024) * 2 + kalibrasi);
    int batteryPercentage = map(tegangan * 100, 290, 400, 0, 100);
    batteryPercentage = constrain(batteryPercentage, 0, 100);

    // Ambil waktu dari NTP
    unsigned long epoch = timeClient.getEpochTime();
    uint64_t ts = (uint64_t)epoch * 1000 + millis() % 1000;

    String pubkey_str = bytesToHexString(publicKey, sizeof(publicKey));
    String sig_str = bytesToHexString(signature, sizeof(signature));

    // Buat payload terenkripsi JSON
    StaticJsonDocument<256> doc;
    doc["id"] = device_id;
    doc["pubkey"] = pubkey_str;
    doc["signature"] = sig_str;
    doc["temperature"] = temperature;
    doc["humidity"] = humidity;
    doc["battery"] = batteryPercentage;
    doc["timestamp"] = ts;

    String json;
    serializeJson(doc, json);

    Serial.println("[ESP8266] JSON Payload:");
    Serial.println(json);

    WiFiClient client;
    if (client.connect(esp32_host, esp32_port)) {
      Serial.println("📡 Mengirim data ke ESP32 Master...");
      client.print(json);
      client.stop();
      Serial.println("✅ Data berhasil dikirim.");
    } else {
      Serial.println("❌ Gagal koneksi ke ESP32 Master.");
    }
  }

  delay(100);
}
