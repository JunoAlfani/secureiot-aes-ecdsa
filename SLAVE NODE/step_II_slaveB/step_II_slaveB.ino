// ===============================
// ✅ ESP8266 Slave Node B – Tahap 2 (Sensor Asli + Kirim JSON)
// ===============================
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <SHA256.h>
#include <DHT.h>
#include <ArduinoJson.h>

extern "C" {
  #include "uECC.h"
}

#define DHTPIN 4
#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);

const char* ssid = "VLAN30_Access";
const char* password = "vlan3012345";
const char* esp32_host = "192.168.30.247";
const uint16_t esp32_port = 8888;

#define EEPROM_SIZE 512
const char* device_id = "1"; // IDENTITAS SLAVE NODE DARI 4-6

uint8_t privateKey[21];
uint8_t publicKey[40];
uint8_t signature[40];

int analogInPin = A0;
float kalibrasi = 0.36;

void readFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  int addr = 0;
  int i = 0;
  while (i < sizeof(device_id)) {
    char c = EEPROM.read(addr++);
    if (c == '\0') break;
    i++;
  }
  for (int j = 0; j < sizeof(privateKey); j++) privateKey[j] = EEPROM.read(addr++);
  for (int j = 0; j < sizeof(publicKey); j++) publicKey[j] = EEPROM.read(addr++);
  for (int j = 0; j < sizeof(signature); j++) signature[j] = EEPROM.read(addr++);
  EEPROM.end();
}

String bytesToHexString(uint8_t* bytes, int len) {
  String hex = "";
  for (int i = 0; i < len; i++) {
    if (bytes[i] < 16) hex += "0";
    hex += String(bytes[i], HEX);
  }
  return hex;
}

void setup() {
  Serial.begin(115200);
  dht.begin();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi terhubung ke VLAN");

  readFromEEPROM();
  Serial.println("✅ Keypair dan Signature berhasil dibaca dari EEPROM");

  // Baca data sensor
  float temperature = round(dht.readTemperature() * 100) / 100.0;
  float humidity = round(dht.readHumidity() * 100) / 100.0;

  int sensorValue = analogRead(analogInPin);
  float tegangan = (((sensorValue * 3.3) / 1024) * 2 + kalibrasi);
  int batteryPercentage = map(tegangan * 100, 290, 400, 0, 100);
  batteryPercentage = constrain(batteryPercentage, 0, 100);

  unsigned long timestamp = millis(); // Jika pakai NTP bisa diganti waktu UTC

  // Format data menjadi JSON
  StaticJsonDocument<512> doc;
  doc["id"] = device_id;
  doc["pubkey"] = bytesToHexString(publicKey, sizeof(publicKey));
  doc["signature"] = bytesToHexString(signature, sizeof(signature));
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["battery"] = batteryPercentage;
  doc["timestamp"] = timestamp;

  String json;
  serializeJson(doc, json);

  // Kirim ke Master
  WiFiClient client;
  if (client.connect(esp32_host, esp32_port)) {
    Serial.println("📡 Mengirim data ke ESP32 Master...");
    client.println(json);
    client.stop();
    Serial.println("✅ Data berhasil dikirim.");
  } else {
    Serial.println("❌ Gagal koneksi ke ESP32 Master.");
  }
}

void loop() {
  // Bisa ditambahkan pengulangan berkala jika diperlukan
}
