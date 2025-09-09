// ===============================
// ✅ ESP8266-1 Slave Node – Tahap 2 (Final: EEPROM + Sensor Asli + Interval 5 Menit)
// ===============================
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <SHA256.h>
#include <DHT.h>
#include <ArduinoJson.h>

extern "C" {
  #include "uECC.h"
}

// === Konfigurasi Sensor & Jaringan ===
#define DHTPIN 4
#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);

const char* ssid = "VLAN20_Access";
const char* password = "vlan2012345";
const char* esp32_host = "192.168.20.5";  // Ganti sesuai IP ESP32 Master
const uint16_t esp32_port = 8888;

#define EEPROM_SIZE 512

char device_id[32];
uint8_t privateKey[21];
uint8_t publicKey[40];
uint8_t signature[40];

int analogInPin = A0;
float kalibrasi = 0.36;

// === Timer Interval (5 menit) ===
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5 * 60 * 1000;

// === Util Fungsi ===
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
}

void loop() {
  if (millis() - lastSendTime >= sendInterval) {
    lastSendTime = millis();

    // Baca data sensor
    float temperature = round(dht.readTemperature() * 100) / 100.0;
    float humidity = round(dht.readHumidity() * 100) / 100.0;

    int sensorValue = analogRead(analogInPin);
    float tegangan = (((sensorValue * 3.3) / 1024) * 2 + kalibrasi);
    int batteryPercentage = map(tegangan * 100, 290, 400, 0, 100);
    batteryPercentage = constrain(batteryPercentage, 0, 100);

    unsigned long timestamp = millis();  // Ganti dengan waktu NTP jika diperlukan

    String pubkey_str = bytesToHexString(publicKey, sizeof(publicKey));
    String sig_str = bytesToHexString(signature, sizeof(signature));

    // Buat JSON payload
    String json = "{";
    json += "\"id\":\"" + String(device_id) + "\",";
    json += "\"pubkey\":\"" + pubkey_str + "\",";
    json += "\"signature\":\"" + sig_str + "\",";
    json += "\"temperature\":" + String(temperature, 1) + ",";
    json += "\"humidity\":" + String(humidity, 1) + ",";
    json += "\"battery\":" + String(batteryPercentage) + ",";
    json += "\"timestamp\":" + String(timestamp);
    json += "}";

    Serial.println("[ESP8266] JSON Payload:");
    Serial.println(json);

    // Kirim ke ESP32 Master
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

  delay(100);  // Jaga loop tetap ringan
}
