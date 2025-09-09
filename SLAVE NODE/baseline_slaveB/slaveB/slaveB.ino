#include <ESP8266WiFi.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#define DHTPIN 4
#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);

const char* ssid = "VLAN30_Access";
const char* password = "vlan3012345";
const char* esp32_host = "192.168.30.247";
const uint16_t esp32_port = 8888;

const char* device_id = "6";
int analogInPin = A0;
float kalibrasi = 0.36;
unsigned int packet_id = 1; // Bertambah setiap pengiriman

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 25200, 60000); // GMT+7

void setup() {
  Serial.begin(115200);
  dht.begin();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi terhubung ke VLAN");

  timeClient.begin();
  while (!timeClient.update()) {
    timeClient.forceUpdate();
  }
}

void loop() {
  float temperature = round(dht.readTemperature() * 100) / 100.0;
  float humidity = round(dht.readHumidity() * 100) / 100.0;

  int sensorValue = analogRead(analogInPin);
  float tegangan = (((sensorValue * 3.3) / 1024) * 2 + kalibrasi);
  int batteryPercentage = map(tegangan * 100, 290, 400, 0, 100);
  batteryPercentage = constrain(batteryPercentage, 0, 100);

  unsigned long epoch = timeClient.getEpochTime();         // detik
  unsigned long ts = epoch * 1000 + millis() % 1000;       // ms

  StaticJsonDocument<256> doc;
  doc["id"] = device_id;
  doc["packet_id"] = packet_id++;
  doc["temp"] = temperature;
  doc["hum"] = humidity;
  doc["batt"] = batteryPercentage;
  doc["ts"] = ts;

  String json;
  serializeJson(doc, json);

  WiFiClient client;
  if (client.connect(esp32_host, esp32_port)) {
    Serial.println("📡 Kirim data ke ESP32 Master...");
    client.println(json);
    client.stop();
    Serial.println("✅ Data terkirim: " + json);
  } else {
    Serial.println("❌ Gagal koneksi ke ESP32 Master.");
  }

  delay(5000); // Kirim setiap 5 detik
}
