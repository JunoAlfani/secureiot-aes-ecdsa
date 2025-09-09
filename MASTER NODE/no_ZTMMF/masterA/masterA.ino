#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* ssid = "Jeisika Eugenia purba";
const char* password = "0407Adrico";
const char* mqttServer = "192.168.1.8";
const int   mqttPort   = 1883;
const char* mqttTopic  = "sensor/data";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WiFiServer tcpServer(8888);

// Unique MQTT client ID
String clientId = "ESP32_TCP_MQTT_" + String((uint32_t)ESP.getEfuseMac(), HEX);

void connectToMQTT() {
  if (mqttClient.connected()) return;

  Serial.print("[MQTT] Menghubungkan... ID: ");
  Serial.println(clientId);

  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("✅ MQTT terhubung.");
  } else {
    Serial.print("❌ MQTT gagal, kode: ");
    Serial.println(mqttClient.state());
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n✅ WiFi terhubung ke jaringan.");
  Serial.println(WiFi.localIP());

  mqttClient.setServer(mqttServer, mqttPort);
  tcpServer.begin();
}

void loop() {
  connectToMQTT();
  mqttClient.loop();

  WiFiClient client = tcpServer.available();
  if (client) {
    Serial.println("\n[ESP32] Koneksi TCP diterima");
    String jsonPayload = client.readStringUntil('\n');
    client.stop();

    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, jsonPayload);
    if (err) {
      Serial.println("[!] JSON tidak valid:");
      Serial.println(jsonPayload);
      return;
    }

    Serial.println("[TCP] Data diterima:");
    Serial.println(jsonPayload);

    if (mqttClient.publish(mqttTopic, jsonPayload.c_str())) {
      Serial.println("[MQTT] ✅ Data dikirim ke broker.");
    } else {
      Serial.println("[MQTT] ❌ Gagal publish.");
    }
  }

  delay(50); // Hindari loop terlalu agresif
}
