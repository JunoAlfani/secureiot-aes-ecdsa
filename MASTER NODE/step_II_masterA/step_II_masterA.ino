// ===============================
// ✅ ESP32 B - Tahap 2: Verifikasi Signature + Enkripsi AES + Kirim ke MQTT (mbedtls version)
// ===============================
#include <WiFi.h>
#include <SHA256.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include "mbedtls/aes.h"
#include "mbedtls/base64.h"

extern "C" {
  #include "uECC.h"
}

const char* ssid = "VLAN20_Access";
const char* password = "vlan2012345";
WiFiServer server(8888);

#define EEPROM_SIZE 512
#define MQTT_SERVER "192.168.10.4"
#define MQTT_PORT 1883
#define MQTT_TOPIC "sensor/data"
#define BASELINE_TEST true


WiFiClient espClient;
PubSubClient mqttClient(espClient);

String whitelist[10];
String signatureList[10];
int whitelistCount = 0;

const char *aes_key_string = "0123456789ABCDEF";
uint8_t aes_key[16], aes_iv[16];

void hexStringToBytes(const String& hex, uint8_t* buffer, int len) {
  for (int i = 0; i < len; i++) {
    String byteString = hex.substring(i * 2, i * 2 + 2);
    buffer[i] = strtoul(byteString.c_str(), nullptr, 16);
  }
}

String hashToHex(uint8_t* hash, int len) {
  String hex = "";
  for (int i = 0; i < len; i++) {
    if (hash[i] < 16) hex += "0";
    hex += String(hash[i], HEX);
  }
  return hex;
}

bool isWhitelisted(String hashStr) {
  for (int i = 0; i < whitelistCount; i++) {
    if (whitelist[i] == hashStr) return true;
  }
  return false;
}

void loadWhitelistFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  int addr = 0;
  whitelistCount = 0;
  while (addr < EEPROM_SIZE && whitelistCount < 10) {
    String pubkeyHash = "";
    char c;
    while (addr < EEPROM_SIZE && (c = EEPROM.read(addr++)) != '\0') pubkeyHash += c;
    String sig = "";
    while (addr < EEPROM_SIZE && (c = EEPROM.read(addr++)) != '\0') sig += c;

    if (pubkeyHash.length() > 0 && sig.length() > 0) {
      whitelist[whitelistCount] = pubkeyHash;
      signatureList[whitelistCount] = sig;
      Serial.printf("[WHITELIST %d] PUB: %s\n", whitelistCount, pubkeyHash.c_str());
      Serial.printf("[WHITELIST %d] SIG: %s\n", whitelistCount, sig.c_str());
      whitelistCount++;
    } else {
      break;
    }
  }
  EEPROM.end();
}

String hashPubKey(uint8_t* pubkey) {
  SHA256 sha;
  uint8_t hash[32];
  sha.reset();
  sha.update(pubkey, 40);
  sha.finalize(hash, sizeof(hash));

  String hex = "";
  for (int i = 0; i < 32; i++) {
    char buf[3];
    sprintf(buf, "%02x", hash[i]);
    hex += buf;
  }
  return hex;
}

size_t pad_pkcs7(uint8_t *data, size_t len, size_t block_size) {
  uint8_t pad_value = block_size - (len % block_size);
  for (int i = 0; i < pad_value; i++) data[len + i] = pad_value;
  return len + pad_value;
}

void encrypt_and_publish(const char* plaintext) {
  uint8_t plainBuf[128];
  uint8_t encBuf[128];
  char base64Out[512];
  size_t encLen, base64Len;

  memcpy(plainBuf, plaintext, strlen(plaintext));
  size_t paddedLen = pad_pkcs7(plainBuf, strlen(plaintext), 16);

  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, (const unsigned char *)aes_key_string, 128);

  for (int i = 0; i < 16; i++) aes_iv[i] = random(0, 256);

  uint8_t iv_copy[16];
  memcpy(iv_copy, aes_iv, 16);
  mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, paddedLen, iv_copy, plainBuf, encBuf);
  mbedtls_aes_free(&aes);
  encLen = paddedLen;

  mbedtls_base64_encode((unsigned char *)base64Out, 512, &base64Len, encBuf, encLen);
  base64Out[base64Len] = '\0';

  char iv_b64[64];
  mbedtls_base64_encode((unsigned char *)iv_b64, 64, &base64Len, aes_iv, 16);
  iv_b64[base64Len] = '\0';

  StaticJsonDocument<256> msg;
  msg["data"] = base64Out;
  msg["iv"] = iv_b64;

  char output[256];
  serializeJson(msg, output);
  mqttClient.publish(MQTT_TOPIC, output);
  Serial.println("[MQTT] Data terenkripsi dipublish ke broker");
  Serial.println(output);
}

void reconnectMQTT() {
  uint64_t chipid = ESP.getEfuseMac();  // ✅ Gantikan getChipId()
  String clientId = "ESP32MasterA-" + String((uint32_t)(chipid >> 32), HEX) + String((uint32_t)chipid, HEX);

  while (!mqttClient.connected()) {
    Serial.println("[MQTT] Mencoba koneksi ke broker...");
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("✅ MQTT terkoneksi ke broker");
      break;
    } else {
      Serial.println("❌ MQTT gagal, retry dalam 2 detik");
      delay(2000);
    }
  }
}


void setup() {
  Serial.begin(115200);
  memcpy(aes_key, aes_key_string, 16);
  randomSeed(micros());

  delay(random(1000, 3000));  // ✅ Delay acak untuk hindari koneksi paralel

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[ESP32] Terhubung ke WiFi");
  Serial.print("[IP ESP32] "); Serial.println(WiFi.localIP());

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  server.begin();
  Serial.println("[ESP32] Server TCP siap menerima sensor");

  loadWhitelistFromEEPROM();
  Serial.printf("[ESP32] %d whitelist dimuat\n", whitelistCount);
}

void loop() {
  // Pastikan mqttClient sudah dideklarasikan global sebelumnya seperti:
  // WiFiClient espClient;
  // PubSubClient mqttClient(espClient);

  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  WiFiClient client = server.available();
  if (client) {
    Serial.println("\n[ESP32] Koneksi masuk dari ESP8266");
    String jsonInput = client.readStringUntil('\n');
    client.stop();

    Serial.println("[ESP32] Payload diterima: " + jsonInput);

    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, jsonInput);
    if (error) {
      Serial.println("[!] Gagal parsing JSON!");
      return;
    }

    const char* id_raw = doc["id"];
    char id[32];
    strncpy(id, id_raw, sizeof(id) - 1);
    id[sizeof(id) - 1] = '\0';

    String pubkey_hex = doc["pubkey"];
    String signature_hex = doc["signature"];
    float temp = doc["temperature"];
    float hum = doc["humidity"];
    int battery = doc["battery"];
    unsigned long timestamp = doc["timestamp"];

    // Hash ID untuk verifikasi
    String payload = id;
    SHA256 sha;
    uint8_t hash[32];
    sha.reset();
    sha.update((const uint8_t*)payload.c_str(), payload.length());
    sha.finalize(hash, sizeof(hash));

    uint8_t pubkey[40];
    uint8_t signature[40];
    hexStringToBytes(pubkey_hex, pubkey, sizeof(pubkey));
    hexStringToBytes(signature_hex, signature, sizeof(signature));

    const struct uECC_Curve_t* curve = uECC_secp160r1();
    bool valid = uECC_verify(pubkey, hash, sizeof(hash), signature, curve);

    // Hash untuk whitelist
    SHA256 shaPub;
    uint8_t pubHash[32];
    shaPub.reset();
    shaPub.update(pubkey, sizeof(pubkey));
    shaPub.finalize(pubHash, sizeof(pubHash));
    String pubkeyHashStr = hashToHex(pubHash, 32);

    String hashStr = hashPubKey(pubkey);
    Serial.println("[ESP32] Hash dari pubkey: " + hashStr);

    if (valid && isWhitelisted(pubkeyHashStr)) {
      Serial.println("✅ Signature VALID & Whitelisted!");
      Serial.printf("[DATA] ID:%s,Temp:%.1f,Hum:%.1f,Batt:%d,Time:%lu\n",
              id, temp, hum, battery, timestamp);

      Serial.print("[DEBUG] Final ID: "); Serial.println(id);
      String rawData = "id:" + String(id) + ",temp:" + String(temp, 2) + ",hum:" + String(hum, 2) + ",batt:" + String(battery) + ",ts:" + String(timestamp);
      while (rawData.length() % 16 != 0) rawData += " ";
      Serial.println("[DEBUG] rawData = " + rawData);

      // ⏱️ Hitung waktu pengiriman dengan enkripsi
      unsigned long t0 = millis();
      encrypt_and_publish(rawData.c_str());
      unsigned long t1 = millis();
      Serial.printf("[ZTMMF] Kirim terenkripsi selesai dalam: %lu ms\n", t1 - t0);
    } else {
      Serial.println("❌ Signature INVALID atau pubkey tidak terdaftar!");
    }
  }
}



