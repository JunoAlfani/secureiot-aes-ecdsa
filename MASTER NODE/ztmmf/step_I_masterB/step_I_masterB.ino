// ===============================
// ✅ ESP32 B - Tahap 1: TCP Server Registrasi PairKey + Signature ke EEPROM
// ===============================
#include <WiFi.h>
#include <SHA256.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

extern "C" {
  #include "uECC.h"
}

const char* ssid = "VLAN30_Access";
const char* password = "vlan3012345";

WiFiServer server(8888);

#define EEPROM_SIZE 512
String whitelist[10];
String signatureList[10];
int whitelistCount = 0;

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

void saveToEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  int addr = 0;
  for (int i = 0; i < whitelistCount; i++) {
    for (int j = 0; j < whitelist[i].length(); j++) EEPROM.write(addr++, whitelist[i][j]);
    EEPROM.write(addr++, '\0');
    for (int j = 0; j < signatureList[i].length(); j++) EEPROM.write(addr++, signatureList[i][j]);
    EEPROM.write(addr++, '\0');
  }
  EEPROM.commit();
  EEPROM.end();
  Serial.println("[EEPROM] Whitelist & Signature disimpan ke EEPROM.");
}

bool isWhitelisted(const String& hashStr) {
  for (int i = 0; i < whitelistCount; i++) {
    if (whitelist[i] == hashStr) return true;
  }
  return false;
}

void addToWhitelist(const String& hashStr, const String& signatureStr) {
  if (!isWhitelisted(hashStr) && whitelistCount < 10) {
    whitelist[whitelistCount] = hashStr;
    signatureList[whitelistCount] = signatureStr;
    Serial.println("[+] Ditambahkan ke whitelist: " + hashStr);
    whitelistCount++;
    saveToEEPROM();
  } else {
    Serial.println("[!] Hash sudah ada atau whitelist penuh!");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("[WiFi] Terhubung");
  Serial.print("Alamat IP ESP32: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("[ESP32] TCP Server aktif di port 8888");
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("\n[ESP32] Koneksi masuk dari Wemos");
    String jsonInput = client.readStringUntil('\n');
    client.stop();

    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, jsonInput);
    if (error) {
      Serial.println("[!] Gagal parsing JSON!");
      return;
    }

    String id = doc["id"];
    String pubkey_hex = doc["pubkey"];
    String signature_hex = doc["signature"];
    Serial.println("[ESP32] Payload diterima: " + pubkey_hex);

    uint8_t pubkey[40];
    hexStringToBytes(pubkey_hex, pubkey, sizeof(pubkey));

    SHA256 sha;
    uint8_t hash[32];
    sha.reset();
    sha.update(pubkey, sizeof(pubkey));
    sha.finalize(hash, sizeof(hash));

    String hashStr = hashToHex(hash, 32);
    Serial.println("[ESP32] Hash dari public key: " + hashStr);

    addToWhitelist(hashStr, signature_hex);
  }
}
