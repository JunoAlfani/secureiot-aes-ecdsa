// ===============================
// ✅ ESP8266-B-5 - Tahap 1: Generate PairKey & Signature
// ===============================
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <SHA256.h>

extern "C" {
  #include "uECC.h"
}

const char* ssid = "VLAN30_Access";
const char* password = "vlan3012345";
const char* esp32_host = "192.168.30.247";
const uint16_t esp32_port = 8888;

#define EEPROM_SIZE 512
const char* device_id = "5"; // IDENTITAS SLAVE NODE DARI 4-6

uint8_t privateKey[21];
uint8_t publicKey[40];
uint8_t signature[40];

String bytesToHexString(uint8_t* bytes, int len) {
  String hex = "";
  for (int i = 0; i < len; i++) {
    if (bytes[i] < 16) hex += "0";
    hex += String(bytes[i], HEX);
  }
  return hex;
}

void saveToEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  int addr = 0;
  for (int i = 0; i < strlen(device_id); i++) EEPROM.write(addr++, device_id[i]);
  EEPROM.write(addr++, '\0');
  for (int i = 0; i < sizeof(privateKey); i++) EEPROM.write(addr++, privateKey[i]);
  for (int i = 0; i < sizeof(publicKey); i++) EEPROM.write(addr++, publicKey[i]);
  for (int i = 0; i < sizeof(signature); i++) EEPROM.write(addr++, signature[i]);
  EEPROM.commit();
  EEPROM.end();
  Serial.println("✅ PairKey & Signature berhasil disimpan ke EEPROM!");
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi terhubung.");

  uECC_set_rng([](uint8_t *dest, unsigned size) {
    while (size--) {
      uint8_t val = 0;
      for (int i = 0; i < 8; ++i) {
        int init = analogRead(A0);
        int count = 0;
        while (analogRead(A0) == init) ++count;
        val = (val << 1) | (count & 1);
      }
      *dest++ = val;
    }
    return 1;
  });

  const struct uECC_Curve_t* curve = uECC_secp160r1();

  if (!uECC_make_key(publicKey, privateKey, curve)) {
    Serial.println("❌ Gagal generate ECC keypair!");
    return;
  }

  // Buat hash dari device_id
  SHA256 sha256;
  uint8_t hash[32];
  sha256.reset();
  sha256.update((const uint8_t*)device_id, strlen(device_id));
  sha256.finalize(hash, sizeof(hash));

  // Buat signature dari hash
  if (!uECC_sign(privateKey, hash, sizeof(hash), signature, curve)) {
    Serial.println("❌ Gagal membuat signature!");
    return;
  }

  Serial.println("[DEBUG] PublicKey: " + bytesToHexString(publicKey, sizeof(publicKey)));
  Serial.println("[DEBUG] Signature: " + bytesToHexString(signature, sizeof(signature)));

  saveToEEPROM();

  String pubkey_str = bytesToHexString(publicKey, sizeof(publicKey));
  String sig_str = bytesToHexString(signature, sizeof(signature));

  // Kirim JSON ke ESP32
  String json = "{";
  json += "\"id\":\"" + String(device_id) + "\",";
  json += "\"pubkey\":\"" + pubkey_str + "\",";
  json += "\"signature\":\"" + sig_str + "\"";
  json += "}";

  WiFiClient client;
  if (client.connect(esp32_host, esp32_port)) {
    Serial.println("📡 Mengirim JSON ke ESP32...");
    client.println(json);
    client.stop();
    Serial.println("✅ JSON berhasil dikirim!");
  } else {
    Serial.println("❌ Gagal koneksi ke ESP32!");
  }
}

void loop() {
  // Kosong
}