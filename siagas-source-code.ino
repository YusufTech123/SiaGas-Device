#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BME680.h>
#include "Adafruit_PM25AQI.h"
#include <WiFi.h>
#include <Adafruit_MAX1704X.h>

bool mqttReady = false;  

// === Sensor ===
Adafruit_BME680 bme;
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
HardwareSerial pmsSerial(2);
Adafruit_MAX17048 max17048; 

// === PIN ===
#define MODEM_RX 21
#define MODEM_TX 20
#define MODEM_BAUD 9600
#define MQ2_PIN 1
#define MQ135_PIN 2
#define MQ6_PIN 3 
#define LED_POWER 10
#define LED_CONN 4
#define LED_HIJAU 14
#define LED_KUNING 12
#define LED_MERAH 13
#define BUZZER_PIN 15
#define CHARGE_STATUS_PIN 19


// === GPRS ===
const char apn[] = "internet";

// === WiFi ===
const char* ssid     = "";
const char* password = "";

// === MQTT ===
const char* mqtt_server = "";
const int   mqtt_port   = 1883;
const char* mqtt_user   = "";
const char* mqtt_pass   = "";

// === Topics ===
const char* topic_mq2   = "siagas/sensor/mq2/value";
const char* topic_mq135 = "siagas/sensor/mq135/value";
const char* topic_mq6   = "siagas/sensor/mq6/value";
const char* topic_level = "siagas/status/level";
const char* topic_temp  = "siagas/sensor/bme680/temperature";
const char* topic_hum   = "siagas/sensor/bme680/humidity";
const char* topic_press = "siagas/sensor/bme680/pressure";
const char* topic_gas   = "siagas/sensor/bme680/gas_resistance";
const char* topic_pm1   = "siagas/sensor/pms5003/pm1";
const char* topic_pm25  = "siagas/sensor/pms5003/pm25";
const char* topic_pm10  = "siagas/sensor/pms5003/pm10";
const char* topic_batt_percentage = "siagas/battery/level";
const char* topic_batt_voltage = "siagas/battery/voltage";
const char* topic_charge_status = "siagas/battery/charge/status";

// Buat Logic Charger
const int samples = 10;          
const float vRef = 3.3;          
const int adcMax = 4095;         
const float threshHigh = 0.8;    
const float threshLow  = 0.7;    
bool charging = false;
bool lastCharging = false;

// === Modem & MQTT Client ===
TinyGsm modem(Serial1);
TinyGsmClient gsmClient(modem);
WiFiClient wifiClient;
PubSubClient mqtt;

// === Timing ===
unsigned long lastSendMQ = 0;
unsigned long lastSendBME = 0;
unsigned long lastSendPMS = 0;
unsigned long lastSendBatt = 0;
const unsigned long intervalMQ = 1000;
const unsigned long intervalBME = 1000;
const unsigned long intervalPMS = 1000;
const unsigned long intervalBatt = 1000;

bool firstLevelMsgIgnored = false;
unsigned long lastMsgTime = 0;
unsigned long timeoutStatus = 5000;

String levelStatus = "aman";
unsigned long lastMQTTReconnectAttempt = 0;
const unsigned long mqttReconnectInterval = 5000;

// === Flag koneksi ===
bool useGPRS = false;
bool useWiFi = false;

// === Cek jaringan ===
unsigned long lastNetCheck = 0;
const unsigned long netCheckInterval = 10000;

// === Fungsi cek & reconnect jaringan dengan auto fallback ===
void checkAndReconnectNetwork() {
  if (useGPRS) {
    if (!modem.isNetworkConnected() || !modem.isGprsConnected()) {
      Serial.println("⚠️ GPRS terputus, coba reconnect...");
      modem.gprsDisconnect();
      delay(2000);
      modem.restart();
      if (modem.waitForNetwork(30000) && modem.gprsConnect(apn)) {
        Serial.println("✅ Reconnect GPRS berhasil!");
        mqtt.setClient(gsmClient);
      } else {
        Serial.println("❌ GPRS gagal, coba fallback ke WiFi...");
        useGPRS = false;
        useWiFi = true;
        WiFi.disconnect(true);
        WiFi.begin(ssid, password);
        unsigned long startAttempt = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
          delay(500);
          Serial.print(".");
        }
        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("✅ Fallback ke WiFi berhasil!");
          mqtt.setClient(wifiClient);
        } else {
          Serial.println("❌ WiFi juga gagal!");
        }
      }
    }
  } else if (useWiFi) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("⚠️ WiFi terputus, coba reconnect...");
      WiFi.disconnect(true);
      delay(1000);
      WiFi.begin(ssid, password);
      unsigned long startAttempt = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
        delay(500);
        Serial.print(".");
      }
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("✅ Reconnect WiFi berhasil!");
        mqtt.setClient(wifiClient);
      } else {
        Serial.println("❌ WiFi gagal, coba fallback ke GPRS...");
        useWiFi = false;
        useGPRS = true;
        modem.gprsDisconnect();
        delay(2000);
        modem.restart();
        if (modem.waitForNetwork(30000) && modem.gprsConnect(apn)) {
          Serial.println("✅ Fallback ke GPRS berhasil!");
          mqtt.setClient(gsmClient);
        } else {
          Serial.println("❌ GPRS juga gagal!");
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_POWER, OUTPUT);
  pinMode(LED_CONN, OUTPUT);
  pinMode(LED_HIJAU, OUTPUT);
  pinMode(LED_KUNING, OUTPUT);
  pinMode(LED_MERAH, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CHARGE_STATUS_PIN, INPUT);

  digitalWrite(LED_POWER, HIGH);
  digitalWrite(LED_CONN, LOW);
  digitalWrite(LED_HIJAU, LOW);
  digitalWrite(LED_KUNING, LOW);
  digitalWrite(LED_MERAH, LOW);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);

  Serial1.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);
  modem.restart();

  while (!mqttReady) {
    Serial.println("Menghubungkan ke GPRS...");
    bool gprsConnected = false;

    if (modem.waitForNetwork(30000)) {
      if (modem.isNetworkConnected()) {
        if (modem.gprsConnect(apn)) {
          gprsConnected = true;
        }
      }
    }

    if (gprsConnected && modem.isGprsConnected()) {
      Serial.println("✅ Berhasil konek GPRS!");
      useGPRS = true;
      mqtt.setClient(gsmClient);
      mqttReady = true;
    } else {
      Serial.println("❌ Gagal konek GPRS, coba WiFi...");
      WiFi.begin(ssid, password);
      unsigned long startAttempt = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
        delay(500);
        Serial.print(".");
      }
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ Berhasil konek WiFi!");
        useWiFi = true;
        mqtt.setClient(wifiClient);
        mqttReady = true;
      } else {
        Serial.println("\n❌ WiFi juga gagal! Coba ulangi lagi...");
        delay(5000); 
      }
    }
  }

  // === BME680 ===
  Wire.begin(8, 9);
  if (bme.begin(0x77, &Wire)) {
    Serial.println("BME680 siap!");
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);
  }

  // === PMS5003 ===
  pmsSerial.begin(9600, SERIAL_8N1, 16, 17);
  if (aqi.begin_UART(&pmsSerial)) {
    Serial.println("PMS5003 siap!");
  }

   // === MAX17048 ===
  if (max17048.begin(&Wire)) {
    Serial.println("MAX17048 siap!");
  } else {
    Serial.println("❌ MAX17048 tidak terdeteksi!");
  }

  // Start koneksi ke MQTT Broker
  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(mqttCallback);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String message = String((char*)payload);

  if (String(topic) == topic_level) {
    if (!firstLevelMsgIgnored) {
      Serial.println("⚠️ Pesan retained pertama diabaikan");
      firstLevelMsgIgnored = true;
      return;
    }

    levelStatus = message;
    lastMsgTime = millis();
    Serial.print("Level status diterima: ");
    Serial.println(levelStatus);

    updateStatusLED();
    handleBuzzer();
  }
}

void checkConnectionTimeout() {
  if (millis() - lastMsgTime > timeoutStatus) {
    Serial.println("⚠️ Status timeout, ML offline");
    levelStatus = "offline";
    digitalWrite(LED_HIJAU, LOW);
    digitalWrite(LED_KUNING, LOW);
    digitalWrite(LED_MERAH, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void bacaDanKirimChargeStatus() {
  long total = 0;
  for (int i = 0; i < samples; i++) {
    total += analogRead(CHARGE_STATUS_PIN);
    delay(5);
  }

  float avgAdc = total / (float)samples;
  float voltage = avgAdc * vRef / adcMax;

  // logika deteksi charging seperti sebelumnya
  if (charging) {
    if (voltage < threshLow) charging = false;
  } else {
    if (voltage > threshHigh) charging = true;
  }

  // SELALU kirim ke MQTT, baik status berubah atau tidak
  String payload = charging ? "true" : "false";
  mqtt.publish(topic_charge_status, payload.c_str(), true);
}

void updateStatusLED() {
  digitalWrite(LED_HIJAU, LOW);
  digitalWrite(LED_KUNING, LOW);
  digitalWrite(LED_MERAH, LOW);
  if (levelStatus == "aman") digitalWrite(LED_HIJAU, HIGH);
  else if (levelStatus == "waspada") digitalWrite(LED_KUNING, HIGH);
  else if (levelStatus == "bahaya") digitalWrite(LED_MERAH, HIGH);
}

void handleBuzzer() {
  static bool buzzerOn = false;
  static unsigned long lastChange = 0;

  if (levelStatus == "offline") {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerOn = false;
    return;
  }

  if (levelStatus == "aman") {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerOn = false;
    return;
  }

  unsigned long now = millis();
  unsigned long onTime = 1000; 
  unsigned long offTime = (levelStatus == "waspada") ? 3000 : 500;

  if (buzzerOn) {
    if (now - lastChange >= onTime) {
      buzzerOn = false;
      digitalWrite(BUZZER_PIN, LOW);
      lastChange = now;
    }
  } else {
    if (now - lastChange >= offTime) {
      buzzerOn = true;
      digitalWrite(BUZZER_PIN, HIGH);
      lastChange = now;
    }
  }
}


// void handleBuzzer() {
//   static bool buzzerOn = false;
//   static unsigned long lastChange = 0;

//   if (levelStatus == "aman") {
//     digitalWrite(BUZZER_PIN, LOW);
//     buzzerOn = false;
//     return;
//   }

//   unsigned long now = millis();
//   unsigned long onTime = 1000; 
//   unsigned long offTime = (levelStatus == "waspada") ? 3000 : 1000;

//   if (buzzerOn) {
//     if (now - lastChange >= onTime) {
//       buzzerOn = false;
//       digitalWrite(BUZZER_PIN, LOW);
//       lastChange = now;
//     }
//   } else {
//     if (now - lastChange >= offTime) {
//       buzzerOn = true;
//       digitalWrite(BUZZER_PIN, HIGH);
//       lastChange = now;
//     }
//   }
// }

void handleConnLED() {
  static bool blinkState = false;
  static unsigned long lastBlink = 0;
  unsigned long now = millis();
  if (mqtt.connected()) {
    if (now - lastBlink >= 100) {
      blinkState = !blinkState;
      digitalWrite(LED_CONN, blinkState);
      lastBlink = now;
    }
  } else {
    digitalWrite(LED_CONN, LOW);
  }
}

void bacaDanKirimMQ() {
  int mq2_value = analogRead(MQ2_PIN);
  int mq135_value = analogRead(MQ135_PIN);
  int mq6_value = analogRead(MQ6_PIN);  

  if (mq2_value > 2500) mq2_value = 2500;  
  if (mq6_value > 2500) mq6_value = 2500;

  char mq2_str[10], mq135_str[10], mq6_str[10];
  itoa(mq2_value, mq2_str, 10);
  itoa(mq135_value, mq135_str, 10);
  itoa(mq6_value, mq6_str, 10);  

  mqtt.publish(topic_mq2, mq2_str);
  mqtt.publish(topic_mq135, mq135_str);
  mqtt.publish(topic_mq6, mq6_str);   
}

void bacaDanKirimBME() {
  if (bme.performReading()) {
    char temp_str[10], hum_str[10], press_str[10], gas_str[15];
    dtostrf(bme.temperature, 4, 2, temp_str);
    dtostrf(bme.humidity, 4, 2, hum_str);
    dtostrf(bme.pressure / 100.0, 4, 2, press_str);
    dtostrf(bme.gas_resistance / 1000.0, 6, 2, gas_str);
    mqtt.publish(topic_temp, temp_str);
    mqtt.publish(topic_hum, hum_str);
    mqtt.publish(topic_press, press_str);
    mqtt.publish(topic_gas, gas_str);
  }
}

void bacaDanKirimPMS() {
  PM25_AQI_Data data;
  if (aqi.read(&data)) {
    char pm1_str[10], pm25_str[10], pm10_str[10];
    itoa(data.pm10_standard, pm1_str, 10);
    itoa(data.pm25_standard, pm25_str, 10);
    itoa(data.pm100_standard, pm10_str, 10);
    mqtt.publish(topic_pm1, pm1_str);
    mqtt.publish(topic_pm25, pm25_str);
    mqtt.publish(topic_pm10, pm10_str);
  }
}

void bacaDanKirimBattery() {
  float level = max17048.cellPercent();
  float volt = max17048.cellVoltage();

  char batt_str[10];
  dtostrf(level, 4, 2, batt_str);
  mqtt.publish(topic_batt_percentage, batt_str);

  char volt_str[10];
  dtostrf(volt, 4, 2, volt_str);
  mqtt.publish(topic_batt_voltage, volt_str);
}

bool reconnectMQTTNonBlocking() {
  Serial.print("🔄 Coba konek MQTT via ");
  Serial.println(useGPRS ? "GPRS" : "WiFi");

  if (mqtt.connect("ESP32-S3-N16R8", mqtt_user, mqtt_pass)) {
    mqtt.subscribe(topic_level, 1);
    Serial.println("✅ Berhasil konek MQTT");
    digitalWrite(LED_CONN, HIGH);
    return true;
  } else {
    Serial.print("❌ Gagal konek MQTT, rc=");
    Serial.println(mqtt.state());
    return false;
  }
}


void loop() {
  if (!mqttReady) {
    handleConnLED();  
    delay(1000); 
    return;
  }

  mqtt.loop();

  unsigned long now = millis();

  if (now - lastNetCheck > netCheckInterval) {
    lastNetCheck = now;
    checkAndReconnectNetwork();
  }

  if (!mqtt.connected()) {
    if (now - lastMQTTReconnectAttempt > mqttReconnectInterval) {
      lastMQTTReconnectAttempt = now;
      reconnectMQTTNonBlocking();
    }
    return;
  }

  checkConnectionTimeout();

  if (now - lastSendMQ >= intervalMQ) {
    lastSendMQ = now;
    bacaDanKirimMQ();
  }
  if (now - lastSendBME >= intervalBME) {
    lastSendBME = now;
    bacaDanKirimBME();
  }
  if (now - lastSendPMS >= intervalPMS) {
    lastSendPMS = now;
    bacaDanKirimPMS();
  }
  if (now - lastSendBatt >= intervalBatt) {
    lastSendBatt = now;
    bacaDanKirimBattery();
    bacaDanKirimChargeStatus();
  }

  handleBuzzer();
  handleConnLED();
  mqtt.loop();
}
