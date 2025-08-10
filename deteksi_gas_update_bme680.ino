#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BME680.h>
#include "Adafruit_PM25AQI.h"

// === Sensor ===
Adafruit_BME680 bme;
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
HardwareSerial pmsSerial(2); 

// === PIN ===
#define MODEM_RX 21
#define MODEM_TX 20
#define MODEM_BAUD 9600
#define MQ2_PIN 1
#define MQ135_PIN 2
#define LED_POWER 10
#define LED_CONN 4
#define LED_HIJAU 14
#define LED_KUNING 12
#define LED_MERAH 13
#define BUZZER_PIN 15

// === GPRS ===
const char apn[] = "internet";

// === MQTT ===
const char* mqtt_server = "mqtt.yusufghazali.com";
const int   mqtt_port   = 1883;
const char* mqtt_user   = "bm$578Bxc1vxDe3btKNpYL&bm64r#8";
const char* mqtt_pass   = "5U5QcwfnZJTaZvBym6p!TuKWvgYLH%";

// === Topics ===
const char* topic_mq2   = "siagas/sensor/mq2/value";
const char* topic_mq135 = "siagas/sensor/mq135/value";
const char* topic_level = "siagas/status/level";
const char* topic_temp  = "siagas/sensor/bme680/temperature";
const char* topic_hum   = "siagas/sensor/bme680/humidity";
const char* topic_press = "siagas/sensor/bme680/pressure";
const char* topic_gas   = "siagas/sensor/bme680/gas_resistance";
const char* topic_pm1   = "siagas/sensor/pms5003/pm1";
const char* topic_pm25  = "siagas/sensor/pms5003/pm25";
const char* topic_pm10  = "siagas/sensor/pms5003/pm10";

// === Modem & MQTT Client ===
TinyGsm modem(Serial1);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

// === Timing ===
unsigned long lastSendMQ = 0;
unsigned long lastSendBME = 0;
unsigned long lastSendPMS = 0;
const unsigned long intervalMQ = 1000;
const unsigned long intervalBME = 1000;
const unsigned long intervalPMS = 1000;

String levelStatus = "aman";
unsigned long lastBuzzToggle = 0;
bool buzzerState = false;
unsigned long lastMQTTReconnectAttempt = 0;
const unsigned long mqttReconnectInterval = 5000;

void setup() {
  Serial.begin(115200);

  pinMode(LED_POWER, OUTPUT);
  pinMode(LED_CONN, OUTPUT);
  pinMode(LED_HIJAU, OUTPUT);
  pinMode(LED_KUNING, OUTPUT);
  pinMode(LED_MERAH, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

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
  Serial.println("Menghubungkan ke GPRS...");
  if (modem.gprsConnect(apn)) {
    Serial.println("Berhasil konek GPRS!");
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

  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(mqttCallback);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String message = String((char*)payload);

  if (String(topic) == topic_level) {
    levelStatus = message;
    Serial.print("Level status diterima: ");
    Serial.println(levelStatus);
    updateStatusLED();
    handleBuzzer();
  }
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

  if (levelStatus == "aman") {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerOn = false;
    return;
  }

  unsigned long now = millis();
  unsigned long onTime = 1000; 
  unsigned long offTime = (levelStatus == "waspada") ? 3000 : 1000;

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

void handleConnLED() {
  static bool blinkState = false;
  static unsigned long lastBlink = 0;
  unsigned long now = millis();
  if (modem.isGprsConnected() && mqtt.connected()) {
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
  char mq2_str[10], mq135_str[10];
  itoa(mq2_value, mq2_str, 10);
  itoa(mq135_value, mq135_str, 10);
  mqtt.publish(topic_mq2, mq2_str);
  mqtt.publish(topic_mq135, mq135_str);
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

bool reconnectMQTTNonBlocking() {
  if (mqtt.connect("ESP32-S3-N16R8", mqtt_user, mqtt_pass)) {
    mqtt.subscribe(topic_level, 1); // QoS 1
    Serial.println("Berhasil konek MQTT");
    return true;
  }
  return false;
}

void loop() {
  mqtt.loop();

  unsigned long now = millis();

  if (!mqtt.connected()) {
    if (now - lastMQTTReconnectAttempt > mqttReconnectInterval) {
      lastMQTTReconnectAttempt = now;
      reconnectMQTTNonBlocking();
    }
    return;
  }

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

  handleBuzzer();
  handleConnLED();
  mqtt.loop();
}
