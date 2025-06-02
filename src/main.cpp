#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// WiFi Configuration
#define WIFI_SSID "satu_dua"
#define WIFI_PASSWORD "satusatu"

// MQTT Configuration
#define MQTT_SERVER "broker.hivemq.com"  // Ganti dengan broker MQTT Anda
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "ESP32_FallDetector_001"
#define MQTT_USERNAME ""  // Kosongkan jika tidak perlu
#define MQTT_PASSWORD ""  // Kosongkan jika tidak perlu

// MQTT Topics
#define TOPIC_STATUS "falldetection/status"
#define TOPIC_LOCATION "falldetection/location"
#define TOPIC_HEARTBEAT "falldetection/heartbeat"
#define TOPIC_ALERT "falldetection/alert"
#define TOPIC_RESET "falldetection/reset"

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

WiFiClient espClient;
PubSubClient mqttClient(espClient);

const int buzzerPin = 23;
const int resetButtonPin = 18;
const int ledPin = 2;

// ========== PENGATURAN SENSITIVITAS ==========
const float FALL_SENSITIVITY = 0.5;      // 0-1: semakin besar = semakin sensitif
const float NORMAL_LOW_MULT = 0.75;       // 0.5-0.8: batas bawah normal
const float NORMAL_HIGH_MULT = 1.3;      // 1.2-2.0: batas atas normal
const float VARIANCE_THRESHOLD = 2;    // 1.0-4.0: deteksi gerakan tidak teratur, semakin kecil = semakin sensitif

const int MIN_FREEFALL_SAMPLES = 4;      // 3-15: jatuh (semakin kecil = sensitif)
const int MIN_ABNORMAL_SAMPLES = 25;     // 5-30: gerakan abnormal

const float alpha = 0.2;                 // 0.1-0.5: smoothing filter
const unsigned long CALIBRATE_MS = 3000; // waktu kalibrasi awal
const unsigned long MOVEMENT_CHECK_INTERVAL = 100; // 50-200ms: frekuensi analisis

// Threshold values (akan di-set otomatis saat kalibrasi)
float TH_NORMAL_LOW = 8.0;    
float TH_NORMAL_HIGH = 12.0;  
float TH_FALL = 5.0;

// System states
enum SystemState {
  STATE_NORMAL = 0,    // Hijau - gerakan normal
  STATE_ABNORMAL = 1,  // Kuning - gerakan tidak normal
  STATE_FALL = 2       // Merah - jatuh (persisten sampai reset)
};

SystemState currentState = STATE_NORMAL;
SystemState previousState = STATE_NORMAL;

bool systemReady = false;        // Flag untuk menandakan sistem siap
bool fallDetected = false;
bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
unsigned long lastLedToggle = 0;
bool ledState = LOW;             // Start dengan LED mati
int fallSampleCount = 0;
int abnormalSampleCount = 0;
float emaAccel;
float emaVariance = 0.0;

// Movement analysis variables
float accelHistory[10] = {0};
int historyIndex = 0;
unsigned long lastMovementCheck = 0;
unsigned long lastMqttReconnect = 0;

String getFormattedTime()
{
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);
  char buffer[25];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buffer);
}

String getStateString(SystemState state) {
  switch(state) {
    case STATE_NORMAL: return "NORMAL";
    case STATE_ABNORMAL: return "ABNORMAL_MOVEMENT";
    case STATE_FALL: return "FALL_DETECTED";
    default: return "UNKNOWN";
  }
}

void initWiFi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nConnected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// Function prototype for resetSystem
void resetSystem();

// Function prototype for connectMQTT
void connectMQTT();

// Function prototype for publishStatus
void publishStatus(String status, bool immediate = false);

// Function prototype for publishAlert
void publishAlert();

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.printf("MQTT message received [%s]: %s\n", topic, message.c_str());
  
  // Handle reset command
  if (String(topic) == TOPIC_RESET && message == "RESET") {
    Serial.println("Remote reset command received");
    resetSystem();
  }
}

void initMQTT() {
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(60);
  
  connectMQTT();
}

void connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println(" connected!");
      
      // Subscribe to reset topic
      mqttClient.subscribe(TOPIC_RESET);
      
      // Send online status
      publishStatus("ONLINE", true);
      
    } else {
      Serial.print(" failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void publishStatus(String status, bool immediate) {
  if (!mqttClient.connected()) {
    if (millis() - lastMqttReconnect > 5000) {
      lastMqttReconnect = millis();
      connectMQTT();
    }
    return;
  }

  DynamicJsonDocument doc(512);
  doc["device_id"] = MQTT_CLIENT_ID;
  doc["status"] = status;
  doc["timestamp"] = getFormattedTime();
  doc["state_code"] = (int)currentState;
  doc["accel_magnitude"] = emaAccel;
  doc["movement_variance"] = emaVariance;
  doc["uptime"] = millis();
  
  // Add GPS location if available
  if (gps.location.isValid()) {
    doc["location"]["lat"] = gps.location.lat();
    doc["location"]["lon"] = gps.location.lng();
    doc["location"]["valid"] = true;
  } else {
    doc["location"]["valid"] = false;
  }

  String jsonString;
  serializeJson(doc, jsonString);
  
  bool published = mqttClient.publish(TOPIC_STATUS, jsonString.c_str(), true); // retained message
  
  if (published) {
    Serial.printf("✅ Status published: %s\n", status.c_str());
  } else {
    Serial.println("❌ Failed to publish status");
  }
  
  // Send alert for fall detection
  if (currentState == STATE_FALL && immediate) {
    publishAlert();
  }
}

void publishAlert() {
  if (!mqttClient.connected()) return;

  DynamicJsonDocument alertDoc(256);
  alertDoc["device_id"] = MQTT_CLIENT_ID;
  alertDoc["alert_type"] = "FALL_DETECTED";
  alertDoc["timestamp"] = getFormattedTime();
  alertDoc["severity"] = "HIGH";
  
  if (gps.location.isValid()) {
    alertDoc["location"]["lat"] = gps.location.lat();
    alertDoc["location"]["lon"] = gps.location.lng();
  }

  String alertJson;
  serializeJson(alertDoc, alertJson);
  
  mqttClient.publish(TOPIC_ALERT, alertJson.c_str(), true);
  Serial.println("🚨 FALL ALERT published to MQTT");
}

void publishHeartbeat() {
  if (!mqttClient.connected()) return;

  DynamicJsonDocument heartbeatDoc(256);
  heartbeatDoc["device_id"] = MQTT_CLIENT_ID;
  heartbeatDoc["timestamp"] = getFormattedTime();
  heartbeatDoc["uptime"] = millis();
  heartbeatDoc["free_heap"] = ESP.getFreeHeap();
  heartbeatDoc["wifi_rssi"] = WiFi.RSSI();
  heartbeatDoc["status"] = getStateString(currentState);

  String heartbeatJson;
  serializeJson(heartbeatDoc, heartbeatJson);
  
  mqttClient.publish(TOPIC_HEARTBEAT, heartbeatJson.c_str());
}

void initMPU()
{
  if (!mpu.begin())
  {
    Serial.println("MPU6050 not found");
    while (1)
      delay(10);
  }
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  Serial.println("Start calibration (remain still)...");

  unsigned long t0 = millis();
  float sum = 0;
  int count = 0;
  while (millis() - t0 < CALIBRATE_MS)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float mag = sqrt(
        a.acceleration.x * a.acceleration.x +
        a.acceleration.y * a.acceleration.y +
        a.acceleration.z * a.acceleration.z);
    sum += mag;
    count++;
    delay(20);
  }
  float baseline = sum / count;
  
  // Set thresholds based on baseline menggunakan multiplier
  TH_FALL = baseline * FALL_SENSITIVITY;           
  TH_NORMAL_LOW = baseline * NORMAL_LOW_MULT;     
  TH_NORMAL_HIGH = baseline * NORMAL_HIGH_MULT;
  
  emaAccel = baseline;
  emaVariance = 0.0;

  Serial.println("Enhanced fall detector ready!");
  Serial.print("Baseline (1g): "); Serial.println(baseline);
  Serial.print("TH_FALL: "); Serial.println(TH_FALL);
  Serial.print("TH_NORMAL_LOW: "); Serial.println(TH_NORMAL_LOW);
  Serial.print("TH_NORMAL_HIGH: "); Serial.println(TH_NORMAL_HIGH);
  
  // Set system ready flag - LED akan menyala setelah ini
  systemReady = true;
}

void initBuzzer()
{
  pinMode(buzzerPin, OUTPUT);
  ledcSetup(0, 1000, 8);
  ledcAttachPin(buzzerPin, 0);
}

void initResetButton()
{
  pinMode(resetButtonPin, INPUT_PULLUP);
}

void initLED()
{
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);  // Start dengan LED mati
}

void playTone(int frequency)
{
  if (frequency > 0)
  {
    ledcWriteTone(0, frequency);
  }
  else
  {
    ledcWriteTone(0, 0);
  }
}

void handleSOS()
{
  static bool isDash = false;
  static unsigned long lastChange = 0;
  const int dashDuration = 300;

  unsigned long now = millis();
  if (now - lastChange >= (isDash ? dashDuration : 100))
  {
    isDash = !isDash;
    lastChange = now;

    if (isDash)
    {
      playTone(800); // Nada dash
    }
    else
    {
      playTone(0); // Hening 100 ms
    }
  }
}

float calculateMovementVariability() {
  if (historyIndex < 5) return 0.0;
  
  float mean = 0.0;
  int samples = min(10, historyIndex);
  
  for (int i = 0; i < samples; i++) {
    mean += accelHistory[i];
  }
  mean /= samples;
  
  float variance = 0.0;
  for (int i = 0; i < samples; i++) {
    variance += pow(accelHistory[i] - mean, 2);
  }
  variance /= samples;
  
  return sqrt(variance);
}

SystemState analyzeMovementPattern(float currentAccel, float variance) {
  // PENTING: Jika sudah fall detected, tetap dalam state fall sampai direset
  if (currentState == STATE_FALL) {
    return STATE_FALL;
  }
  
  // Deteksi jatuh (prioritas tertinggi)
  if (currentAccel < TH_FALL) {
    fallSampleCount++;
    if (fallSampleCount >= MIN_FREEFALL_SAMPLES) {
      return STATE_FALL;
    }
  } else {
    fallSampleCount = 0;
  }
  
  // Deteksi gerakan abnormal
  bool accelOutOfRange = (currentAccel < TH_NORMAL_LOW) || (currentAccel > TH_NORMAL_HIGH);
  bool highVariability = variance > VARIANCE_THRESHOLD;
  
  if (accelOutOfRange || highVariability) {
    abnormalSampleCount++;
    if (abnormalSampleCount >= MIN_ABNORMAL_SAMPLES) {
      return STATE_ABNORMAL;
    }
  } else {
    abnormalSampleCount = max(0, abnormalSampleCount - 2);
  }
  
  return STATE_NORMAL;
}

void resetSystem()
{
  currentState = STATE_NORMAL;
  previousState = STATE_NORMAL;
  fallDetected = false;
  fallSampleCount = 0;
  abnormalSampleCount = 0;
  emaAccel = (TH_NORMAL_LOW + TH_NORMAL_HIGH) / 2;
  emaVariance = 0.0;
  ledcWriteTone(0, 0);
  
  // Reset history buffer
  for (int i = 0; i < 10; i++) {
    accelHistory[i] = 0.0;
  }
  historyIndex = 0;
  
  Serial.println("System reset - ready for new detection");
  publishStatus("RESET", true); // Report reset status
}

void checkResetButton()
{
  int reading = digitalRead(resetButtonPin);
  if (reading == LOW)
  {
    if (!buttonPressed && (millis() - lastDebounceTime) > debounceDelay)
    {
      buttonPressed = true;
      lastDebounceTime = millis();
      resetSystem();
    }
  }
  else
  {
    buttonPressed = false;
  }
}

void handleLED()
{
  // LED hanya menyala jika sistem sudah ready (WiFi connected & calibrated)
  if (!systemReady) {
    digitalWrite(ledPin, LOW);  // LED mati jika belum ready
    return;
  }
  
  switch(currentState) {
    case STATE_NORMAL:
      digitalWrite(ledPin, HIGH);  // LED menyala terus (hijau)
      break;
      
    case STATE_ABNORMAL:
      // Blink lambat untuk status kuning
      if (millis() - lastLedToggle > 500) {
        lastLedToggle = millis();
        ledState = !ledState;
        digitalWrite(ledPin, ledState);
      }
      break;
      
    case STATE_FALL:
      // Blink cepat untuk status merah
      if (millis() - lastLedToggle > 100) {
        lastLedToggle = millis();
        ledState = !ledState;
        digitalWrite(ledPin, ledState);
      }
      break;
  }
}

void handleBuzzer() {
  switch(currentState) {
    case STATE_NORMAL:
      playTone(0);  // Tidak ada suara
      break;
      
    case STATE_ABNORMAL:
      // Beep singkat setiap 2 detik
      static unsigned long lastAbnormalBeep = 0;
      if (millis() - lastAbnormalBeep > 2000) {
        lastAbnormalBeep = millis();
        playTone(600);
        delay(100);
        playTone(0);
      }
      break;
      
    case STATE_FALL:
      handleSOS();  // SOS pattern
      break;
  }
}

void setup()
{
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  
  initLED();        // LED mati dulu
  initBuzzer();
  initResetButton();
  
  Serial.println("Initializing system...");
  initWiFi();       // Connect ke WiFi
  
  configTime(7 * 3600, 0, "pool.ntp.org");
  delay(1000);
  
  initMQTT();       // Connect ke MQTT Broker
  initMPU();        // Kalibrasi & set systemReady = true
  
  Serial.println("✅ System fully initialized and ready!");
}

void loop()
{
  // Maintain MQTT connection
  if (!mqttClient.connected()) {
    if (millis() - lastMqttReconnect > 5000) {
      lastMqttReconnect = millis();
      connectMQTT();
    }
  }
  mqttClient.loop();

  checkResetButton();
  handleLED();

  // Hanya proses sensor jika sistem sudah ready
  if (!systemReady) {
    delay(100);
    return;
  }

  // Baca sensor MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float mag = sqrt(
      a.acceleration.x * a.acceleration.x +
      a.acceleration.y * a.acceleration.y +
      a.acceleration.z * a.acceleration.z);
  
  // Update EMA
  emaAccel = alpha * mag + (1 - alpha) * emaAccel;
  
  // Update history buffer untuk analisis pola
  if (millis() - lastMovementCheck > MOVEMENT_CHECK_INTERVAL) {
    lastMovementCheck = millis();
    
    accelHistory[historyIndex % 10] = emaAccel;
    historyIndex++;
    
    // Hitung variabilitas gerakan
    emaVariance = calculateMovementVariability();
    
    // Analisis pola gerakan dan tentukan state
    previousState = currentState;
    currentState = analyzeMovementPattern(emaAccel, emaVariance);
    
    // Log perubahan state
    if (currentState != previousState) {
      Serial.printf("State changed: %s -> %s\n", 
                   getStateString(previousState).c_str(), 
                   getStateString(currentState).c_str());
      
      if (currentState == STATE_FALL) {
        Serial.println("🚨 FALL DETECTED! System locked in fall state until reset.");
        if (gps.location.isValid()) {
          Serial.printf("Location: %.6f, %.6f\n", gps.location.lat(), gps.location.lng());
        }
        fallDetected = true;
      } else if (currentState == STATE_ABNORMAL) {
        Serial.println("⚠️ ABNORMAL MOVEMENT DETECTED");
      } else {
        Serial.println("✅ Movement normalized");
      }
      
      publishStatus(getStateString(currentState), true);  // Report immediately on state change
    }
  }

  // Handle GPS
  while (gpsSerial.available() > 0)
  {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  // Handle buzzer based on current state
  handleBuzzer();

  // Heartbeat - kirim data secara berkala (setiap 5 detik)
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 5000)
  {
    lastHeartbeat = millis();
    publishHeartbeat();
    
    Serial.printf("📡 Heartbeat: %s (Accel: %.2f, Var: %.2f)\n", 
                 getStateString(currentState).c_str(), emaAccel, emaVariance);
  }

  delay(10); // Small delay to prevent watchdog reset
}