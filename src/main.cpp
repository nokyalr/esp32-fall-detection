#include <WiFi.h>
#include <FirebaseESP32.h>
#include <time.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define WIFI_SSID "satu_dua"
#define WIFI_PASSWORD "satusatu"
#define FIREBASE_HOST "emergencyfalldetection-725cb-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "wRBzA1N2VoFQrzmEOizH5eWc66HDliiiD3fvWoH5"

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

const int buzzerPin = 23;
const int resetButtonPin = 12;
const int ledPin = 2;

// ========== PENGATURAN SENSITIVITAS ==========
const float FALL_SENSITIVITY = 0.6;      // 0-1: semakin besar = semakin sensitif
const float NORMAL_LOW_MULT = 0.75;       // 0.5-0.8: batas bawah normal
const float NORMAL_HIGH_MULT = 1.3;      // 1.2-2.0: batas atas normal
const float VARIANCE_THRESHOLD = 2;    // 1.0-4.0: deteksi gerakan tidak teratur, semakin kecil = semakin sensitif

const int MIN_FREEFALL_SAMPLES = 3;      // 3-15: jatuh (semakin kecil = sensitif)
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
}

void initFirebase()
{
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Firebase.setBool(fbdo, "/esp32/online", true);
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

void reportStatus()
{
  String status = getStateString(currentState);
  String timestamp = getFormattedTime();
  float latitude = 0.0;
  float longitude = 0.0;

  if (gps.location.isValid())
  {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }

  bool ok = true;
  ok &= Firebase.setString(fbdo, "/fall_detection/status", status);
  ok &= Firebase.setString(fbdo, "/fall_detection/timestamp", timestamp);
  ok &= Firebase.setFloat(fbdo, "/fall_detection/location/lat", latitude);
  ok &= Firebase.setFloat(fbdo, "/fall_detection/location/lon", longitude);
  ok &= Firebase.setInt(fbdo, "/fall_detection/state_code", (int)currentState);
  ok &= Firebase.setFloat(fbdo, "/fall_detection/accel_magnitude", emaAccel);
  ok &= Firebase.setFloat(fbdo, "/fall_detection/movement_variance", emaVariance);

  if (ok)
  {
    Serial.printf("✅ Status reported: %s\n", status.c_str());
  }
  else
  {
    Serial.print("❌ Error reporting status: ");
    Serial.println(fbdo.errorReason());
  }
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
  reportStatus(); // Report reset status
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
  initFirebase();   // Connect ke Firebase
  
  initMPU();        // Kalibrasi & set systemReady = true
  
  Serial.println("✅ System fully initialized and ready!");
}

void loop()
{
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
      
      reportStatus();  // Report immediately on state change
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
  if (Firebase.ready() && (millis() - lastHeartbeat > 5000))
  {
    lastHeartbeat = millis();

    String status = getStateString(currentState);
    String timestamp = getFormattedTime();
    float latitude = 0.0;
    float longitude = 0.0;

    if (gps.location.isValid())
    {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
    }

    time_t now = time(NULL);

    bool ok = true;
    ok &= Firebase.setString(fbdo, "/fall_detection/status", status);
    ok &= Firebase.setString(fbdo, "/fall_detection/timestamp", timestamp);
    ok &= Firebase.setFloat(fbdo, "/fall_detection/location/lat", latitude);
    ok &= Firebase.setFloat(fbdo, "/fall_detection/location/lon", longitude);
    ok &= Firebase.setBool(fbdo, "/esp32/online", true);
    ok &= Firebase.setInt(fbdo, "/esp32/last_seen", now);
    ok &= Firebase.setInt(fbdo, "/fall_detection/state_code", (int)currentState);
    ok &= Firebase.setFloat(fbdo, "/fall_detection/accel_magnitude", emaAccel);
    ok &= Firebase.setFloat(fbdo, "/fall_detection/movement_variance", emaVariance);

    if (!ok)
    {
      Serial.print("⚠️ Heartbeat gagal: ");
      Serial.println(fbdo.errorReason());
    }
    else
    {
      Serial.printf("📡 Heartbeat: %s (Accel: %.2f, Var: %.2f)\n", 
                   status.c_str(), emaAccel, emaVariance);
    }
  }
}