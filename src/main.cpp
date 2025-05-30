#include <WiFi.h>
#include <FirebaseESP32.h>
#include <time.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define WIFI_SSID "DIMZT_05"
#define WIFI_PASSWORD "Surabaya"
#define FIREBASE_HOST "emergencyfalldetection-725cb-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "wRBzA1N2VoFQrzmEOizH5eWc66HDliiiD3fvWoH5"

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

const int buzzerPin = 23;
const int resetButtonPin = 18;
const int ledPin = 2;

const int dotDuration = 100;
const int shortPause = dotDuration;
const int dashDuration = dotDuration * 3;
const int longPause = dotDuration * 7;
const unsigned long ledBlinkInterval = 100;

float TH_LOW = 7.0;
const float alpha = 0.3;
const unsigned long CALIBRATE_MS = 3000;
const int MIN_FREEFALL_SAMPLES = 3;

bool fallDetected = false;
bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
unsigned long lastLedToggle = 0;
bool ledState = HIGH;
int fallSampleCount = 0;
float emaAccel;

// SOS tone handling
unsigned long sosLastTime = 0;
int sosStep = 0;
bool tonePlaying = false;
unsigned long toneEndTime = 0;

// SOS Morse Code: 1 = dot, 3 = dash, 0 = pause, -1 = done
int sosSequence[] = {1, 1, 1, 0, 3, 3, 3, 0, 1, 1, 1, 0, -1};

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
  Firebase.setBool(fbdo, "/esp32/online", true); // saat ESP32 baru hidup
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
  TH_LOW = baseline * 0.65;
  emaAccel = baseline;

  Serial.println("Fall detector ready!");
  Serial.print("Baseline (1g): ");
  Serial.println(baseline);
  Serial.print("TH_LOW: ");
  Serial.println(TH_LOW);
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
  digitalWrite(ledPin, HIGH);
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

void reportFall()
{
  String status = "FALL DETECTED";
  String timestamp = getFormattedTime();
  float latitude = 0.0;
  float longitude = 0.0;

  if (gps.location.isValid())
  {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }
  else
  {
    Serial.println("Using last known location (GPS not available)");
  }

  bool ok = true;
  ok &= Firebase.setString(fbdo, "/fall_detection/status", status);
  ok &= Firebase.setString(fbdo, "/fall_detection/timestamp", timestamp);
  ok &= Firebase.setFloat(fbdo, "/fall_detection/location/lat", latitude);
  ok &= Firebase.setFloat(fbdo, "/fall_detection/location/lon", longitude);

  if (ok)
  {
    Serial.println("✅ Fall event reported to Firebase!");
  }
  else
  {
    Serial.print("❌ Error reporting fall: ");
    Serial.println(fbdo.errorReason());
  }
}

void resetSystem()
{
  fallDetected = false;
  fallSampleCount = 0;
  emaAccel = TH_LOW * 1.5;
  ledcWriteTone(0, 0);
  digitalWrite(ledPin, HIGH);
  sosStep = 0;
  toneEndTime = 0;
  Serial.println("System reset - ready for new detection");
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
  if (fallDetected)
  {
    if (millis() - lastLedToggle > ledBlinkInterval)
    {
      lastLedToggle = millis();
      ledState = !ledState;
      digitalWrite(ledPin, ledState);
    }
  }
  else
  {
    digitalWrite(ledPin, HIGH);
  }
}

void printGPSLocation()
{
  if (gps.location.isValid())
  {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
  }
  else
  {
    Serial.println("GPS location not available");
  }
}

void setup()
{
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  initWiFi();
  initBuzzer();
  initResetButton();
  initLED();
  configTime(7 * 3600, 0, "pool.ntp.org");
  delay(1000);
  initFirebase();
  initMPU();
}

unsigned long lastHeartbeat = 0;

void loop()
{
  checkResetButton();
  handleLED();

  if (!fallDetected)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float mag = sqrt(
        a.acceleration.x * a.acceleration.x +
        a.acceleration.y * a.acceleration.y +
        a.acceleration.z * a.acceleration.z);
    emaAccel = alpha * mag + (1 - alpha) * emaAccel;

    if (emaAccel < TH_LOW)
    {
      fallSampleCount++;
      if (fallSampleCount >= MIN_FREEFALL_SAMPLES)
      {
        fallDetected = true;
        Serial.println("Fall detected");
        printGPSLocation();
        reportFall();
        lastLedToggle = millis();
      }
    }
    else
    {
      fallSampleCount = 0;
    }
  }

  while (gpsSerial.available() > 0)
  {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  if (fallDetected)
  {
    handleSOS();
  }

  // Heartbeat - kirim data walaupun tidak jatuh
  if (Firebase.ready() && (millis() - lastHeartbeat > 5000)) // setiap 5 detik
  {
    lastHeartbeat = millis();

    String status = fallDetected ? "FALL DETECTED" : "Normal";
    String timestamp = getFormattedTime();
    float latitude = 0.0;
    float longitude = 0.0;

    if (gps.location.isValid())
    {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
    }

    time_t now = time(NULL); // waktu epoch sekarang

    bool ok = true;
    ok &= Firebase.setString(fbdo, "/fall_detection/status", status);
    ok &= Firebase.setString(fbdo, "/fall_detection/timestamp", timestamp);
    ok &= Firebase.setFloat(fbdo, "/fall_detection/location/lat", latitude);
    ok &= Firebase.setFloat(fbdo, "/fall_detection/location/lon", longitude);
    ok &= Firebase.setBool(fbdo, "/esp32/online", true);
    ok &= Firebase.setInt(fbdo, "/esp32/last_seen", now);

    if (!ok)
    {
      Serial.print("⚠️ Heartbeat gagal: ");
      Serial.println(fbdo.errorReason());
    }
    else
    {
      Serial.println("📡 Heartbeat Firebase update");
    }
  }
}
