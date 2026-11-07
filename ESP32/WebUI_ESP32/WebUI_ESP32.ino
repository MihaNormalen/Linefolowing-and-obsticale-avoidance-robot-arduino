#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <NewPing.h>
#include "index.h"

// --- WI-FI SETTINGS (Optional/Background) ---
const char* ssid = "YOUR_WIFI_NAME";     // Vnesite ime vašega Wi-Fi omrežja
const char* password = "YOUR_WIFI_PASSWORD"; // Vnesite geslo Wi-Fi omrežja

const bool ENABLE_AP_FALLBACK = true;
const char* ap_ssid = "ESP32-Robot-AP";
const char* ap_password = "12345678";

// --- SERVERS ---
WebServer server(80);
WebSocketsServer webSocket(81);

// --- PINS (ESP32 & TB6612FNG Drivers) ---
#define STBY 23
#define PWMA1 25
#define AIN1_1 26
#define AIN2_1 27  
#define PWMB1 4
#define BIN1_1 16
#define BIN2_1 17  

#define PWMA2 32
#define AIN1_2 33
#define AIN2_2 21  
#define PWMB2 19
#define BIN1_2 18
#define BIN2_2 22  

#define IR_LEFT   15
#define IR_CENTER 35
#define IR_RIGHT  34  

#define TRIGGER_PIN 14
#define ECHO_PIN    2  
#define SERVO_PIN   13  
#define MAX_DISTANCE 200

// --- CONFIGURABLE SETTINGS ---
int OBSTACLE_THRESHOLD = 20; 
int MIN_CLEARANCE = 30;      
int BASE_SPEED = 90;         
int PIVOT_SPEED = 110;       

// DEFAULT TO RUNNING AUTONOMOUSLY ON BOOT
bool isRunning = true;
bool irEnabled = true;
String currentMode = "line"; 

const int SCAN_CENTER = 90;
enum State { LINE_FOLLOW, OBSTACLE_AVOID, SCANNING, REVERSING, MANUAL_MODE };
State currentState = LINE_FOLLOW;

unsigned long stateStartTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long obstacleAvoidTimer = 0;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Servo scanServo;

float filteredDistance = MAX_DISTANCE;
const float SMOOTHING_FACTOR = 0.3; 

// --- MOTOR DRIVE ---
void driveTank(int leftSpeed, int rightSpeed) {
  digitalWrite(STBY, HIGH);
  
  int lSpeed = constrain(abs(leftSpeed), 0, 255);
  analogWrite(PWMA1, lSpeed); analogWrite(PWMB1, lSpeed);
  if (leftSpeed >= 0) {
    digitalWrite(AIN1_1, HIGH); digitalWrite(AIN2_1, LOW);
    digitalWrite(BIN1_1, HIGH); digitalWrite(BIN2_1, LOW);
  } else {
    digitalWrite(AIN1_1, LOW); digitalWrite(AIN2_1, HIGH);
    digitalWrite(BIN1_1, LOW); digitalWrite(BIN2_1, HIGH);
  }
  
  int rSpeed = constrain(abs(rightSpeed), 0, 255);
  analogWrite(PWMA2, rSpeed); analogWrite(PWMB2, rSpeed);
  if (rightSpeed >= 0) {
    digitalWrite(AIN1_2, HIGH); digitalWrite(AIN2_2, LOW);
    digitalWrite(BIN1_2, HIGH); digitalWrite(BIN2_2, LOW);
  } else {
    digitalWrite(AIN1_2, LOW); digitalWrite(AIN2_2, HIGH);
    digitalWrite(BIN1_2, LOW); digitalWrite(BIN2_2, HIGH);
  }
}

void stopMotors() {
  analogWrite(PWMA1, 0); analogWrite(PWMB1, 0);
  analogWrite(PWMA2, 0); analogWrite(PWMB2, 0);
  digitalWrite(AIN1_1, LOW); digitalWrite(AIN2_1, LOW);
  digitalWrite(BIN1_1, LOW); digitalWrite(BIN2_1, LOW);
  digitalWrite(AIN1_2, LOW); digitalWrite(AIN2_2, LOW);
  digitalWrite(BIN1_2, LOW); digitalWrite(BIN2_2, LOW);
  digitalWrite(STBY, LOW);
}

// --- WEBSOCKET & TELEMETRY ---
String getStateString() {
  if (!isRunning) return "Paused";
  if (currentMode == "manual") return "Manual Drive";
  switch (currentState) {
    case LINE_FOLLOW: return "Autonomous / Line";
    case REVERSING: return "Reversing";
    case OBSTACLE_AVOID: return "Avoiding Obstacle";
    default: return "Running";
  }
}

void sendTelemetry() {
  StaticJsonDocument<256> doc;
  doc["distance"] = (int)filteredDistance;
  doc["battery"] = 5.0;
  doc["state"] = getStateString();
  doc["irLeft"] = digitalRead(IR_LEFT);
  doc["irCenter"] = digitalRead(IR_CENTER);
  doc["irRight"] = digitalRead(IR_RIGHT);

  String jsonString;
  serializeJson(doc, jsonString);
  webSocket.broadcastTXT(jsonString);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    StaticJsonDocument<512> doc;
    if (deserializeJson(doc, payload, length)) return;

    if (doc.containsKey("type") && strcmp(doc["type"], "settings") == 0) {
      BASE_SPEED = doc["baseSpeed"] | BASE_SPEED;
      PIVOT_SPEED = doc["pivotSpeed"] | PIVOT_SPEED;
      OBSTACLE_THRESHOLD = doc["obstacleThreshold"] | OBSTACLE_THRESHOLD;
      irEnabled = doc["irEnabled"] | irEnabled;
      currentMode = doc["mode"] | currentMode;
      isRunning = doc["isRunning"] | isRunning;

      if (!isRunning) stopMotors();
    } 
    else if (doc.containsKey("type") && strcmp(doc["type"], "joystick") == 0) {
      // If user uses joystick, instantly switch to manual mode
      currentMode = "manual";
      isRunning = true;

      float x = doc["x"]; 
      float y = doc["y"]; 

      if (abs(x) < 0.05 && abs(y) < 0.05) {
        stopMotors();
        return;
      }

      float throttle = -y; 
      float steering = x;  

      int leftSpeed = (throttle + steering) * BASE_SPEED;
      int rightSpeed = (throttle - steering) * BASE_SPEED;
      driveTank(leftSpeed, rightSpeed);
    }
  }
}

void updateDistance() {
  unsigned int cm = sonar.ping_cm();
  if (cm > 0 && cm < MAX_DISTANCE) {
    if ((float)cm < filteredDistance) filteredDistance = (float)cm;
    else filteredDistance = SMOOTHING_FACTOR * (float)cm + (1.0 - SMOOTHING_FACTOR) * filteredDistance;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(STBY, OUTPUT);
  pinMode(PWMA1, OUTPUT); pinMode(AIN1_1, OUTPUT); pinMode(AIN2_1, OUTPUT);
  pinMode(PWMB1, OUTPUT); pinMode(BIN1_1, OUTPUT); pinMode(BIN2_1, OUTPUT);
  pinMode(PWMA2, OUTPUT); pinMode(AIN1_2, OUTPUT); pinMode(AIN2_2, OUTPUT);
  pinMode(PWMB2, OUTPUT); pinMode(BIN1_2, OUTPUT); pinMode(BIN2_2, OUTPUT);

  pinMode(IR_LEFT, INPUT); pinMode(IR_CENTER, INPUT); pinMode(IR_RIGHT, INPUT);
  
  ESP32PWM::allocateTimer(0);
  scanServo.setPeriodHertz(50); 
  scanServo.attach(SERVO_PIN, 500, 2400);
  scanServo.write(SCAN_CENTER); 
  stopMotors();

  // Non-blocking Wi-Fi attempt in background (won't freeze robot if router is missing)
  WiFi.begin(ssid, password);
  unsigned long startWiFi = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startWiFi < 4000) {
    delay(200);
  }
  if (WiFi.status() != WL_CONNECTED && ENABLE_AP_FALLBACK) {
    WiFi.softAP(ap_ssid, ap_password);
  }

  MDNS.begin("robo");
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", index_html); });
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // --- 2 SECOND DELAY ON BOOT BEFORE MOVING ---
  Serial.println("Booting up... Robot will start in 2 seconds.");
  delay(2000); 
  stateStartTime = millis();
}

void loop() {
  server.handleClient();
  webSocket.loop();
  
  unsigned long currentTime = millis();
  
  if (currentTime - lastTelemetryTime >= 200) {
    lastTelemetryTime = currentTime;
    sendTelemetry();
  }

  if (!isRunning) {
    stopMotors();
    return;
  }

  // If manual mode is active via web browser joystick, skip autonomous loop
  if (currentMode == "manual") {
    return;
  }

  updateDistance(); 

  // --- SMOOTH AUTONOMOUS OBSTACLE / LINE BEHAVIOR ---
  if (currentMode == "line" || currentMode == "circle" || currentMode == "square") {
    
    // If an obstacle is detected too close, back up and turn smoothly instead of jittering
    if (filteredDistance < OBSTACLE_THRESHOLD && filteredDistance > 0) {
      driveTank(-BASE_SPEED * 0.8, -BASE_SPEED * 0.8); // Back up briefly
      delay(300);
      driveTank(PIVOT_SPEED, -PIVOT_SPEED);            // Turn away
      delay(400);
      return;
    }

    // Normal forward movement or IR tracking
    bool centerIR = digitalRead(IR_CENTER);
    
    if (irEnabled && centerIR == HIGH) {
      // Follows line/surface
      driveTank(BASE_SPEED, BASE_SPEED);
    } else {
      // Gentle cruising / searching forward if no line or IR is disabled
      driveTank(BASE_SPEED, BASE_SPEED);
    }
  }
}