#include <ESP32Servo.h>
#include <NewPing.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// --- WI-FI PODATKI (Vpišite svoje) ---
const char* ssid = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";

WebSocketsServer webSocket(81);

// --- PIN DEFINICIJE ZA ESP32 IN DVA TB6612FNG GONILNIKA ---
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

// --- IR SENZORJI ---
#define IR_LEFT   15
#define IR_CENTER 35
#define IR_RIGHT  34  

// --- ULTRAZVOČNI SENZOR IN SERVO ---
#define TRIGGER_PIN 14
#define ECHO_PIN    2  
#define SERVO_PIN   13  
#define MAX_DISTANCE 200

// --- NASTAVLJIVE KONSTANTE ---
int OBSTACLE_THRESHOLD = 15; 
int MIN_CLEARANCE = 25;      
int BASE_SPEED = 140;        
int PIVOT_SPEED = 170;       
int LIGHT_ADJUST_SPEED = 110; 
int PID_P = 50;
int PID_I = 25;
bool lineFollowingEnabled = true;

// --- SERVO KOTI ---
const int SCAN_CENTER = 90;
const int SCAN_LEFT = 150;
const int SCAN_RIGHT = 30;

// --- ČASOVNE KONSTANTE ---
const unsigned long START_DELAY_TIME = 3000;  
const unsigned long REVERSE_TIME = 1000;      
const unsigned long AVOID_TURN_TIME = 300;   
const unsigned long AVOID_FORWARD_TIME = 500; 
const unsigned long SERVO_SETTLE_TIME = 350; 
const unsigned long LOST_LINE_TIMEOUT = 20; 
const unsigned long STATE_INTERVAL = 10;     
const unsigned long STATE_COOLDOWN_TIME = 50; 

enum State { LINE_FOLLOW, OBSTACLE_AVOID, SCANNING, REVERSING, MANUAL_IDLE };
enum Direction { LEFT, RIGHT, NONE }; 
State currentState = LINE_FOLLOW;
Direction lastDirection = NONE; 

unsigned long stateStartTime = 0;
unsigned long lastSeenTime = 0; 
unsigned long lastStateExecuteTime = 0;
unsigned long lastStateChangeTime = 0; 
unsigned long lastTelemetryTime = 0;
int avoidStep = 0; 

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Servo scanServo;

float filteredDistance = MAX_DISTANCE;
const float SMOOTHING_FACTOR = 0.4;
int lastSpeed = -1; 

// -----------------------------------------------------------------------
// GIBANJE MOTORJEV
// -----------------------------------------------------------------------
void setAllMotorSpeeds(int speed) {
  int s = constrain(speed, 0, 255);
  if (s != lastSpeed) {
    analogWrite(PWMA1, s);
    analogWrite(PWMB1, s);
    analogWrite(PWMA2, s);
    analogWrite(PWMB2, s);
    lastSpeed = s;
  }
}

void moveForward(int speed) {
  digitalWrite(STBY, HIGH);
  setAllMotorSpeeds(speed);
  digitalWrite(AIN1_1, HIGH); digitalWrite(AIN2_1, LOW);
  digitalWrite(BIN1_1, HIGH); digitalWrite(BIN2_1, LOW);
  digitalWrite(AIN1_2, HIGH); digitalWrite(AIN2_2, LOW);
  digitalWrite(BIN1_2, HIGH); digitalWrite(BIN2_2, LOW);
}

void moveBackward(int speed) {
  digitalWrite(STBY, HIGH);
  setAllMotorSpeeds(speed);
  digitalWrite(AIN1_1, LOW); digitalWrite(AIN2_1, HIGH);
  digitalWrite(BIN1_1, LOW); digitalWrite(BIN2_1, HIGH);
  digitalWrite(AIN1_2, LOW); digitalWrite(AIN2_2, HIGH);
  digitalWrite(BIN1_2, LOW); digitalWrite(BIN2_2, HIGH);
}

void pivotLeft(int speed) {
  digitalWrite(STBY, HIGH);
  setAllMotorSpeeds(speed);
  digitalWrite(AIN1_1, LOW); digitalWrite(AIN2_1, HIGH);
  digitalWrite(BIN1_1, LOW); digitalWrite(BIN2_1, HIGH);
  digitalWrite(AIN1_2, HIGH); digitalWrite(AIN2_2, LOW);
  digitalWrite(BIN1_2, HIGH); digitalWrite(BIN2_2, LOW);
}

void pivotRight(int speed) {
  digitalWrite(STBY, HIGH);
  setAllMotorSpeeds(speed);
  digitalWrite(AIN1_1, HIGH); digitalWrite(AIN2_1, LOW);
  digitalWrite(BIN1_1, HIGH); digitalWrite(BIN2_1, LOW);
  digitalWrite(AIN1_2, LOW); digitalWrite(AIN2_2, HIGH);
  digitalWrite(BIN1_2, LOW); digitalWrite(BIN2_2, HIGH);
}

void stopMotors() {
  setAllMotorSpeeds(0); 
  digitalWrite(AIN1_1, LOW); digitalWrite(AIN2_1, LOW);
  digitalWrite(BIN1_1, LOW); digitalWrite(BIN2_1, LOW);
  digitalWrite(AIN1_2, LOW); digitalWrite(AIN2_2, LOW);
  digitalWrite(BIN1_2, LOW); digitalWrite(BIN2_2, LOW);
  digitalWrite(STBY, LOW);
}

// -----------------------------------------------------------------------
// WEBSOCKET UKAZI (JSON - Posodobljeno za JsonDocument)
// -----------------------------------------------------------------------
void handleWebSocketMessage(uint8_t *payload, size_t length) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) return;

  if (doc.containsKey("type")) {
    String type = doc["type"].as<String>();
    
    if (type == "settings") {
      if (doc["baseSpeed"]) BASE_SPEED = doc["baseSpeed"];
      if (doc["pivotSpeed"]) PIVOT_SPEED = doc["pivotSpeed"];
      if (doc["adjustSpeed"]) LIGHT_ADJUST_SPEED = doc["adjustSpeed"];
      if (doc["obstacleThreshold"]) OBSTACLE_THRESHOLD = doc["obstacleThreshold"];
      if (doc["minClearance"]) MIN_CLEARANCE = doc["minClearance"];
      if (doc["pidP"]) PID_P = doc["pidP"];
      if (doc["pidI"]) PID_I = doc["pidI"];
    } 
    else if (type == "toggleLine") {
      lineFollowingEnabled = doc["enabled"];
      if (!lineFollowingEnabled) {
        currentState = MANUAL_IDLE;
        stopMotors();
      } else {
        currentState = LINE_FOLLOW;
      }
    }
    else if (type == "manualCmd") {
      String cmd = doc["cmd"].as<String>();
      currentState = MANUAL_IDLE;
      if (cmd == "forward") moveForward(BASE_SPEED);
      else if (cmd == "backward") moveBackward(BASE_SPEED);
      else if (cmd == "stop") stopMotors();
    }
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED: break;
    case WStype_CONNECTED: break;
    case WStype_TEXT:
      handleWebSocketMessage(payload, length);
      break;
    default: break;
  }
}

void sendTelemetry() {
  bool l = (digitalRead(IR_LEFT) == LOW);
  bool c = (digitalRead(IR_CENTER) == LOW);
  bool r = (digitalRead(IR_RIGHT) == LOW);

  JsonDocument doc;
  doc["distance"] = (int)filteredDistance;
  doc["irLeft"] = l;
  doc["irCenter"] = c;
  doc["irRight"] = r;
  doc["battery"] = 5.1; 
  doc["speed"] = (lastSpeed >= 0 ? (lastSpeed * 100) / 255 : 0);
  doc["temp"] = 28.5; 
  
  switch(currentState) {
    case LINE_FOLLOW: doc["state"] = "Line Following"; break;
    case OBSTACLE_AVOID: doc["state"] = "Avoiding Obstacle"; break;
    case SCANNING: doc["state"] = "Scanning"; break;
    case REVERSING: doc["state"] = "Reversing"; break;
    case MANUAL_IDLE: doc["state"] = "Manual / Idle"; break;
  }

  String jsonString;
  serializeJson(doc, jsonString);
  webSocket.broadcastTXT(jsonString);
}

// -----------------------------------------------------------------------
// LOGIKA ROBOTA
// -----------------------------------------------------------------------
void updateDistance() {
  unsigned int cm = sonar.ping_cm();
  if (cm > 0 && cm < MAX_DISTANCE) {
    filteredDistance = SMOOTHING_FACTOR * (float)cm + (1.0 - SMOOTHING_FACTOR) * filteredDistance;
  } else {
    if (filteredDistance > MAX_DISTANCE) filteredDistance = MAX_DISTANCE;
  }
}

void changeState(State newState) {
  currentState = newState;
  stateStartTime = millis();
  lastStateExecuteTime = millis(); 
  lastStateChangeTime = millis(); 
  stopMotors(); 
  avoidStep = 0;
}

void readIRSensors(bool& L, bool& C, bool& R) {
  L = (digitalRead(IR_LEFT) == LOW);
  C = (digitalRead(IR_CENTER) == LOW);
  R = (digitalRead(IR_RIGHT) == LOW);
  if (L || C || R) lastSeenTime = millis(); 
}

void handleLineFollowing() {
  if (filteredDistance < OBSTACLE_THRESHOLD && filteredDistance > 0) {
    changeState(REVERSING);
    return;
  }

  bool L, C, R;
  readIRSensors(L, C, R);
  
  if (millis() - lastStateExecuteTime < STATE_INTERVAL) return;
  lastStateExecuteTime = millis();

  if ( (!L && C && !R) || (L && C && R) || (L && !C && R) ) {
      moveForward(BASE_SPEED);
      lastDirection = NONE;
  }
  else if (!L && C && R) {
      pivotLeft(LIGHT_ADJUST_SPEED);
      lastDirection = LEFT;
  } 
  else if (L && C && !R) {
      pivotRight(LIGHT_ADJUST_SPEED);
      lastDirection = RIGHT;
  }
  else if (!L && !C && R) {
      pivotLeft(PIVOT_SPEED);
      lastDirection = LEFT;
  }
  else if (L && !C && !R) {
      pivotRight(PIVOT_SPEED);
      lastDirection = RIGHT;
  } 
  else if (!L && !C && !R) {
      if (millis() - lastSeenTime > LOST_LINE_TIMEOUT) {
          if (lastDirection == LEFT) pivotLeft(PIVOT_SPEED); 
          else if (lastDirection == RIGHT) pivotRight(PIVOT_SPEED); 
          else pivotLeft(PIVOT_SPEED); 
      } else {
          stopMotors();
      }
  }
  else {
      stopMotors();
  }
}

void handleReversing(unsigned long currentTime) {
  if (currentTime - stateStartTime < REVERSE_TIME) {
    moveBackward(BASE_SPEED);
  } else {
    changeState(SCANNING);
  }
}

void handleScanning(unsigned long currentTime) {
  static int leftDist = 0, rightDist = 0;
  if (avoidStep > 0 && currentTime - stateStartTime < SERVO_SETTLE_TIME) return; 

  switch (avoidStep) {
    case 0: scanServo.write(SCAN_CENTER); stateStartTime = currentTime; avoidStep++; break;
    case 1: scanServo.write(SCAN_LEFT); stateStartTime = currentTime; avoidStep++; break;
    case 2: 
      leftDist = (int)filteredDistance;
      scanServo.write(SCAN_RIGHT); 
      stateStartTime = currentTime;
      avoidStep++;
      break;
    case 3: 
      rightDist = (int)filteredDistance;
      scanServo.write(SCAN_CENTER);
      if (rightDist > MIN_CLEARANCE || leftDist > MIN_CLEARANCE) {
          if (rightDist > leftDist) lastDirection = RIGHT;
          else lastDirection = LEFT;
          changeState(OBSTACLE_AVOID);
      } else {
          changeState(REVERSING); 
      }
      avoidStep = 0;
      break;
  }
}

void handleObstacleAvoidance(unsigned long currentTime) {
  if (currentTime - lastStateExecuteTime < STATE_INTERVAL) return;
  lastStateExecuteTime = currentTime; 

  bool bypassRight = (lastDirection == RIGHT);
  unsigned long requiredTime = (avoidStep == 1 || avoidStep == 3) ? AVOID_FORWARD_TIME : AVOID_TURN_TIME; 

  if (currentTime - stateStartTime >= requiredTime) {
    avoidStep++;
    stateStartTime = currentTime;
  }

  switch (avoidStep) {
    case 0: (bypassRight) ? pivotLeft(PIVOT_SPEED) : pivotRight(PIVOT_SPEED); break;
    case 1: moveForward(BASE_SPEED); break;
    case 2: (bypassRight) ? pivotRight(PIVOT_SPEED) : pivotLeft(PIVOT_SPEED); break;
    case 3: moveForward(BASE_SPEED); break;
    case 4: (bypassRight) ? pivotRight(PIVOT_SPEED) : pivotLeft(PIVOT_SPEED); break;
    case 5: changeState(LINE_FOLLOW); break;
  }
}

// -----------------------------------------------------------------------
// SETUP & LOOP
// -----------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  
  Serial.println("");
  Serial.println("Wi-Fi je uspešno povezan!");
  Serial.print("IP naslov ESP32 je: ");
  Serial.println(WiFi.localIP());

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  pinMode(STBY, OUTPUT);
  pinMode(PWMA1, OUTPUT); pinMode(AIN1_1, OUTPUT); pinMode(AIN2_1, OUTPUT);
  pinMode(PWMB1, OUTPUT); pinMode(BIN1_1, OUTPUT); pinMode(BIN2_1, OUTPUT);
  pinMode(PWMA2, OUTPUT); pinMode(AIN1_2, OUTPUT); pinMode(AIN2_2, OUTPUT);
  pinMode(PWMB2, OUTPUT); pinMode(BIN1_2, OUTPUT); pinMode(BIN2_2, OUTPUT);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_CENTER, INPUT); 
  pinMode(IR_RIGHT, INPUT);
  
  scanServo.attach(SERVO_PIN, 500, 2400);
  scanServo.write(SCAN_CENTER); 
  
  stopMotors();
  stateStartTime = millis();
  delay(START_DELAY_TIME);
}

void loop() {
  webSocket.loop();
  unsigned long currentTime = millis();
  
  if (currentTime - lastTelemetryTime > 200) {
    updateDistance();
    sendTelemetry();
    lastTelemetryTime = currentTime;
  }

  if (currentState == MANUAL_IDLE) return;

  if (currentTime - lastStateChangeTime < STATE_COOLDOWN_TIME) { // Popravljeno iz stateChangeTime v lastStateChangeTime
    stopMotors(); 
    return; 
  }

  switch (currentState) {
    case LINE_FOLLOW: 
      if (lineFollowingEnabled) handleLineFollowing(); 
      else stopMotors();
      break;
    case REVERSING: handleReversing(currentTime); break;
    case SCANNING: handleScanning(currentTime); break;
    case OBSTACLE_AVOID: handleObstacleAvoidance(currentTime); break;
    case MANUAL_IDLE: break;
  }
}