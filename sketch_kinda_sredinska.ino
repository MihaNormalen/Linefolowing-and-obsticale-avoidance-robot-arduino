// Sledilec Črte in Izogibalec Oviran (Verzija 8.0 - Združeno)
//
// Ta koda združuje:
// 1. Vaše želene nastavitve pinov in hitrosti (iz V7.3 / sketch_nov6f_Kinda_ok.ino).
// 2. Popravljeno logiko sledenja s 3 senzorji (iz V7.13), ki pravilno vozi naravnost.
// 3. 5-sekundni zamik ob zagonu.
// 4. Ohranjeno neblokirajočo FSM logiko, optimizacijo motorjev in hiter servo.

#include <NewPing.h>
#include <Servo.h>
#include <AFMotor.h>

// --- PIN DEFINICIJE (Po vaših nastavitvah) ---
#define IR_LEFT A5          
#define IR_CENTER A4        
#define IR_RIGHT A2         
#define TRIGGER_PIN A1      
#define ECHO_PIN A0         
#define SERVO_PIN 10        
#define MAX_DISTANCE 200    

// --- NASTAVITEV MOTORJEV (AFMotor Shield) ---
AF_DCMotor motorFL(3), motorFR(2), motorRL(4), motorRR(1);

// --- KONSTANTE ZA HITROST IN PRAGE (Po vaših nastavitvah) ---
const int OBSTACLE_THRESHOLD = 10; 
const int MIN_CLEARANCE = 20;      
const int BASE_SPEED = 80;        
const int PIVOT_SPEED = 130;       
const int LIGHT_ADJUST_SPEED = 100; 

// --- SERVO KOTI ---
const int SCAN_CENTER = 90;
const int SCAN_LEFT = 150;
const int SCAN_RIGHT = 30;

// --- KONSTANTE ZA ČAS ---
const unsigned long START_DELAY_TIME = 3000;  // NOVO: 5 sekund zamik
const unsigned long REVERSE_TIME = 1000;      
const unsigned long AVOID_TURN_TIME = 300;   
const unsigned long AVOID_FORWARD_TIME = 500; 
const unsigned long SERVO_SETTLE_TIME = 350; 
const unsigned long LOST_LINE_TIMEOUT = 20; // Čas pred agresivnim iskanjem
const unsigned long STATE_INTERVAL = 10;     
const unsigned long STATE_COOLDOWN_TIME = 50; 

// --- SPREMENLJIVKE STANJA ---
enum State { LINE_FOLLOW, OBSTACLE_AVOID, SCANNING, REVERSING };
enum Direction { LEFT, RIGHT, NONE }; // Potrebno za logiko V7.13
State currentState = LINE_FOLLOW;
Direction lastDirection = NONE; // Potrebno za logiko V7.13
unsigned long stateStartTime = 0;
unsigned long lastSeenTime = 0; // Potrebno za logiko V7.13
unsigned long lastStateExecuteTime = 0;
unsigned long stateChangeTime = 0; 
int avoidStep = 0; 

// --- SENZORJI ---
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Servo scanServo;

float filteredDistance = MAX_DISTANCE;
const float SMOOTHING_FACTOR = 0.4;
int lastSpeed = -1; 

// -----------------------------------------------------------------------
// LOW-LEVEL FUNKCIJE GIBANJA (Optimizirane)
// -----------------------------------------------------------------------

void setAllMotorSpeeds(int speed) {
  int s = constrain(speed, 0, 255);
  if (s != lastSpeed) {
    motorFL.setSpeed(s);
    motorFR.setSpeed(s);
    motorRL.setSpeed(s);
    motorRR.setSpeed(s);
    lastSpeed = s;
  }
}

void moveForward(int speed) {
  setAllMotorSpeeds(speed);
  motorFL.run(FORWARD);
  motorFR.run(FORWARD);
  motorRL.run(FORWARD);
  motorRR.run(FORWARD);
}

void moveBackward(int speed) {
  setAllMotorSpeeds(speed);
  motorFL.run(BACKWARD);
  motorFR.run(BACKWARD);
  motorRL.run(BACKWARD);
  motorRR.run(BACKWARD);
}

void pivotLeft(int speed) {
  setAllMotorSpeeds(speed);
  motorFL.run(BACKWARD);
  motorRL.run(BACKWARD);
  motorFR.run(FORWARD);
  motorRR.run(FORWARD);
}

void pivotRight(int speed) {
  setAllMotorSpeeds(speed);
  motorFL.run(FORWARD);
  motorRL.run(FORWARD);
  motorFR.run(BACKWARD);
  motorRR.run(BACKWARD);
}

void stopMotors() {
  setAllMotorSpeeds(0); 
  motorFL.run(RELEASE);
  motorFR.run(RELEASE);
  motorRL.run(RELEASE);
  motorRR.run(RELEASE);
}

// -----------------------------------------------------------------------
// FUNKCIJE STANJA IN SENZORJEV
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
  stateChangeTime = millis(); 
  stopMotors(); 
  avoidStep = 0;
  
  Serial.print("MENJAVA STANJA -> "); 
  switch(newState) {
    case LINE_FOLLOW: Serial.println("SLEDENJE CRTI"); break;
    case REVERSING: Serial.println("VZVRATNA VOZNJA"); break;
    case SCANNING: Serial.println("SKENIRANJE"); break;
    case OBSTACLE_AVOID: Serial.println("IZOGIBANJE OVIRI"); break;
  }
}

void readIRSensors(bool& L, bool& C, bool& R) {
  // PREDPOSTAVLJENA LOGIKA: LOW = Zaznano črno črto (1), HIGH = Bel (0)
  L = (digitalRead(IR_LEFT) == LOW);
  C = (digitalRead(IR_CENTER) == LOW);
  R = (digitalRead(IR_RIGHT) == LOW);
  
  if (L || C || R) {
    lastSeenTime = millis(); 
  }
}

// -----------------------------------------------------------------------
// FUNKCIJE ZA RAVNANJE S STANJI
// -----------------------------------------------------------------------

// ***********************************************************************
// *** USPEŠNA LOGIKA SLEDENJA (iz V7.13) ***
// ***********************************************************************
void handleLineFollowing() {
  // 1. Preveri oviro
  if (filteredDistance < OBSTACLE_THRESHOLD) {
    changeState(REVERSING);
    return;
  }

  bool L, C, R;
  readIRSensors(L, C, R);
  
  // State Delay Guard
  if (millis() - lastStateExecuteTime < STATE_INTERVAL) return;
  lastStateExecuteTime = millis();

  // Logika SLEDENJA ČRTI (POPRAVLJEN VRSTNI RED)

  // 1. Vsa stanja za VOŽNJO NARAVNOST (Najvišja prioriteta)
  // (0,1,0) = Samo center na črti
  // (1,1,1) = Prečna črta
  // (1,0,1) = Nenavadno stanje (prej klicalo STOP), zdaj gre naravnost
  if ( (!L && C && !R) || (L && C && R) || (L && !C && R) ) {
      moveForward(BASE_SPEED); // Uporabi vašo hitrost 80
      lastDirection = NONE;
  }
  // 2. NEŽEN ZAVOJ LEVO (C+R)
  else if (!L && C && R) {
      pivotLeft(LIGHT_ADJUST_SPEED); // Uporabi vašo hitrost 80
      lastDirection = LEFT;
  } 
  // 3. NEŽEN ZAVOJ DESNO (L+C)
  else if (L && C && !R) {
      pivotRight(LIGHT_ADJUST_SPEED); // Uporabi vašo hitrost 80
      lastDirection = RIGHT;
  }
  // 4. OSTER ZAVOJ LEVO (SAMO R)
  else if (!L && !C && R) {
      pivotLeft(PIVOT_SPEED); // Uporabi vašo hitrost 180
      lastDirection = LEFT;
  }
  // 5. OSTER ZAVOJ DESNO (SAMO L)
  else if (L && !C && !R) {
      pivotRight(PIVOT_SPEED); // Uporabi vašo hitrost 180
      lastDirection = RIGHT;
  } 
  // 6. IZGUBLJENA ČRTA (Vsi FALSE: 0,0,0)
  else if (!L && !C && !R) {
      // Agresivno iskanje v smeri zadnjega zavoja, če je preteklo preveč časa
      if (millis() - lastSeenTime > LOST_LINE_TIMEOUT) {
          if (lastDirection == LEFT) {
            pivotLeft(PIVOT_SPEED); 
          } else if (lastDirection == RIGHT) {
            pivotRight(PIVOT_SPEED); 
          } else {
             pivotLeft(PIVOT_SPEED); 
          }
      } else {
          stopMotors(); // Kratka pavza pred iskanjem
      }
  }
  // 7. OSTALI PRIMERI - Varnostna ustavitev
  else {
      stopMotors();
  }
}
// ***********************************************************************
// *** Konec logike sledenja ***
// ***********************************************************************


void handleReversing(unsigned long currentTime) {
  if (currentTime - stateStartTime < REVERSE_TIME) {
    moveBackward(BASE_SPEED); // Uporabi vašo hitrost 80
  } else {
    changeState(SCANNING);
  }
}

void handleScanning(unsigned long currentTime) {
  static int leftDist = 0, rightDist = 0;
  
  if (avoidStep > 0 && currentTime - stateStartTime < SERVO_SETTLE_TIME) {
    return; 
  }

  switch (avoidStep) {
    case 0: 
      scanServo.write(SCAN_CENTER); 
      stateStartTime = currentTime; 
      avoidStep++;
      break;

    case 1: 
      scanServo.write(SCAN_LEFT); 
      stateStartTime = currentTime;
      avoidStep++;
      break;

    case 2: 
      leftDist = (int)filteredDistance;
      Serial.print("Levo: "); Serial.print(leftDist); 
      scanServo.write(SCAN_RIGHT); 
      stateStartTime = currentTime;
      avoidStep++;
      break;

    case 3: 
      rightDist = (int)filteredDistance;
      Serial.print(", Desno: "); Serial.println(rightDist);
      scanServo.write(SCAN_CENTER);
      
      if (rightDist > MIN_CLEARANCE || leftDist > MIN_CLEARANCE) {
          if (rightDist > leftDist) {
            lastDirection = RIGHT;
            Serial.println("-> Obvoz DESNO");
          } else {
            lastDirection = LEFT;
            Serial.println("-> Obvoz LEVO");
          }
          changeState(OBSTACLE_AVOID);
      } else {
          Serial.println("-> Premalo prostora. Vzvratna vožnja.");
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
  
  unsigned long requiredTime;
  if (avoidStep == 1 || avoidStep == 3) {
      requiredTime = AVOID_FORWARD_TIME; 
  } else {
      requiredTime = AVOID_TURN_TIME; 
  }

  if (currentTime - stateStartTime >= requiredTime) {
    avoidStep++;
    stateStartTime = currentTime;
  }

  switch (avoidStep) {
    case 0: 
      (bypassRight) ? pivotLeft(PIVOT_SPEED) : pivotRight(PIVOT_SPEED); // Uporabi 180
      break;
    case 1: 
      moveForward(BASE_SPEED); // Uporabi 80
      break;
    case 2: 
      (bypassRight) ? pivotRight(PIVOT_SPEED) : pivotLeft(PIVOT_SPEED); // Uporabi 180
      break;
    case 3: 
      moveForward(BASE_SPEED); // Uporabi 80
      break;
    case 4: 
      (bypassRight) ? pivotRight(PIVOT_SPEED) : pivotLeft(PIVOT_SPEED); // Uporabi 180
      break;
    case 5: 
      changeState(LINE_FOLLOW);
      break;
  }
}

// -----------------------------------------------------------------------
// SETUP IN LOOP
// -----------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
  Serial.println("--- ROBOT ZAGNAN: V8.0 (Združena Logika) ---");

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_CENTER, INPUT); 
  pinMode(IR_RIGHT, INPUT);
  
  scanServo.attach(SERVO_PIN);
  scanServo.write(SCAN_CENTER); 
  
  stopMotors();
  stateStartTime = millis();

  // NOVO: 5-sekundni zamik ob zagonu
  Serial.println("Cakam 5 sekund...");
  delay(START_DELAY_TIME);
  Serial.println("ZACENJAM!");
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - stateChangeTime < STATE_COOLDOWN_TIME) {
    stopMotors(); 
    return; 
  }

  updateDistance(); 
  
  switch (currentState) {
    case LINE_FOLLOW:
      handleLineFollowing();
      break;
    case REVERSING:
      handleReversing(currentTime);
      break;
    case SCANNING:
      handleScanning(currentTime);
      break;
    case OBSTACLE_AVOID:
      handleObstacleAvoidance(currentTime);
      break;
  }
}
