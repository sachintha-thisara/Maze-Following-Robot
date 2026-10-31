// ========================================================
// MAZE LOGIC (L/R HAND) + FLOAT MEDIAN FILTER + WALL RECOVERY
// FIXED: KICKSTART + SMART 180 TURN + PRECISION 1-WALL TRACKING
// ========================================================

const int BASE_PWM = 80;
const int TURN_PWM = 160;      
const float LEFT_TRIM  = 0.93;
const float RIGHT_TRIM = 0.91;

const int TURN_TIME = 220;           
const int TURN_TIME_180 = 380;       
const int ALIGN_TIME = 160;          
const int POST_TURN_PAUSE = 1000;    
const int CLEAR_JUNCTION_TIME = 450;

const float FRONT_CRASH_DIST = 5.0;      
const int BACKUP_TIME = 300;         
const int BACKUP_PWM = 80;          
const unsigned long STALL_TIMEOUT = 1200; 

// --- TUNED TARGET DISTANCE (IN CM) ---
float TARGET_DIST =1.0;         // Auto-updates when between 2 walls, starts at 5.5cm
const float WALL_DETECT_DIST = 25.0; 
const float MIN_SAFE_DIST = 3.0;     

const int ENA = 5;
const int IN1 = 6;  
const int IN2 = 7;  
const int IN3 = 8;  
const int IN4 = 4;  
const int ENB = 9;

const int SWITCH_PIN = 3;        
const int IR_PIN = A2;        
const int TRIG_F = 12; const int ECHO_F = 13;
const int TRIG_L = 10; const int ECHO_L = 11;
const int TRIG_R = A0; const int ECHO_R = A1;

const float wallThreshold = 15.0;    

char currentAction = 'S';
bool isStopped = true;

// ---------- Wall-follow PD controller ----------
const float KP        = 2.5;      // Slightly stronger proportional hold
const float KD        = 1.4;      
const float D_FILTER  = 0.4;      
const int   MAX_STEER = 40;

float lastErr = 0;
float dErrF = 0;                      
unsigned long lastPid = 0;
int lastWallMode = 0; 

// ---------- Float Sensor Filtering & Debounce ----------
float bufF[3] = {50.0, 50.0, 50.0};
float bufL[3] = {5.5, 5.5, 5.5};
float bufR[3] = {5.5, 5.5, 5.5};
int filterIdx = 0;

const int OPEN_CONFIRM_COUNT = 2; 
int leftOpenStreak  = 0;
int rightOpenStreak = 0;

// ---------- Stall Detection Trackers ----------
unsigned long lastMoveTime = 0;
float lastStallF = 0, lastStallL = 0, lastStallR = 0;

void setup() {
  Serial.begin(115200);
 
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
 
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(IR_PIN, INPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
 
  stopMotors();
  delay(2000);
  resetSensorBuffers();
}

// High-precision float distance (detects 0.1cm changes)
float getExactDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0) return 999.0;
  return (duration * 0.0343) / 2.0;  
}

float medianOf3(float a, float b, float c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

void resetSensorBuffers() {
  float f = getExactDistance(TRIG_F, ECHO_F); delay(10);
  float l = getExactDistance(TRIG_L, ECHO_L); delay(10);
  float r = getExactDistance(TRIG_R, ECHO_R); delay(10);

  // Auto-calibrate TARGET_DIST if robot is currently between two walls
  if (l < WALL_DETECT_DIST && r < WALL_DETECT_DIST) {
    TARGET_DIST = (l + r) / 2.0;
  }

  for (int i = 0; i < 3; i++) {
    bufF[i] = f; bufL[i] = l; bufR[i] = r;
  }
  leftOpenStreak = 0;
  rightOpenStreak = 0;
  lastWallMode = 0;
  
  lastMoveTime = millis();
  lastStallF = f; lastStallL = l; lastStallR = r;
}

void driveForwardRaw() {
  if (isStopped) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, 150 * LEFT_TRIM); 
    analogWrite(ENB, 150 * RIGHT_TRIM);
    delay(30);
    isStopped = false;
  }
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, BASE_PWM * LEFT_TRIM);
  analogWrite(ENB, BASE_PWM * RIGHT_TRIM);
}

void moveStraight(float dLeft, float dRight) {
  if (isStopped) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, 150 * LEFT_TRIM); 
    analogWrite(ENB, 150 * RIGHT_TRIM);
    delay(30);
    isStopped = false;
    
    lastErr = 0; 
    dErrF = 0; 
    lastPid = millis();
  }

  int leftSpeed  = BASE_PWM * LEFT_TRIM;
  int rightSpeed = BASE_PWM * RIGHT_TRIM;
  float err = 0;
  int currentWallMode = 0;

  // MODE 1: Two walls present
  if (dLeft < WALL_DETECT_DIST && dRight < WALL_DETECT_DIST) {
    currentWallMode = 1;
    err = (dLeft - dRight) * 0.5; // Scaled so 1cm drift = 1.0 err (matches single-wall mode)
    
    // Continuously learn the true center of the corridor when fairly centered
    if (abs(dLeft - dRight) < 3.0) {
      TARGET_DIST = 0.8 * TARGET_DIST + 0.2 * ((dLeft + dRight) / 2.0);
    }
    
    if (abs(err) <= 0.3) err = 0; // Tiny 3mm deadband to prevent motor buzz
  }
  // MODE 2: Only Left wall present (Right side is open)
  else if (dLeft < WALL_DETECT_DIST && dRight >= WALL_DETECT_DIST) {
    currentWallMode = 2;
    err = dLeft - TARGET_DIST;    // Full strength (removed * 0.5 and 2cm blind spot)
    if (abs(err) <= 0.3) err = 0;
  }
  // MODE 3: Only Right wall present (Left side is open)
  else if (dRight < WALL_DETECT_DIST && dLeft >= WALL_DETECT_DIST) {
    currentWallMode = 3;
    err = TARGET_DIST - dRight;   // Full strength (removed * 0.5 and 2cm blind spot)
    if (abs(err) <= 0.3) err = 0;
  }

  if (currentWallMode != 0) {
    unsigned long now = millis();
    float dt = (now - lastPid) / 1000.0;
    if (dt <= 0 || dt > 0.25 || currentWallMode != lastWallMode) {
      lastErr = err; dErrF = 0; dt = 0.02;
    }
    dErrF += D_FILTER * ((err - lastErr) / dt - dErrF);
    lastErr = err; lastPid = now;

    int correction = constrain((int)(KP * err + KD * dErrF), -MAX_STEER, MAX_STEER);
    leftSpeed -= correction; rightSpeed += correction;
  } else {
    lastErr = 0; dErrF = 0; lastPid = millis();
  }

  lastWallMode = currentWallMode;
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void spinRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  analogWrite(ENA, TURN_PWM * LEFT_TRIM);
  analogWrite(ENB, TURN_PWM * RIGHT_TRIM);
}

void spinLeft() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, TURN_PWM * LEFT_TRIM);
  analogWrite(ENB, TURN_PWM * RIGHT_TRIM);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);    analogWrite(ENB, 0);
  isStopped = true;
}

void hardBrake() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);   analogWrite(ENB, 255);
  delay(100); 
  stopMotors();
}

void checkWhiteLine() {
  if (digitalRead(IR_PIN) == LOW) {
    hardBrake();
    while (true) {}
  }
}

void backUpLittle(float dLeft, float dRight) {
  hardBrake();
  delay(150);

  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, BACKUP_PWM * LEFT_TRIM);
  analogWrite(ENB, BACKUP_PWM * RIGHT_TRIM);
  delay(BACKUP_TIME);

  hardBrake();
  delay(200);

  if (dLeft < dRight && dLeft < WALL_DETECT_DIST) {
    spinRight(); delay(100); hardBrake(); delay(100);
  } else if (dRight < dLeft && dRight < WALL_DETECT_DIST) {
    spinLeft(); delay(100); hardBrake(); delay(100);
  }

  resetSensorBuffers();
}

void recoverFromWall(char wallSide) {
  stopMotors(); delay(100);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, BACKUP_PWM * LEFT_TRIM); 
  analogWrite(ENB, BACKUP_PWM * RIGHT_TRIM);
  delay(BACKUP_TIME); 
  if (wallSide == 'L') { spinRight(); delay(150); } 
  else { spinLeft(); delay(150); }
  stopMotors(); delay(100);
  resetSensorBuffers(); 
}

void executeTurn(char direction, bool isSideJunction) {
  if (isSideJunction) { 
    float fDist = getExactDistance(TRIG_F, ECHO_F);
    if (fDist > FRONT_CRASH_DIST + 3.0) {
      driveForwardRaw(); 
      delay(ALIGN_TIME); 
    }
  }
  
  hardBrake(); 
  delay(300);

  if (direction == 'L') { 
    spinLeft(); 
    delay(TURN_TIME); 
  } 
  else if (direction == 'R') { 
    spinRight(); 
    delay(TURN_TIME); 
  } 
  else if (direction == 'B') { 
    float exactLeft = getExactDistance(TRIG_L, ECHO_L);
    delay(10);
    float exactRight = getExactDistance(TRIG_R, ECHO_R);

    if (exactRight >= exactLeft) {
      spinRight();
    } else {
      spinLeft();
    }
    delay(TURN_TIME_180); 
  }

  hardBrake(); 
  delay(POST_TURN_PAUSE); 

  resetSensorBuffers();

  if (direction != 'B') {
    unsigned long start = millis();
    while (millis() - start < CLEAR_JUNCTION_TIME) {
      checkWhiteLine();
      
      float safeDist = getExactDistance(TRIG_F, ECHO_F);
      if (safeDist <= FRONT_CRASH_DIST) {
        backUpLittle(getExactDistance(TRIG_L, ECHO_L), getExactDistance(TRIG_R, ECHO_R));
        break;
      }
      
      float dL = getExactDistance(TRIG_L, ECHO_L);
      float dR = getExactDistance(TRIG_R, ECHO_R);
      moveStraight(dL, dR); 
      
      delay(15); 
    }
  }
  
  currentAction = 'T'; 
  resetSensorBuffers();
}

void loop() {
  checkWhiteLine();
  bool useLeftHandRule = (digitalRead(SWITCH_PIN) == HIGH);

  bufF[filterIdx] = getExactDistance(TRIG_F, ECHO_F); delay(10);
  bufL[filterIdx] = getExactDistance(TRIG_L, ECHO_L); delay(10);
  bufR[filterIdx] = getExactDistance(TRIG_R, ECHO_R); delay(10);
  filterIdx = (filterIdx + 1) % 3;

  float distFront = medianOf3(bufF[0], bufF[1], bufF[2]);
  float distLeft  = medianOf3(bufL[0], bufL[1], bufL[2]);
  float distRight = medianOf3(bufR[0], bufR[1], bufR[2]);

  if (distFront <= FRONT_CRASH_DIST) {
    backUpLittle(distLeft, distRight);
    return;
  }

  if (distLeft < MIN_SAFE_DIST) { recoverFromWall('L'); return; } 
  else if (distRight < MIN_SAFE_DIST) { recoverFromWall('R'); return; }

  if (abs(distFront - lastStallF) > 1.0 || abs(distLeft - lastStallL) > 1.0 || abs(distRight - lastStallR) > 1.0) {
    lastMoveTime = millis();
    lastStallF = distFront;
    lastStallL = distLeft;
    lastStallR = distRight;
  } else if (millis() - lastMoveTime > STALL_TIMEOUT) {
    backUpLittle(distLeft, distRight);
    return;
  }

  if (distLeft >= WALL_DETECT_DIST) leftOpenStreak++; else leftOpenStreak = 0;
  if (distRight >= WALL_DETECT_DIST) rightOpenStreak++; else rightOpenStreak = 0;

  bool frontBlocked = (distFront < wallThreshold);
  bool leftOpen  = frontBlocked ? (distLeft >= WALL_DETECT_DIST)  : (leftOpenStreak >= OPEN_CONFIRM_COUNT);
  bool rightOpen = frontBlocked ? (distRight >= WALL_DETECT_DIST) : (rightOpenStreak >= OPEN_CONFIRM_COUNT);

  if (useLeftHandRule) {
    if (leftOpen) executeTurn('L', !frontBlocked);
    else if (!frontBlocked) { moveStraight(distLeft, distRight); currentAction = 'S'; }
    else if (rightOpen) executeTurn('R', false);
    else executeTurn('B', false);
  } 
  else {
    if (rightOpen) executeTurn('R', !frontBlocked);
    else if (!frontBlocked) { moveStraight(distLeft, distRight); currentAction = 'S'; }
    else if (leftOpen) executeTurn('L', false);
    else executeTurn('B', false);
  }
}
