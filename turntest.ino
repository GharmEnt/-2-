#include <SPI.h>
#include <MFRC522.h>

const int TRIG_TOP = 8;
const int ECHO_TOP = 9;
const int TRIG_BOTTOM = 11;
const int ECHO_BOTTOM = 10;

const int ENA = 2;
const int ENB = 7;
const int IN1 = 3;
const int IN2 = 4;
const int IN3 = 5;
const int IN4 = 6;

const int MOVE_SPEED_START = 90;
const int MOVE_SPEED_NORMAL = 70;
const int TURN_SPEED = 160;
const int FORWARD_DISTANCE = 7;
const float DETECTION_DISTANCE = 30.0;
const int NUM_SCAN_POSITIONS = 4;
const int TURN_90_DEGREES_MS = 690;
const float BACKWARD_RATIO = 0.85;
const int RAMP_DOWN_TIME = 200;
const float APPROACH_DISTANCE = 10.0;
const int BACKWARD_FROM_OBJECT_MS = 1000;

#define RST_PIN_TOP    47
#define SS_PIN_TOP     49
#define RST_PIN_BOTTOM 48
#define SS_PIN_BOTTOM  53

MFRC522 mfrc522_top(SS_PIN_TOP, RST_PIN_TOP);
MFRC522 mfrc522_bottom(SS_PIN_BOTTOM, RST_PIN_BOTTOM);

struct ColorUID {
    const char* colorName;
    byte uid[4];
};

ColorUID bottomColors[] = {
    {"Синий",   {146, 59, 27, 33}},
    {"Красный", {162, 235, 216, 33}},
    {"Зеленый", {146, 119, 102, 33}},
    {"Черный",  {210, 70, 16, 32}}
};

ColorUID topColors[] = {
    {"Синий",   {67, 29, 136, 29}},
    {"Синий",   {131, 195, 11, 28}},
    {"Красный", {211, 56, 205, 27}},
    {"Красный", {115, 43, 3, 28}},
    {"Зеленый", {195, 34, 203, 27}},
    {"Зеленый", {111, 11, 111, 11}},
    {"Белый",   {179, 51, 8, 28}},
    {"Белый",   {131, 80, 238, 27}}
};

unsigned long forwardStartTime = 0;
unsigned long forwardDuration = 0;
bool isMovingForward = false;
String detectedBottomColor = "";
String detectedColors[NUM_SCAN_POSITIONS];
int currentScanPosition = 0;
bool targetFound = false;
int foundPosition = -1;
unsigned long rotationStartTime = 0;
unsigned long rotationDuration = 0;

float getDistance(int trigPin, int echoPin);
String readRFIDColorWithRetry(MFRC522 &mfrc522, ColorUID colors[], int colorCount, int maxAttempts, int intervalMs);
String readRFIDColor(MFRC522 &mfrc522, ColorUID colors[], int colorCount);
void moveForward(int speed);
void moveForwardRamp();
void moveBackward(int speed);
void moveBackwardRamp();
void turnLeft(int speed);
void turnRight(int speed);
void stopMotor();
int calculateTimeForDistance(float distance_cm);
void scanPosition(int position);
void findAndGoToTargetColor();
void rotateAndScanForTarget();
void returnToStartPosition();
void searchForObjectAndCompare();
void goToPosition(int position);

void setup() {
    Serial.begin(9600);
    SPI.begin();
    
    mfrc522_top.PCD_Init();
    mfrc522_bottom.PCD_Init();
    
    pinMode(TRIG_TOP, OUTPUT);
    pinMode(ECHO_TOP, INPUT);
    pinMode(TRIG_BOTTOM, OUTPUT);
    pinMode(ECHO_BOTTOM, INPUT);
    
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    stopMotor();
}

void loop() {
    for (int i = 0; i < NUM_SCAN_POSITIONS; i++) {
        detectedColors[i] = "";
    }
    currentScanPosition = 0;
    targetFound = false;
    foundPosition = -1;
    rotationDuration = 0;
    
    scanPosition(0);
    
    for (int i = 1; i < NUM_SCAN_POSITIONS; i++) {
        turnLeft(TURN_SPEED);
        delay(TURN_90_DEGREES_MS);
        stopMotor();
        
        scanPosition(i);
    }
    
    returnToStartPosition();
    
    searchForObjectAndCompare();
    
    while(true);
}

void scanPosition(int position) {
    forwardDuration = 0;
    forwardStartTime = 0;
    
    moveForwardRamp();
    
    while (getDistance(TRIG_BOTTOM, ECHO_BOTTOM) <= 5.0) {
        moveForward(MOVE_SPEED_NORMAL);
        delay(50);
    }
    stopMotor();
    
    if (forwardStartTime > 0) {
        forwardDuration = millis() - forwardStartTime;
    }
    
    detectedBottomColor = readRFIDColorWithRetry(mfrc522_bottom, bottomColors, 4, 5, 500);
    
    detectedColors[position] = detectedBottomColor;
    
    if (forwardDuration > 0) {
        unsigned long backwardTime = forwardDuration * BACKWARD_RATIO;
        
        moveBackwardRamp();
        delay(backwardTime);
        stopMotor();
    } else {
        moveBackwardRamp();
        delay(2000);
        stopMotor();
    }
}

void returnToStartPosition() {
    turnLeft(TURN_SPEED);
    delay(TURN_90_DEGREES_MS);
    stopMotor();
    delay(200);
}

void searchForObjectAndCompare() {
    bool objectFound = false;
    
    rotationStartTime = millis();
    
    turnLeft(TURN_SPEED);
    
    while (!objectFound) {
        float distance = getDistance(TRIG_TOP, ECHO_TOP);
        
        if (distance <= DETECTION_DISTANCE && distance > 2.0) {
            stopMotor();
            
            rotationDuration = millis() - rotationStartTime;
            
            objectFound = true;
            break;
        }
        
        delay(30);
    }
    
    if (!objectFound) {
        return;
    }
    
    turnLeft(TURN_SPEED);
    delay(100);
    stopMotor();
    
    float targetDistance = APPROACH_DISTANCE;
    
    moveForwardRamp();
    
    while (getDistance(TRIG_TOP, ECHO_TOP) > targetDistance + 2.0) {
        moveForward(MOVE_SPEED_NORMAL / 2);
        delay(100);
        
        float currentDist = getDistance(TRIG_TOP, ECHO_TOP);
        
        if (currentDist < 5.0) {
            break;
        }
    }
    stopMotor();
    
    String objectColor = readRFIDColorWithRetry(mfrc522_top, topColors, 8, 5, 500);
    
    if (objectColor == "") {
        return;
    }
    
    foundPosition = -1;
    for (int i = 0; i < NUM_SCAN_POSITIONS; i++) {
        if (detectedColors[i] == objectColor) {
            foundPosition = i;
            break;
        }
    }
    
    if (foundPosition == -1) {
        return;
    }
    
    moveBackwardRamp();
    delay(BACKWARD_FROM_OBJECT_MS);
    stopMotor();
    
    if (rotationDuration > 0) {
        turnRight(TURN_SPEED);
        delay(rotationDuration);
        stopMotor();
    }
    
    goToPosition(foundPosition);
}

void goToPosition(int position) {
    int turnsNeeded = position;
    
    for (int i = 0; i < turnsNeeded; i++) {
        turnLeft(TURN_SPEED);
        delay(TURN_90_DEGREES_MS);
        stopMotor();
        delay(200);
    }
    
    if (position == 3) {
        turnRight(TURN_SPEED);
        delay(TURN_90_DEGREES_MS);
        stopMotor();
        delay(200);
    }
    
    forwardDuration = 0;
    forwardStartTime = 0;
    
    moveForwardRamp();
    
    while (getDistance(TRIG_BOTTOM, ECHO_BOTTOM) <= 5.0) {
        moveForward(MOVE_SPEED_NORMAL);
        delay(50);
    }
    stopMotor();
    
    if (forwardStartTime > 0) {
        forwardDuration = millis() - forwardStartTime;
        unsigned long backwardTime = forwardDuration * BACKWARD_RATIO;
        
        moveBackwardRamp();
        delay(backwardTime);
        stopMotor();
    }
}

void moveForwardRamp() {
    int speedSteps = 4;
    int stepDelay = RAMP_DOWN_TIME / speedSteps;
    
    moveForward(MOVE_SPEED_START);
    delay(stepDelay);
    
    moveForward(85);
    delay(stepDelay);
    
    moveForward(80);
    delay(stepDelay);
    
    moveForward(75);
    delay(stepDelay);
    
    moveForward(MOVE_SPEED_NORMAL);
    
    if (!isMovingForward) {
        forwardStartTime = millis();
        isMovingForward = true;
    }
}

void moveBackwardRamp() {
    int speedSteps = 4;
    int stepDelay = RAMP_DOWN_TIME / speedSteps;
    
    moveBackward(MOVE_SPEED_START);
    delay(stepDelay);
    
    moveBackward(85);
    delay(stepDelay);
    
    moveBackward(80);
    delay(stepDelay);
    
    moveBackward(75);
    delay(stepDelay);
    
    moveBackward(MOVE_SPEED_NORMAL);
    isMovingForward = false;
}

float getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration == 0) {
        return 500.0;
    }
    
    float distance = duration * 0.034 / 2;
    return distance;
}

String readRFIDColorWithRetry(MFRC522 &mfrc522, ColorUID colors[], int colorCount, int maxAttempts, int intervalMs) {
    String result = "";
    int attempts = 0;
    
    while (result == "" && attempts < maxAttempts) {
        attempts++;
        
        result = readRFIDColor(mfrc522, colors, colorCount);
        
        if (result == "" && attempts < maxAttempts) {
            delay(intervalMs);
        }
    }
    
    return result;
}

String readRFIDColor(MFRC522 &mfrc522, ColorUID colors[], int colorCount) {
    if (!mfrc522.PICC_IsNewCardPresent()) {
        return "";
    }
    
    if (!mfrc522.PICC_ReadCardSerial()) {
        return "";
    }
    
    for (int i = 0; i < colorCount; i++) {
        bool match = true;
        for (byte j = 0; j < 4; j++) {
            if (mfrc522.uid.uidByte[j] != colors[i].uid[j]) {
                match = false;
                break;
            }
        }
        if (match) {
            mfrc522.PICC_HaltA();
            return String(colors[i].colorName);
        }
    }
    
    mfrc522.PICC_HaltA();
    return "Неизвестный";
}

int calculateTimeForDistance(float distance_cm) {
    float time_per_cm = 50.0;
    return (int)(distance_cm * time_per_cm);
}

void moveForward(int speed) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
}

void moveBackward(int speed) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
}

void turnLeft(int speed) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    isMovingForward = false;
}

void turnRight(int speed) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    isMovingForward = false;
}

void stopMotor() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}