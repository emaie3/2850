#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

VL53L0X handSensor;
VL53L0X voteSensor;
Servo gate;

// Pins
const int HAND_XSHUT = 2;
const int VOTE_XSHUT = 3;
const int SERVO_PIN = 9;

// Sensor addresses
const uint8_t HAND_ADDRESS = 0x30;
const uint8_t VOTE_ADDRESS = 0x31;

// Servo settings
const int CLOSED_ANGLE = 0;
const int OPEN_ANGLE = 90;
const unsigned long OPEN_TIME = 300;

// Hand detection
const int HAND_TRIGGER = 100;
const int HAND_RESET = 120;
bool handReady = true;

// Calibrated voting distances
const int holeMin[4] = {250, 326, 426, 486};
const int holeMax[4] = {325, 425, 485, 550};

// Duplicate vote prevention
const unsigned long MIN_LOCKOUT = 600;
const unsigned long CLEAR_TIME = 150;
const int CLEAR_DISTANCE = 500;

bool voteReady = true;
unsigned long lastVoteTime = 0;
unsigned long clearStart = 0;

// Non-blocking servo timing
bool gateOpen = false;
unsigned long gateOpenedAt = 0;

void releaseSensor(int pin) {
  pinMode(pin, INPUT);
}

void shutDownSensor(int pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Hold both sensors in shutdown
  shutDownSensor(HAND_XSHUT);
  shutDownSensor(VOTE_XSHUT);
  delay(20);

  // Initialise hand sensor
  releaseSensor(HAND_XSHUT);
  delay(20);

  handSensor.setTimeout(100);

  if (!handSensor.init()) {
    Serial.println("ERROR:HAND_SENSOR");
    while (1);
  }

  handSensor.setAddress(HAND_ADDRESS);

  // Initialise voting sensor
  releaseSensor(VOTE_XSHUT);
  delay(20);

  voteSensor.setTimeout(100);

  if (!voteSensor.init()) {
    Serial.println("ERROR:VOTE_SENSOR");
    while (1);
  }

  voteSensor.setAddress(VOTE_ADDRESS);

  handSensor.startContinuous(30);
  voteSensor.startContinuous(20);

  gate.attach(SERVO_PIN);
  gate.write(CLOSED_ANGLE);

  Serial.println("READY");
}

void loop() {
  unsigned long now = millis();

  // -------- SENSOR 1: HAND + SERVO --------

  int handDistance = handSensor.readRangeContinuousMillimeters();
  Serial.print("Hand: ");
  Serial.print(handDistance);
  Serial.print(" mm | Ready: ");
  Serial.println(handReady ? "YES" : "NO");

  if (!handSensor.timeoutOccurred()) {

    if (handReady && !gateOpen &&
        handDistance > 0 && handDistance < HAND_TRIGGER) {

      gate.write(OPEN_ANGLE);
      gateOpen = true;
      gateOpenedAt = now;
      handReady = false;
    }

    if (handDistance > HAND_RESET) {
      handReady = true;
    }
  }

  // Close gate after OPEN_TIME
  if (gateOpen && now - gateOpenedAt >= OPEN_TIME) {
    gate.write(CLOSED_ANGLE);
    gateOpen = false;
  }

  // -------- SENSOR 2: BALL VOTING --------

  int distance = voteSensor.readRangeContinuousMillimeters();

  // Print voting sensor distance
  Serial.print("Vote sensor distance: ");
  Serial.print(distance);
  Serial.println(" mm");

  if (voteSensor.timeoutOccurred()) {
    clearStart = 0;
    delay(100);
    return;
  }

  int vote = 0;

  for (int i = 0; i < 4; i++) {
    if (distance >= holeMin[i] && distance <= holeMax[i]) {
      vote = i + 1;
      break;
    }
  }

  if (voteReady && vote != 0) {
    Serial.print("VOTE:");
    Serial.println(vote);

    voteReady = false;
    lastVoteTime = now;
    clearStart = 0;
  }

  // Rearm after path has been clear
  if (!voteReady) {

    if (distance > CLEAR_DISTANCE) {
      if (clearStart == 0) clearStart = now;
    } else {
      clearStart = 0;
    }

    if (now - lastVoteTime >= MIN_LOCKOUT &&
        clearStart != 0 &&
        now - clearStart >= CLEAR_TIME) {

      voteReady = true;
      clearStart = 0;
    }
  }

  delay(100);
}