
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
const int HAND_TRIGGER = 150;
const int HAND_RESET = 200;
bool handReady = true;

// Your calibrated voting distances
const int holeMin[4] = {180, 240, 300, 360};
const int holeMax[4] = {210, 270, 320, 380};

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
  // XSHUT is low-voltage logic: release, don't drive HIGH.
  pinMode(pin, INPUT);
}

void shutDownSensor(int pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Hold both sensors in shutdown initially.
  shutDownSensor(HAND_XSHUT);
  shutDownSensor(VOTE_XSHUT);
  delay(20);

  // Start hand sensor and change its address.
  releaseSensor(HAND_XSHUT);
  delay(20);

  handSensor.setTimeout(100);
  if (!handSensor.init()) {
    Serial.println("ERROR:HAND_SENSOR");
    while (1);
  }
  handSensor.setAddress(HAND_ADDRESS);

  // Start voting sensor and change its address.
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
  Serial.print("Hand distance: ");
  Serial.print(handDistance);
  Serial.println(" mm");
  delay(100);


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

  // Close the gate after OPEN_TIME.
  if (gateOpen && now - gateOpenedAt >= OPEN_TIME) {
    gate.write(CLOSED_ANGLE);
    gateOpen = false;
  }

  // -------- SENSOR 2: BALL VOTING --------
  int distance = voteSensor.readRangeContinuousMillimeters();

  if (voteSensor.timeoutOccurred()) {
    clearStart = 0;
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

  // Rearm only after the path has been clear long enough.
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
}
