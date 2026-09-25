/*
 * xiao_send_pwm.ino
 *
 * Receives ASCII lines over USB serial from the host (makarena.py):
 *   "a<speed> b<steer>\n"
 *     speed: -500..500
 *     steer: -400..400
 *
 * Mixes speed + steer into differential left/right motor commands and
 * drives two H-bridge channels (PWM duty + direction pin each).
 *
 * Safety: if no valid command is received within WATCHDOG_MS, both
 * motors are stopped. This guards against a dropped USB connection or
 * a hung host script leaving the robot driving unattended.
 *
 * Wiring (adjust pins to match your H-bridge):
 *   Left motor  PWM -> LEFT_PWM_PIN,  direction -> LEFT_DIR_PIN
 *   Right motor PWM -> RIGHT_PWM_PIN, direction -> RIGHT_DIR_PIN
 *
 * Mounting orientation: if the robot is mounted upside down (rolled
 * 180 degrees around its forward axis), what was the left motor is now
 * physically on the right side, and each motor's "forward" spin
 * direction now drives its wheel backward. SWAP_LR and INVERT_DIR
 * below correct for that without touching the control logic.
 */

const int LEFT_PWM_PIN = D0;
const int LEFT_DIR_PIN = D1;
const int RIGHT_PWM_PIN = D2;
const int RIGHT_DIR_PIN = D3;

const bool SWAP_LR = true;    // robot mounted upside down: L/R positions are swapped
const bool INVERT_DIR = true; // robot mounted upside down: forward spin direction is reversed

const long BAUDRATE = 115200;

const int MAX_PWM = 500;   // must match Python's MAX_PWM
const int MAX_STEER = 400; // must match Python's MAX_STEER
const int PWM_RANGE = 500; // analogWrite() duty ceiling used below

const unsigned long WATCHDOG_MS = 500;

String lineBuf;
unsigned long lastCommandMs = 0;

void setMotor(int pwmPin, int dirPin, int value) {
  // value: -PWM_RANGE..PWM_RANGE
  bool forward = value >= 0;
  int duty = constrain(abs(value), 0, PWM_RANGE);
  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, duty);
}

void stopMotors() {
  setMotor(LEFT_PWM_PIN, LEFT_DIR_PIN, 0);
  setMotor(RIGHT_PWM_PIN, RIGHT_DIR_PIN, 0);
}

void applyCommand(int speed, int steer) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  steer = constrain(steer, -MAX_STEER, MAX_STEER);

  // Differential mix, then rescale into the PWM output range.
  int left = speed + steer;
  int right = speed - steer;

  int maxMag = MAX_PWM + MAX_STEER;
  left = map(left, -maxMag, maxMag, -PWM_RANGE, PWM_RANGE);
  right = map(right, -maxMag, maxMag, -PWM_RANGE, PWM_RANGE);

  if (INVERT_DIR) {
    left = -left;
    right = -right;
  }

  if (SWAP_LR) {
    setMotor(LEFT_PWM_PIN, LEFT_DIR_PIN, right);
    setMotor(RIGHT_PWM_PIN, RIGHT_DIR_PIN, left);
  } else {
    setMotor(LEFT_PWM_PIN, LEFT_DIR_PIN, left);
    setMotor(RIGHT_PWM_PIN, RIGHT_DIR_PIN, right);
  }
}

// Parses "a<int> b<int>" -> speed, steer. Returns true on success.
bool parseLine(const String &line, int &speed, int &steer) {
  int aIdx = line.indexOf('a');
  int bIdx = line.indexOf('b');
  if (aIdx < 0 || bIdx < 0 || bIdx <= aIdx) return false;

  String aStr = line.substring(aIdx + 1, bIdx);
  String bStr = line.substring(bIdx + 1);
  aStr.trim();
  bStr.trim();
  if (aStr.length() == 0 || bStr.length() == 0) return false;

  speed = aStr.toInt();
  steer = bStr.toInt();
  return true;
}

void setup() {
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  analogWriteRange(PWM_RANGE);

  stopMotors();

  Serial.begin(BAUDRATE);
  lineBuf.reserve(32);
  lastCommandMs = millis();
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      int speed, steer;
      if (parseLine(lineBuf, speed, steer)) {
        applyCommand(speed, steer);
        lastCommandMs = millis();
      }
      lineBuf = "";
    } else if (c != '\r') {
      lineBuf += c;
      if (lineBuf.length() > 64) lineBuf = ""; // guard against garbage/noise
    }
  }

  if (millis() - lastCommandMs > WATCHDOG_MS) {
    stopMotors();
  }
}
