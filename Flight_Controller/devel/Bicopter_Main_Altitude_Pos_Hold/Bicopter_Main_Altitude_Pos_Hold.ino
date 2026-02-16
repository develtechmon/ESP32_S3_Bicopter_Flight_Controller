/*
  ================================================================================
  Project     : ESP32-S3 Bicopter Flight Controller
  File        : Bicopter_Main_Altitude_Hold.ino
  Description : Bicopter firmware with MTF-02 altitude hold.
                ALL variables declared here.
                Altitude_Hold.ino contains functions only (no declarations).

  Modules:
    Altitude_Hold.ino  — altitude hold functions
    Position_Hold.ino  — position hold functions
    MTF02_Library_CM.h — MTF-02 sensor library

  Bicopter layout (viewed from front):
    [Motor Left] ------- FRAME ------- [Motor Right]
         |                                    |
    [Servo Left]                       [Servo Right]

  Control mapping:
    Roll  → motor differential (left/right thrust)
    Pitch → servo tilt (forward/back)
    Yaw   → opposite servo tilt

  Channels:
    CH1 → Roll
    CH2 → Pitch
    CH3 → Throttle
    CH4 → Yaw
    CH5 → Arm/Disarm (yaw right = arm, yaw left = disarm)
    CH6 → Altitude Hold on/off (> 1500 = ON)
    CH7 → Position Hold on/off (> 1500 = ON, requires CH6 active)

  Revision History:
    Version  Date        Author        Description
    -------  ----------  ------------  -----------------------------------------
    1.0      2025-07-30  Lukas Johnny  Initial bicopter release
    1.1      2025-08-15  Lukas Johnny  Added MTF-02 altitude hold module
    1.2      2025-08-16  Lukas Johnny  Added MTF-02 position hold module
  ================================================================================
*/

#include <Wire.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#include "MTF02_Library_CM.h"

// ==== Pin Definitions ====
#define LED_PIN         48
#define NUM_LEDS        1
#define PPM_PIN         13
#define CUSTOM_SDA_PIN  4
#define CUSTOM_SCL_PIN  5
#define SERVO_RIGHT_PIN 11
#define SERVO_LEFT_PIN  10
#define MOTOR_RIGHT_PIN 9
#define MOTOR_LEFT_PIN  8

// ==== PPM Configuration ====
#define NUM_CHANNELS       8
#define PPM_SYNC_THRESHOLD 3000
#define CHANNEL_MIN        1000
#define CHANNEL_MAX        2000

// ==== Servo Configuration ====
#define SERVO_CENTER 1500
#define SERVO_MIN    1200
#define SERVO_MAX    1800

// ==== ESC Configuration ====
int ESCfreq        = 400;
int ThrottleIdle   = 1180;
int ThrottleCutOff = 1000;

// ==== Timing ====
const float dt = 0.004;   // 250Hz
uint32_t LoopTimer;

// ==== LED ====
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ==== Servo and Motor Objects ====
Servo servoRight;
Servo servoLeft;
Servo motorRight;
Servo motorLeft;

// ==== Receiver Variables ====
volatile int ReceiverValue[NUM_CHANNELS] = {1500,1500,1000,1500,1500,1500,1500,1500};
volatile int channelIndex = 0;
volatile unsigned long lastTime = 0;
int channelValues[NUM_CHANNELS];

// ==== IMU Variables ====
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float KalmanAngleRoll             = 0;
float KalmanUncertaintyAngleRoll  = 4;
float KalmanAnglePitch            = 0;
float KalmanUncertaintyAnglePitch = 4;
volatile float Kalman1DOutput[]   = {0, 0};

// ==== Calibration Values ====
float RateCalibrationRoll  =  1.73;
float RateCalibrationPitch = -0.30;
float RateCalibrationYaw   =  0.39;
float AccXCalibration      =  0.01;
float AccYCalibration      = -0.03;
float AccZCalibration      =  0.08;

// ==== Angle PID Gains ====
// Pitch gains raised — servo axis needs more authority than motor axis
// Trace: 10deg error × PAnglePitch(3.0) = 30 → × PRatePitch(1.5) = 45 InputPitch
// → 45 × 1.5 servo multiplier = 67.5 µs servo movement — visible and controllable
float PAngleRoll  = 1.5,  IAngleRoll  = 0.3,  DAngleRoll  = 0.01;
float PAnglePitch = 3.0,  IAnglePitch = 0.0,  DAnglePitch = 0.0;  // raised, I/D zeroed

// ==== Rate PID Gains ====
float PRateRoll  = 0.5,  IRateRoll  = 1.5,  DRateRoll  = 0.008;
float PRatePitch = 1.5,  IRatePitch = 0.0,  DRatePitch = 0.0;    // raised, I/D zeroed
float PRateYaw   = 0.8,  IRateYaw   = 1.2,  DRateYaw   = 0.005;

// ==== PID State Variables ====
float DesiredAngleRoll,  DesiredAnglePitch;
float ErrorAngleRoll,    ErrorAnglePitch;
float PrevErrorAngleRoll  = 0,  PrevErrorAnglePitch  = 0;
float PrevItermAngleRoll  = 0,  PrevItermAnglePitch  = 0;

float DesiredRateRoll,  DesiredRatePitch,  DesiredRateYaw;
float ErrorRateRoll,    ErrorRatePitch,    ErrorRateYaw;
float PrevErrorRateRoll  = 0,  PrevErrorRatePitch  = 0,  PrevErrorRateYaw  = 0;
float PrevItermRateRoll  = 0,  PrevItermRatePitch  = 0,  PrevItermRateYaw  = 0;

float InputRoll, InputPitch, InputYaw, InputThrottle;
float MotorInputRight, MotorInputLeft;
float ServoInputRight, ServoInputLeft;

// ==== Servo Filtering ====
float filteredServoLeft  = SERVO_CENTER;
float filteredServoRight = SERVO_CENTER;
const float SERVO_ALPHA  = 0.6;

// ==== D-term Filtering ====
float filteredDRatePitch = 0;
float filteredDRateYaw   = 0;
const float D_FILTER_ALPHA = 0.15;

// ==== Servo Rate Limiting ====
float prevServoLeft  = SERVO_CENTER;
float prevServoRight = SERVO_CENTER;
const float MAX_SERVO_RATE = 30;

// ==== Arming ====
bool armed = false;
unsigned long armDisarmTimer = 0;
const unsigned long armHoldTime = 1000;

// ================================================================================
// ALTITUDE HOLD VARIABLES (used by Altitude_Hold.ino functions)
// ================================================================================

// MTF-02 filtering
#define LIDAR_BUFFER_SIZE 21
int   lidarHistTab[LIDAR_BUFFER_SIZE];
int   lidarHistIdx        = 0;
int   lidarSum            = 0;
float complementaryFiltered = 0;
const float COMP_FILTER_ALPHA = 0.75;

// Altitude hold state
bool  altitudeHoldActive  = false;
float groundReference     = 0;
float targetAltitude      = 0;
float currentAltitude     = 0;
int   baseThrottle        = 0;

// Landing mode
bool  landingMode             = false;
float landingStartAltitude    = 0;
unsigned long landingStartTime = 0;
const float LANDING_RATE_CMS          = 15.0;
const int   THROTTLE_DEADBAND         = 25;
const int   LANDING_THROTTLE_THRESHOLD = 1400;

// Altitude PID
float altPID_error       = 0;
float altPID_errorPrev   = 0;
float altPID_integral    = 0;
float altPID_output      = 0;
float altPID_Kp          = 1.0;
float altPID_Ki          = 0.25;
float altPID_Kd          = 0.5;
float altPID_integralMax = 100.0;

// Velocity-based derivative
float altitudeVelocity        = 0;
unsigned long altitudeHoldStartTime = 0;

// ================================================================================
// POSITION HOLD VARIABLES (used by Position_Hold.ino functions)
// ================================================================================

// Position hold state
bool  positionHoldActive      = false;
unsigned long positionHoldStartTime = 0;

// Position (cm, relative to activation point)
float currentPositionX = 0;
float currentPositionY = 0;
float targetPositionX  = 0;
float targetPositionY  = 0;

// Velocity targets (cm/s)
float targetVelocityX = 0;
float targetVelocityY = 0;

// Position → Velocity PID gains
float posToVel_Kp = 0.6;
float posToVel_Ki = 0.05;
float posToVel_Kd = 0.1;

// Velocity → Angle PID gains — split for roll and pitch
// Roll: through motor differential (fast) — lower gains fine
// Pitch: through angle+rate PID chain with raised gains — keep vel gains moderate
float vel_Kp_Roll  = 0.35;
float vel_Ki_Roll  = 0.0;
float vel_Kd_Roll  = 0.01;
float vel_Kp_Pitch = 0.35;   // start same as roll — raise if no response
float vel_Ki_Pitch = 0.0;    // zero until P is stable
float vel_Kd_Pitch = 0.0;    // zero — D on servo axis causes buzz

// Position PID state
float posX_error = 0, posX_errorPrev = 0, posX_integral = 0;
float posY_error = 0, posY_errorPrev = 0, posY_integral = 0;

// Velocity PID state
float velX_error = 0, velX_errorPrev = 0, velX_integral = 0;
float velY_error = 0, velY_errorPrev = 0, velY_integral = 0;

// Angle corrections fed into DesiredAngleRoll / DesiredAnglePitch
float positionCorrectionRoll  = 0;
float positionCorrectionPitch = 0;

// Stick deadband for position hold input
const int STICK_DEADBAND = 50;

// Position velocity filter state (declared here so they reset on activation)
// Position velocity filter state — shared between a_updatePosition and d_calculatePositionPID
float pidVelFilteredX     = 0;
float pidVelFilteredY     = 0;
unsigned long posLastUpdate = 0;

// Stick smoothing state (declared here so they reset on activation)
float smoothRollInput  = 0;
float smoothPitchInput = 0;

// Derivative filter state for position PID (declared here so they reset on activation)
float filtered_posD_X = 0;
float filtered_posD_Y = 0;
float filtered_velD_X = 0;
float filtered_velD_Y = 0;

// ================================================================================
// FUNCTION PROTOTYPES
// ================================================================================
void IRAM_ATTR ppmInterruptHandler();
void read_receiver();
void read_imu();
void kalman_1d(float& KalmanState, float& KalmanUncertainty,
               float KalmanInput, float KalmanMeasurement);
void reset_pid();
float constrain_float(float value, float min_val, float max_val);
float rate_limit(float current, float previous, float max_rate);

// ================================================================================
// PPM INTERRUPT
// ================================================================================
void IRAM_ATTR ppmInterruptHandler() {
  unsigned long currentTime = micros();
  unsigned long pulseWidth  = currentTime - lastTime;
  lastTime = currentTime;
  if (pulseWidth > PPM_SYNC_THRESHOLD) {
    channelIndex = 0;
  } else if (channelIndex < NUM_CHANNELS) {
    if (pulseWidth < CHANNEL_MIN) pulseWidth = CHANNEL_MIN;
    else if (pulseWidth > CHANNEL_MAX) pulseWidth = CHANNEL_MAX;
    ReceiverValue[channelIndex] = pulseWidth;
    channelIndex++;
  }
}

// ================================================================================
// READ RECEIVER
// ================================================================================
void read_receiver() {
  noInterrupts();
  for (int i = 0; i < NUM_CHANNELS; i++) channelValues[i] = ReceiverValue[i];
  interrupts();
}

// ================================================================================
// KALMAN FILTER
// ================================================================================
void kalman_1d(float& KalmanState, float& KalmanUncertainty,
               float KalmanInput, float KalmanMeasurement) {
  KalmanState       = KalmanState + dt * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + dt * dt * 4 * 4;
  float KalmanGain  = KalmanUncertainty / (KalmanUncertainty + 3 * 3);
  KalmanState       = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

// ================================================================================
// READ IMU
// ================================================================================
void read_imu() {
  Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x43); Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll  = (float)GyroX / 65.5 - RateCalibrationRoll;
  RatePitch = (float)GyroY / 65.5 - RateCalibrationPitch;
  RateYaw   = (float)GyroZ / 65.5 - RateCalibrationYaw;
  AccX = (float)AccXLSB / 4096 - AccXCalibration;
  AccY = (float)AccYLSB / 4096 - AccYCalibration;
  AccZ = (float)AccZLSB / 4096 - AccZCalibration;
  AngleRoll  = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29578;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29578;
}

// ================================================================================
// RESET PID
// ================================================================================
void reset_pid() {
  PrevErrorAngleRoll = 0;  PrevErrorAnglePitch = 0;
  PrevItermAngleRoll = 0;  PrevItermAnglePitch = 0;
  PrevErrorRateRoll  = 0;  PrevErrorRatePitch  = 0;  PrevErrorRateYaw  = 0;
  PrevItermRateRoll  = 0;  PrevItermRatePitch  = 0;  PrevItermRateYaw  = 0;
  filteredServoLeft  = SERVO_CENTER;  filteredServoRight = SERVO_CENTER;
  prevServoLeft      = SERVO_CENTER;  prevServoRight     = SERVO_CENTER;
  filteredDRatePitch = 0;
  filteredDRateYaw   = 0;
}

// ================================================================================
// HELPERS
// ================================================================================
float constrain_float(float value, float min_val, float max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

float rate_limit(float current, float previous, float max_rate) {
  float delta = current - previous;
  if (abs(delta) > max_rate) return (delta > 0) ? previous + max_rate : previous - max_rate;
  return current;
}

// ================================================================================
// SETUP
// ================================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Bicopter Starting...");

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, OUTPUT);
  strip.begin();
  strip.show();

  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(100);
    digitalWrite(LED_BUILTIN, LOW);  delay(100);
  }

  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);
  delay(100);

  Wire.setClock(400000);
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
  delay(250);
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  delay(100);

  // MTF-02 init + ground calibration (Altitude_Hold.ino)
  initializeAltitudeSystem();

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);

  Serial.println("Initializing ESCs...");
  delay(1000);
  motorRight.attach(MOTOR_RIGHT_PIN, 1000, 2000); delay(500); motorRight.setPeriodHertz(ESCfreq);
  motorLeft.attach(MOTOR_LEFT_PIN,   1000, 2000); delay(500); motorLeft.setPeriodHertz(ESCfreq);
  motorRight.writeMicroseconds(1000);
  motorLeft.writeMicroseconds(1000);
  delay(2000);

  servoRight.setPeriodHertz(200); servoLeft.setPeriodHertz(200);
  servoRight.attach(SERVO_RIGHT_PIN, SERVO_MIN, SERVO_MAX);
  servoLeft.attach(SERVO_LEFT_PIN,   SERVO_MIN, SERVO_MAX);
  servoRight.writeMicroseconds(SERVO_CENTER);
  servoLeft.writeMicroseconds(SERVO_CENTER);
  delay(1000);

  filteredServoLeft  = SERVO_CENTER; filteredServoRight = SERVO_CENTER;
  prevServoLeft      = SERVO_CENTER; prevServoRight     = SERVO_CENTER;

  strip.setPixelColor(0, strip.Color(0, 255, 0)); strip.show(); delay(1000);
  strip.setPixelColor(0, strip.Color(0, 0, 0));   strip.show();

  Serial.println("Setup complete!");
  LoopTimer = micros();
}

// ================================================================================
// MAIN LOOP
// ================================================================================
void loop() {
  read_receiver();

  // ---- Arming / Disarming ----
  if (channelValues[2] < 1050) {
    if (!armed && channelValues[3] > 1900) {
      if (armDisarmTimer == 0) armDisarmTimer = millis();
      else if (millis() - armDisarmTimer > armHoldTime) {
        armed = true;
        strip.setPixelColor(0, strip.Color(255, 255, 0)); strip.show(); delay(500);
        strip.setPixelColor(0, strip.Color(0, 0, 0));     strip.show();
        armDisarmTimer = 0;
        Serial.println("ARMED");
      }
    } else if (armed && channelValues[3] < 1100) {
      if (armDisarmTimer == 0) armDisarmTimer = millis();
      else if (millis() - armDisarmTimer > armHoldTime) {
        armed              = false;
        altitudeHoldActive = false;
        landingMode        = false;
        positionHoldActive      = false;
        positionCorrectionRoll  = 0;
        positionCorrectionPitch = 0;
        pidVelFilteredX = 0; pidVelFilteredY = 0;
        filtered_posD_X = 0; filtered_posD_Y = 0;
        filtered_velD_X = 0; filtered_velD_Y = 0;
        posLastUpdate   = 0;
        strip.setPixelColor(0, strip.Color(255, 255, 255)); strip.show(); delay(500);
        strip.setPixelColor(0, strip.Color(0, 0, 0));       strip.show();
        armDisarmTimer = 0;
        reset_pid();
        Serial.println("DISARMED");
      }
    } else { armDisarmTimer = 0; }
  } else { armDisarmTimer = 0; }

  // ---- Disarmed: stop everything ----
  if (!armed) {
    motorRight.writeMicroseconds(ThrottleCutOff);
    motorLeft.writeMicroseconds(ThrottleCutOff);
    servoRight.writeMicroseconds(SERVO_CENTER);
    servoLeft.writeMicroseconds(SERVO_CENTER);
    while (micros() - LoopTimer < (dt * 1000000));
    LoopTimer = micros();
    return;
  }

  // ---- Altitude Hold (Altitude_Hold.ino) ----
  a_updateAltitude();
  b_handleAltitudeHold();
  c_handlePilotAltitudeInput();
  d_calculateAltitudePID();

  // ---- Position Hold (Position_Hold.ino) ----
  a_updatePosition();
  b_handlePositionHold();
  c_handlePilotPositionInput();
  d_calculatePositionPID();

  // ---- IMU ----
  read_imu();

  // ---- Kalman ----
  kalman_1d(KalmanAngleRoll,  KalmanUncertaintyAngleRoll,  RateRoll,  AngleRoll);
  KalmanAngleRoll            = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch            = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
  KalmanAngleRoll  = constrain_float(KalmanAngleRoll,  -20, 20);
  KalmanAnglePitch = constrain_float(KalmanAnglePitch, -20, 20);

  // ---- Desired Angles ----
  // Both roll and pitch fed through angle+rate PID chain
  // Correction limits raised — pitch needs more authority than quad to drive servo
  if (positionHoldActive) {
    DesiredAngleRoll  = 0.1 * (channelValues[0] - 1500) + positionCorrectionRoll;
    DesiredAnglePitch = 0.1 * (channelValues[1] - 1500) + positionCorrectionPitch;
    DesiredAngleRoll  = constrain_float(DesiredAngleRoll,  -8,  8);
    DesiredAnglePitch = constrain_float(DesiredAnglePitch, -10, 10);
  } else {
    DesiredAngleRoll  = 0.1 * (channelValues[0] - 1500);
    DesiredAnglePitch = 0.1 * (channelValues[1] - 1500);
    positionCorrectionRoll  = 0;
    positionCorrectionPitch = 0;
  }
  DesiredRateYaw = 0.15 * (channelValues[3] - 1500);
  InputThrottle = calculateThrottleOutput();

  // ---- Angle PID: Roll ----
  ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  float PtAR = PAngleRoll * ErrorAngleRoll;
  float ItAR = PrevItermAngleRoll + IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * dt / 2;
  ItAR = constrain_float(ItAR, -200, 200);
  float DtAR = DAngleRoll * (ErrorAngleRoll - PrevErrorAngleRoll) / dt;
  DesiredRateRoll = constrain_float(PtAR + ItAR + DtAR, -200, 200);
  PrevErrorAngleRoll = ErrorAngleRoll; PrevItermAngleRoll = ItAR;

  // ---- Angle PID: Pitch ----
  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
  float PtAP = PAnglePitch * ErrorAnglePitch;
  float ItAP = PrevItermAnglePitch + IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * dt / 2;
  ItAP = constrain_float(ItAP, -200, 200);
  float DtAP = DAnglePitch * (ErrorAnglePitch - PrevErrorAnglePitch) / dt;
  DesiredRatePitch = constrain_float(PtAP + ItAP + DtAP, -200, 200);
  PrevErrorAnglePitch = ErrorAnglePitch; PrevItermAnglePitch = ItAP;

  // ---- Rate PID: Roll ----
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  float PtRR = PRateRoll * ErrorRateRoll;
  float ItRR = PrevItermRateRoll + IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * dt / 2;
  ItRR = constrain_float(ItRR, -300, 300);
  float DtRR = DRateRoll * (ErrorRateRoll - PrevErrorRateRoll) / dt;
  InputRoll = constrain_float(PtRR + ItRR + DtRR, -400, 400);
  PrevErrorRateRoll = ErrorRateRoll; PrevItermRateRoll = ItRR;

  // ---- Rate PID: Pitch ----
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  float PtRP = PRatePitch * ErrorRatePitch;
  float ItRP = PrevItermRatePitch + IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * dt / 2;
  ItRP = constrain_float(ItRP, -300, 300);
  float DtRP = DRatePitch * (ErrorRatePitch - PrevErrorRatePitch) / dt;
  filteredDRatePitch = D_FILTER_ALPHA * DtRP + (1 - D_FILTER_ALPHA) * filteredDRatePitch;
  InputPitch = constrain_float(PtRP + ItRP + filteredDRatePitch, -400, 400);
  PrevErrorRatePitch = ErrorRatePitch; PrevItermRatePitch = ItRP;

  // ---- Rate PID: Yaw ----
  ErrorRateYaw = DesiredRateYaw - RateYaw;
  float PtRY = PRateYaw * ErrorRateYaw;
  float ItRY = PrevItermRateYaw + IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * dt / 2;
  ItRY = constrain_float(ItRY, -300, 300);
  float DtRY = DRateYaw * (ErrorRateYaw - PrevErrorRateYaw) / dt;
  filteredDRateYaw = D_FILTER_ALPHA * DtRY + (1 - D_FILTER_ALPHA) * filteredDRateYaw;
  InputYaw = constrain_float(PtRY + ItRY + filteredDRateYaw, -400, 400);
  PrevErrorRateYaw = ErrorRateYaw; PrevItermRateYaw = ItRY;

  if (InputThrottle > 1800) InputThrottle = 1800;

  // ---- Bicopter Mixing ----
  MotorInputLeft  = InputThrottle - InputRoll;
  MotorInputRight = InputThrottle + InputRoll;
  ServoInputLeft  = SERVO_CENTER - InputPitch * 1.5 - InputYaw;
  ServoInputRight = SERVO_CENTER + InputPitch * 1.5 - InputYaw;

  // ---- Servo Rate Limit + Filter ----
  ServoInputLeft  = rate_limit(ServoInputLeft,  prevServoLeft,  MAX_SERVO_RATE);
  ServoInputRight = rate_limit(ServoInputRight, prevServoRight, MAX_SERVO_RATE);
  filteredServoLeft  = SERVO_ALPHA * ServoInputLeft  + (1 - SERVO_ALPHA) * filteredServoLeft;
  filteredServoRight = SERVO_ALPHA * ServoInputRight + (1 - SERVO_ALPHA) * filteredServoRight;
  filteredServoLeft  = constrain_float(filteredServoLeft,  SERVO_MIN, SERVO_MAX);
  filteredServoRight = constrain_float(filteredServoRight, SERVO_MIN, SERVO_MAX);
  prevServoLeft  = ServoInputLeft;
  prevServoRight = ServoInputRight;

  // ---- Motor Limits ----
  MotorInputRight = constrain_float(MotorInputRight, ThrottleIdle, 2000);
  MotorInputLeft  = constrain_float(MotorInputLeft,  ThrottleIdle, 2000);

  // ---- Emergency Stop ----
  if (channelValues[2] < 1030) {
    MotorInputRight    = ThrottleCutOff;
    MotorInputLeft     = ThrottleCutOff;
    filteredServoLeft  = SERVO_CENTER;
    filteredServoRight = SERVO_CENTER;
    prevServoLeft      = SERVO_CENTER;
    prevServoRight     = SERVO_CENTER;
    altitudeHoldActive = false;
    landingMode        = false;
    positionHoldActive      = false;
    positionCorrectionRoll  = 0;
    positionCorrectionPitch = 0;
    pidVelFilteredX = 0; pidVelFilteredY = 0;
    filtered_posD_X = 0; filtered_posD_Y = 0;
    filtered_velD_X = 0; filtered_velD_Y = 0;
    posLastUpdate   = 0;
    reset_pid();
  }

  // ---- Write Outputs ----
  motorRight.writeMicroseconds((int)MotorInputRight);
  motorLeft.writeMicroseconds((int)MotorInputLeft);
  servoRight.writeMicroseconds((int)filteredServoRight);
  servoLeft.writeMicroseconds((int)filteredServoLeft);

  while (micros() - LoopTimer < (dt * 1000000));
  LoopTimer = micros();
}
