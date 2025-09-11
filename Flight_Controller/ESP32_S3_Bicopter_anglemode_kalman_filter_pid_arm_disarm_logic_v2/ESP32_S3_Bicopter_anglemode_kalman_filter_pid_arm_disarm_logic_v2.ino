/*
  Bicopter Flight Controller - Stable Version
  ESP32-S3 with MPU6050, 2 servos, 2 motors
  Tested for Arduino IDE compilation
*/

#include <Wire.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>

// ==== Pin Definitions ====
#define LED_PIN 48  
#define NUM_LEDS 1
#define PPM_PIN 13
#define CUSTOM_SDA_PIN 11
#define CUSTOM_SCL_PIN 12
#define SERVO_RIGHT_PIN 8
#define SERVO_LEFT_PIN 7
#define MOTOR_RIGHT_PIN 9
#define MOTOR_LEFT_PIN 6

// ==== PPM Configuration ====
#define NUM_CHANNELS 8
#define PPM_SYNC_THRESHOLD 3000
#define CHANNEL_MIN 1000
#define CHANNEL_MAX 2000

// ==== Servo Configuration ====
#define SERVO_CENTER 1500
#define SERVO_MIN 1100
#define SERVO_MAX 1900

// ==== ESC Configuration ====
int ESCfreq = 400;
int ThrottleIdle = 1180;
int ThrottleCutOff = 1000;

// ==== Timing ====
const float dt = 0.004;  // 250Hz loop rate
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
float KalmanAngleRoll = 0;
float KalmanUncertaintyAngleRoll = 4;
float KalmanAnglePitch = 0;
float KalmanUncertaintyAnglePitch = 4;

// ==== Calibration Values ====
float RateCalibrationRoll = -1.97;
float RateCalibrationPitch = 1.83;
float RateCalibrationYaw = 0.34;
float AccXCalibration = 0.01;
float AccYCalibration = 0.01;
float AccZCalibration = 0.08;

// ==== PID Gains ====
// Angle PID (outer loop)
float PAngleRoll = 1.5;
float IAngleRoll = 0.3;
float DAngleRoll = 0.01;
float PAnglePitch = 1.8;
float IAnglePitch = 0.3;
float DAnglePitch = 0.01;

// Rate PID (inner loop)
float PRateRoll = 0.5;
float IRateRoll = 1.5;
float DRateRoll = 0.008;
float PRatePitch = 0.6;
float IRatePitch = 1.8;
float DRatePitch = 0.01;
float PRateYaw = 0.8;
float IRateYaw = 1.2;
float DRateYaw = 0.005;

// ==== PID Variables ====
float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll = 0, PrevErrorAnglePitch = 0;
float PrevItermAngleRoll = 0, PrevItermAnglePitch = 0;

float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float PrevErrorRateRoll = 0, PrevErrorRatePitch = 0, PrevErrorRateYaw = 0;
float PrevItermRateRoll = 0, PrevItermRatePitch = 0, PrevItermRateYaw = 0;

float InputRoll, InputPitch, InputYaw, InputThrottle;
float MotorInputRight, MotorInputLeft;
float ServoInputRight, ServoInputLeft;

// ==== Arming Variables ====
bool armed = false;
unsigned long armDisarmTimer = 0;
const unsigned long armHoldTime = 1000;

// ==== Function Prototypes ====
void IRAM_ATTR ppmInterruptHandler();
void read_receiver();
void read_imu();
void kalman_1d(float& KalmanState, float& KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
void reset_pid();
float constrain_float(float value, float min_val, float max_val);

// ==== PPM Interrupt Handler ====
void IRAM_ATTR ppmInterruptHandler() {
  unsigned long currentTime = micros();
  unsigned long pulseWidth = currentTime - lastTime;
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

// ==== Read Receiver Values ====
void read_receiver() {
  noInterrupts();
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channelValues[i] = ReceiverValue[i];
  }
  interrupts();
}

// ==== Kalman Filter - Fixed with References ====
void kalman_1d(float& KalmanState, float& KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  // Prediction step
  KalmanState = KalmanState + dt * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + dt * dt * 4 * 4;
  
  // Update step
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
}

// ==== Read IMU Data ====
void read_imu() {
  // Configure and read accelerometer
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  
  // Configure and read gyroscope
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  
  // Convert and calibrate
  RateRoll = (float)GyroX / 65.5 - RateCalibrationRoll;
  RatePitch = (float)GyroY / 65.5 - RateCalibrationPitch;
  RateYaw = (float)GyroZ / 65.5 - RateCalibrationYaw;
  
  AccX = (float)AccXLSB / 4096 - AccXCalibration;
  AccY = (float)AccYLSB / 4096 - AccYCalibration;
  AccZ = (float)AccZLSB / 4096 - AccZCalibration;
  
  // Calculate angles
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29578;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29578;
}

// ==== Reset PID States ====
void reset_pid() {
  PrevErrorAngleRoll = 0;
  PrevErrorAnglePitch = 0;
  PrevItermAngleRoll = 0;
  PrevItermAnglePitch = 0;
  PrevErrorRateRoll = 0;
  PrevErrorRatePitch = 0;
  PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0;
  PrevItermRatePitch = 0;
  PrevItermRateYaw = 0;
}

// ==== Constrain Float ====
float constrain_float(float value, float min_val, float max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

// ==== Setup ====
void setup() {
  Serial.begin(115200);
  Serial.println("Bicopter Starting...");
  
  // Initialize LED
  pinMode(LED_BUILTIN, OUTPUT);
  strip.begin();
  strip.show();
  
  // Startup blink
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  
  // Setup PPM
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);
  delay(100);
  
  // Setup I2C and MPU6050
  Wire.setClock(400000);
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
  delay(250);
  
  // Wake up MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
  
  // Setup PWM
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Initialize motors
  Serial.println("Initializing ESCs...");
  delay(1000);
  
  motorRight.attach(MOTOR_RIGHT_PIN, 1000, 2000);
  delay(500);
  motorRight.setPeriodHertz(ESCfreq);
  
  motorLeft.attach(MOTOR_LEFT_PIN, 1000, 2000);
  delay(500);
  motorLeft.setPeriodHertz(ESCfreq);
  
  // ESC initialization
  motorRight.writeMicroseconds(1000);
  motorLeft.writeMicroseconds(1000);
  delay(2000);
  
  // Initialize servos
  servoRight.setPeriodHertz(300);
  servoLeft.setPeriodHertz(300);
  servoRight.attach(SERVO_RIGHT_PIN, SERVO_MIN, SERVO_MAX);
  servoLeft.attach(SERVO_LEFT_PIN, SERVO_MIN, SERVO_MAX);
  
  // Center servos
  servoRight.writeMicroseconds(SERVO_CENTER);
  servoLeft.writeMicroseconds(SERVO_CENTER);
  delay(1000);
  
  // Ready signal
  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();
  delay(1000);
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();
  
  Serial.println("Setup complete!");
  LoopTimer = micros();
}

// ==== Main Loop ====
void loop() {
  // Read receiver
  read_receiver();
  
  // Arming logic
  if (channelValues[2] < 1050) {
    // Arm check
    if (!armed && channelValues[3] > 1900) {
      if (armDisarmTimer == 0) {
        armDisarmTimer = millis();
      } else if (millis() - armDisarmTimer > armHoldTime) {
        armed = true;
        strip.setPixelColor(0, strip.Color(255, 255, 0));
        strip.show();
        delay(500);
        strip.setPixelColor(0, strip.Color(0, 0, 0));
        strip.show();
        armDisarmTimer = 0;
        Serial.println("ARMED");
      }
    }
    // Disarm check
    else if (armed && channelValues[3] < 1100) {
      if (armDisarmTimer == 0) {
        armDisarmTimer = millis();
      } else if (millis() - armDisarmTimer > armHoldTime) {
        armed = false;
        strip.setPixelColor(0, strip.Color(255, 255, 255));
        strip.show();
        delay(500);
        strip.setPixelColor(0, strip.Color(0, 0, 0));
        strip.show();
        armDisarmTimer = 0;
        reset_pid();
        Serial.println("DISARMED");
      }
    } else {
      armDisarmTimer = 0;
    }
  } else {
    armDisarmTimer = 0;
  }
  
  // If not armed, stop motors
  if (!armed) {
    motorRight.writeMicroseconds(ThrottleCutOff);
    motorLeft.writeMicroseconds(ThrottleCutOff);
    servoRight.writeMicroseconds(SERVO_CENTER);
    servoLeft.writeMicroseconds(SERVO_CENTER);
    
    while (micros() - LoopTimer < (dt * 1000000));
    LoopTimer = micros();
    return;
  }
  
  // Read IMU
  read_imu();
  
  // Apply Kalman filter
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  
  // Limit angles
  KalmanAngleRoll = constrain_float(KalmanAngleRoll, -20, 20);
  KalmanAnglePitch = constrain_float(KalmanAnglePitch, -20, 20);
  
  // Get desired values from transmitter
  DesiredAngleRoll = 0.1 * (channelValues[0] - 1500);
  DesiredAnglePitch = 0.1 * (channelValues[1] - 1500);
  InputThrottle = channelValues[2];
  DesiredRateYaw = 0.15 * (channelValues[3] - 1500);
  
  // Angle PID for Roll
  ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  float PtermAngleRoll = PAngleRoll * ErrorAngleRoll;
  float ItermAngleRoll = PrevItermAngleRoll + IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * dt / 2;
  ItermAngleRoll = constrain_float(ItermAngleRoll, -200, 200);
  float DtermAngleRoll = DAngleRoll * (ErrorAngleRoll - PrevErrorAngleRoll) / dt;
  float PIDAngleRoll = PtermAngleRoll + ItermAngleRoll + DtermAngleRoll;
  DesiredRateRoll = constrain_float(PIDAngleRoll, -200, 200);
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermAngleRoll;
  
  // Angle PID for Pitch
  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
  float PtermAnglePitch = PAnglePitch * ErrorAnglePitch;
  float ItermAnglePitch = PrevItermAnglePitch + IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * dt / 2;
  ItermAnglePitch = constrain_float(ItermAnglePitch, -200, 200);
  float DtermAnglePitch = DAnglePitch * (ErrorAnglePitch - PrevErrorAnglePitch) / dt;
  float PIDAnglePitch = PtermAnglePitch + ItermAnglePitch + DtermAnglePitch;
  DesiredRatePitch = constrain_float(PIDAnglePitch, -200, 200);
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermAnglePitch;
  
  // Rate PID for Roll
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  float PtermRateRoll = PRateRoll * ErrorRateRoll;
  float ItermRateRoll = PrevItermRateRoll + IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * dt / 2;
  ItermRateRoll = constrain_float(ItermRateRoll, -300, 300);
  float DtermRateRoll = DRateRoll * (ErrorRateRoll - PrevErrorRateRoll) / dt;
  InputRoll = constrain_float(PtermRateRoll + ItermRateRoll + DtermRateRoll, -400, 400);
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRateRoll;
  
  // Rate PID for Pitch
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  float PtermRatePitch = PRatePitch * ErrorRatePitch;
  float ItermRatePitch = PrevItermRatePitch + IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * dt / 2;
  ItermRatePitch = constrain_float(ItermRatePitch, -300, 300);
  float DtermRatePitch = DRatePitch * (ErrorRatePitch - PrevErrorRatePitch) / dt;
  InputPitch = constrain_float(PtermRatePitch + ItermRatePitch + DtermRatePitch, -400, 400);
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermRatePitch;
  
  // Rate PID for Yaw
  ErrorRateYaw = DesiredRateYaw - RateYaw;
  float PtermRateYaw = PRateYaw * ErrorRateYaw;
  float ItermRateYaw = PrevItermRateYaw + IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * dt / 2;
  ItermRateYaw = constrain_float(ItermRateYaw, -300, 300);
  float DtermRateYaw = DRateYaw * (ErrorRateYaw - PrevErrorRateYaw) / dt;
  InputYaw = constrain_float(PtermRateYaw + ItermRateYaw + DtermRateYaw, -400, 400);
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermRateYaw;
  
  // Limit throttle
  if (InputThrottle > 1800) InputThrottle = 1800;
  
  // Bicopter mixing
  // Motors control thrust and roll
  MotorInputLeft = InputThrottle - InputRoll * 0.5;
  MotorInputRight = InputThrottle + InputRoll * 0.5;
  
  // Servos control pitch and yaw
  ServoInputLeft = SERVO_CENTER - InputPitch * 2.0 - InputYaw * 1.5;
  ServoInputRight = SERVO_CENTER + InputPitch * 2.0 - InputYaw * 1.5;
  
  // Apply limits
  MotorInputRight = constrain_float(MotorInputRight, ThrottleIdle, 2000);
  MotorInputLeft = constrain_float(MotorInputLeft, ThrottleIdle, 2000);
  ServoInputRight = constrain_float(ServoInputRight, SERVO_MIN, SERVO_MAX);
  ServoInputLeft = constrain_float(ServoInputLeft, SERVO_MIN, SERVO_MAX);
  
  // Emergency stop
  if (channelValues[2] < 1030) {
    MotorInputRight = ThrottleCutOff;
    MotorInputLeft = ThrottleCutOff;
    ServoInputRight = SERVO_CENTER;
    ServoInputLeft = SERVO_CENTER;
    reset_pid();
  }
  
  // Write outputs
  motorRight.writeMicroseconds(MotorInputRight);
  motorLeft.writeMicroseconds(MotorInputLeft);
  servoRight.writeMicroseconds(ServoInputRight);
  servoLeft.writeMicroseconds(ServoInputLeft);
  
  // Maintain loop timing
  while (micros() - LoopTimer < (dt * 1000000));
  LoopTimer = micros();
}
