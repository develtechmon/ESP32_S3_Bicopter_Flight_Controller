#include <Wire.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>

// ==== Define the pin for the built-in LED. Change this if your board uses a different pin ====
#define LED_PIN 48  
#define NUM_LEDS 1

// ===== PPM Definitions =====
#define PPM_PIN 13             // Pin where the PPM signal is connected
#define NUM_CHANNELS 8         // Total number of channels in the PPM stream
#define PPM_SYNC_THRESHOLD 3000 // Pulse width (µs) above which a sync pulse is assumed
#define CHANNEL_MIN 1000       // Minimum valid pulse width (µs)
#define CHANNEL_MAX 2000       // Maximum valid pulse width (µs)

// Define your custom I2C pins (change these as needed)
#define CUSTOM_SDA_PIN 11   // Example: GPIO3
#define CUSTOM_SCL_PIN 12   // Example: GPIO4

// ===== Servo and Motor Definitions =====
#define SERVO_RIGHT_PIN 8         // Pin for servo 1 (right servo for pitch/yaw)
#define SERVO_LEFT_PIN 7         // Pin for servo 2 (left servo for pitch/yaw)
#define MOTOR_RIGHT_PIN 9        // Pin for motor 1 (right motor) - treated as servo
#define MOTOR_LEFT_PIN 6        // Pin for motor 2 (left motor) - treated as servo
#define SERVO_CENTER 1500     // Center position for servos (microseconds)
#define SERVO_MIN 1000        // Minimum servo position (microseconds)
#define SERVO_MAX 1900        // Maximum servo position (microseconds)

// Motor ESC frequency (BLHeli_S typically works best at 400Hz or lower)
int ESCfreq = 400;

// ===== Flight Controller / PID Declarations =====
uint32_t LoopTimer;
volatile float MotorInputRight, MotorInputLeft;  // Only 2 motors for bicopter
volatile float ServoInputRight, ServoInputLeft;  // Servo inputs for pitch/yaw control
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw, AccXCalibration, AccYCalibration, AccZCalibration;

// Global variables for PPM
volatile int ReceiverValue[NUM_CHANNELS] = {1500,1500,1000,1500,1500,1500,1500,1500};
volatile int channelIndex = 0;
volatile unsigned long lastTime = 0;
int channelValues[NUM_CHANNELS];

// PID parameters for angle control (pitch control is more important in bicopter)
// float PAngleRoll = 4, PAnglePitch = 4;  // Pitch needs stronger response
// float IAngleRoll = 0.3, IAnglePitch = 0.8;
// float DAngleRoll = 0.005, DAnglePitch = 0.01;

// float PAngleRoll = 4, PAnglePitch = 4;  // Pitch stronger for bicopter stability
// float IAngleRoll = 0.5, IAnglePitch = 0.5;  // Higher I for pitch hold
// float DAngleRoll = 0.007, DAnglePitch = 0.007; // More damping for pitch

float PAngleRoll = 1.5, PAnglePitch = 2.0;    // Much lower P gains
float IAngleRoll = 0.1, IAnglePitch = 0.2;    // Very low I gains
float DAngleRoll = 0.002, DAnglePitch = 0.005; // Low D gains


// PID parameters for rate control
// float PRateRoll = 0.5, PRatePitch = 1.2;    // Pitch needs stronger rate control
// float IRateRoll = 1.5, IRatePitch = 2.5;
// float DRateRoll = 0.006, DRatePitch = 0.012;

// float PRateRoll = 0.625, PRatePitch = 0.625;
// float IRateRoll = 2.1, IRatePitch = 2.1;
// float DRateRoll = 0.0088, DRatePitch = DRateRoll;

float PRateRoll = 0.3, PRatePitch = 0.4;      // Very low rate P
float IRateRoll = 0.8, IRatePitch = 1.0;      // Low rate I
float DRateRoll = 0.003, DRatePitch = 0.005;  // Minimal rate D

// // Yaw control (keep low to prevent oscillations)
// float PRateYaw = 2.0;
// float IRateYaw = 1.0;
// float DRateYaw = 0;

// float PRateYaw = 4;
// float IRateYaw = 3;
// float DRateYaw = 0;

float PRateYaw = 0.8;    // Very low yaw P
float IRateYaw = 0.3;    // Very low yaw I  
float DRateYaw = 0;      // No yaw D

// Throttle limits
int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;
int led_time = 500;

volatile float PtermRoll;
volatile float ItermRoll;
volatile float DtermRoll;
volatile float PIDOutputRoll;
volatile float PtermPitch;
volatile float ItermPitch;
volatile float DtermPitch;
volatile float PIDOutputPitch;
volatile float PtermYaw;
volatile float ItermYaw;
volatile float DtermYaw;
volatile float PIDOutputYaw;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;

// Kalman filters for angle estimation
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
volatile float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
volatile float Kalman1DOutput[] = {0, 0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

// Battery Parameters
float Voltage;

// PWM configuration: Not needed anymore - using servo control for motors
// const int pwmFrequency = 20000; // No longer needed
// const int pwmResolution = 8;    // No longer needed

// Motor and servo objects
Servo servoRight;  // Right servo (controls pitch and yaw)
Servo servoLeft;  // Left servo (controls pitch and yaw - mirrored)
Servo motorRight;  // Right motor (A2212/15T) - controlled as servo
Servo motorLeft;  // Left motor (A2212/15T) - controlled as servo

// Time step (seconds)
const float t = 0.004; 

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ===== Arming/Disarming Variables =====
bool armed = false;             // Drone armed state
unsigned long armDisarmTimer = 0;
const unsigned long armHoldTime = 1000; // milliseconds required to hold stick

// ===== PPM Interrupt Handler =====
// Use IRAM_ATTR for faster interrupt handling on the ESP32.
void IRAM_ATTR ppmInterruptHandler() {
  unsigned long currentTime = micros();
  unsigned long pulseWidth = currentTime - lastTime;
  lastTime = currentTime;

  if (pulseWidth > PPM_SYNC_THRESHOLD) {
    // A long pulse is assumed to be the sync pulse – reset channel index.
    channelIndex = 0;
  } else if (channelIndex < NUM_CHANNELS) {
    // Store a valid channel pulse (constrained to valid range)
    if (pulseWidth < CHANNEL_MIN) pulseWidth = CHANNEL_MIN;
    else if (pulseWidth > CHANNEL_MAX) pulseWidth = CHANNEL_MAX;
    ReceiverValue[channelIndex] = pulseWidth;
    channelIndex++;
  }
}

// ==== Read Battery Voltage ====
void battery_voltage(void) {
  Voltage = (float)analogRead(1) / 237; // GPIO1
}

// ===== Helper Function to Safely Copy Receiver Values =====
void read_receiver(int *channelValues) {
  noInterrupts();
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channelValues[i] = ReceiverValue[i];
  }
  interrupts();
}

// ===== Simple 1D Kalman Filter =====
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + (t * KalmanInput);
  KalmanUncertainty = KalmanUncertainty + (t * t * 4 * 4); // IMU variance (4 deg/s)
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3); // error variance (3 deg)
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState; 
  Kalman1DOutput[1] = KalmanUncertainty;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  strip.begin();       // Initialize the NeoPixel strip
  strip.show();        // Turn all pixels off as an initial state

  // Flash built-in LED a few times for startup indication
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(led_time);
    digitalWrite(LED_BUILTIN, LOW);
    delay(led_time);
  }
  
  // ----- Setup PPM Receiver -----
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);
  delay(100);

  // ----- Setup MPU6050 (I2C) -----
  Wire.setClock(400000);
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // ----- Setup ESP32 PWM Timers for ESCs and Servos -----
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // ----- Initialize Motors (ESCs) -----
  Serial.println("Initializing ESCs...");
  delay(1000);

  // ----- Setup PWM for Motor Drivers - Now using Servo control -----
  motorRight.attach(MOTOR_RIGHT_PIN,1000,2000);
  delay(500);
  motorRight.setPeriodHertz(ESCfreq);

  motorLeft.attach(MOTOR_LEFT_PIN,1000,2000);
  delay(500);
  motorLeft.setPeriodHertz(ESCfreq);

  // Start with minimum throttle for ESC initialization
  motorRight.writeMicroseconds(1000);
  motorLeft.writeMicroseconds(1000);
  delay(2000);  // Give ESCs time to initialize

  // ----- Setup Servos -----
  servoRight.setPeriodHertz(50);
  servoLeft.setPeriodHertz(50);
  servoRight.attach(SERVO_RIGHT_PIN, SERVO_MIN, SERVO_MAX);
  servoLeft.attach(SERVO_LEFT_PIN, SERVO_MIN, SERVO_MAX);
  
  // Initialize servos to center position
  servoRight.writeMicroseconds(SERVO_CENTER);
  servoLeft.writeMicroseconds(SERVO_CENTER);
  delay(1000);

  // ----- Calibration Values for A2212 Motors -----
  RateCalibrationRoll=-1.98;
  RateCalibrationPitch=1.96;
  RateCalibrationYaw=0.38;
  AccXCalibration=0.06;
  AccYCalibration=-0.01;
  AccZCalibration=-0.08;

  // Green LED to indicate normal startup
  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();
  delay(1000);
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();

  LoopTimer = micros();
}

void loop() {
  read_receiver(channelValues);

  // ----- Arming/Disarming Logic -----
  // Check if throttle (channel 2) is low enough to allow arming/disarming
  if (channelValues[2] < 1050) {
    // To arm: yaw (channel 3) high
    if (!armed && channelValues[3] > 1900) {
      if (armDisarmTimer == 0) {
        armDisarmTimer = millis();
      } else if (millis() - armDisarmTimer > armHoldTime) {
        armed = true;
        // Yellow LED (Red + Green) to indicate arming
        strip.setPixelColor(0, strip.Color(255, 255, 0));
        strip.show();
        delay(1000);
        strip.setPixelColor(0, strip.Color(0, 0, 0));
        strip.show();
        armDisarmTimer = 0;
      }
    }
    // To disarm: yaw (channel 3) low
    else if (armed && channelValues[3] < 1100) {
      if (armDisarmTimer == 0) {
        armDisarmTimer = millis();
      } else if (millis() - armDisarmTimer > armHoldTime) {
        armed = false;
        // White LED (all colors) to indicate disarming
        strip.setPixelColor(0, strip.Color(255, 255, 255));
        strip.show();
        delay(1000);
        strip.setPixelColor(0, strip.Color(0, 0, 0));
        strip.show();
        armDisarmTimer = 0;
      }
    } else {
      armDisarmTimer = 0;
    }
  } else {
    armDisarmTimer = 0;
  }
  
  // If not armed, immediately cut off motor output and reset PID integrals
  if (!armed) {
    MotorInputRight = MotorInputLeft = ThrottleCutOff;
    ServoInputRight = ServoInputLeft = SERVO_CENTER;
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
    
    // Update motor and servo outputs
    motorRight.writeMicroseconds(MotorInputRight);
    motorLeft.writeMicroseconds(MotorInputLeft);
    
    servoRight.writeMicroseconds(ServoInputRight);
    servoLeft.writeMicroseconds(ServoInputLeft);
    
    while (micros() - LoopTimer < (t * 1000000));
    LoopTimer = micros();
    return; // Skip the rest of the control loop if disarmed
  }

  // ----- Read MPU6050 Data -----
  // Request accelerometer data
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
  
  // Request gyroscope data
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
  
  RateRoll  = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw   = (float)GyroZ / 65.5;
  AccX      = (float)AccXLSB / 4096;
  AccY      = (float)AccYLSB / 4096;
  AccZ      = (float)AccZLSB / 4096;

  // Apply calibration offsets
  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;
  AccX      -= AccXCalibration;
  AccY      -= AccYCalibration;
  AccZ      -= AccZCalibration;

  // ----- Calculate Angles from Accelerometer -----
  AngleRoll  = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;
  
  // Run simple Kalman filters for both angles
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Clamp the filtered angles to ±20 degrees
  KalmanAngleRoll = (KalmanAngleRoll > 20) ? 20 : ((KalmanAngleRoll < -20) ? -20 : KalmanAngleRoll);
  KalmanAnglePitch = (KalmanAnglePitch > 20) ? 20 : ((KalmanAnglePitch < -20) ? -20 : KalmanAnglePitch);

  // ----- Set Desired Angles and Throttle from Receiver Inputs -----
  DesiredAngleRoll  = 0.1 * (ReceiverValue[0] - 1500);
  DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
  InputThrottle     = ReceiverValue[2];
  DesiredRateYaw    = 0.15 * (ReceiverValue[3] - 1500);

  // --- Angle PID for Roll ---
  ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
  PIDOutputRoll = constrain(PtermRoll + ItermRoll + DtermRoll, -400, 400);
  DesiredRateRoll = PIDOutputRoll;
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;

  // --- Angle PID for Pitch ---
  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
  PIDOutputPitch = constrain(PtermPitch + ItermPitch + DtermPitch, -400, 400);
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // ----- Rate PID Calculations -----
  ErrorRateRoll  = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw   = DesiredRateYaw - RateYaw;

  // Roll Rate PID
  PtermRoll = PRateRoll * ErrorRateRoll;
  ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
  PIDOutputRoll = constrain(PtermRoll + ItermRoll + DtermRoll, -400, 400);
  InputRoll = PIDOutputRoll;
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;

  // Pitch Rate PID
  PtermPitch = PRatePitch * ErrorRatePitch;
  ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
  PIDOutputPitch = constrain(PtermPitch + ItermPitch + DtermPitch, -400, 400);
  InputPitch = PIDOutputPitch;
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  // Yaw Rate PID
  PtermYaw = PRateYaw * ErrorRateYaw;
  ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
  ItermYaw = constrain(ItermYaw, -400, 400);
  DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
  PIDOutputYaw = constrain(PtermYaw + ItermYaw + DtermYaw, -400, 400);
  InputYaw = PIDOutputYaw;
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;

  if (InputThrottle > 2000) { 
    InputThrottle = 2000;
  }

  // ----- BICOPTER MIXING -----
  // For bicopter: 2 motors provide lift and roll control, 2 servos provide pitch and yaw control
  
  // Motor mixing: differential thrust for roll control
  // MotorInputLeft = InputThrottle - InputRoll;  // Left motor
  // MotorInputRight = InputThrottle + InputRoll;  // Right motor
  
  MotorInputLeft = InputThrottle - (InputRoll * 0.5);   // Reduced roll authority
  MotorInputRight = InputThrottle + (InputRoll * 0.5);  // Reduced roll authority

  // Servo mixing: collective for pitch, differential for yaw
  // When controller pitch is pushed down, both servos should tilt motors down (forward pitch)
  // When controller yaw is pushed, servos should move opposite to each other
  ServoInputLeft = SERVO_CENTER - InputPitch - InputYaw;  // Left servo (mirrored for yaw)
  ServoInputRight = SERVO_CENTER + InputPitch - InputYaw;  // Right servo

  // Clamp motor outputs to safe range
  MotorInputRight = constrain(MotorInputRight, ThrottleIdle, 2000);
  MotorInputLeft = constrain(MotorInputLeft, ThrottleIdle, 2000);

  // Clamp servo outputs to safe range
  ServoInputRight = constrain(ServoInputRight, SERVO_MIN, SERVO_MAX);
  ServoInputLeft = constrain(ServoInputLeft, SERVO_MIN, SERVO_MAX);

  // If throttle is too low, reset PID integrals and cut motors
  //if (ReceiverValue[2] < 1030) {
  if  (channelValues[2] < 1030 {
    MotorInputRight = MotorInputLeft = ThrottleCutOff;
    ServoInputRight = ServoInputLeft = SERVO_CENTER;
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
  }

  // --- Update motor and servo outputs using writeMicroseconds ---
  motorRight.writeMicroseconds(MotorInputRight);
  motorLeft.writeMicroseconds(MotorInputLeft);

  // Update servo positions using writeMicroseconds as requested
  servoRight.writeMicroseconds(ServoInputRight);
  servoLeft.writeMicroseconds(ServoInputLeft);

  while (micros() - LoopTimer < (t * 1000000));
  LoopTimer = micros();
}
