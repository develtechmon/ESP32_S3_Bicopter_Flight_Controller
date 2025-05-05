#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>

// ===== PPM Definitions =====
#define PPM_PIN 13             // Pin where the PPM signal is connected
#define NUM_CHANNELS 8         // Total number of channels in the PPM stream
#define PPM_SYNC_THRESHOLD 3000 // Pulse width (µs) above which a sync pulse is assumed
#define CHANNEL_MIN 1000       // Minimum valid pulse width (µs)
#define CHANNEL_MAX 2000       // Maximum valid pulse width (µs)

// ===== Flight Controller / PID Declarations =====
uint32_t LoopTimer;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float MotorInput1, MotorInput2;

// Servo control variables
float ServoInput1, ServoInput2;
int ServoCenter = 1500;        // Center position for servos
int ServoMin = 1000;           // Minimum servo position
int ServoMax = 2000;           // Maximum servo position
int ServoMaxTravel = 400;      // Maximum travel from center position (in µs)

// Global variables for PPM
volatile int ReceiverValue[NUM_CHANNELS] = {1500,1500,1000,1500,1500,1500,1500,1500};
volatile int channelIndex = 0;
volatile unsigned long lastTime = 0;

// Changed ESC frequency to 400Hz - better for BLHeli_S
int ESCfreq = 400;
int channelValues[NUM_CHANNELS];

// Throttle limits
int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;

// Servo and motor objects
Servo mot1;  // Left motor
Servo mot2;  // Right motor
Servo servo1; // Left servo
Servo servo2; // Right servo

// Pin definitions
const int mot1_pin = 6;    // Left motor pin
const int mot2_pin = 9;    // Right motor pin
const int servo1_pin = 7;  // Left servo pin
const int servo2_pin = 8;  // Right servo pin

// Time step (seconds)
const float t = 0.004; 

// Joystick deadzone (to prevent small movements from affecting control)
const int DEADZONE = 30;

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
    if(pulseWidth < CHANNEL_MIN) pulseWidth = CHANNEL_MIN;
    else if(pulseWidth > CHANNEL_MAX) pulseWidth = CHANNEL_MAX;
    ReceiverValue[channelIndex] = pulseWidth;
    channelIndex++;
  }
}

// ===== Helper Function to Safely Copy Receiver Values =====
void read_receiver(int *channelValues) {
  noInterrupts();
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channelValues[i] = ReceiverValue[i];
  }
  interrupts();
}

// Apply deadzone to joystick inputs
int applyDeadzone(int value, int center, int deadzone) {
  if (abs(value - center) < deadzone) {
    return center;
  }
  return value;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Bicopter Servo Control Initializing...");

  // ----- Setup PPM Receiver -----
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);

  // ----- Setup ESP32 PWM Timers for Motor ESCs and Servos -----
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  Serial.println("Starting ESC initialization sequence...");
  delay(3000); // Give time for ESCs to power up
  
  // ----- Attach Motors with correct range and frequency -----
  mot1.attach(mot1_pin, 1000, 2000);
  mot2.attach(mot2_pin, 1000, 2000);

  // Set PWM frequency - BLHeli_S works better at 400Hz or lower
  mot1.setPeriodHertz(ESCfreq);
  mot2.setPeriodHertz(ESCfreq);
    
  // Initialize motors to low throttle
  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  delay(4000);
  
  // ----- Attach Servos -----
  servo1.setPeriodHertz(50); // Standard 50Hz servo frequency
  servo2.setPeriodHertz(50);
  servo1.attach(servo1_pin, ServoMin, ServoMax);
  servo2.attach(servo2_pin, ServoMin, ServoMax);
  
  // Center servos at startup
  servo1.writeMicroseconds(ServoCenter);
  servo2.writeMicroseconds(ServoCenter);
  delay(1000);
  
  Serial.println("Waiting for throttle stick to be in low position...");
  // Wait until a valid throttle reading is received via PPM (channel index 2)
  while (true) {
    read_receiver(channelValues);
    // Assume channel 2 (index 2) is throttle; wait until it is near low position
    if (channelValues[2] > 1000 && channelValues[2] < 1050) {
      Serial.println("Throttle at minimum. System ready!");
      break;
    }
    delay(100);
  }
}

void loop() {
  read_receiver(channelValues);

  // Read inputs from receiver channels
  // For clarity, mapping the channels to their functions:
  // Channel 0: Roll (right stick X-axis) - not used for servos
  // Channel 1: Pitch (right stick Y-axis) - used for servo pitch control
  // Channel 2: Throttle (left stick Y-axis) - used for motor control
  // Channel 3: Yaw (left stick X-axis) - used for servo yaw control

  // Apply deadzone to prevent small inadvertent movements
  int roll = applyDeadzone(channelValues[0], 1500, DEADZONE);
  int pitch = applyDeadzone(channelValues[1], 1500, DEADZONE);
  int throttle = channelValues[2]; // No deadzone for throttle
  int yaw = applyDeadzone(channelValues[3], 1500, DEADZONE);

  // Process inputs for motors (throttle control)
  InputThrottle = throttle;
  
  // Smooth throttle changes to prevent ESC cutout
  static float lastThrottle = 1000;
  float maxChange = 20; // Max allowed change per loop
  
  // Limit the rate of throttle change
  if (InputThrottle > lastThrottle + maxChange)
    InputThrottle = lastThrottle + maxChange;
  else if (InputThrottle < lastThrottle - maxChange)
    InputThrottle = lastThrottle - maxChange;
    
  lastThrottle = InputThrottle;
  
  // Apply throttle to motors
  MotorInput1 = InputThrottle;
  MotorInput2 = InputThrottle;
  
  // Constrain motor values to safe range
  MotorInput1 = constrain(MotorInput1, 1000, 2000);
  MotorInput2 = constrain(MotorInput2, 1000, 2000);

  // Process inputs for servos (pitch and yaw control)
  // Map joystick inputs to appropriate servo movements
  
  // Calculate pitch control - CORRECTED DIRECTION
  // Right stick up (pitch < 1500) = servos move backward = pitch down
  // Right stick down (pitch > 1500) = servos move forward = pitch up
  InputPitch = map(pitch, 1000, 2000, -ServoMaxTravel, ServoMaxTravel);
  
  // Calculate yaw control
  // Left stick left (yaw < 1500) = servos move outward for left yaw
  // Left stick right (yaw > 1500) = servos move outward for right yaw
  InputYaw = map(yaw, 1000, 2000, ServoMaxTravel, -ServoMaxTravel);
  
  // Servo mixing for pitch and yaw
  // Note: The servos are mirrored, so one needs positive values 
  // while the other needs negative values for the same movement
  
  // For left servo (servo1):
  // - For forward pitch (pitch down): subtract from center (move servo forward)
  // - For yaw left/right: subtract from center (move servo outward)
  ServoInput1 = ServoCenter - (InputPitch * 0.5) - (InputYaw * 0.5);
  
  // For right servo (servo2):
  // - For forward pitch (pitch down): add to center (move servo forward) - MIRRORED
  // - For yaw left/right: subtract from center (move servo outward) - SAME DIRECTION
  ServoInput2 = ServoCenter + (InputPitch * 0.5) - (InputYaw * 0.5);
  
  // Constrain servo values to safe range
  ServoInput1 = constrain(ServoInput1, ServoMin, ServoMax);
  ServoInput2 = constrain(ServoInput2, ServoMin, ServoMax);

  // ----- Write Motor PWM Signals -----
  mot1.writeMicroseconds(MotorInput1);
  mot2.writeMicroseconds(MotorInput2);
  
  // ----- Write Servo PWM Signals -----
  servo1.writeMicroseconds(ServoInput1);
  servo2.writeMicroseconds(ServoInput2);

  // Debug output
  Serial.print("Roll [µs]: "); Serial.print(channelValues[0]);
  Serial.print("  Pitch [µs]: "); Serial.print(channelValues[1]);
  Serial.print("  Throttle [µs]: "); Serial.print(channelValues[2]);
  Serial.print("  Yaw [µs]: "); Serial.print(channelValues[3]);
  Serial.print("  Motors: L="); Serial.print(MotorInput1);
  Serial.print(" R="); Serial.print(MotorInput2);
  Serial.print("  Servos: L="); Serial.print(ServoInput1);
  Serial.print(" R="); Serial.println(ServoInput2);
  
  // Add a small delay to prevent serial buffer overflow
  delay(10);
}
