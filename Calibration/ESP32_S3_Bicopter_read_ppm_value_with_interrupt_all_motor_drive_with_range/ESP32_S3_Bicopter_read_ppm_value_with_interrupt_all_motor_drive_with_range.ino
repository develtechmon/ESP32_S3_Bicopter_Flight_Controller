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
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

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

// Servo objects to control motors
Servo mot1;
Servo mot2;

const int mot1_pin = 6;
const int mot2_pin = 9;

// Time step (seconds)
const float t = 0.004; 

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

void setup() {
  Serial.begin(115200);
  
  // ----- Setup PPM Receiver -----
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);

  // ----- Setup ESP32 PWM Timers for Motor ESCs -----
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  
  Serial.println("Starting ESC initialization sequence...");
  delay(3000); // Give time for ESCs to power up
  
  // ----- Attach Motors with correct range and frequency -----
  mot1.attach(mot1_pin, 1000, 2000);
  mot2.attach(mot2_pin, 1000, 2000);
  
  // Set PWM frequency - BLHeli_S works better at 400Hz or lower
  mot1.setPeriodHertz(ESCfreq);
  mot2.setPeriodHertz(ESCfreq);
  
  // Proper ESC initialization sequence
  Serial.println("Sending minimum throttle signal...");
  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  delay(4000); // Give ESCs time to recognize the low signal
  
  Serial.println("ESC initialization complete!");
  
  // Optional: Throttle safety check
  Serial.println("Waiting for low throttle position...");
  while (true) {
    read_receiver(channelValues);
    // Check if throttle is at minimum position
    if (channelValues[2] < 1100) {
      Serial.println("Throttle at minimum. Ready!");
      break;
    }
    delay(100);
  }
}

void loop() {
  read_receiver(channelValues);

  // Get throttle input (channel 3 in most systems)
  InputThrottle = channelValues[2];
  
  // Add safety to prevent sudden throttle jumps
  static float lastThrottle = 1000;
  float maxChange = 20; // Max allowed change per loop
  
  // Limit the rate of throttle change
  if (InputThrottle > lastThrottle + maxChange)
    InputThrottle = lastThrottle + maxChange;
  else if (InputThrottle < lastThrottle - maxChange)
    InputThrottle = lastThrottle - maxChange;
    
  lastThrottle = InputThrottle;
  
  // Apply throttle to both motors
  MotorInput1 = InputThrottle;
  MotorInput2 = InputThrottle;

  // ----- Write Motor PWM Signals -----
  mot1.writeMicroseconds(MotorInput1);
  mot2.writeMicroseconds(MotorInput2);

  // Optional: Print receiver channels for debugging
  Serial.print("Roll [µs]: "); Serial.print(channelValues[0]);
  Serial.print("  Pitch [µs]: "); Serial.print(channelValues[1]);
  Serial.print("  Throttle [µs]: "); Serial.print(channelValues[2]);
  Serial.print("  Yaw [µs]: "); Serial.print(channelValues[3]);
  Serial.print("  Motor1: "); Serial.print(MotorInput1);
  Serial.print("  Motor2: "); Serial.println(MotorInput2);
  
  delay(10); // Short delay to prevent serial buffer overrun
}
