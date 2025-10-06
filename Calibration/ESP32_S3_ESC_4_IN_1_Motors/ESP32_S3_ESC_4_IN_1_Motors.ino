#include <ESP32Servo.h>

// Create 4 servo objects for 4 ESCs
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

// Define GPIO pins - CHANGE THESE to your actual pins
const int ESC1_PIN = 1;
const int ESC2_PIN = 2;
const int ESC3_PIN = 3;
const int ESC4_PIN = 5;

// ESC calibration values (in microseconds)
const int MIN_THROTTLE = 1000;  // Minimum pulse width (stopped)
const int MAX_THROTTLE = 2000;  // Maximum pulse width (full speed)
const int ARM_THROTTLE = 1000;  // Arming value

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-S3 ESC Control Starting...");
  
  // Attach ESCs to pins with min/max pulse width
  // Standard PWM frequency is 50Hz for ESCs
  esc1.setPeriodHertz(50);
  esc2.setPeriodHertz(50);
  esc3.setPeriodHertz(50);
  esc4.setPeriodHertz(50);
  
  esc1.attach(ESC1_PIN, MIN_THROTTLE, MAX_THROTTLE);
  esc2.attach(ESC2_PIN, MIN_THROTTLE, MAX_THROTTLE);
  esc3.attach(ESC3_PIN, MIN_THROTTLE, MAX_THROTTLE);
  esc4.attach(ESC4_PIN, MIN_THROTTLE, MAX_THROTTLE);
  
  Serial.println("ESCs attached to pins");
  
  // CRITICAL: ESC Arming Sequence
  // Send minimum throttle for 2-3 seconds to arm ESCs
  Serial.println("Arming ESCs... Send minimum throttle");
  armESCs();
  
  Serial.println("ESCs armed! Ready to control motors.");
  Serial.println("Starting test sequence in 2 seconds...");
  delay(2000);
}

void loop() {
  // Test sequence: gradually increase then decrease throttle
  testThrottleRamp();
  delay(3000);
}

// Arm all ESCs by sending minimum throttle signal
void armESCs() {
  esc1.writeMicroseconds(MIN_THROTTLE);
  esc2.writeMicroseconds(MIN_THROTTLE);
  esc3.writeMicroseconds(MIN_THROTTLE);
  esc4.writeMicroseconds(MIN_THROTTLE);
  delay(3000);  // Wait for ESC beeps indicating arming complete
}

// Set all motors to same throttle value
void setAllMotors(int throttle) {
  // Constrain throttle to safe range
  throttle = constrain(throttle, MIN_THROTTLE, MAX_THROTTLE);
  
  esc1.writeMicroseconds(throttle);
  esc2.writeMicroseconds(throttle);
  esc3.writeMicroseconds(throttle);
  esc4.writeMicroseconds(throttle);
  
  Serial.print("Throttle set to: ");
  Serial.println(throttle);
}

// Set individual motor throttle
void setMotor(int motorNum, int throttle) {
  throttle = constrain(throttle, MIN_THROTTLE, MAX_THROTTLE);
  
  switch(motorNum) {
    case 1:
      esc1.writeMicroseconds(throttle);
      break;
    case 2:
      esc2.writeMicroseconds(throttle);
      break;
    case 3:
      esc3.writeMicroseconds(throttle);
      break;
    case 4:
      esc4.writeMicroseconds(throttle);
      break;
  }
}

// Test function: ramp throttle up and down
void testThrottleRamp() {
  Serial.println("Starting throttle ramp test...");
  
  // Ramp up from min to 30% throttle
  for (int throttle = MIN_THROTTLE; throttle <= MIN_THROTTLE + 300; throttle += 10) {
    setAllMotors(throttle);
    delay(50);
  }
  
  delay(1000);
  
  // Ramp back down to minimum
  for (int throttle = MIN_THROTTLE + 300; throttle >= MIN_THROTTLE; throttle -= 10) {
    setAllMotors(throttle);
    delay(50);
  }
  
  Serial.println("Test complete");
}

// Emergency stop - call this to immediately stop all motors
void emergencyStop() {
  setAllMotors(MIN_THROTTLE);
  Serial.println("EMERGENCY STOP - All motors stopped");
}