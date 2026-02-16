// ================================================================================
// Project     : ESP32-S3 Bicopter Flight Controller
// File        : Altitude_Hold.ino
// Description : Altitude hold functions using MTF-02 built-in rangefinder.
//               All variables declared in Bicopter_Main_Altitude_Hold.ino
//
// Channel map:
//   Channel 6 (index 6) > 1500 → altitude hold ON
//   Throttle stick < 1400      → landing mode
//   Throttle stick outside ±25 deadband → nudge target altitude
// ================================================================================

// ================================================================================
// PRIVATE: Read raw distance from MTF-02 internal rangefinder
// ================================================================================
int readMTF02Distance() {
  if (mtf02_is_distance_valid()) {
    return mtf02_get_raw_distance_cm();
  }
  return 0;
}

// ================================================================================
// PRIVATE: Circular buffer filter
// ================================================================================
int applyCircularBufferFilter(int newValue) {
  lidarHistTab[lidarHistIdx] = newValue;
  lidarSum += lidarHistTab[lidarHistIdx];
  lidarSum -= lidarHistTab[(lidarHistIdx + 1) % LIDAR_BUFFER_SIZE];
  lidarHistIdx++;
  if (lidarHistIdx == LIDAR_BUFFER_SIZE) lidarHistIdx = 0;
  return lidarSum / (LIDAR_BUFFER_SIZE - 1);
}

// ================================================================================
// PRIVATE: Complementary filter with tilt correction
// ================================================================================
int applyComplementaryFilter(int newValue) {
  float rollRad        = KalmanAngleRoll  * 0.0174533;
  float pitchRad       = KalmanAnglePitch * 0.0174533;
  float ang_from_level = sqrt(rollRad * rollRad + pitchRad * pitchRad);

  float angleCorrectedValue = newValue * cos(ang_from_level);

  complementaryFiltered = COMP_FILTER_ALPHA * angleCorrectedValue
                        + (1.0 - COMP_FILTER_ALPHA) * complementaryFiltered;
  return (int)complementaryFiltered;
}

// ================================================================================
// PRIVATE: Velocity from altitude delta — used for D term
// ================================================================================
void calculateAltitudeVelocity() {
  static float tf_alt_prev = 0;
  altitudeVelocity = (currentAltitude - tf_alt_prev) / dt;
  tf_alt_prev = currentAltitude;
}

// ================================================================================
// PUBLIC: Called once in setup()
// ================================================================================
void initializeAltitudeSystem() {
  mtf02_init();
  delay(100);

  // Enable external altitude once here only — not inside the loop
  mtf02_enable_external_altitude(true);
  mtf02_set_external_altitude_timeout(100);

  // Zero the circular buffer
  for (int i = 0; i < LIDAR_BUFFER_SIZE; i++) {
    lidarHistTab[i] = 0;
  }
  lidarSum     = 0;
  lidarHistIdx = 0;

  // Zero filter state
  complementaryFiltered = 0;
  altitudeVelocity      = 0;
  altitudeHoldStartTime = 0;

  // Ground reference calibration — 50 readings over 500ms
  Serial.println("Calibrating ground reference...");
  int   validReadings = 0;
  float sum           = 0;

  for (int i = 0; i < 50; i++) {
    mtf02_update();
    int rawReading = readMTF02Distance();
    if (rawReading > 0) {
      int circBuf  = applyCircularBufferFilter(rawReading);
      int compFilt = applyComplementaryFilter(circBuf);
      sum += compFilt;
      validReadings++;
    }
    delay(10);
    if (i % 10 == 0) digitalWrite(2, !digitalRead(2));
  }

  if (validReadings > 5) {
    groundReference = sum / validReadings;
    Serial.print("Ground reference: ");
    Serial.print(groundReference);
    Serial.println(" cm");
  } else {
    groundReference = 0;
    Serial.print("Using default ground reference: ");
    Serial.println(groundReference);
  }

  Serial.println("MTF-02 altitude system ready");
}

// ================================================================================
// STEP 1: Read sensor, filter, update currentAltitude each loop
// ================================================================================
void a_updateAltitude() {
  mtf02_update();

  int rawReading = readMTF02Distance();

  if (rawReading > 0) {
    int circBuf  = applyCircularBufferFilter(rawReading);
    int compFilt = applyComplementaryFilter(circBuf);

    currentAltitude = compFilt - groundReference;
    if (currentAltitude < 0) currentAltitude = 0;

    calculateAltitudeVelocity();

    // Feed filtered altitude back to MTF-02 — matches your working version exactly
    mtf02_set_external_altitude(complementaryFiltered);

  } else {
    mtf02_enable_external_altitude(false);
  }

  // Serial.print("currentAltitude: ");
  // Serial.print(currentAltitude);
  // Serial.print(" - ");
}

// ================================================================================
// STEP 2: Activate or deactivate altitude hold via switch
// ================================================================================
void b_handleAltitudeHold() {
  bool altHoldSwitch = (ReceiverValue[6] > 1500);

  if (altHoldSwitch && !altitudeHoldActive && armed && currentAltitude > 15) {
    altitudeHoldActive = true;
    targetAltitude     = currentAltitude;
    baseThrottle       = ReceiverValue[2];
    landingMode        = false;

    // Reset PID state on activation
    altPID_error     = 0;
    altPID_errorPrev = 0;
    altPID_integral  = 0;
    altPID_output    = 0;

    Serial.print("Altitude Hold ACTIVATED at ");
    Serial.print(targetAltitude);
    Serial.println(" cm");
  }

  if (!altHoldSwitch && altitudeHoldActive) {
    altitudeHoldActive = false;
    landingMode        = false;
    altPID_output      = 0;
    Serial.println("Altitude Hold DEACTIVATED");
  }
}

// ================================================================================
// STEP 3: Pilot throttle stick adjusts target altitude or triggers landing
// ================================================================================
void c_handlePilotAltitudeInput() {
  if (!altitudeHoldActive) return;

  int throttleInput  = ReceiverValue[2];
  int throttleCenter = 1500;
  int throttleError  = throttleInput - throttleCenter;

  if (throttleInput < LANDING_THROTTLE_THRESHOLD) {
    if (!landingMode) {
      landingMode          = true;
      landingStartAltitude = currentAltitude;
      landingStartTime     = millis();
      Serial.println("LANDING MODE ACTIVATED - Gentle descent");
    }

    float elapsedTime    = (millis() - landingStartTime) / 1000.0;
    float descentDistance = LANDING_RATE_CMS * elapsedTime;
    targetAltitude        = landingStartAltitude - descentDistance;

    if (targetAltitude < 8) targetAltitude = 8;

    if (throttleInput < 1050 && currentAltitude < 20) {
      altitudeHoldActive = false;
      landingMode        = false;
      Serial.println("Landing complete - altitude hold disabled");
    }
    return;

  } else {
    if (landingMode) {
      landingMode    = false;
      targetAltitude = currentAltitude;
      Serial.println("Landing mode CANCELLED");
    }
  }

  if (abs(throttleError) > THROTTLE_DEADBAND) {
    float altitudeChangeRate = (throttleError - (throttleError > 0 ? THROTTLE_DEADBAND : -THROTTLE_DEADBAND)) * 0.03;

    if (altitudeChangeRate >  15) altitudeChangeRate =  15;
    if (altitudeChangeRate < -10) altitudeChangeRate = -10;

    targetAltitude += altitudeChangeRate * dt * 80;

    if (targetAltitude <  10) targetAltitude =  10;
    if (targetAltitude > 300) targetAltitude = 300;
  }

  Serial.print("targetAltitude: ");
  Serial.print(targetAltitude);
  Serial.print(" - ");
}

// ================================================================================
// STEP 4: PID — outputs altPID_output added to base throttle
// ================================================================================
void d_calculateAltitudePID() {
  if (!altitudeHoldActive) {
    altPID_output   = 0;
    altPID_integral = 0;
    return;
  }

  altPID_error = targetAltitude - currentAltitude;

  if (abs(altPID_error) < 3.0) altPID_error = 0;

  unsigned long timeSinceActivation = millis() - altitudeHoldStartTime;
  float gainScale = 1.0;
  if (timeSinceActivation < 2000) {
    gainScale = (float)timeSinceActivation / 2000.0;
    gainScale = constrain(gainScale, 0.5, 1.0);
  }

  float effective_Kp = altPID_Kp * gainScale;
  float effective_Ki = altPID_Ki * gainScale;
  float effective_Kd = altPID_Kd * gainScale;

  float P_term = effective_Kp * altPID_error;

  altPID_integral += altPID_error * dt;
  if (altPID_integral >  altPID_integralMax) altPID_integral =  altPID_integralMax;
  if (altPID_integral < -altPID_integralMax) altPID_integral = -altPID_integralMax;
  float I_term = effective_Ki * altPID_integral;

  float D_term = effective_Kd * altitudeVelocity;

  altPID_output = P_term + I_term - D_term;

  if (altPID_output >  150) altPID_output =  150;
  if (altPID_output < -150) altPID_output = -150;
}

// ================================================================================
// PUBLIC: Returns final throttle for motor mixing
// ================================================================================
int calculateThrottleOutput() {
  if (altitudeHoldActive) {
    int throttle;

    if (landingMode) {
      throttle = baseThrottle - 50 + (int)altPID_output;
    } else {
      throttle = baseThrottle + (int)altPID_output;
    }

    if (throttle > 1750) throttle = 1750;
    if (throttle < 1100) throttle = 1100;
    return throttle;

  } else {
    return ReceiverValue[2];
  }
}
