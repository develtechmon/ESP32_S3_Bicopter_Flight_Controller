// ================================================================================
// Project     : ESP32-S3 Bicopter Flight Controller
// File        : Position_Hold.ino
// Description : Position hold functions using MTF-02 optical flow.
//               All variables declared in Bicopter_Main_Altitude_Hold.ino
//
// Changes from working 5inch quad version:
//   1. Switch moved to CH7 (channelValues[7]) — CH6 is altitude hold
//   2. mtf02_update() removed — already called in a_updateAltitude()
//   3. t → dt to match bicopter main file timing variable
//   4. Requires altitudeHoldActive before activating — optical flow
//      scaling is only valid when altitude hold is stable
//   5. If altitude hold drops, position hold auto-deactivates
//
// Channel map:
//   CH7 (channelValues[7]) > 1500 → position hold ON
//   Requires CH6 altitude hold already active
// ================================================================================

// ================================================================================
// STEP 1: Read optical flow velocities, integrate into position
// NOTE: mtf02_update() NOT called here — already called in a_updateAltitude()
// ================================================================================
void a_updatePosition() {
  if (!positionHoldActive) return;

  // Only integrate on valid frames — skip bad frames without zeroing
  if (!mtf02_is_velocity_valid()) return;

  float currentVelX = mtf02_get_velocity_x_cms();
  float currentVelY = mtf02_get_velocity_y_cms();

  // Update shared filter
  pidVelFilteredX = 0.7 * pidVelFilteredX + 0.3 * currentVelX;
  pidVelFilteredY = 0.7 * pidVelFilteredY + 0.3 * currentVelY;

  unsigned long now = millis();

  if (posLastUpdate > 0) {
    float dtPos = (now - posLastUpdate) / 1000.0f;
    if (dtPos > 0 && dtPos < 0.1f) {
      currentPositionX += pidVelFilteredX * dtPos;
      currentPositionY += pidVelFilteredY * dtPos;
    }
  }
  posLastUpdate = now;
}

// ================================================================================
// STEP 2: Activate or deactivate position hold via switch
// Requires altitude hold already active — flow scaling depends on stable height
// ================================================================================
void b_handlePositionHold() {
  bool posHoldSwitch = (channelValues[7] > 1500);

  // ACTIVATION — requires altitude hold already running
  if (posHoldSwitch && !positionHoldActive && armed && altitudeHoldActive && currentAltitude > 15) {
    positionHoldActive    = true;
    positionHoldStartTime = millis();

    currentPositionX = 0;
    currentPositionY = 0;
    targetPositionX  = 0;
    targetPositionY  = 0;

    targetVelocityX = 0;
    targetVelocityY = 0;

    // Reset PID state
    posX_integral = 0; posX_error = 0; posX_errorPrev = 0;
    posY_integral = 0; posY_error = 0; posY_errorPrev = 0;
    velX_integral = 0; velX_error = 0; velX_errorPrev = 0;
    velY_integral = 0; velY_error = 0; velY_errorPrev = 0;

    // Reset filter state — prevents residual velocity from bleeding in at activation
    pidVelFilteredX  = 0;
    pidVelFilteredY  = 0;
    posLastUpdate    = 0;
    smoothRollInput  = 0;
    smoothPitchInput = 0;
    filtered_posD_X  = 0;
    filtered_posD_Y  = 0;
    filtered_velD_X  = 0;
    filtered_velD_Y  = 0;

    positionCorrectionRoll  = 0;
    positionCorrectionPitch = 0;

    Serial.println("Position Hold ACTIVATED");
  }

  // DEACTIVATION via switch
  if (!posHoldSwitch && positionHoldActive) {
    positionHoldActive      = false;
    targetVelocityX         = 0;
    targetVelocityY         = 0;
    positionCorrectionRoll  = 0;
    positionCorrectionPitch = 0;
    Serial.println("Position Hold DEACTIVATED");
  }

  // Auto-deactivate if altitude hold drops
  if (!altitudeHoldActive && positionHoldActive) {
    positionHoldActive      = false;
    targetVelocityX         = 0;
    targetVelocityY         = 0;
    positionCorrectionRoll  = 0;
    positionCorrectionPitch = 0;
    Serial.println("Position Hold OFF - altitude hold lost");
  }
}

// ================================================================================
// STEP 3: Pilot stick input shifts velocity target
// ================================================================================
void c_handlePilotPositionInput() {
  if (!positionHoldActive) return;

  int rollInput  = channelValues[0] - 1500;
  int pitchInput = channelValues[1] - 1500;

  // Use global variables — reset properly on activation
  smoothRollInput  = 0.8 * smoothRollInput  + 0.2 * rollInput;
  smoothPitchInput = 0.8 * smoothPitchInput + 0.2 * pitchInput;

  if (abs(smoothRollInput) > STICK_DEADBAND) {
    targetVelocityX = -smoothRollInput * 0.03;
    targetVelocityX = constrain(targetVelocityX, -20, 20);
  } else {
    targetVelocityX = 0;
  }

  if (abs(smoothPitchInput) > STICK_DEADBAND) {
    targetVelocityY = smoothPitchInput * 0.03;
    targetVelocityY = constrain(targetVelocityY, -20, 20);
  } else {
    targetVelocityY = 0;
  }

  // When sticks centered, sync position target to current position
  if (abs(smoothRollInput) <= STICK_DEADBAND && abs(smoothPitchInput) <= STICK_DEADBAND) {
    unsigned long now = millis();
    if (now - posLastUpdate > 200) {
      targetPositionX  = currentPositionX;
      targetPositionY  = currentPositionY;
    }
  }
}

// ================================================================================
// STEP 4: Dual-loop velocity-based PID
// Outer loop: position error → velocity target
// Inner loop: velocity error → angle correction
// ================================================================================
void d_calculatePositionPID() {
  if (!positionHoldActive) {
    targetVelocityX         = 0;
    targetVelocityY         = 0;
    positionCorrectionRoll  = 0;
    positionCorrectionPitch = 0;
    return;
  }

  float currentVelX = mtf02_get_velocity_x_cms();
  float currentVelY = mtf02_get_velocity_y_cms();

  // On invalid frame — keep last correction, don't zero it
  // Original quad behavior: filter updates even on invalid, correction continues
  // Zeroing on invalid frames causes snap-to-zero then snap-back oscillation
  if (!mtf02_is_velocity_valid()) return;

  // Update filter only on valid frames
  pidVelFilteredX = 0.7 * pidVelFilteredX + 0.3 * currentVelX;
  pidVelFilteredY = 0.7 * pidVelFilteredY + 0.3 * currentVelY;

  // ---- Outer Loop: Position → Velocity ----
  if (abs(targetVelocityX) < 1.0 && abs(targetVelocityY) < 1.0) {

    posX_error = targetPositionX - currentPositionX;
    posY_error = targetPositionY - currentPositionY;

    if (abs(posX_error) < 15.0) posX_error = 0;
    if (abs(posY_error) < 15.0) posY_error = 0;

    // X-axis position PID
    float posP_X = posToVel_Kp * posX_error;

    posX_integral += posX_error * dt;
    posX_integral  = constrain(posX_integral, -10.0, 10.0);
    float posI_X   = posToVel_Ki * posX_integral;

    float posD_X_raw = (posX_error - posX_errorPrev) / dt;
    filtered_posD_X  = 0.8 * filtered_posD_X + 0.2 * posD_X_raw;
    float posD_X     = posToVel_Kd * filtered_posD_X;
    posX_errorPrev   = posX_error;

    float generatedVelX = posP_X + posI_X + posD_X;
    generatedVelX       = constrain(generatedVelX, -15.0, 15.0);

    // Y-axis position PID
    float posP_Y = posToVel_Kp * posY_error;

    posY_integral += posY_error * dt;
    posY_integral  = constrain(posY_integral, -10.0, 10.0);
    float posI_Y   = posToVel_Ki * posY_integral;

    float posD_Y_raw = (posY_error - posY_errorPrev) / dt;
    filtered_posD_Y  = 0.8 * filtered_posD_Y + 0.2 * posD_Y_raw;
    float posD_Y     = posToVel_Kd * filtered_posD_Y;
    posY_errorPrev   = posY_error;

    float generatedVelY = posP_Y + posI_Y + posD_Y;
    generatedVelY       = constrain(generatedVelY, -15.0, 15.0);

    targetVelocityX = generatedVelX;
    targetVelocityY = generatedVelY;
  }

  // ---- Inner Loop: Velocity → Angle ----
  velX_error = targetVelocityX - pidVelFilteredX;
  velY_error = targetVelocityY - pidVelFilteredY;

  if (abs(velX_error) < 5.0) velX_error = 0;
  if (abs(velY_error) < 5.0) velY_error = 0;

  // X-axis velocity PID — roll via motor differential (fast response)
  float velP_X = vel_Kp_Roll * velX_error;

  velX_integral += velX_error * dt;
  velX_integral  = constrain(velX_integral, -8.0, 8.0);
  float velI_X   = vel_Ki_Roll * velX_integral;

  float velD_X_raw = (velX_error - velX_errorPrev) / dt;
  filtered_velD_X  = 0.85 * filtered_velD_X + 0.15 * velD_X_raw;
  float velD_X     = vel_Kd_Roll * filtered_velD_X;
  velX_errorPrev   = velX_error;

  float velOutput_X = velP_X + velI_X + velD_X;
  velOutput_X       = constrain(velOutput_X, -6.0, 6.0);

  // Y-axis velocity PID — pitch via servo tilt
  float velP_Y = vel_Kp_Pitch * velY_error;

  velY_integral += velY_error * dt;
  velY_integral  = constrain(velY_integral, -8.0, 8.0);
  float velI_Y   = vel_Ki_Pitch * velY_integral;

  float velD_Y_raw = (velY_error - velY_errorPrev) / dt;
  filtered_velD_Y  = 0.85 * filtered_velD_Y + 0.15 * velD_Y_raw;
  float velD_Y     = vel_Kd_Pitch * filtered_velD_Y;
  velY_errorPrev   = velY_error;

  float velOutput_Y = velP_Y + velI_Y + velD_Y;
  velOutput_Y       = constrain(velOutput_Y, -5.0, 5.0);

  // Convert to angle corrections
  // Roll: positive = right motor more thrust = tilts left (resists rightward drift)
  // Pitch: positive = servo tilts forward, negative = tilts backward
  positionCorrectionRoll  =  velOutput_X * 0.8;
  positionCorrectionPitch =  velOutput_Y * 0.8;

  positionCorrectionRoll  = constrain(positionCorrectionRoll,  -5.0, 5.0);
  positionCorrectionPitch = constrain(positionCorrectionPitch, -5.0, 5.0);
}
