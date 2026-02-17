// ================================================================================
// Project     : ESP32-S3 Bicopter Flight Controller
// File        : Position_Hold.ino
// Description : Velocity-based position hold using MTF-02 optical flow.
//               Ported from working quadcopter implementation with bicopter fixes:
//                 - No static locals: all state in globals (declared in main)
//                 - Split velocity PID gains: Roll (motor diff) vs Pitch (servo chain)
//                 - Correct channel map: CH6 (idx 5) = alt hold, CH7 (idx 6) = pos hold
//                 - Alt hold required as prerequisite for pos hold
//                 - dt used throughout (not t)
//
// Dual-loop architecture:
//   Position error [cm] → [Outer PID] → velocity target [cm/s]
//   Velocity error [cm/s] → [Inner PID] → angle correction [deg]
//   positionCorrectionRoll/Pitch → injected into DesiredAngleRoll/Pitch in main loop
//
// Axis notes:
//   X (Roll)  : motor differential — fast, direct. Uses vel_Kp_Roll etc.
//   Y (Pitch) : servo → angle PID → rate PID chain — 3 layers of lag. Uses vel_Kp_Pitch etc.
//
// All variables declared in Bicopter_Main_Altitude_Hold.ino
// ================================================================================

// ================================================================================
// STEP 1: Integrate optical flow velocity → position estimate each loop
// ================================================================================
void a_updatePosition() {
    // NOTE: mtf02_update() already called in a_updateAltitude(). Don't call again.
    // Double-calling at 100Hz causes packet loss on the UART.

    if (!positionHoldActive) return;

    // Raw velocities from MTF-02 (cm/s, body frame)
    float currentVelX = mtf02_get_velocity_x_cms();
    float currentVelY = mtf02_get_velocity_y_cms();

    // Low-pass filter: suppress optical flow noise.
    // Analogy: heavy flywheel — small bumps barely shift it,
    //          but sustained motion moves it steadily.
    const float VEL_FILTER_ALPHA = 0.7f;
    pidVelFilteredX = VEL_FILTER_ALPHA * pidVelFilteredX + (1.0f - VEL_FILTER_ALPHA) * currentVelX;
    pidVelFilteredY = VEL_FILTER_ALPHA * pidVelFilteredY + (1.0f - VEL_FILTER_ALPHA) * currentVelY;

    // Integrate filtered velocity → position (dead reckoning)
    unsigned long now = millis();
    if (posLastUpdate > 0) {
        float dtPos = (now - posLastUpdate) / 1000.0f;
        if (dtPos > 0.0f && dtPos < 0.1f) {   // sanity gate: reject stalls/wraps
            currentPositionX += pidVelFilteredX * dtPos;
            currentPositionY += pidVelFilteredY * dtPos;
        }
    }
    posLastUpdate = now;
}

// ================================================================================
// STEP 2: Activate / deactivate position hold via CH7
// Prerequisites: armed + altitudeHoldActive (CH6) + altitude > 15cm
// ================================================================================
void b_handlePositionHold() {
    bool posHoldSwitch = (channelValues[7] > 1500);   // CH7 = index 6

    // ---- ACTIVATION ----
    if (posHoldSwitch && !positionHoldActive && armed && altitudeHoldActive && currentAltitude > 15) {

        positionHoldActive    = true;
        positionHoldStartTime = millis();

        // Zero reference at activation point
        currentPositionX = 0.0f;  currentPositionY = 0.0f;
        targetPositionX  = 0.0f;  targetPositionY  = 0.0f;
        targetVelocityX  = 0.0f;  targetVelocityY  = 0.0f;

        // Clear all PID state — stale integrals cause a lurch on activation
        posX_integral = 0.0f;  posX_error = 0.0f;  posX_errorPrev = 0.0f;
        posY_integral = 0.0f;  posY_error = 0.0f;  posY_errorPrev = 0.0f;
        velX_integral = 0.0f;  velX_error = 0.0f;  velX_errorPrev = 0.0f;
        velY_integral = 0.0f;  velY_error = 0.0f;  velY_errorPrev = 0.0f;

        // Clear filter states (all declared as globals in main)
        pidVelFilteredX = 0.0f;  pidVelFilteredY = 0.0f;
        filtered_posD_X = 0.0f;  filtered_posD_Y = 0.0f;
        filtered_velD_X = 0.0f;  filtered_velD_Y = 0.0f;
        smoothRollInput = 0.0f;  smoothPitchInput = 0.0f;
        posLastUpdate   = millis();

        positionCorrectionRoll  = 0.0f;
        positionCorrectionPitch = 0.0f;

        Serial.println("Position Hold ACTIVATED");
        Serial.print("  Alt: "); Serial.print(currentAltitude); Serial.println(" cm");
    }

    // ---- DEACTIVATION: switch off ----
    if (!posHoldSwitch && positionHoldActive) {
        positionHoldActive      = false;
        targetVelocityX         = 0.0f;  targetVelocityY         = 0.0f;
        positionCorrectionRoll  = 0.0f;  positionCorrectionPitch = 0.0f;
        Serial.println("Position Hold DEACTIVATED (switch)");
    }

    // ---- DEACTIVATION: alt hold killed underneath us ----
    // If you kill CH6, pos hold must also die — it depends on altitude stability
    if (positionHoldActive && !altitudeHoldActive) {
        positionHoldActive      = false;
        targetVelocityX         = 0.0f;  targetVelocityY         = 0.0f;
        positionCorrectionRoll  = 0.0f;  positionCorrectionPitch = 0.0f;
        Serial.println("Position Hold DEACTIVATED (alt hold off)");
    }
}

// ================================================================================
// STEP 3: Pilot stick → velocity targets
// Outside deadband → command velocity (relative flight)
// Inside deadband  → freeze target position (hold here)
// ================================================================================
void c_handlePilotPositionInput() {
    if (!positionHoldActive) return;

    int rawRoll  = ReceiverValue[0] - 1500;   // CH1 ±500
    int rawPitch = ReceiverValue[1] - 1500;   // CH2 ±500

    // Smooth stick to prevent jerky velocity commands.
    // Analogy: pushing through light mud — you feel the stick, but it
    //          doesn't snap the drone to a new velocity instantaneously.
    const float STICK_SMOOTH_ALPHA = 0.8f;
    smoothRollInput  = STICK_SMOOTH_ALPHA * smoothRollInput  + (1.0f - STICK_SMOOTH_ALPHA) * rawRoll;
    smoothPitchInput = STICK_SMOOTH_ALPHA * smoothPitchInput + (1.0f - STICK_SMOOTH_ALPHA) * rawPitch;

    bool rollActive  = (abs(smoothRollInput)  > STICK_DEADBAND);
    bool pitchActive = (abs(smoothPitchInput) > STICK_DEADBAND);

    // Scale: 0.03 cm/s per µs → full deflection (~450µs past deadband) ≈ ±13.5 cm/s
    const float STICK_VEL_SCALE = 0.03f;

    targetVelocityX = rollActive  ? constrain(-smoothRollInput  * STICK_VEL_SCALE, -20.0f, 20.0f) : 0.0f;
    targetVelocityY = pitchActive ? constrain( smoothPitchInput * STICK_VEL_SCALE, -20.0f, 20.0f) : 0.0f;

    // Both sticks centered: update hold target to current position
    // 200ms debounce — position estimate is noisy, don't thrash the target every 4ms
    if (!rollActive && !pitchActive) {
        static unsigned long lastTargetUpdate = 0;
        unsigned long now = millis();
        if (now - lastTargetUpdate > 200) {
            targetPositionX  = currentPositionX;
            targetPositionY  = currentPositionY;
            lastTargetUpdate = now;
        }
    }
}

// ================================================================================
// STEP 4: Dual-loop PID → positionCorrectionRoll / positionCorrectionPitch
//
// OUTER LOOP (position → velocity): only when sticks centered
// INNER LOOP (velocity → angle):    always
//
// KEY BICOPTER DIFFERENCE vs quad:
//   Roll  (X): vel_Kp_Roll  — motor diff, fast direct response
//   Pitch (Y): vel_Kp_Pitch — servo→angle→rate chain, 3 layers of lag, needs lower gain
//
// Analogy: Roll is a sports car (instant throttle response).
//          Pitch is a truck (3 gearboxes between pedal and wheels).
//          Same gain = one is nervous, other is dead.
// ================================================================================
void d_calculatePositionPID() {
    if (!positionHoldActive) {
        targetVelocityX         = 0.0f;  targetVelocityY         = 0.0f;
        positionCorrectionRoll  = 0.0f;  positionCorrectionPitch = 0.0f;
        return;
    }

    // Bail on stale sensor — don't fly on dead reckoning alone
    if (!mtf02_is_velocity_valid()) {
        positionCorrectionRoll  = 0.0f;
        positionCorrectionPitch = 0.0f;
        return;
    }

    // =========================================================================
    // OUTER LOOP: Position error [cm] → velocity target [cm/s]
    // Only runs when pilot is not commanding velocity (sticks centered)
    // =========================================================================
    if (abs(targetVelocityX) < 1.0f && abs(targetVelocityY) < 1.0f) {

        posX_error = targetPositionX - currentPositionX;
        posY_error = targetPositionY - currentPositionY;

        // Deadband: optical flow has ~3-5cm noise floor at hover — don't chase it
        if (abs(posX_error) < 5.0f) posX_error = 0.0f;
        if (abs(posY_error) < 5.0f) posY_error = 0.0f;

        // --- X outer PID ---
        float posP_X     = posToVel_Kp * posX_error;
        posX_integral   += posX_error * dt;
        posX_integral    = constrain(posX_integral, -10.0f, 10.0f);
        float posI_X     = posToVel_Ki * posX_integral;
        // Heavy D filter (alpha=0.2) — position derivative is very noisy
        float posD_X_raw = (posX_error - posX_errorPrev) / dt;
        filtered_posD_X  = 0.8f * filtered_posD_X + 0.2f * posD_X_raw;
        float posD_X     = posToVel_Kd * filtered_posD_X;
        posX_errorPrev   = posX_error;
        float generatedVelX = constrain(posP_X + posI_X + posD_X, -15.0f, 15.0f);

        // --- Y outer PID ---
        float posP_Y     = posToVel_Kp * posY_error;
        posY_integral   += posY_error * dt;
        posY_integral    = constrain(posY_integral, -10.0f, 10.0f);
        float posI_Y     = posToVel_Ki * posY_integral;
        float posD_Y_raw = (posY_error - posY_errorPrev) / dt;
        filtered_posD_Y  = 0.8f * filtered_posD_Y + 0.2f * posD_Y_raw;
        float posD_Y     = posToVel_Kd * filtered_posD_Y;
        posY_errorPrev   = posY_error;
        float generatedVelY = constrain(posP_Y + posI_Y + posD_Y, -15.0f, 15.0f);

        targetVelocityX = generatedVelX;
        targetVelocityY = generatedVelY;
    }

    // =========================================================================
    // INNER LOOP: Velocity error [cm/s] → angle correction [degrees]
    // Uses split gains — Roll and Pitch are mechanically different axes
    // =========================================================================
    velX_error = targetVelocityX - pidVelFilteredX;
    velY_error = targetVelocityY - pidVelFilteredY;

    // Velocity deadband: ~3 cm/s is sensor noise floor, don't amplify it
    if (abs(velX_error) < 3.0f) velX_error = 0.0f;
    if (abs(velY_error) < 3.0f) velY_error = 0.0f;

    // --- X (Roll) inner PID — motor differential, fast axis ---
    float velP_X     = vel_Kp_Roll * velX_error;
    velX_integral   += velX_error * dt;
    velX_integral    = constrain(velX_integral, -8.0f, 8.0f);
    float velI_X     = vel_Ki_Roll * velX_integral;
    float velD_X_raw = (velX_error - velX_errorPrev) / dt;
    filtered_velD_X  = 0.85f * filtered_velD_X + 0.15f * velD_X_raw;
    float velD_X     = vel_Kd_Roll * filtered_velD_X;
    velX_errorPrev   = velX_error;
    float velOutput_X = constrain(velP_X + velI_X + velD_X, -6.0f, 6.0f);

    // --- Y (Pitch) inner PID — servo chain, slow axis ---
    float velP_Y     = vel_Kp_Pitch * velY_error;
    velY_integral   += velY_error * dt;
    velY_integral    = constrain(velY_integral, -8.0f, 8.0f);
    float velI_Y     = vel_Ki_Pitch * velY_integral;
    float velD_Y_raw = (velY_error - velY_errorPrev) / dt;
    filtered_velD_Y  = 0.85f * filtered_velD_Y + 0.15f * velD_Y_raw;
    float velD_Y     = vel_Kd_Pitch * filtered_velD_Y;
    velY_errorPrev   = velY_error;
    float velOutput_Y = constrain(velP_Y + velI_Y + velD_Y, -6.0f, 6.0f);

    // =========================================================================
    // Velocity output → angle corrections
    positionCorrectionRoll  = constrain( velOutput_X * 0.8f, -5.0f, 5.0f);  // REMOVE minus sign
    positionCorrectionPitch = constrain( velOutput_Y * 0.8f, -5.0f, 5.0f);  // Keep as-is
}