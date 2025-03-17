void moveDoor(bool direction, int target) {
  digitalWrite(DIR_PIN, direction); // Set motor direction

  // Determine current position and distance to target
  int currentPos = encoderCount;
  int totalDistance = abs(target - currentPos);

  // Split the motion into 3 zones
  int accelDistance = totalDistance * 0.2;  // 20% for acceleration
  int decelDistance = totalDistance * 0.2;  // 20% for deceleration
  int cruiseDistance = totalDistance - accelDistance - decelDistance;

  int speed = MIN_SPEED;
  analogWrite(PWM_PIN, speed);

  Serial.print("[MOVING] ");
  Serial.println(direction == HIGH ? "Opening..." : "Closing...");

  while ((direction == HIGH && encoderCount < target) || (direction == LOW && encoderCount > target)) {
    int traveled = abs(encoderCount - currentPos);
    int remaining = abs(target - encoderCount);

    // === Speed Calculation ===
    if (traveled < accelDistance) {
      // Acceleration zone
      speed = map(traveled, 0, accelDistance, MIN_SPEED, MAX_SPEED);
    } else if (remaining < decelDistance) {
      // Deceleration zone
      speed = map(remaining, 0, decelDistance, MIN_SPEED, MAX_SPEED);
    } else {
      // Cruise zone
      speed = MAX_SPEED;
    }

    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    analogWrite(PWM_PIN, speed);

    // Debug output
    Serial.print("[ENC] Encoder: ");
    Serial.print(encoderCount);
    Serial.print(" | Speed: ");
    Serial.println(speed);

    // Emergency stop logic (same as before)
    if (direction == HIGH && digitalRead(CLOSE_BTN) == LOW) {
      Serial.println("[EMERGENCY] Open interrupted by Close. Stopping...");
      emergencyStop();
      handleClose();
      return;
    }
    if (direction == LOW && digitalRead(OPEN_BTN) == LOW) {
      Serial.println("[EMERGENCY] Close interrupted by Open. Stopping...");
      emergencyStop();
      handleOpen();
      return;
    }

    delay(50); // Polling delay
  }

  analogWrite(PWM_PIN, 0); // Stop motor
  Serial.println("[DONE] Door movement complete.");
  isOpening = false;
  isClosing = false;
  Serial.println("========================================");
}
