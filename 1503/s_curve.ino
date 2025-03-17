float smoothStep(float x) {
  return 3 * x * x - 2 * x * x * x;
}

void moveDoor(bool direction, int target) {
  digitalWrite(DIR_PIN, direction); // Set direction

  int startPos = encoderCount;
  int totalDistance = abs(target - startPos);
  int accelDistance = totalDistance * 0.2;
  int decelDistance = totalDistance * 0.2;
  int cruiseDistance = totalDistance - accelDistance - decelDistance;

  Serial.print("[MOVING-S-CURVE] ");
  Serial.println(direction == HIGH ? "Opening..." : "Closing...");

  while ((direction == HIGH && encoderCount < target) || (direction == LOW && encoderCount > target)) {
    int currentPos = abs(encoderCount - startPos);
    int remaining = abs(target - encoderCount);
    int speed;

    if (currentPos < accelDistance) {
      // Acceleration S-curve
      float x = (float)currentPos / accelDistance;
      speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * smoothStep(x);
    } else if (remaining < decelDistance) {
      // Deceleration S-curve
      float x = (float)remaining / decelDistance;
      speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * smoothStep(x);
    } else {
      // Cruise
      speed = MAX_SPEED;
    }

    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    analogWrite(PWM_PIN, speed);

    // Debug
    Serial.print("[ENC] Encoder: ");
    Serial.print(encoderCount);
    Serial.print(" | Speed: ");
    Serial.println(speed);

    // Emergency Stop Logic
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

    delay(50);
  }

  analogWrite(PWM_PIN, 0); // Stop motor
  Serial.println("[DONE] Door movement complete.");
  isOpening = false;
  isClosing = false;
  Serial.println("========================================");
}
