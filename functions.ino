void acquireAndProcessAngle()  //This function samples the encoder via SPI, calculates the angle, velocity, and acceleration, and updates their respective moving averages.
{
  uint16_t newAngle = magAlpha.readAngleRaw();   // Acquire raw angle from MagAlpha MPS encoder

  // Calculate the angular difference, considering the circular nature of angles
  double angleDifference = (newAngle - lastAngle) / 180;

  // Correct for wrap-around when transitioning from 360 to 0 degrees
  if (angleDifference < -180) {
    angleDifference += 360;
  }
  // Correct for wrap-around when transitioning from 0 to 360 degrees
  else if (angleDifference > 180) {
    angleDifference -= 360;
  }

  totalRotation += angleDifference; // Accumulate total rotation for tracking

  // Update the moving average of the angle
  angleAvg.push(totalRotation); // Adds the current total rotation to the moving average buffer
  angleMovingAverage = angleAvg.get(); // Computes the moving average of the angle

  // Calculate and cap the angular velocity
  angleVelocity = angleMovingAverage - prevAngleMovingAverage;  // Compute the velocity as the first derivative of the angle moving average
  if (angleVelocity > scroll_vel_max) // Cap the velocity to a maximum value
  {
    angleVelocity = scroll_vel_max;
  }
  if (angleVelocity < -scroll_vel_max) // Cap the velocity to a minimum value
  {
    angleVelocity = -scroll_vel_max;
  }

  // Update the moving average of the velocity
  veloAvg.push(angleVelocity); // Adds the current velocity to the moving average buffer
  velocityMovingAverage = veloAvg.get(); // Computes the moving average of the velocity

  // Calculate and update the moving average of the acceleration
  acceleration = velocityMovingAverage - prevVelAvg;  // Compute the acceleration as the first derivative of the velocity moving average
  accAvg.push(acceleration); // Adds the current acceleration to the moving average buffer
  accMovingAverage = accAvg.get();  // Computes the moving average of the acceleration

  // Store previous states for use in the next iteration
  prevAngleMovingAverage = angleMovingAverage;
  prevVelAvg = velocityMovingAverage;
  lastAngle = newAngle;

}

void idlecheck() {  // This function checks if the wheel has been idle for a specified duration and handles the device's idle state accordingly.

  // Uncomment for debugging: Serial.println("Checking idle state");
  digitalWrite(pin_MPS_enable, HIGH);  // Activate the MPS encoder
  acquireAndProcessAngle();
  int difference = abs(abs(totalRotation) - abs(prevTotalRotation));



  // Disable idle mode if the wheel has moved significantly
  if (difference > 50)  // Threshold to prevent accidental idle disable
  {
    // Uncomment for debugging: Serial.println("Idle disabled");
    isIdle = false;
    digitalWrite(pin_MPS_enable, HIGH);  // Keep the MPS encoder active
    idleCounts = 0;
  } else {
    idleCounts += 1;
  }

  // Enable idle mode if the wheel has been inactive for long enough
  if (idleCounts > idleCountThreshold) // Check if the idle count exceeds the threshold
  {
    isIdle = true;
    digitalWrite(pin_MPS_enable, LOW);  // Deactivate the MPS encoder
    // Uncomment for debugging: Serial.println("Idle enabled");
  }

  // Reboot the system if it has been idle for an extended period
  if (idleCounts > 3600) // Check if idle for 3600 seconds (1 hour)
  {
    NVIC_SystemReset(); // Reboot the system
    // Uncomment for debugging: Serial.println("Rebooting");
  }
  prevTotalRotation = totalRotation;
}

void scroll_calcs() // This function calculates the scroll distance based on velocity and acceleration, and updates the scroll distance moving average.
{
  scroll_distance = scroll_base_distance + abs(scroll_scale * velocityMovingAverage) + abs(scroll_acc * pow(accMovingAverage, 2)); // Calculate scroll distance based on velocity and acceleration
  scrollAvg.push(scroll_distance);  // Adds the current scroll distance to the moving average buffer
  scrollAverage = scrollAvg.get();  // Computes the moving average of the scroll distance
}

void startAdvertising(void) // This function sets up and starts Bluetooth advertising for device pairing.
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_MOUSE);
  Bluefruit.Advertising.addService(blehid);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // Set advertising interval in units of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(90);    // Set duration of fast advertising mode in seconds
  Bluefruit.Advertising.start(0);              // Start advertising indefinitely
}

void scroll_android(int16_t x, int16_t y, bool tipTouch) // This function sends a touchpad HID report for Android devices, simulating touchpad scrolling.
{
  uint8_t report[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // Initialize a blank HID report
  report[0] = tipTouch ? 0x03 : 0x00;  // Set the touch state (clicked or not clicked)
  uint8_t pressure = 50; // Placeholder for pressure value
  uint8_t size = 50; // Placeholder for size value
  report[1] = pressure;   // Set pressure (currently not functional)
  report[2] = size;     // Set size (currently not functional)
  report[3] = x & 0xff; // Set the lower byte of the x-coordinate
  report[4] = (x >> 8) & 0xff;// Set the upper byte of the x-coordinate
  report[5] = y & 0xff; // Set the lower byte of the y-coordinate
  report[6] = (y >> 8) & 0xff;// Set the upper byte of the y-coordinate
  blehid.inputReport(0x01, report, sizeof(report)); // Send the HID report for Android with report ID 2
}

void scroll_PC(int16_t input)  // This function sends a HID report for PC devices, simulating the Microsoft Surface Dial scrolling.
{
  input = PC_scroll_scale * (input - scroll_base_distance); // Adjust the input by scaling and offsetting based on base distance
  uint8_t report[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };  // Initialize a blank HID report
  report[0] = 0;  // No button click
  report[1] = input & 0xFF;  // Set the lower byte of the input
  report[2] = (input >> 8) & 0xFF;  // Set the upper byte of the input
  blehid.inputReport(0x03, report, sizeof(report)); // Send the HID report for PC with report ID 3
}

void clickMouseButton(uint8_t buttonNumber, bool isPressed) // This function sends a HID report to simulate mouse button clicks.
{
  if (buttonNumber < 1 || buttonNumber > 4) {
    Serial.println("Invalid button number. Please choose a value between 1 and 4."); // Validate the button number
    return;
  }

  // Initialize a blank HID report
  uint8_t report[4] = {0x00, 0x00, 0x00, 0x00};

  // Set or clear the appropriate bit for the button state
  if (isPressed) {
    report[0] |= (1 << (buttonNumber - 1)); // Press the specified button
  } else {
    report[0] &= ~(1 << (buttonNumber - 1)); // Release the specified button
  }

  // Send the HID report with report ID 2
  blehid.inputReport(0x02, report, sizeof(report));
}

//////// MPS encoder functions
uint8_t readRegister_MA780(uint8_t address, bool print) {

#define READ_REG_COMMAND    (0b010 << 13)
#define WRITE_REG_COMMAND   (0b100 << 13)
  uint16_t  reg_frame1, reg_frame2;
  uint8_t  reg_contents;
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer16(READ_REG_COMMAND | ((address & 0x1F) << 8) | 0x00);

  digitalWrite(SPI_CS_PIN, HIGH);
  delayMicroseconds(1); // Wait for 1us to respect tIdleReg of 750ns before register readout
  digitalWrite(SPI_CS_PIN, LOW);
  reg_frame1 = SPI.transfer16(0x0000); // Ignore this transfer result as it's not needed for the register read

  digitalWrite(SPI_CS_PIN, HIGH);
  delayMicroseconds(1); // Wait for 1us to respect tIdleReg of 750ns before the next register readout
  digitalWrite(SPI_CS_PIN, LOW);

  reg_frame2 = SPI.transfer16(0x0000); // Capture the entire 16-bit frame directly.

  digitalWrite(SPI_CS_PIN, HIGH);
  delayMicroseconds(1); // Wait for 1us to respect tIdleReg of 750ns after register readout

  reg_contents = reg_frame1 & 0xFF; // Extract the last 8 bits from reg_frame1 which contain the register value

  // Optionally print the register contents to the serial monitor
  if (print) {
    Serial.print("Read Register[");
    Serial.print(address);
    Serial.print("]    BIN=");
    for (int bit = 7; bit >= 0; --bit) {
      Serial.print((reg_contents >> bit) & 1);
    }
    Serial.print(" HEX=");
    Serial.print(reg_contents, HEX);
    Serial.print(" DEC=");
    Serial.println(reg_contents, DEC);
  }
  return reg_contents;
}

void readAllRegisters() // This function reads all relevant MPS encoder registers and optionally prints their values.
{
  uint8_t registers[] = {0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 14, 22, 26};
  for (int i = 0; i < sizeof(registers); i++) {
    readRegister_MA780(registers[i], true); // Read and print each register
  }
}

void mouse_back()   // This experimental function simulates a mouse back button press using a combination of delays and HID reports.
{
  delay(200); // Delay to simulate holding the button
  scroll_android(x_start, y_pos, false); // Call scroll function with no touch
  Serial.println("Click back"); // Print action to serial monitor
  delay(200); // Delay to simulate holding the button
  clickMouseButton(4, true); // Press the back button (button 4)
  delay(100); // Delay to simulate holding the button
  clickMouseButton(4, false); // Release the back button
  delay(1000); // Delay to simulate end of action
  // return;
}
