void acquireAndProcessAngle()  //This does all the heavy lifting. Samples enocder from SPI and stores to moving average. 
 {
  uint16_t newAngle = magAlpha.readAngleRaw();   // Acquire angle from MagAlpha MPS encoder

  // Calculate the difference, taking wrap-around into account
  double angleDifference = (newAngle - lastAngle) / 180;

  // Handle wrap-around from 360 to 0
  if (angleDifference < -180) {
    angleDifference += 360;
  }
  // Handle wrap-around from 0 to 360
  else if (angleDifference > 180) {
    angleDifference -= 360;
  }

  totalRotation += angleDifference; // Accumulate total rotation


  /////// angle averages
  angleAvg.push(totalRotation); //adds to angle moving average buffer
  angleMovingAverage = angleAvg.get(); //adds to angle moving average buffer

  /////// velocity averages
  angleVelocity = angleMovingAverage - prevAngleMovingAverage;  // Compute the first derivative of the angle moving average (velocity)
  if (angleVelocity > scroll_vel_max) //velocity cap
  {
    angleVelocity = scroll_vel_max;
  }
   if (angleVelocity < -scroll_vel_max) //velocity cap
  {
    angleVelocity = -scroll_vel_max;
  }

  
  veloAvg.push(angleVelocity); //adds to angle moving average buffer
  velocityMovingAverage = (veloAvg.get());

  //acceleration average
  acceleration = velocityMovingAverage - prevVelAvg;  // Compute the first derivative of the angle moving average (velocity)
  accAvg.push(acceleration); //adds to angle moving average buffer
  accMovingAverage = (accAvg.get());  //computes average from buffer 

  /////previous states
  prevAngleMovingAverage = angleMovingAverage;
  prevVelAvg = velocityMovingAverage;
  lastAngle = newAngle;

}

void idlecheck() {  //this function checks to see if the wheel has moved in the last X secondss. Needs to be very short to not crash ISR. 
  
  //Serial.println("Checking idle state");  //only enable this for debug. Might crash ISR
  digitalWrite(pin_MPS_enable, HIGH);  // wake up and enable pin MPS enocder
  acquireAndProcessAngle();
  int difference = abs(abs(totalRotation) - abs(prevTotalRotation));


  if (difference > 50)  //disables idle. Important to make high enough that picking it up doesn't kick it out of idle
  {
    if (isIdle == true)
    {
      //  Serial.println("Idle disabled ");   //only enable this for debug. Might crash ISR
    }

    isIdle = false;                      // More than 360 degrees of rotation, not idle
    digitalWrite(pin_MPS_enable, HIGH);  // enable pin MPS enocder
    idleCounts = 0;

  } else {
    idleCounts = idleCounts + 1;
  }

  
  if (idleCounts > idleCountThreshold) //enables idle
  {
    isIdle = true;                      // Less than 360 degrees of rotation, considered idle
    digitalWrite(pin_MPS_enable, LOW);  // disable pin MPS enocder
    // Serial.println("Idle enabled");  //only enable this for debug. Might crash ISR
  }

   if (idleCounts > 3600) //if idle for an 3600 seconds(hour), reboot
  {
    NVIC_SystemReset(); //reboot
    // Serial.println("Rebooting");  //only enable this for debug. Might crash ISR
  }
  prevTotalRotation = totalRotation;
}



void scroll_calcs() //computes how many pixels to scroll by on each loop event
{
  scroll_distance = scroll_base_distance + abs(scroll_scale * velocityMovingAverage) + abs(scroll_acc * pow(accMovingAverage, 2)); //most important equation 
  scrollAvg.push(scroll_distance);  //adds to moving average buffer 
  scrollAverage = (scrollAvg.get());  //computes average from buffer 
}



void startAdvertising(void) //bluetooth pairing
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_MOUSE);
  Bluefruit.Advertising.addService(blehid);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(90);    // number of seconds in fast mode
  Bluefruit.Advertising.start(0);              // 0 = Don't stop advertising after n seconds
}



void scroll_android(int16_t x, int16_t y, bool tipTouch) //uses the touchpad HID for Android scrolling
{
  uint8_t report[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //build blank report
  report[0] = tipTouch ? 0x03 : 0x00;  // clicked or not clicked
  uint8_t pressure;
  uint8_t size;
  pressure = 50;
  size = 50;
  report[1] = pressure;   //these dont do anything yet
  report[2] = size;     //these dont do anything yet
  report[3] = x & 0xff; //first byte
  report[4] = (x >> 8) & 0xff;//second byte
  report[5] = y & 0xff; //first byte
  report[6] = (y >> 8) & 0xff;//second byte
  blehid.inputReport(0x01, report, sizeof(report)); //construct and send usb hid byte report for Android on report ID 2
}


void scroll_PC(int16_t input)  //uses the Microsoft surface dial HID for PC scrolling
{
  input = PC_scroll_scale * (input - scroll_base_distance); //we have to remove the base because the PC mode is inremental instead of absolute
  uint8_t report[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };  //build blank report
  report[0] = 0;  //no button click
  report[1] = input & 0xFF;  //first byte
  report[2] = (input >> 8) & 0xFF;  //second byte
  blehid.inputReport(0x03, report, sizeof(report)); //construct and send usb hid byte report for PC on report ID 3
}


void clickMouseButton(uint8_t buttonNumber, bool isPressed) //Third device, can be used for back click. Not currently used.
{
  if (buttonNumber < 1 || buttonNumber > 4) {
    Serial.println("Invalid button number. Please choose a value between 1 and 4.");
    return;
  }

  // Initialize the report. Adjust the size if your report structure is different.
  // Assuming 1 byte for buttons, 1 byte for padding, 2 bytes for X and Y axis.
  uint8_t report[4] = {0x00, 0x00, 0x00, 0x00};

  // Set or clear the appropriate bit for the button
  if (isPressed) {
    report[0] |= (1 << (buttonNumber - 1)); // Press the button
  } else {
    report[0] &= ~(1 << (buttonNumber - 1)); // Release the button
  }

  // Send the report
  // Assuming you have a function to send a report given its ID and the report data
  // Replace `sendReport` with whatever function you use to send HID reports
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
  delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 750ns before register readout
  digitalWrite(SPI_CS_PIN, LOW);
  reg_frame1 = SPI.transfer16(0x0000); //this is the angle. We ignore this.

  digitalWrite(SPI_CS_PIN, HIGH);
  delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 750ns before register readout
  digitalWrite(SPI_CS_PIN, LOW);

  reg_frame2 = SPI.transfer16(0x0000); // Capture the entire 16-bit frame directly.

  digitalWrite(SPI_CS_PIN, HIGH);
  delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 750ns after register readout

  reg_contents = reg_frame1 & 0xFF; // Extract the last 8 bits from reg_frame1

  //print to serial case
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


void readAllRegisters() //reads all MPS encoder memory registers
{
  uint8_t registers[] = {0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 14, 22, 26};
  for (int i = 0; i < sizeof(registers); i++) {
    readRegister_MA780(registers[i], true);
  }
}


void mouse_back()   //experimental back button
{
  delay(200); // Hold the button for 100 milliseconds
  scroll_android(x_start, y_pos, false);
  Serial.println("Click back");
  delay(200); // Hold the button for 200 milliseconds
  clickMouseButton(4, true); // Press button 4 (back button)
  delay(100); // Hold the button for 100 milliseconds
  clickMouseButton(4, false); // Release  button 4 (back button)
  delay(1000); // Hold the button for 1000 milliseconds
  // return;
}
