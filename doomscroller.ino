#include <bluefruit.h>
#include <math.h>
#include <MagAlpha.h>   //third party library, need to download
#include <MovingAveragePlus.h>  //third party library, need to download


volatile bool verbose_angle = true;   //for printing telementry to serial port for debug. Use with BeterSerialPlotter

///// Android user adjustable paramters - speed responses
double scroll_threshold = 0.22;    //min velocity to start scrolling. betwen 0.1 and 0.8. Too low will cause it to never settle
int scroll_time = 4;             // milliseconds of main loop.  Should be between 2-20ms. Too slow will cause lag in android
double scroll_base_distance = 3;  //min pixels to scroll in velocity calculation
double scroll_scale = 11;         //Scroll speed gain factor. between 5 -20
double scroll_vel_max = 8;         //Sets max to scroll speed. between 5 -20
double scroll_acc = 50;           //between 10-100 if you want acceleration boosted response - IE flicking. Instragram benefits from this being higher.

///// PC user adjustable paramters - speed responses
double PC_scroll_scale = 0.25;  //Scroll speed gain factor for PC dial driver. This scales down the default android calculations.

///// User adjustable paramters - speed responses
int idleCountThreshold = 30;  // seconds until idle mode starts
const char* deviceName = "Doomscroller168";  //bluetooth device name



MovingAveragePlus<double> angleAvg(15); // Initializes a moving average for angle with a window size of 6
MovingAveragePlus<double> veloAvg(10); // Initializes a moving average for velocity with a window size of 10
MovingAveragePlus<double> accAvg(7);   // Initializes a moving average for acceleration with a window size of 7
MovingAveragePlus<double> scrollAvg(5); // Initializes a moving average for scroll distance with a window size of 5


//// system
unsigned long time_ms; // Variable to store time in milliseconds
int x_start = 5000; // Initial x-coordinate on screen  for scrolling action. Range is 0-10000
int y_start = 5000; // Initial y-coordinate for scrolling action.  Range is 0-10000
#define ANDROID_REPORT_ID 1 // Report ID for Android HID reports
#define PC_REPORT_ID 3 // Report ID for PC HID reports
#define UART_BAUDRATE 115200 // Baud rate for UART serial communication
#define SPI_SCLK_FREQUENCY 5000000 // SPI clock frequency in Hz. Don't go above 10mhz 
#define SPI_CS_PIN 0 // Chip Select pin for SPI communication
#define pin_MPS_enable 1 // Pin to enable/disable MPS (Magnetic Position Sensor)
#define TIMER_IDLE_INTERVAL_MS 1000 // Interval in milliseconds to check for idle state. Determines how fast it wakes up 



//// Idle detection parameters
volatile double prevTotalRotation ; // Stores the previous total rotation value for idle detection
int idleCounts;                 // Counter for consecutive idle checks
volatile bool isIdle = false; // Flag to indicate if the device is currently idle
volatile bool isClicked = false; // Flag to indicate if a click action has occurred (not directly related to idle detection but may be used for UI interaction)
volatile double currentTime; // Variable to store the current time, potentially for idle time tracking
volatile double idleEvents;  // Counter for the number of times the device has been detected as idle
unsigned long lastIdleCheckTime = 0;  // Stores the last time the idle check was performed
const unsigned long idleCheckInterval = TIMER_IDLE_INTERVAL_MS;  // Idle check interval in milliseconds

//// Position and velocity variables
double newAngle; // Stores the latest angle measured from the sensor
volatile double angleVelocity; // Stores the rate of change of angle over time, making it accessible in interrupt context
volatile double acceleration; // Stores the rate of change of velocity, indicating how quickly the velocity is increasing or decreasing
double scroll_distance; // Calculates how far to scroll based on velocity and acceleration
volatile double angleMovingAverage; // A moving average of the angle to smooth out rapid fluctuations
volatile double velocityMovingAverage; // A moving average of the velocity for smoother scrolling action
volatile double accMovingAverage; // A moving average of the acceleration to stabilize erratic changes
volatile double scrollAverage; // A moving average of the scroll distance to ensure consistent scrolling speed
volatile double prevAngleMovingAverage ; // Stores the previous angle moving average for comparison in the next cycle
volatile double prevVelAvg; // Stores the previous velocity average for calculating acceleration
volatile double lastAngle; // The last angle measurement used to calculate the new velocity
volatile double totalRotation; // Sum of all angle changes to track overall rotation
volatile double angle; // Current angle measurement from the sensor
volatile double angle_prev; // Previous angle measurement for calculating velocity
int y_pos; // Current y position for scrolling action
int click_ypos; // Y position when a click action is detected


//HID descriptor to define USB device type.
uint8_t hid_report_descriptor[] PROGMEM = {

  // begin Android Touchpad HID descriptor
  0x05, 0x0d, /* USAGE_PAGE (Digitizer) */
  0x09, 0x04, /* USAGE (Touch Screen) */
  0xa1, 0x01, /* COLLECTION (Application) */
  0x85, ANDROID_REPORT_ID, /*    REPORT_ID (1) */

  /* declare a finger collection */
  0x09, 0x20, /*   Usage (Stylus) */
  0xA1, 0x00, /*   Collection (Physical) */

  /* Declare a finger touch (finger up/down) */
  0x09, 0x42, /*     Usage (Tip Switch) */
  0x09, 0x32, /*     USAGE (In Range) */
  0x15, 0x00, /*     LOGICAL_MINIMUM (0) */
  0x25, 0x01, /*     LOGICAL_MAXIMUM (1) */
  0x75, 0x01, /*     REPORT_SIZE (1) */
  0x95, 0x02, /*     REPORT_COUNT (2) */
  0x81, 0x02, /*     INPUT (Data,Var,Abs) */
  /* Declare the remaining 6 bits of the first data byte as constant -> the driver will ignore them */
  0x75, 0x01, /*     REPORT_SIZE (1) */
  0x95, 0x06, /*     REPORT_COUNT (6) */
  0x81, 0x01, /*     INPUT (Cnst,Ary,Abs) */
  /* End of finger collection */
  0xc0, /* END_COLLECTION */

  /* Pressure */
  0x09, 0x30,       /*     USAGE (Pressure) */
  0x26, 0xFF, 0x00, /*     LOGICAL_MAXIMUM (255) */
  0x75, 0x08,       /*     REPORT_SIZE (8) */
  0x95, 0x01,       /*     REPORT_COUNT (1) */
  0x81, 0x02,       /*     INPUT (Data,Var,Abs) */

  /* Size */
  0x09, 0x56,       /*     USAGE (Size) */
  0x15, 0x00,       /*     LOGICAL_MINIMUM (0) */
  0x26, 0xFF, 0x00, /*     LOGICAL_MAXIMUM (255) */
  0x75, 0x08,       /*     REPORT_SIZE (8) */
  0x95, 0x01,       /*     REPORT_COUNT (1) */
  0x81, 0x02,       /*     INPUT (Data,Var,Abs) */

  /* Define absolute X and Y coordinates of 16 bit each (percent values multiplied with 100) */
  /* http://www.usb.org/developers/hidpage/Hut1_12v2.pdf */
  /* Chapter 16.2 says: "In the Stylus collection a Pointer physical collection will contain the axes reported by the stylus." */
  0x05, 0x01,       /*     Usage Page (Generic Desktop) */
  0x09, 0x01,       /*     Usage (Pointer) */
  0xA1, 0x00,       /*     Collection (Physical) */
  0x09, 0x30,       /*        Usage (X) */
  0x09, 0x31,       /*        Usage (Y) */
  0x16, 0x00, 0x00, /*        Logical Minimum (0) */
  0x26, 0x10, 0x27, /*        Logical Maximum (10000) */
  0x36, 0x00, 0x00, /*        Physical Minimum (0) */
  0x46, 0x10, 0x27, /*        Physical Maximum (10000) */
  0x66, 0x00, 0x00, /*        UNIT (None) */
  0x75, 0x10,       /*        Report Size (16), */
  0x95, 0x02,       /*        Report Count (2), */
  0x81, 0x02,       /*        Input (Data,Var,Abs) */

  /* End of pointer collection */
  0xc0, /* END_COLLECTION */

  /* End of touchscreen application collection */
  0xc0, /* END_COLLECTION */



  /* Mouse Descriptor starts here */  //This is for the experimental back button click
  0x05, 0x01, /* USAGE_PAGE (Generic Desktop) */
  0x09, 0x02, /* USAGE (Mouse) */
  0xa1, 0x01, /* COLLECTION (Application) */
  0x85, 0x02, /*   REPORT_ID (2) */
  0x09, 0x01, /*   USAGE (Pointer) */
  0xa1, 0x00, /*   COLLECTION (Physical) */
  0x05, 0x09, /*     USAGE_PAGE (Button) */
  0x19, 0x01, /*     USAGE_MINIMUM (Button 1) */
  0x29, 0x04, /*     USAGE_MAXIMUM (Button 3) */
  0x15, 0x00, /*     LOGICAL_MINIMUM (0) */
  0x25, 0x01, /*     LOGICAL_MAXIMUM (1) */
  0x95, 0x04, /*     REPORT_COUNT (3) */
  0x75, 0x01, /*     REPORT_SIZE (1) */
  0x81, 0x02, /*     INPUT (Data,Var,Abs) */
  0x95, 0x01, /*     REPORT_COUNT (1) */
  0x75, 0x04, /*     REPORT_SIZE (5) */
  0x81, 0x03, /*     INPUT (Cnst,Var,Abs) */
  0x05, 0x01, /*     USAGE_PAGE (Generic Desktop) */
  0x09, 0x30, /*     USAGE (X) */
  0x09, 0x31, /*     USAGE (Y) */
  0x15, 0x81, /*     LOGICAL_MINIMUM (-127) */
  0x25, 0x7f, /*     LOGICAL_MAXIMUM (127) */
  0x75, 0x08, /*     REPORT_SIZE (8) */
  0x95, 0x02, /*     REPORT_COUNT (2) */
  0x81, 0x06, /*     INPUT (Data,Var,Rel) */
  0x09, 0x38, /*     USAGE (Wheel) */
  0x15, 0x81, /*     LOGICAL_MINIMUM (-127) */
  0x25, 0x7f, /*     LOGICAL_MAXIMUM (127) */
  0x75, 0x08, /*     REPORT_SIZE (8) */
  0x95, 0x01, /*     REPORT_COUNT (1) */
  0x81, 0x06, /*     INPUT (Data,Var,Rel) */
  0xc0, /* END_COLLECTION */   /* End of mouse physical collection */
  0xc0, /* END_COLLECTION */   /* End of mouse application collection */



  //////// Begin PC dial driver
  0x05, 0x01,       //# Usage Page (Generic Desktop)
  0x09, 0x0e,       //# Usage (System Multi-Axis Controller)
  0xa1, 0x01,       //# Collection (Application)
  0x85, PC_REPORT_ID,  //#   Report Id (Radial Controller)
  0x05, 0x0d,       //#   Usage Page (Digitizers)
  0x09, 0x21,       //#   Usage (Puck)
  0xa1, 0x00,       //#   Collection (Physical)

  //# Button
  0x05, 0x09,       //#     Usage Page (Buttons)
  0x09, 0x01,       //#     Usage (Button 1)
  0x15, 0x00,       //#     Logical Minimum (0)
  0x25, 0x01,       //#     Logical Maximum (1)
  0x75, 0x01,       //#     Report Size (1)
  0x95, 0x01,       //#     Report Count (1)
  0x81, 0x02,       //#     Input (Data,Var,Abs)

  //# Padding
  0x75, 0x07,       //#     Report Size (7)
  0x95, 0x01,       //#     Report Count (1)
  0x81, 0x01,       //#     Input (Data,Var,Abs)

  //# Rotation
  0x05, 0x01,       //#     Usage Page (Generic Desktop)
  0x09, 0x37,       //#     Usage (Dial)
  0x55, 0x0f,       //#     Unit Exponent (-1)
  0x65, 0x14,       //#     Unit (Degrees, English Rotation)
  0x36, 0xf0, 0xf1, //#     Physical Minimum (-3600)
  0x46, 0x10, 0x0e, //#     Physical Maximum (3600)
  0x16, 0xf0, 0xf1, //#     Logical Minimum (-3600)
  0x26, 0x10, 0x0e, //#     Logical Maximum (3600)
  0x75, 0x10,       //#     Report Size (16)
  0x95, 0x01,       //#     Report Count (1)
  0x81, 0x06,       //#     Input (Data,Var,Rel)

  0xc0,             //#   End Collection
  0xc0,             //# End Collection
  // End PC dial driver

};

BLEDis bledis; // Initialize BLE device information service
BLEHidGeneric blehid = BLEHidGeneric(3); // Initialize BLE HID with 3 as the parameter, possibly indicating the number of HID reports

MagAlpha magAlpha; // Initialize an instance of MagAlpha, likely a magnetic position sensor
void acquireAndProcessAngle(); // Declaration of a function to acquire and process angle data


void setup() {
  Serial.begin(UART_BAUDRATE); // Initialize serial communication at a baud rate defined by UART_BAUDRATE
  delay(200); // Short delay to ensure serial communication is established
  Serial.print(deviceName);
  Serial.println(" booting");

  //configure bluetooth
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX); // Configure the maximum bandwidth for the Bluetooth peripheral
  Bluefruit.begin(1, 1); // Initialize Bluefruit library with 1 peripheral and 1 central
  Bluefruit.setName(deviceName); // Set the Bluetooth device name
  bledis.setManufacturer("Andrew Industries"); // Set the manufacturer name in the BLE device information service
  bledis.setModel("Doomscroller V0.1"); // Set the model name in the BLE device information service
  bledis.begin(); // Start the BLE device information service
  uint16_t input_report_lengths[] = {7, 4, 3}; // Define report lengths for each HID device (touchpad, mouse, dial)
  blehid.setReportMap(hid_report_descriptor, sizeof(hid_report_descriptor));  // Set the HID report map using the descriptor and its size
  blehid.setReportLen(input_report_lengths, NULL, NULL); // Set the report lengths for input, output, and feature reports
  blehid.begin(); // Initialize the BLE HID service
  Bluefruit.Periph.setConnInterval(9, 16);  // Set the connection interval range (8 can be unstable, so it's the minimum). Devices try to negociate to a lower rate (increased latency)
  startAdvertising(); // Start BLE advertising to make the device discoverable


  pinMode(1, OUTPUT);  //enable pin MPS encoder
  digitalWrite(1, HIGH);  // enable pin MPS enocder
  magAlpha.begin(SPI_SCLK_FREQUENCY, MA_SPI_MODE_3, SPI_CS_PIN); //mps encoder spi bus
  magAlpha.writeRegister(2, 0);  //MPS encoder BCT side shaft config. imporant.
  magAlpha.writeRegister(3, 2);  //MPS encoder enable y triming
  magAlpha.writeRegister(14, 0xB0);  //set filter to value 11 (78hz)
  magAlpha.writeRegister(9, 128); //binary for 1000000 (reverse direction)
  Serial.print("MPS encoder BCT setup");

  //  // enables background ISR timer for idle detection. Maybe we can make this not use ISR?
  //  if (ITimer1.attachInterruptInterval(TIMER_IDLE_INTERVAL_MS * 1000, idlecheck)) {
  //    Serial.println(F("Idle Timer started successfully"));
  //  }


  ////Bluetooth connection loop. Tries to connect for 10 seconds.
  while (!Bluefruit.connected()) { // Keep trying to connect until successful
    digitalWrite(LED_BUILTIN, LOW);        // Turn the LED on to indicate connection attempt (LOW because the LED is active low on most boards)
    delay(100);                            // Wait for 100 milliseconds before toggling LED
    digitalWrite(LED_BUILTIN, HIGH);       // Turn the LED off to indicate waiting state
    delay(100);                            // Wait for another 100 milliseconds before next attempt
    if (millis() - time_ms > 10000) {  // Check if 10 seconds have passed since the start of the connection attempt
      // If unable to connect within 10 seconds, take action
      Serial.println("Failed to connect within 10 seconds. Suspending."); // Notify via Serial that the device is suspending attempts
      suspendLoop(); // Call a function to suspend operation or go to sleep mode
      break;  // Exit the while loop to stop trying to connect
    }
  }

  time_ms = millis(); // Reset the timer to current time for next operations
  Serial.print(deviceName);
  Serial.println(" booted");

}



void loop() {
  /////// idle stuff
  unsigned long currentTime = millis();
  if (currentTime - lastIdleCheckTime >= idleCheckInterval)  //Checks for idle state every 1000ms
  {
    idlecheck();  // Wakes up encoder and looks for change in angle.
    lastIdleCheckTime = currentTime;  // Update the last check time
  }

  if (isIdle)  //check to see if xx degrees of rotation has occured in the last XX seconds. If not, skip for battery savings
  {
    Serial.print(deviceName);
    Serial.println(" is idle");
    delay(500);
    return; //skip the rest of the program
  }



  //Verbose debug printing at 200ms. Use with betterserialplotter
  static unsigned long lastPrintTime = 0;
  if (verbose_angle && millis() - lastPrintTime >= 200) {
    lastPrintTime = millis();

    Serial.print(angleMovingAverage);
    Serial.print("\t");

    Serial.print(velocityMovingAverage);
    Serial.print("\t");

    Serial.print(accMovingAverage);
    Serial.print("\t");

    Serial.print(scrollAverage);
    Serial.print("\t");

    Serial.print(isClicked);
    Serial.print("\t");

    Serial.print(idleEvents);
    Serial.print("\t");

    Serial.println(y_pos);
  }



  acquireAndProcessAngle();   //// acquire encoder signal and process moving averages
  scroll_calcs();   // calculate the distance in pixels to scroll based on velocity


  //////////////  logic for scroll events ///////////////////

  if (velocityMovingAverage < -scroll_threshold)    // Negtive velocity - scroll down
  {
    //Serial.println("Velocity is negative. Scroll down ");
    if (isClicked == false)  //Unclicked mouse state . Move extra to escape 40px controls zone.
    {
      scroll_android(x_start, y_pos, true);  //touchpad event. calls scroll event to be transmitted through HID report
      click_ypos = y_pos;
      delay(scroll_time);
      y_pos -= 41;
      scroll_android(x_start, y_pos, true);  //touchpad event. calls scroll event to be transmitted through HID report
      isClicked = true;
      delay(scroll_time);
      isIdle = false;

    }

    if (isClicked == true)  //Clicked mouse state. 99% of events.
    {
      y_pos += scrollAverage;  // y_pos is the vertical location of the cursor
      scroll_android(x_start, y_pos, true); //calls scroll event to be transmitted through HID report
      scroll_PC(-scrollAverage); //calls scroll event to be transmitted through HID report
      isClicked = true;
      delay(scroll_time);
      isIdle = false;
    }


    if (y_pos < 2500 || y_pos > 7500)  // Reset y_pos to the middle if it goes out of bounds. Note that upper and lower 25% shouldn't be used due to menus
    {
      if ( abs(click_ypos - y_pos) > 50 && isClicked == true) //don't allow a lift until min move threshold, to prevent accidental clicks
      {
        y_pos = 5000;
        scroll_android(x_start, y_pos, false);
        Serial.println("Y position reset to middle");
      }
    }
    idleEvents = 0;
  }    // end of negtive velocity - scroll down case




  if (velocityMovingAverage > scroll_threshold) //// Positive velocity - scroll up //////////////////
  {
    //Serial.println("Velocity is negative");
    if (isClicked == false)  //Unclicked mouse state . Move extra to escape 40px controls zone.
    {
      scroll_android(x_start, y_pos, true);  //touchpad event. calls scroll event to be transmitted through HID report
      click_ypos = y_pos;
      delay(scroll_time);
      y_pos -= -41;
      scroll_android(x_start, y_pos, true); //touchpad event. calls scroll event to be transmitted through HID report
      isClicked = true;
      delay(scroll_time);
      isIdle = false;
      click_ypos = y_pos;
    }

    if (isClicked == true)  // Negtive velocity - scroll down)
    {
      y_pos -= scrollAverage;
      scroll_android(x_start, y_pos, true); //click mouse
      scroll_PC(scrollAverage);
      delay(scroll_time);
      isClicked = true;
    }

    if (y_pos < 2500 || y_pos > 7500) // Reset y_pos to the middle if it goes out of bounds. Note that upper and lower 25% shouldn't be used due to menus
    {
      if ( abs(click_ypos - y_pos) > 50 && isClicked == true) //don't allow a lift until min move threshold, to prevent accidental clicks
      {
        y_pos = 5000;
        scroll_android(x_start, y_pos, false);
        Serial.println("Y position reset to middle");
      }
    }
    idleEvents = 0;
  }


  ////////////////////   zero velocty. Stop scroll  ///////////////////
  //This one has been very problematic. The big issue is lifting the finger touch state before enough scrolling has happened. This results in accidental button clicks.
  if (velocityMovingAverage < scroll_threshold && velocityMovingAverage > -scroll_threshold)
  {
    if (idleEvents >= 80 && abs(accMovingAverage) < 0.2) // Don't let go of mouse unless stationary for time enough time.
    {
      //// clickgaurd
      if ( abs(click_ypos - y_pos) < 50 && isClicked == true)  //If we haven't moved more than 45 pixels, force a force to prevent accidental clicks. Very important.
      {
        Serial.println("Click gaurd triggered ");
        y_pos += 10;
        scroll_android(x_start, y_pos, true);
        delay(scroll_time);
        y_pos += 10;
        scroll_android(x_start, y_pos, true);
        delay(scroll_time);
        scroll_android(x_start, y_pos, false);
        isClicked = false;
        //y_pos = y_start;
        delay(scroll_time);
        idleEvents = 0;
        return;
      }
      scroll_android(x_start, y_pos, false); //release mouse click
      isClicked = false;
      click_ypos = y_pos;
      //y_pos = y_start;
      idleEvents = 0;
      delay(scroll_time);
    }
    delay(1);
    idleEvents = idleEvents + 1;
  }



  if (velocityMovingAverage < -40 && accMovingAverage < -4 && isClicked == true) // experimental back click on flick backwards. Not currently used.
  {
    // mouse_back()
  }
  ////   end of scrolling logic



  delay(1); //important to have a min delay
}
