## Step 1: Set Up the Seeed nRF52840 on Arduino IDE

Install the Arduino IDE: Download from the official Arduino website.
Add the Seeed Board to Arduino IDE:
Go to File > Preferences, and insert this URL into "Additional Boards Manager URLs":
arduino
Copy code
https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
Navigate to Tools > Board > Boards Manager, search for "Seeed nRF52 Boards", and install it.
Choose Seeed XIAO BLE from Tools > Board > Seeed nRF52 Boards.
Driver Installation (if necessary):
Follow the steps on the Seeed Studio wiki page for XIAO BLE.
## Step 2: Install Required Libraries

MPS MagAlpha and MovingAveragePlus Libraries:
Open Sketch > Include Library > Manage Libraries....
Search and install "MPS MagAlpha" and "MovingAveragePlus".
## Step 3: Upload Code

Connect the Doomscroller: If the device isn't recognized as a COM port, perform a magnetic triggered reset by swiping a magnet across the back PCB before uploading your code.
Post-Upload Reset: Ensure to perform a magnetic reset right after the code is flashed.



## Troubleshooting and FAQ

### Device not working with iPhone: ###
* iOS functionality is currently not supported. Contact Andrew for alternatives such as store credit or a return.

### Adjusting Scroll Speed and Responsiveness: ###
* Modify user settings in the main .ino file. Adjust scale and max velocity for performance. Experiment with the buffer lengths in moving averages for desired responsiveness.

### Connection Issues: ###
* Latency or choppy scroll action can often be fixed by a magnetic swipe before pairing to optimize Bluetooth update rates.

### Entering Bootloader Mode: ###
* Perform a double magnetic tap on the "C" letter of the PCB to enter bootloader mode, mimicking a double-click of the onboard reset switch. This will present the device as a removable USB drive for firmware updates.
