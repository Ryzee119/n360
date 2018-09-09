/*
  This sketch is used for the n360 PCB (See http://n360-usb.com/). It is based on the USB Host Shield Library for the MAX3421 IC.
  See https://github.com/felis/USB_Host_Shield_2.0
  It supports up to four Xbox 360 controllers using the Xbox 360 Wireless Receiver.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.

  **PCB LAYOUT**
  The microcontroller used to control the USB Host IC (Max3421) is the Atmega328PB. The layout is effectively a clone of a Arduino Pro
  Mini 3.3V (https://store.arduino.cc/arduino-pro-mini) with some minor differences and the USB Host Mini
  https://www.circuitsathome.com/usb-host-shield-hardware-manual/) so it is directly compatible with the Arduino IDE and the
  USB_Host_Shield_2.0 arduino library. Make sure you select the Arduino Pro Mini 3.3V 8Mhz in the dropdown lists in the arduino IDE.

  With some C code experience, the information available at https://github.com/felis/USB_Host_Shield_2.0 and the requirements
  below and my code as an example, it should be possible to add support for any number of USB Game Controllers or modify my
  code to suit your needs.

  The secondary microcontroller on the PCB (The STM32 series chip, labelled 'U3') handles all the low level controller protocol and responds
  in real-time to all the various N64 console commands to simulate up to 4 genuine N64 controllers with peripherals and has a direct bus
  connection to the SRAM which simulates a Controller Pak so you don't need to worry about that.
  This is referred to as the 'N64 Protocol Microcontroller' below for clarity.



  **SERIAL DATA OUTPUT TO N64 PROTOCOL MICROCONTOLLER**
  All data is sent to the N64 Protocol Microocontroller over a standard UART interface.
  The output serial tx pin can only output the specific datastring required for the N64 Protocol Microocontroller. Any other data
  sent on the serial port will prevent the communication from working. Ensure that any other data (like debug data) being sent is
  removed or commented out before final testing. The serial port should use 500kbaud and the data buffer must be exactly 21 bytes long.
  Provided you send this exact datastream to the N64 Protocol Microcontroller, the finer details of the N64 controller protocol
  will be handled by that.

  The output data buffer must be exactly 21 bytes long regardless of how many controllers are connected and has the following format:
    byte0
  [0xA*(note1), ...

  byte1 (each button has 1 bit in this order note4)        byte2 (each button has 1 bit in this order note4)
  P1_A|P1_B|P1_Z|P1_START|P1_DU|P1_DD|P1_DL|P1_DR,     P1_RESET(note2)|NOTUSED|P1_LB|P1_RB|P1_CU|P1_CD|P1_CL|P1_CR,  ...

  byte3 (note3)               byte4 (note3)                     byte5 (note 5)
  X-axis (signed char),    Y-axis (signed char),    lowest significatn bit is the Xbox Back button bit, ...

  byte 06,07,08,09,10 is as per byte 1,2,3,4,5 but for player 2
  byte 11,12,13,14,15 is as per byte 1,2,3,4,5 but for player 3
  byte 16,17,18,19,20 is as per byte 1,2,3,4,5 but for player 4

  Note1: The first byte always starts with 0xA*. The lower 4 bits has a bit set which indicates if that particular controller is connected.
         Bit 0 is set if player 1 is connected, the bit is cleared if the controller is disconnected or disconnects
         Bit 1 is set if player 2 is connected, the bit is cleared if the controller is disconnected or disconnects
         Bit 2 is set if player 3 is connected, the bit is cleared if the controller is disconnected or disconnects
         Bit 3 is set if player 4 is connected, the bit is cleared if the controller is disconnected or disconnects

  Note2: The reset bit is set on a genuine controller when you press LB+RB+START. Otherwise it is just zero.
     The N64 console is probably quite happy with this always being zero and the analog stick reset is not really required for
     a modern analog stick, but included 'just in case'.

  Note3: On a genuine controller, X-axis fully left ~= -81, X-axis fully right ~= +81, Y-axis fully up ~= +81, Y-axis fully down ~=-81.

  Note4: A bit corresponding to a button press is 1 when the button is pressed and 0 when the button is released.

  Note5: This byte is 0x01 when the BACK is pressed on the Xbox 360 controller, 0x00 if it is not pressed. The N64 Protocol
  Microcontroller looks for several button combos to select which controller peripheral is installed.
     These rely on the BACK button. The combos are:
     BACK+D-LEFT = No peripheral for that controller
     BACK+D-RIGHT = Install a Rumblepak in that controller
     BACK+D-UP = Install a Mempack peripheral in that controller (Bank #1, for player 1 only)
     BACK+D-DOWN = Install a Mempack peripheral in that controller (Bank #2, for player 1 only)
     The N64 Protocol Microcontroller will send a request to rumble for ~150ms to indicate a successful combo press.
     See **RUMBLE INPUTS** below.
     All other inputs will be ignored by the N64 Protocol Microcontroller whilst BACK is being pressed to prevent the button
     combos creating inputs into the game you are playing. So make sure the BACK button bit is cleared when not required.
     All this is hardcoded into the N64 Protocol Microcontroller and cannot be changed. You can just need to set a button
     to be your 'BACK' in this program.


  **RUMBLE INPUTS**
  The N64 Protocol Microcontroller has an output gpio pin for each controller to indicate if the n64 console has requested a
  rumble for that specific controller.
  This pin is also set on a successful button combo (see Note 5 above).
  The pin is high when the controller should be rumbling, and is low when the controller should not be rumbling.
  The corresponding Arduino input pins for each controller are as follows:
  #define P1_RUMBLE_PIN 3
  #define P2_RUMBLE_PIN 4
  #define P3_RUMBLE_PIN 16
  #define P4_RUMBLE_PIN 17
  These defines can be included in your program if you wish to add rumble support. The Arduino should then communicate with
  the USB Host IC to trigger rumbling for the game controller. i.e if P1_RUMBLE_PIN is high then Xbox.setRumbleOn(255, 255, 0);



  **N64 CONSOLE RESET OUTPUT**
  N64_RESET_PIN should always be set to an input to allow the console to boot.
  To initiate a reset, set the pin to an output then pull this pin LOW for a few hundred millseconds then set to an input again.
  It is defined in the below program as:
  #define N64_RESET_PIN 2



  **USB HOST IC RESET**
  USB_RESET_PIN must be set to HIGH for the USB Host IC to function.
  The USB Host IC (MAX3421) can be reset by pulling USB_RESET_PIN low for a brief period of time which should be done at the
  beginning of the program. It is defined in the below program as:
  #define USB_RESET_PIN 8



  **SOME OTHER FEATURES**
  I have added the ability the change the precision of the analog stick. If you Press 'Y' on the Xbox360 controller
  the maximum span of the analog stick will toggle from the range defined in RANGE_DIV_STD to approximately +/-53.
  This allows finer control if required and can help with aiming etc. A 'fine' rumble indicates the finer precision.
  A 'rough' rumble indicates the default precision. These can be adjusted by modifying RANGE_DIV_STD and RANGE_DIV_ALT defines.

  The analog stick deadzone can be adjusted by modifying DEADZONE_LOW and DEADZONE_HIGH defines.

  There is 4 solder pads on the PCB labelled USER. These are generic pads that you can use for whatever purposes you want.
  The default state of these is inputs and don't do anything. The silkscreen marking near the pad indicates the corresponding
  Arduino pin. These are defined as USER_GPIO1,2,3 and 4 in the code below.

*/


#include <XBOXRECV.h>
#include <SPI.h>

//N64 Buttons
#define N64_A   1UL<<15
#define N64_B   1UL<<14
#define N64_Z   1UL<<13
#define N64_ST  1UL<<12
#define N64_DU  1UL<<11
#define N64_DD  1UL<<10
#define N64_DL  1UL<<9
#define N64_DR  1UL<<8
#define N64_RES 1UL<<7
#define NOTUSED 1UL<<6
#define N64_LB  1UL<<5
#define N64_RB  1UL<<4
#define N64_CU  1UL<<3
#define N64_CD  1UL<<2
#define N64_CL  1UL<<1
#define N64_CR  1UL<<0

///Buttons sent to the N64 Microcontroller that aren;t n64 controller buttons
#define XBX_BCK  1UL<<0   //Required for button combos.


//RANGE_DIV_STD, will make +/- 84 points on axis 32768/390~=84.
//+/- 32768 is the range from the Xbox 360 Controller. Adjust this is you want to change to standard range.
#define RANGE_DIV_STD 390

#define SEND_PERIOD 0       //ms between controller data transfer, want it fairly regulary to minimise input lag. 0=fast as possible!
#define DEADZONE_LOW 0.25   //Deadzone for centre of analog stick, adjust to your controller.
#define DEADZONE_HIGH 0.05  //Deadzone for outside edge of analog stick, adjust to your controller.

//Arduino Pin mappings

//To reset the N64 console, set the pin to an output and pull low briefly to force the n64 console to reset.
//(The 'R' pad on the PCB must be connected to the n64 motherboard).
#define N64_RESET_PIN 2

//This pin should be an output and should always be high. Pull low briefly to reset the USB Host IC (MAX3421).
//Should be a done at boot up to ensure everything has a known state.
#define USB_RESET_PIN 8

//Rumble pins should be an input and is high if the N64 Protocol Microcontroller has requested a rumble (i.e from a game).
//Low is no rumble required.
#define P1_RUMBLE_PIN 3
#define P2_RUMBLE_PIN 4
#define P3_RUMBLE_PIN 16
#define P4_RUMBLE_PIN 17



//USER GPIO - Can be used for other purposes
//there is 4 solder pads on the PCB for this purpose. Silk screen markings correspond to the Arduino GPIO numbers.
#define USER_GPIO1 5
#define USER_GPIO2 6
#define USER_GPIO3 7
#define USER_GPIO4 19

//USB objects
USB Usb;
XBOXRECV Xbox(&Usb);

//Controller variables. All arrays of 4 to handle 4 controllers.
uint16_t btn_state[4]   = {0, 0, 0, 0}; //16 bit variables that hold all the button states from the Xbox 360 Controllers
uint8_t  rumble_pin[4]  = {P1_RUMBLE_PIN, P2_RUMBLE_PIN, P3_RUMBLE_PIN, P4_RUMBLE_PIN}; //input pins for each controller rumble request pin.

//Other variables
int8_t   tx_buf[40]; //The main output buffer to the N64 Protocol Microcontroller.
unsigned long transmit_timer = 0; //Stores a timestamp which is used to time the interval between output data transmission
unsigned long reset_timer = 0UL; //Stores a timestamp which is used to time how long the reset button is held after the RESET button combo.


//This function applies a scaled radial deadzone both at the central position and the outer edge.
void apply_deadzone(float* pOutX, float* pOutY, float x, float y, float deadZoneLow, float deadZoneHigh) {
  float magnitude = sqrtf(x * x + y * y);
  if (magnitude > deadZoneLow) {
    //Scale such that output magnitude is in the range [0.0f, 1.0f]
    float legalRange = 1.0f - deadZoneHigh - deadZoneLow;
    float normalizedMag = (magnitude - deadZoneLow) / legalRange;
    if (normalizedMag > 1.0f) {
      normalizedMag = 1.0f;
    }
    float scale = normalizedMag / magnitude;
    *pOutX = x * scale;
    *pOutY = y * scale;
  }
  else {
    //Stick is in the inner dead zone
    *pOutX = 0.0f;
    *pOutY = 0.0f;
  }
}

void setup() {
  Serial.begin(500000); //Needs to be 500kbaud

  //Setup GPIO
  pinMode(N64_RESET_PIN, INPUT); //N64 RESET pin is tri-stated (input) when a reset is NOT required. i.e should normally be input.
  pinMode(USB_RESET_PIN, OUTPUT);
  pinMode(rumble_pin[0], INPUT);
  pinMode(rumble_pin[1], INPUT);
  pinMode(rumble_pin[2], INPUT);
  pinMode(rumble_pin[3], INPUT);

  //User GPIO is set to input, but can be changed to ouput if you want to use them
  pinMode(USER_GPIO1, INPUT);
  pinMode(USER_GPIO2, INPUT);
  pinMode(USER_GPIO3, INPUT);
  pinMode(USER_GPIO4, INPUT);

  //Reset the USB Host Controller to ensure it's at a known state for startup. Generally improves reliability.
  digitalWrite(USB_RESET_PIN, LOW);
  delay(10); //wait 10ms
  digitalWrite(USB_RESET_PIN, HIGH);

  //Initialise the USB Host Controller using the USB Host Shield Library
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start      ")); //This serial print command can remain is it will only print if something has gone wrong.
    while (1); //halt - something is wrong
  }
  //Serial.print(F("\r\nXbox Wireless Receiver Library Started")); need to comment out because it breaks datastream to n64 microcontroller.
  transmit_timer = millis();
}

void loop() {
  uint8_t num_controllers = 0;
  Usb.Task();
  if (Xbox.XboxReceiverConnected) {
    for (uint8_t i = 0; i < 4; i++) {
      if (Xbox.Xbox360Connected[i]) {
        //The first byte of the datastream has a bit to indicate which controllers are connected.
        //The bit is set here for the corresponding controller.
        num_controllers |= 1 << i;

        //Xbox controller button mapping - this can be adjusted to your preference
        if (Xbox.getButtonPress(A, i))     btn_state[i] |= N64_A;
        if (Xbox.getButtonPress(B, i))     btn_state[i] |= N64_B;
        if (Xbox.getButtonPress(X, i))     btn_state[i] |= N64_B;
        if (Xbox.getButtonPress(L2, i))    btn_state[i] |= N64_Z;
        if (Xbox.getButtonPress(R2, i))    btn_state[i] |= N64_Z;
        if (Xbox.getButtonPress(START, i)) btn_state[i] |= N64_ST;
        if (Xbox.getButtonPress(UP, i))    btn_state[i] |= N64_DU;
        if (Xbox.getButtonPress(DOWN, i))  btn_state[i] |= N64_DD;
        if (Xbox.getButtonPress(LEFT, i))  btn_state[i] |= N64_DL;
        if (Xbox.getButtonPress(RIGHT, i)) btn_state[i] |= N64_DR;
        if (Xbox.getButtonPress(L1, i))    btn_state[i] |= N64_LB;
        if (Xbox.getButtonPress(R1, i))    btn_state[i] |= N64_RB;
        if (Xbox.getButtonPress(R3, i))    btn_state[i] |= N64_CU | N64_CD | N64_CL | N64_CR; //Activate all C-buttons as the same time. Is there ever a need to press all 4?

        //Digitise right analog stick for C buttons (17000 is approximately 50% movement on the analog stick, adjust to set which point the c button activates.
        if (Xbox.getAnalogHat(RightHatY, i) < -17000) {
          btn_state[i] |= N64_CD;
        } else if (Xbox.getAnalogHat(RightHatY, i) > 17000) {
          btn_state[i] |= N64_CU;
        }
        if (Xbox.getAnalogHat(RightHatX, i) < -17000) {
          btn_state[i] |= N64_CL;
        } else if (Xbox.getAnalogHat(RightHatX, i) > 17000) {
          btn_state[i] |= N64_CR;
        }

        //This is the button that must be held for button combos to work.
        //Default is XBX_BCK which is the Xbox 360 Controller 'Back' button.
        uint8_t extra_btns = 0x00;
        if (Xbox.getButtonPress(BACK, i))  extra_btns |= XBX_BCK;

        //Outputs a signal to the N64 reset pin to reset the console on LB+RB+A+B+DD+DR combo
        if (btn_state[i]&N64_LB && btn_state[i]&N64_RB && btn_state[i]&N64_A && btn_state[i]&N64_B && btn_state[i]&N64_DD && btn_state[i]&N64_DR) {
          //Set reset pin low, will simulate the reset button being pushed on the N64 console.
          pinMode(N64_RESET_PIN, OUTPUT);
          digitalWrite(N64_RESET_PIN, LOW);
          reset_timer = millis();
        }

        //Commands that are sent to the controller are sorted here
        //If the rumble request pin for the corresponding controller is high, we should be rumbling.
        //If the pin is low we should not be rumbling.
        if (digitalRead(rumble_pin[i]) == HIGH) {
          Xbox.setRumbleOn(255, 255, i);
        } else if (digitalRead(rumble_pin[i]) == LOW) {
          Xbox.setRumbleOn(0, 0, i);
        } else if (Xbox.getButtonPress(BACK, i) && Xbox.getButtonPress(START, i)) {
          //Shutdown controller by pressing BACK and START button
          Xbox.setLedRaw(0x80, i); //Sending 0x80 to the setLedRaw command just happens to end up the same command as the shutdown magic packet
        }

        //Get X,Y axis from main analog stick and apply inner and outer deadzone correction
        float x, y;
        apply_deadzone(&x, &y, Xbox.getAnalogHat(LeftHatX, i) / 32768.0, Xbox.getAnalogHat(LeftHatY, i) / 32768.0, DEADZONE_LOW, DEADZONE_HIGH);
        x *= 32768.0;
        y *= 32768.0;

        //Build output buffer
        //tx_buf[0] is set just before the serial transmission, not here. As the 'i' variable increments, the position in the tx_buf array
        //offsets accordingly for each controller.
        tx_buf[i * 5 + 1] = btn_state[i] >> 8; //The higher 8 bits are placed first
        tx_buf[i * 5 + 2] = btn_state[i];      //Then the lower 8 bits
        tx_buf[i * 5 + 3] = x / RANGE_DIV_STD;  //Divide the x,y values by the range_div to get a proper range.
        tx_buf[i * 5 + 4] = y / RANGE_DIV_STD;
        tx_buf[i * 5 + 5] = extra_btns;
        btn_state[i] = 0x0000; //Reset the button presses for the next loop


      } else { //If the controller isn't connected to the receiver...
        num_controllers &= ~(1 << i);
      }
    }
  }

  //If the reset pin was set from a controller combo, reset to pin state to an INPUT after ~500ms.
  if (reset_timer != 0 && (millis() - reset_timer) > 500UL) {
    pinMode(N64_RESET_PIN, INPUT);
    digitalWrite(N64_RESET_PIN, HIGH);
    reset_timer = 0UL;
  }

  //Send data to the N64 Protocol Microcontroller
  transmit_timer = millis();
  tx_buf[0] = 0xA0 | num_controllers; //num_controllers should have a corresponding bit set for each controller that is connected.
  Serial.flush();
  Serial.write((uint8_t *)tx_buf, (int)21); //Send the serial data!
  memset(tx_buf, 0x00, sizeof(tx_buf)); //and clear the transmit buffer

}


