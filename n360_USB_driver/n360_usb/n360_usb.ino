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
#include <math.h>
#include <avr/wdt.h>
#include "n360_usb.h"
#include "src/XBOXUSB8BITDO.h"



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
XBOXRECV Xbox360Wireless(&Usb);
XBOXUSB8BITDO Xbox360Wired1(&Usb);
XBOXUSB8BITDO Xbox360Wired2(&Usb);
XBOXUSB8BITDO Xbox360Wired3(&Usb);
XBOXUSB8BITDO Xbox360Wired4(&Usb);
XBOXUSB8BITDO *Xbox360Wired[4] = {&Xbox360Wired1, &Xbox360Wired2, &Xbox360Wired3, &Xbox360Wired4};

//Controller variables. All arrays of 4 to handle 4 controllers.
uint16_t btn_state[4]   = {0, 0, 0, 0}; //16 bit variables that hold all the button states from the Xbox 360 Controllers
uint16_t range_div[4]   = {RANGE_DIV_STD, RANGE_DIV_STD, RANGE_DIV_STD, RANGE_DIV_STD}; //The current range divider for each controller.
uint8_t  rumble_flag[4] = {0, 0, 0, 0}; //A flag is set here if the controller is currently rumbling.
uint8_t  rumble_pin[4]  = {P1_RUMBLE_PIN, P2_RUMBLE_PIN, P3_RUMBLE_PIN, P4_RUMBLE_PIN}; //input pins for each controller rumble request pin.
unsigned long rumble_timer[4] = {0UL, 0UL, 0UL, 0UL}; //Stores a timestamp which is used to time how long the rumble has been on.
unsigned long command_timer[4] = {0UL, 0UL, 0UL, 0UL}; //Stores a timestamp which is used to time how long the rumble has been on.
unsigned long disconnection_timer[4] = {0UL, 0UL, 0UL, 0UL}; //Stores a timestamp which is used to time how long the rumble has been on.

//Other variables
int8_t   tx_buf[40]; //The main output buffer to the N64 Protocol Microcontroller.
unsigned long transmit_timer = 0; //Stores a timestamp which is used to time the interval between output data transmission
unsigned long reset_timer = 0UL; //Stores a timestamp which is used to time how long the reset button is held after the RESET button combo.
uint8_t goldeneye_mode = 0; //Goldeneye mode uses the secondary analog stick and take advantage of Goldeneye's ability to use two n64 controllers.

//Array that alters the angular magnitude dependant on the angle the the controller stick is going. This corrects for the
//octagonal shape of the original Nintendo 64. A 90degree window is covered, each bit in the array covers 2degrees.
//i.e octa_correction[0] is the magnitude (radius from the zero point) between 0-2degrees from the x-axis.
//octa_correction[1] is the magnitude between 2-4degrees from the x-axis. etc..
#define c 32768/RANGE_DIV_STD/83
static const uint8_t octa_correction[46] = {83 * c,  83 * c,  82 * c,  82 * c,  82 * c,  82 * c,  82 * c,
                                            82 * c,  82 * c,  82 * c,  83 * c,  83 * c,  82 * c,  83 * c,  84 * c,  85 * c,  86 * c,  87 * c,  88 * c,
                                            89 * c,  90 * c,  91 * c,  91 * c,  90 * c,  89 * c,  88 * c,  87 * c,  87 * c,  86 * c,  86 * c,  85 * c,
                                            84 * c,  83 * c,  83 * c,  82 * c,  82 * c,  82 * c,  82 * c,  82 * c,  82 * c,  82 * c,  82 * c,
                                            82 * c,  83 * c,  83 * c,  83 * c
                                           };



void(* resetFunc) (void) = 0; //declare reset function @ address 0


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
  delay(500);
  //Reset the USB Host Controller to ensure it's at a known state for startup. Generally improves reliability.
  digitalWrite(USB_RESET_PIN, LOW);
  delay(50); //wait 10ms
  digitalWrite(USB_RESET_PIN, HIGH);
  delay(50); //wait 10ms


  //Initialise the USB Host Controller using the USB Host Shield Library
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start  "));
    delay(500);
    resetFunc();
    while (1); //halt - something is wrong
  }
  //Serial.print(F("\r\nXbox Wireless Receiver Library Started")); need to comment out because it breaks datastream to n64 microcontroller.
  transmit_timer = millis();

}

void loop() {
  uint8_t num_controllers = 0;
  Usb.busprobe();
  //Usb.Task();
  for (uint8_t i = 0; i < 4; i++) {
    Usb.Task();
    if (controllerConnected(i)) {
      //The first byte of the datastream has a bit to indicate which controllers are connected.
      //The bit is set here for the corresponding controller.
      num_controllers |= 1 << i;

      //Xbox controller button mapping - this can be adjusted to your preference
      if (getButtonPress(A, i))     btn_state[i] |= N64_A;
      if (getButtonPress(B, i))     btn_state[i] |= N64_B;
      if (getButtonPress(X, i))     btn_state[i] |= N64_B;
      if (getButtonPress(L2, i))    btn_state[i] |= N64_Z;
      if (getButtonPress(R2, i))    btn_state[i] |= N64_Z;
      if (getButtonPress(START, i)) btn_state[i] |= N64_ST;
      if (getButtonPress(UP, i))    btn_state[i] |= N64_DU;
      if (getButtonPress(DOWN, i))  btn_state[i] |= N64_DD;
      if (getButtonPress(LEFT, i))  btn_state[i] |= N64_DL;
      if (getButtonPress(RIGHT, i)) btn_state[i] |= N64_DR;
      if (getButtonPress(L1, i))    btn_state[i] |= N64_LB;
      if (getButtonPress(R1, i))    btn_state[i] |= N64_RB;
      if (getButtonPress(R3, i))    btn_state[i] |= N64_CU | N64_CD | N64_CL | N64_CR; //Activate all C-buttons as the same time. Is there ever a need to press all 4?

      //Digitise right analog stick for C buttons (17000 is approximately 50% movement on the analog stick, adjust to set which point the c button activates.
      if (getAnalogHat(RightHatY, i) < -17000) {
        btn_state[i] |= N64_CD;
      } else if (getAnalogHat(RightHatY, i) > 17000) {
        btn_state[i] |= N64_CU;
      }
      if (getAnalogHat(RightHatX, i) < -17000) {
        btn_state[i] |= N64_CL;
      } else if (getAnalogHat(RightHatX, i) > 17000) {
        btn_state[i] |= N64_CR;
      }

      //This is the button that must be held for button combos to work.
      //Default is XBX_BCK which is the Xbox 360 Controller 'Back' button.
      uint8_t extra_btns = 0x00;
      if (getButtonPress(BACK, i))  extra_btns |= XBX_BCK;


      //Reset analog stick. This is part of the N64 protocol. LB+RB+START
      //The RESET bit in the n64 button map is set to one, the start button is forced to zero.
      //No action is really required for a modern controller and probably not required at all but implemented for completeness.
      if (btn_state[i]&N64_LB && btn_state[i]&N64_RB &&  btn_state[i]&N64_ST) {
        btn_state[i] |= N64_RES;
        btn_state[i] &= ~(0x0000 | N64_ST); //Clear the start button bit
      }


      //Outputs a signal to the N64 reset pin to reset the console on LB+RB+A+B+DD+DR combo
      if (btn_state[i]&N64_LB && btn_state[i]&N64_RB && btn_state[i]&N64_A && btn_state[i]&N64_B && btn_state[i]&N64_DD && btn_state[i]&N64_DR) {
        //Set reset pin low, will simulate the reset button being pushed on the N64 console.
        pinMode(N64_RESET_PIN, OUTPUT);
        digitalWrite(N64_RESET_PIN, LOW);
        reset_timer = millis();
      }


      //All the below functions that can activate a rumble or send some other command to the controller below are part of an else-if chain.
      //This ensures that only 1 rumble activation request occurs per loop. I found there was issues if multiple rumble requests occured in a loop.

      //This will check if the controller is already rumbling. If so, turn off after 300ms.
      if((millis() - command_timer[i]) > 16){
        if (rumble_timer[i] != 0 && (millis() - rumble_timer[i]) > 300 && rumble_flag[i] != 1) {
          setRumbleOn(0, 0, i);
          rumble_timer[i] = 0UL;
        }
        //Controller rumble requests from the N64 Protocol Microcontroller are sorted here
        //If the rumble request pin for the corresponding controller is high, we should be rumbling.
        //If the pin is low we should not be rumbling. We implement a rumble counter variable to force a minimum rumble time of 100ms
        //During testing I found that the Xbox rumble motors are a bit slower to run up, so very short rumbles won't be felt otherwise.
        else if (digitalRead(rumble_pin[i]) == HIGH && rumble_flag[i] == 0) {
          setRumbleOn(255, 255, i);
          rumble_timer[i] = millis();
          rumble_flag[i] = 1;
  
        } else if (digitalRead(rumble_pin[i]) == LOW && (millis() - rumble_timer[i]) > 100 && rumble_flag[i] == 1) {
          rumble_timer[i] = 0UL;
          setRumbleOn(0, 0, i);
          rumble_flag[i] = 0;
  
        } else if (goldeneye_mode && i == 0) { //goldeneye mode is a specific mode I made for Goldeneye. This is described further down the code.
          //Converts millis to quarter seconds, then on every even LED1 is on, odd LED2 is on.
          //A poor mans way of flashing between the top two LEDs at 2Hz without adding another counter.
          if ((uint16_t)(millis() / 250) % 2) {
            setLedOn(LED1, i);
          } else {
            setLedOn(LED2, i);
          }
  
        } else if (getButtonPress(BACK, i) && getButtonPress(START, i)) {
          //Shutdown controller by pressing BACK and START button
          if(Xbox360Wireless.Xbox360Connected[i] && (millis()-disconnection_timer[i])>1000) {
            Xbox360Wireless.disconnect(i);
            btn_state[i]=0;
            disconnection_timer[i]=millis();
          }
          
        }
        command_timer[i]=millis();
      }


      //Get X,Y axis from main analog stick and apply inner and outer deadzone correction
      float x, y;
      apply_deadzone(&x, &y, getAnalogHat(LeftHatX, i) / 32768.0, getAnalogHat(LeftHatY, i) / 32768.0, DEADZONE_LOW, DEADZONE_HIGH);
      x *= 32768.0;
      y *= 32768.0;


      /** THIS SECTION IS CALLED GOLDEN EYE MODE AND IS A CUSTOM ARRANGEMENT MADE SPECIFICALLY FOR GOLDENEYE/PERFECT DARK
          This uses the secondary analog stick and takes advantage of the games ability to use two n64 controllers at the same time
          for dual analog stick inputs (Move and Aim). The code will trick the console into thinking a second controller has been installed
          and the Xbox 360 Controller movements that would normally correspond to the 2nd controller are directed accordingly.
          For this to work the game must be configured to controller layout "2.1  PLENTY". This is done by pressing START after
          starting a level and navigating through the menu. You can optionally change the controller stick inversion as the default
          is INVERTED for the aiming. This is in the game options too.
          The default configuration is fairly standard to a modern FPS game.
          Left Trigger = AIM with reticle
          Right Trigger = FIRE
          Right Stick = Look Around. The vertical aiming inversion can be enabled/disabled in the in-game options.
          Left Stick = Walk Forward/Backward, Strafe left/right
          B = Reload/Interact
          A = Change Weapon
          Navigating menus in goldeneye mode can be difficult. If you hold the right stick in, the left stick will act as per normal.
          Alternatively, hold the left trigger and the right analog stick will control the cursor.
      */
      //Toggle goldeneye mode by holding LB and RB then pressing the XBOX button.
      if (i==0 && (getButtonClick(XBOX, i) || getButtonClick(BACK, i)) && getButtonPress(L1, i) && getButtonPress(R1, i)) {
        goldeneye_mode ^= 1;
        if (!goldeneye_mode) {
          setLedOn(LED1, 0); //If goldeneye mode disabled, set led back to 1st quadrant.
        }
      }


      if (goldeneye_mode && i == 0) {
        float x1, y1, temp1;
        range_div[0] = RANGE_DIV_STD;
        num_controllers |= 1 << 1; //Force enable player 2 controller as the 2nd analog stick is sent to the 2nd controller port

        btn_state[0] &= ~(N64_Z); //Clear Z button input then reinstate below accordingly
        if (getButtonPress(R2, 0))    btn_state[0] |= N64_Z; //Back right trigger will be a 'Z' press from player 1 controller and is 'FIRE'
        if (getButtonPress(L2, 0))    btn_state[1] |= N64_Z; //Back left trigger will be a 'Z' press from player 2 controller and is 'AIM'
        btn_state[0] &= ~(N64_CD | N64_CU | N64_CL | N64_CR); //C-buttons must be cleared in goldeneye mode as the right stick isn't C-buttons anymore.


        //Get x,y axis for secondary stick and apply deadzone correction
        apply_deadzone(&x1, &y1, getAnalogHat(RightHatX, 0) / 32768.0, getAnalogHat(RightHatY, 0) / 32768.0, DEADZONE_LOW, DEADZONE_HIGH);
        x1 *= 32768.0 * 0.7;
        y1 *= 32768.0 * 0.7;

        //Holding left trigger shows the aiming reticule. The analog sticks are completely swapped here so that the right stick remains aiming.
        if (getButtonPress(L2, 0)) {
          //Swap x-axis between analog sticks
          temp1 = x;
          x = x1;
          x1 = temp1;
          //Swap y-axis between analog sticks
          temp1 = y;
          y = y1;
          y1 = temp1;
        }

        //To help navigate menus, hold the right analog stick in to maintain the default analog stick settings, otherwise the x-axis is swapped
        //during normal gameplay. Makes the control arrangement better.
        if (!getButtonPress(R3, 0) && !getButtonPress(L2, 0)) {
          //Just swap the X-axis between analog sticks.
          temp1 = x;
          x = x1;
          x1 = temp1;
        }

        //These variables are forced into Player 2 controller data locations so they appear to come from a player 2 controller
        tx_buf[1 * 5 + 1] = btn_state[1] >> 8; //The higher 8 bits are placed first
        tx_buf[1 * 5 + 2] = btn_state[1];      //Then the lower 8 bits
        tx_buf[1 * 5 + 3] = x1 / RANGE_DIV_STD;  //Right analog stick axis is put into the spot player 2 axis would normally be
        tx_buf[1 * 5 + 4] = y1 / RANGE_DIV_STD;
      } //end goldeneye mode.


      //Correct for octagonal shape of an original N64 analog stick.
      if (!goldeneye_mode) { //Apply only when the fine precision hasnt been enabled and not in goldeneye mode.
        //Now that deadzones have been applied, Correct for octagonal shape on an original N64 using a lookup table.
        int angle = atan2(y, x) * 180 / 3.14159; //Need to know the angle
        float magnitude = sqrtf(x * x + y * y);

       if(magnitude>30000){
         if(angle>-10 && angle<10) y=0;
         if(angle>80 && angle<100) x=0;
         if(angle>170 && angle<190) y=0;
         if(angle>-100 && angle<-80) x=0;

         /*
         if(angle>30 && angle <60){
            y=64.0/84.0*32768.0; x=58.0/84.0*32768.0;
         }
         
         if(angle>120 && angle <150){
            y=64.0/84.0*32768.0; x=-58.0/84.0*32768.0;
         }

         if(angle>-60 && angle <-30){
            y=-64.0/84.0*32768.0; x=58.0/84.0*32768.0;
         }

         if(angle>-150 && angle <-120){
            y=-64.0/84.0*32768.0; x=-58.0/84.0*32768.0;
         }*/
       }
        
        
        if (angle < 0) angle *= -1; //make sure the results is always between 0 and 90deg
        if (angle > 90) angle = 90 - (angle - 90); //make sure the results is always between 0 and 90deg
        angle /= 2; //Lookup table is in 2degree increments
        range_div[i] = 32768 / octa_correction[angle]; //Apply corrected magnitude
      }



      //Build output buffer
      //tx_buf[0] is set just before the serial transmission, not here. As the 'i' variable increments, the position in the tx_buf array
      //offsets accordingly for each controller.
      tx_buf[i * 5 + 1] = btn_state[i] >> 8; //The higher 8 bits are placed first
      tx_buf[i * 5 + 2] = btn_state[i];      //Then the lower 8 bits
      tx_buf[i * 5 + 3] = x / range_div[i];  //Divide the x,y values by the range_div to get a proper range.
      tx_buf[i * 5 + 4] = y / range_div[i];
      tx_buf[i * 5 + 5] = extra_btns;
      btn_state[i] = 0x0000; //Reset the button presses for the next loop

      if (goldeneye_mode) {
        btn_state[1] = 0x0000; //Reset the button presses for the next loop for player 2 controller if in goldeneye mode.
      }


    } else { //If the controller isn't connected to the receiver...
      //The first byte of the datastream has a bit to indicate which controllers are connected.
      //The bit is cleared here because the controller isn't connected.
      //In goldeneye mode player 2 is disabled when player 1 controller disconnects
      if (goldeneye_mode && i == 0) {
        num_controllers &= ~(1 << 0); //Disable player 1 controller
        num_controllers &= ~(1 << 1); //Disable player 2 controller if player 1 controller isnt connected in goldeneye mode because p2 is forced on by p1.
      } else if (!goldeneye_mode) {
        num_controllers &= ~(1 << i); //If not in goldeneye mode, disable controllers as per normal by disabling the corresponding bit.
      }
    }
  }

  //If the reset pin was set from a controller combo, reset to pin state to an INPUT after ~500ms.
  if (reset_timer != 0 && (millis() - reset_timer) > 500UL) {
    pinMode(N64_RESET_PIN, INPUT);
    digitalWrite(N64_RESET_PIN, HIGH);
    reset_timer = 0UL;
  }
  
  //Send data periodically on serial port.
  if ((millis() - transmit_timer) > SEND_PERIOD) {
    transmit_timer = millis();
    tx_buf[0] = 0xA0 | num_controllers; //num_controllers should have a corresponding bit set for each controller that is connected.
    Serial.flush();
    Serial.write((uint8_t *)tx_buf, (int)21); //Send the serial data!
    memset(tx_buf, 0x00, sizeof(tx_buf)); //and clear the transmit buffer
  }
}



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


//Parse button presses for each type of controller
uint8_t getButtonPress(ButtonEnum b, uint8_t controller) {
  if (Xbox360Wireless.Xbox360Connected[controller])
    return Xbox360Wireless.getButtonPress(b, controller);

  if (Xbox360Wired[controller]->Xbox360Connected)
    return Xbox360Wired[controller]->getButtonPress(b);

  return 0;
}

uint8_t getButtonClick(ButtonEnum b, uint8_t controller) {
  if (Xbox360Wireless.Xbox360Connected[controller])
    return Xbox360Wireless.getButtonClick(b, controller);

  if (Xbox360Wired[controller]->Xbox360Connected)
    return Xbox360Wired[controller]->getButtonClick(b);

  return 0;
}




//Parse analog stick requests for each type of controller.
int16_t getAnalogHat(AnalogHatEnum a, uint8_t controller) {
  if (Xbox360Wireless.Xbox360Connected[controller])
    return Xbox360Wireless.getAnalogHat(a, controller);

  if (Xbox360Wired[controller]->Xbox360Connected)
    return Xbox360Wired[controller]->getAnalogHat(a);

  return 0;
}

//Parse rumble activation requests for each type of controller.
void setRumbleOn(uint8_t lValue, uint8_t rValue, uint8_t controller) {
  if (Xbox360Wireless.Xbox360Connected[controller])
    Xbox360Wireless.setRumbleOn(lValue, rValue, controller);

  if (Xbox360Wired[controller]->Xbox360Connected)
    Xbox360Wired[controller]->setRumbleOn(lValue, rValue); //If you have an externally power USB 2.0 hub you can uncomment this to enable rumble

}

//Parse LED activation requests for each type of controller.
void setLedOn(LEDEnum led, uint8_t controller) {
  if (Xbox360Wireless.Xbox360Connected[controller])
    Xbox360Wireless.setLedOn(led, controller);

  if (Xbox360Wired[controller]->Xbox360Connected)
    Xbox360Wired[controller]->setLedOn(led);

}

bool controllerConnected(uint8_t controller) {
  if (Xbox360Wireless.XboxReceiverConnected && Xbox360Wireless.Xbox360Connected[controller]){
    return true;
    
  } else if (Xbox360Wired[controller]->Xbox360Connected){
    return true;
  
  } else {
    return false;
  }

  
}
