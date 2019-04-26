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



///Buttons sent to the N64 Microcontroller that aren't n64 controller buttons
#define XBX_BCK  1UL<<0   //Required for button combos.


//RANGE_DIV_STD, will make +/- 84 points on axis 32768/390~=84.
//+/- 32768 is the range from the Xbox 360 Controller. Adjust this is you want to change to standard range.
#define RANGE_DIV_STD 390 

#define SEND_PERIOD 0       //ms between controller data transfer, want it fairly regulary to minimise input lag. 0=fast as possible!
#define DEADZONE_LOW 0.25   //Deadzone for centre of analog stick, adjust to your controller.
#define DEADZONE_HIGH 0.05  //Deadzone for outside edge of analog stick, adjust to your controller.


void apply_deadzone(float* pOutX, float* pOutY, float x, float y, float deadZoneLow, float deadZoneHigh);
uint8_t getButtonPress(ButtonEnum b, uint8_t controller);
uint8_t getButtonClick(ButtonEnum b, uint8_t controller);
int16_t getAnalogHat(AnalogHatEnum a, uint8_t controller);
void setRumbleOn(uint8_t lValue, uint8_t rValue, uint8_t controller);
void setLedOn(LEDEnum led, uint8_t controller);
bool controllerConnected(uint8_t controller);
