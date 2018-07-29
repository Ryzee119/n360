# n360 USB drive Source

This is the source code for the USB drive for my n360 PCB (See http://n360-usb.com/). The code communicates with the MAX3421 USB Host Controller IC and is based on the USB Host Shield Library (https://github.com/felis/USB_Host_Shield_2.0)

The data from the Xbox 360 controllers is then sent to the Nintendo 64 via serial to the Low Level N64 Protocol Microcontroller at high speed for final output to the Nintendo 64 console.


# Build
Download the latest version of the Arduino IDE (https://www.arduino.cc/en/main/software). Ensure you install the Arduino Host Shield Library from the Library Manager:
![alt text](https://i.imgur.com/7ZfBsUC.png)
Select the Board Type "Arduino Pro or Pro Mini" from the Board selection list, then select the Processor type "ATmega328P (3.3V, 8Mhz)"
![alt text](https://i.imgur.com/lJ7mr9g.png)

Download the .ino file from this repository and open it in the Arduino IDE.

Once connected as per the instructions at http://n360-usb.com/usb/ press the Upload button. This will compile and upload the code to the USB Host Driver.
