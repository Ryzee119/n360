# n360 USB Driver Source

This is the source code for the USB driver for my n360 PCB (See http://n360-usb.com/). The code is written for the ATmega328PB and communicates with the MAX3421 USB Host Controller IC and is based on the USB Host Shield Library (https://github.com/felis/USB_Host_Shield_2.0)

The data from the Xbox 360 controllers is then sent via serial to the Low Level N64 Protocol Microcontroller at high speed for final output to the Nintendo 64 console.


| Driver | Description | Author |
| --- | --- | --- |
| `n360_usb` | This is the default driver for 4 x Xbox 360 Wireless Controllers with a USB Receiver. This is what is programmed on the n360 PCB from new.| Ryzee119 |
| `n360_usb_barebone` | This is a Wireless Xbox 360 Controller Driver with no bells and whistles. Good starting point for development. | Ryzee119 |
| `n360_usb_xbone` | Use up to 4x **WIRED** Xbox One Controllers. If using more than one controller you must use an externally powered USB Hub. No LED indications, LB+RB+BACK to enable Goldeneye mode.| Ryzee119 |


# Building Code
Download the latest version of the Arduino IDE (https://www.arduino.cc/en/main/software). Ensure you install the Arduino Host Shield Library from the Library Manager:
![alt text](https://i.imgur.com/7ZfBsUC.png)
Select the Board Type "Arduino Pro or Pro Mini" from the Board selection list, then select the Processor type "ATmega328P (3.3V, 8Mhz)"
![alt text](https://i.imgur.com/lJ7mr9g.png)

Download the .ino file from this repository and open it in the Arduino IDE.
One opened you can compile by clicking the 'tick' in the top left corner in the Arduino IDE to compile.
![alt text](https://i.imgur.com/tD5O3KC.png)

# Programming the n360
Connect a serial programming interface like https://www.sparkfun.com/products/9873 to the 6 pin programming interface on the n360 PCB. Ensure that it can support up to 500kbaud speeds. You may need to solder a 6 pin header on the n360 PCB, alternatively if you just place the programmer into the holes on the PCB it will generally make contact with all the required points anyway. The 'DTR' pin is used to reset the Atmega328PB automatically to accept serial programming so is required.

In the Arduino IDE, goto Tools>Port and ensure you have selected the COM Port corresponding to your serial programmer. Mine happens to be COM5.
![alt text](https://i.imgur.com/ppySRhk.png)

The programmer will not supply power to the n360 PCB, so you must power on the Nintendo 64 before programming. After programming you may need to power cycle the Nintendo 64 for the n360 PCB to correctly reset and initialise.

This will not impact any existing saved games on the Controller Pak.

![alt text](http://n360-usb.com/wp-content/uploads/2018/08/arduino_header.png)

![alt text](http://n360-usb.com/wp-content/uploads/2018/09/n3603-768x874.jpg)


# Debugging
You can debug through the "Serial Monitor". Ensure that the correct COM Port has been selected and the baud rate should be changed to 500kbaud. You will probably gets lots of unreadable garbage. This is the binary data being sent to the N64 Protocol Microcontroller.

Comment out `Serial.write((uint8_t *)tx_buf, (int)21);` at the end of the code to stop this. This will ofcourse prevent the N64 Protocol Microcontroller from receiving data, but you can then include `Serial.print` throughout your code to debug the USB Host controller.

Once happy, you will need to comment out all you `Serial.Print' statements and re-add `Serial.write((uint8_t *)tx_buf, (int)21);`


