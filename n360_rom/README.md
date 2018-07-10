# n360 test rom

This custom Nintendo 64 ROM was written to assist in testing of my n360 PCB design. This can also be used to test a normal N64 controller.
To run this ROM on the N64 console, you must have a flashcart (everdrive64 for example). It can also run in low level emulators such as cen64.


# Build
To build this file, libdragon is required. This can be found at https://github.com/DragonMinded/libdragon

After libdragon and its dependancies have been installed, download the MakeFile and ctest_360.c and place them in a folder called ctest_360 within the libdragon directory. The source code can then be built with

```sh
sudo npm run tc -- make -c ctest_360
```
This will generate a .z64 file that can be loaded onto your flash cart.

![alt text](https://i.imgur.com/4NDem8B.jpg)
