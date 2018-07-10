# n360 test rom

This custom Nintendo 64 ROM was written to assist in testing of my n360 PCB design. This can also be used to test a normal N64 controller.
To run this ROM on the N64 console, you must have a flashcart (everdrive64 for example). It can also run in low level emulators such as cen64.


# Build
libdragon is required. This can be found at https://github.com/DragonMinded/libdragon
After libragon and its dependancies have been installed the source file can be compiled with:
```sh
sudo npm run tc -- make -c ctest_360
```
This will generate a .z64 file that can be loaded onto your flash cart.

![alt text](https://i.imgur.com/4NDem8B.jpg)
