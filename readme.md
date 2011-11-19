# RGB Tumbler readme

This is the firmware source code for the RGB Tumbler project. Read more about the project (including schematics and board layouts) here: [http://atomslagstyrken.dk/arduino/tag/rgbtumbler/](http://atomslagstyrken.dk/arduino/tag/rgbtumbler/).

## Compilation

If you are using a Mac with Xcode you can open the PWM.xcodeproj project and select either "build", "flash" or "fuse" from the targets dropdown and then choose Product -> Build.

As the names indicate, "build" will just compile the project (handy for verifying the syntax), "flash" will compile and flash to the device and "fuse" will set the devices fuses as specified in the Makefile.

If you are _not_ using a Mac with Xcode, just use the normal AVR Libc build commands: `make flash` or `make fuse`.

## Configuration

Edit the Makefile to specify programmer and port. I am using an AVRISP mkII on the USB port.

# Disclaimer

Use at your own risk. And the code is most likely not complete yet...