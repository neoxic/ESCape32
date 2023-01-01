ESCape32
========

Firmware for 32-bit BLDC motor electronic speed controllers that aims to Keep-It-Super-Simple. It is designed to deliver smooth and efficient motor drive, fast transitions from a complete stop to full throttle, robust direction reversals, and maximum hardware support.


Features
--------

+ Servo PWM with automatic throttle calibration
+ iBUS, S.BUS, DSHOT 300/600, bidirectional DSHOT
+ Proportional brake / drag brake
+ Variable PWM frequency
+ KISS telemetry
+ Customizable sounds
+ Configuration/firmware updates over the signal wire using a generic USB-TTL adapter


Installation
------------

The latest release can be downloaded [here](https://github.com/neoxic/ESCape32/releases).

Visit the [Wiki](https://github.com/neoxic/ESCape32/wiki) to get started with ESCape32.


Dependencies
------------

+ cmake
+ arm-none-eabi-gcc
+ arm-none-eabi-binutils
+ arm-none-eabi-newlib
+ libopencm3
+ stlink


Building from source
--------------------

To build all targets, run:

```
cmake -B build
cd build
make
```

To flash a particular target using an ST-LINK probe, run for example:

```
make flash-BOOT_F0_PA2
make flash-F051_PA2_A
```
