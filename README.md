ESCape32
========

Firmware for 32-bit BLDC motor electronic speed controllers that aims at simplicity. It is designed to deliver smooth and efficient motor drive, fast transitions from a complete stop to full throttle, robust direction reversals, and maximum hardware support.


Features
--------

+ Servo PWM, Oneshot125, automatic throttle calibration
+ DSHOT 300/600/1200, bidirectional DSHOT, extended telemetry
+ Analog/serial/iBUS/SBUS input mode
+ KISS/iBUS/S.Port telemetry
+ DSHOT 3D mode, turtle mode, beacon, LED, programming
+ Sine startup mode (crawler mode)
+ Proportional brake, drag brake
+ Temperature/voltage/current protection
+ Variable PWM frequency, active freewheeling
+ Customizable startup music
+ Configuration via CLI using a USB-TTL adapter or Betaflight passthrough


Installation
------------

The list of compatible ESCs can be found [here](https://github.com/neoxic/ESCape32/wiki/Targets).

The latest release can be downloaded [here](https://github.com/neoxic/ESCape32/releases).

Visit the [ESCape32 Wiki](https://github.com/neoxic/ESCape32/wiki) for more information.


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

To flash a particular target using an ST-LINK programmer, run:

```
make flash-<target>
```
