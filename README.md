ESCape32
========

Firmware for 32-bit BLDC motor electronic speed controllers that aims for simplicity. It is designed to deliver smooth and efficient motor drive, fast transitions from a complete stop to full throttle, robust direction reversals, and maximum hardware support.


Features
--------

+ Servo PWM, Oneshot125, automatic throttle calibration
+ DSHOT 300/600/1200, bidirectional DSHOT, extended telemetry
+ Analog/serial/iBUS/SBUS/SBUS2/CRSF input mode
+ KISS/iBUS/S.Port/CRSF telemetry
+ DSHOT 3D mode, turtle mode, beacon, LED, programming
+ Sine startup mode, brushed mode, hybrid mode (sensored/sensorless)
+ Proportional brake, drag brake
+ Temperature/voltage/current protection
+ Variable PWM frequency, active freewheeling
+ Customizable startup music
+ Propeller positioning feature to stop the propeller in a specific position


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

Use `LIBOPENCM3_DIR` to specify a path to LibOpenCM3 if it is not in the system root:

```
git clone https://github.com/libopencm3/libopencm3.git
make -C libopencm3 TARGETS='stm32/f0 stm32/g0 stm32/g4 stm32/l4'
cmake -B build -D LIBOPENCM3_DIR=libopencm3
```

Use `CMAKE_INSTALL_PREFIX` to specify an alternative system root:

```
cmake -B build -D CMAKE_INSTALL_PREFIX=~/local
```

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


Building on GitHub
------------------

+ Fork the repository
+ Go to _Actions_
+ Run the _Build ESCape32_ workflow


Propeller Positioning Feature
-----------------------------

The propeller positioning feature allows the propeller to stop in a specific position in airplane mode to avoid damage during landing. This feature requires a dedicated position sensor to detect the desired propeller position.

### Enabling the Propeller Positioning Feature

To enable the propeller positioning feature, set the `propeller_positioning` configuration option to `1` in the `src/common.h` file.

### Required Hardware

The propeller positioning feature requires a dedicated position sensor that can be connected to a GPIO pin. The sensor should provide a signal when the desired propeller position is reached.

### Using the Propeller Positioning Feature

1. Connect the position sensor to a free GPIO pin on the ESC.
2. Enable the propeller positioning feature in the configuration.
3. The ESC firmware will use the position sensor to stop the propeller in the desired position when the airplane mode is activated.
