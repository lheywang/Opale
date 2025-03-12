================
Devices
================

This folder store the base source code (.c and .h files) that are used by any 
devices directly connected to the nRF5340 IC. Any device that required a dedicated
driver code is placed in another folder.

For example, you'll find the .c and .h for PWM control (and a small abstraction layer over them),
but not the full c driver for the IMU for example !