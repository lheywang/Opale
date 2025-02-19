================
Application
================

This folder contain all of the sources (.h and .c) files that are used by the application core.

----------------
Tasks
----------------

The network core is used actively to :

* Acquire measures 

 * Via UART on the GPS ans 9 axis IMU
 * Via it's integrated ADC for the feedback of servo engines
 * Via I2C for 2 other accelerometers and a barometer

* Filter and compute the required correction for the rocket
* Command outputs to control the different actuators

 * PWM for the four servo engines on the wings
 * PWM for the parachute servo engines
 * I2C for the optionnal GPIOs 

* Store measurements into dedicated EEPROMS for future analysis