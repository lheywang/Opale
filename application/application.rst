================
Application
================

This folder contain all of the sources (.h and .c / .cpp) files that are used by the application core.

----------------
Tasks
----------------

There is two core : The app core, and the network core. 
The app core is used to acquire data, compute trajectories, ensure safety and so...
The network core is used to ensure the UI over BT (start !).

----------------
Threads
----------------

There four threads that run concurrently on the MCU : 

* Safety thread : Ensure that the safety conditions are met before starting the rocket. Enable the rocket engine starter and some hardware locks.
* Measurement thread : Acquire the data, compute basic values on it and send if to the other threads.
* Logger thread : Log the data incomming at a known rate, and store it to the EEPROM
* Controller thread : Compute the position and control the servo engines
