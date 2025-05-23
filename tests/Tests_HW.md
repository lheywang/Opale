# TEST HW

## SWD Section

### nRF

[x] SWD Detection by host computer
[x] Able to program the chip

## PWM Section :

### RGB Led

[x] Led 1

### Engines commands

[x] Principle (global system is working)
[x] Servo 1
[x] Servo 2
[x] Servo 3
[ ] Servo 4 (No signal on PWM input. Incorrect soldering.)
[ ] Parachute

### Feedback tests

[x] Principle
[ ] Servo 1
[ ] Servo 2
[ ] Servo 3
[ ] Servo 4
[ ] Parachute

### Buzzer

[x] Buzzer (extremely quiet. Not enough voltage ? current ?)

## UART Section

### DEBUG OUT

[x] Able to log outputs on the debug
[ ] Able to send messages on the debug

### IMU

[ ] UART waveforms are correct on the bus (read)
[ ] UART waveforms are correct on the bus (write)
[ ] IMU respond to answers
[ ] All functions of the IMU are tested

### TESEO

[x] UART waveforms are correct on the bus (read) --> No FIX
[ ] UART waveforms are correct on the bus (write)
[ ] TESEO respond to answers
[ ] All functions of the TESEO are tested

## GPIO

### Reset

[x] GPIO does strobe to 0 when booting.

### Safety toggling

[x] Able to toggle the latch (Hardware change output state. No reading by the MCU. Incorrect soldering.)
[ ] Latch output match our requirements

### Engines

[ ] Engines are starting correctly
[ ] Engines are blocked by the AND condition
