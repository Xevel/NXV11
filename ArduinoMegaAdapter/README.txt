Arduino Neato XV-11 Laser Distance Scanner 
Motor control board v0.2 by Cheng-Lung Lee

XV-11 LDS adapter reads LDS data from RX3 then relay to TX. Also extract the speed 
data from the data stream to do speed control on LDS motor. Everythin can power 
from USB no extra power required.

This code is tested on Arduino Mega 1280
I/O:
Motor drive by low side driver IPS041L connect to PWM Pin4, Motor power from 5V
Neato XV-11 LDS Vcc(red) : 5V
Neato XV-11 LDS TX(Orange) : RX3 
