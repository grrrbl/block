# block
This is an implementation of a half automatic block system for modell railroads as it is used in russia and some other osshd countries. The Hardware is based on an ATtiny44 and some shift registers. For convenience it would be possible to use an AtMega 328p etc. Until now I haven't  made a layout for a PCB, but it's still on my ToDo List.

The block interface communicates with other devices over TTL serial. I don't use a crystall at the moment. So it depends on the callibration of the microcontrollers oscillator for the serial communication to work stable. I worked around the limitations induced by the jitter by just using 4 bit words for serial communication.

## Code Structure

## Block System function
