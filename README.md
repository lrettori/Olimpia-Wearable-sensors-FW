# Olimpia-Wearable-sensors-FW
Firmware of the sensor system Olimpia

First release:
Original version, pre-2024. Several major issues, regarding both Bluetooth and serial communication, leading to missing bytes, message-shift, sending of old messages, and more.
Never released version, fixed with second release (which is actually the first used).

Second release:
Released in may 2024. Fixed the issues of the first version. One last non-solved (but managed) bug regarding the serial communication between microcontroller and BLE module of sensor nodes.

Third release:
October 2024. Introduced a sinchronization message sent from the PC to the dongle. First release revealed pre-existing bug, solved with the commit pushed on oct 17th.
