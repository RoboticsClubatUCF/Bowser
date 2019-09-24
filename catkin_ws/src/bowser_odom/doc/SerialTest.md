# Serial Test
1. Get a USB-RS232-3.3 cable.
2. Connect the RX labeled wire to TXD0 on the Jetson TX2.
3. Connect the TX labeled wire to RXD0 on the Jetson TX2.
4. Connect the GND to each other.
5. Connect the USB to another computer and open Arduino to check /dev/ttyUSB0 at 9600 baude rate.
6. Open Putty on the Jetson using serial line /dev/ttyS0 at 9600 baude rate.
