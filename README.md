# OBD2Scan
Sketch to scan OBDII with an M2

A sketch that will automatically attempt to determine speed and connection status for the two CAN buses on the M2. 

It then queries for ECUs and investigates what sort of things those ECUs support.

If an SD card is inserted into the M2 it writes a log file with the supported PIDs.
