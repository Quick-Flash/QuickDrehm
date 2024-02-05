# SBUS Setup
Most users will be using ELRS to generate SBUS commands to the flight controller.
What we want to make sure of with our SBUS setup is that failsafes will be seen as a failsafe.
FrSky should work by default.

### FrSky
This should work out of the box.

### ELRS Users
When setting up SBUS failsafe ensure that "SBUS Failsafe" is set to sending failsafe output.
Otherwise failsafe will not be seen by the flight controller!

### All other SBUS
Ensure that on failsafe that your receiver is sending failsafe packets.
Otherwise failsafe will not be seen by the flight controller!

# Safety!!!
Testing should be mandatory for your SBUS link. 
If it is not setup correctly it is quite possible that during failsafe a dangerous fly away will occur.

## Testing
Debug the `failsafe` variable and ensure that it shows 0 when your radio is turned on and 1 when you turn off your radio.
