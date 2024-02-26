# Radio Controller Setup
The flight controller does not know what you want your RC channels to do before you have setup the radio code.
The flight controller does not know the ranges of the RC channels before you have setup the radio code.

## Radio Channels
Comment in the lines
```
// delay(1000); // Add extra delay so that we can get a radio connection first. Increase value if things aren't working.
// findRcChannelLimits(RC_ARM); // RC limits printed to serial monitor. Paste these in radio.ino, then comment this out forever.
```
in main.ino inside of `void setup()`.

The `delay(1000)` is important as it gives some time for your receiver to connect to your radio and start sending SBUS signals to the flight controller.
If the flight controller is not receiving valid SBUS signals before reaching this line you may get invalid results.

Power on your radio controller and the teensy4.0 flight controller via USB.
Open the file `global_defines.h` and find:
```
// TODO ensure that these match the channels they go to, this is setup for standard opentx/edgetx channel order AETR
#define RC_ROLL 0 // can also be called RC_AILERON if desired
#define RC_PITCH 1 // can also be called RC_ELEVATOR if desired
#define RC_THROTTLE 2
#define RC_YAW 3 // can also be called RC_RUDDER if desired
#define RC_ARM 4 // if using ELRS this channel should always be your arming channel

// TODO rename to match aux channels function if desired
#define RC_AUX1 5 // rename if you want channels to have mode names
#define RC_AUX2 6 // rename if you want channels to have mode names
#define RC_AUX3 7 // rename if you want channels to have mode names
#define RC_AUX4 8 // rename if you want channels to have mode names
#define RC_AUX5 9 // rename if you want channels to have mode names
#define RC_AUX6 10 // rename if you want channels to have mode names
#define RC_AUX7 11 // rename if you want channels to have mode names
```
Watch the serial monitor and see which channels are moving as you move your sticks/switches.
Make sure that the names of RC channels match the channel that the radio is sending found via the serial monitor.
Note: If using ELRS then `#define RC_ARM 4` should always be used.
The `RC_AUX` should be renamed to match what you want that switches function to be.
You need to rename this in all files. Use the IDE to automate this for you.
A naming example could be changing `#define RC_AUX1 5` to be `#define RC_FLIGHT_MODE 5` 
or `#define RC_FLIGHT_CONFIGURATION 5` or even `#define RC_TRANSITION 5`.
`RC_` should still be used as the start of the name to make indexing the `rc_channels[]` more memorable.


## Radio Ranges
This is continued from `Radio Channels` above.
Check the serial monitor and you should see all your radio channels displaying a min, max and current value.
Move all your sticks and switches to their extreme values, then set your sticks to their middle position.

In the file `radio.ino` edit the function call to `midpointRangeScalerInit()` inside the function `initRcScalers()`.
You should modify it so that the `174.0f, 992.0f, 1811.0f, // input min, mid and max` line matches the serial monitor output.

Comment in the lines 
```
delay(1000); // Add extra delay so that we can get a radio connection first. Increase value if things aren't working.
findRcChannelLimits(RC_ARM); // RC limits printed to serial monitor. Paste these in radio.ino, then comment this out forever.
```
in main.ino inside of `void setup()` when you are finished.