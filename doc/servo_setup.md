# Servo Setup
## Wiring
All servos should be wired according to the wiring diagram with servo 0 being the first servo.
![Wiring Diagram](images/Teensy-Wiring-Diagram.png)

In `global_defines.h` `SERVO_0` to `SERVO_8` are mapped to match the wiring diagram.

## Renaming Servos to Match Their Use
In `global_defines.h` the servo defines for the servos you are using should be renamed in all files.
Use Arduino IDE functions to ensure that you have renamed the defines in all files.
Example `SERVO_0` could be renamed to `SERVO_RIGHT_AILERON`, `SERVO_LANDING_GEAR`, `SERVO_FPV_CAMERA_TILT`, `SERVO_RUDDER` or any other name to match its use.
I highly recommend keeping the prefix `SERVO` in all names to help "namespace" these names and remove any confusion with other similarly named variables or defines.


## Setting Servo Mid and Endpoints
Modify the `controlMixer()` function in main.ino so that only one servo at a time will move as you move a radio stick.
Example `servo_commands[SERVO_0] = rc_channels[RC_ROLL] * 90.0f;`
The 90.0f multiplier to the rc_channels is important as the servos by default expect -90.0 to 90.0 as the input range.
Run this code and ensure that you can move a servo.
Go to the `servo.ino` file and in the `initServoScales()` function find the `servoScalerInitHelper()` function call that matches to your servo.
Modify the inputs to the `servoScalerInitHelper()` calls to match what your servo is actually doing.
A positive servo value should map to control surfaces pointing up in relation to the aircraft.

The function `servoScalerInitHelper()` takes 6 inputs.
```
void servoScalerInitHelper(
    boundedRangeScaler_t &servoScales, 
    float angle_min, float angle_max, 
    float output_min, float output_mid, float output_max
) 
```

The first input is the `servoScales` it modifies.

The second and third input `angle_min` and `angle_max`. They set the input limits that you are using. This should be mapped to the amount of physical movement you want your control surfaces to have, not to servo arm movement.
Doing this makes it easier to set up the servo mixers and makes things like flaps for instance easier to setup.

The fourth, fifth and sixth inputs `output_min`, `output_mid` and `output_max`. They determine what the software will send as a min, mid, and max command to the servo.
If the servo is moving too much increasing the min/lowering the max will reduce servo movement. If the control surface does not center correctly changing the mid will change this behaviour.

### Examples

If your control surface is really only moving +-45 degrees and are centering correctly you would only modify the 2nd and 3rd inputs:
```
servoScalerInitHelper(
  servoScales[SERVO_0], 
  45.0f, -45.0f, // Servo control surface angle min and max, use a higher then lower number to swap directions
  0.0f, 0.5f, 1.0f // Servo output min, mid, and max. Modify is servo isn't centering or moving to far. Only takes a range of 0.0-1.0
);
```

If the control surface is moving too far when moving with your radio you might do something like this instead:
```
servoScalerInitHelper(
  servoScales[SERVO_0], 
  90.0f, -90.0f, // Servo control surface angle min and max, use a higher then lower number to swap directions
  0.2f, 0.5f, 0.8f // Servo output min, mid, and max. Modify is servo isn't centering or moving to far. Only takes a range of 0.0-1.0
);
```

If the servo is not centering correctly when your radio input is centered you might do something like this instead:
```
servoScalerInitHelper(
  servoScales[SERVO_0], 
  90.0f, -90.0f, // Servo control surface angle min and max, use a higher then lower number to swap directions
  0.0f, 0.6f, 1.0f // Servo output min, mid, and max. Modify is servo isn't centering or moving to far. Only takes a range of 0.0-1.0
);
```

If the servo is moving the wrong way (ie positive input is moving the control surface down) you would do this:
```
servoScalerInitHelper(
  servoScales[SERVO_0], 
  -90.0f, 90.0f, // Servo control surface angle min and max, use a higher then lower number to swap directions
  0.0f, 0.6f, 1.0f // Servo output min, mid, and max. Modify is servo isn't centering or moving to far. Only takes a range of 0.0-1.0
);
```