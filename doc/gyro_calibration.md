# Gyro Calibration

Digital IMU tend to have small amounts of bias to both their internal accelerometer and gyros. 
The gyro calibration is meant to find this bias so that the code can add a bias correction after reading the accelerometer and gyro.
This is important to reduce drift and help understand the direction of gravity better.

## How To
Comment in the line
`// calculateGyroBias(); // Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.`
in main.ino inside of `void setup()`.
This will run the code which will calculate the gyro bias.
Once you have done this set your aircraft on a flat surface and power on the teensy4.0 flight controller via USB.
Do not touch, move, or bump the aircraft during this process or you will get a poor calibration.
Watch the serial monitor and wait until it prints out the `gyro_bias` and `acc_bias`.
Copy the `gyro_bias` and `acc_bias` to your clipboard.
In the file `global_defines.h` replace the `gyro_bias` and `acc_bias` found there with the biases just found.

The code will not continue past this line when `calculateGyroBias();` is compiled.
So it is important that you comment this line of code out after finding the bias.
