// Teensy Flight Controller - QuickDrehm
// Authors: Kevin Plaizer, Nicholas Rehm
// Version: Alpha 1.0
 
//========================================================================================================================//

//CREDITS + SPECIAL THANKS
/*
Some elements inspired by:
http://www.brokking.net/ymfc-32_main.html

Madgwick filter function adapted from:
https://github.com/arduino-libraries/MadgwickAHRS


Thank you to:
RcGroups 'jihlein' - IMU implementation overhaul + SBUS implementation.
Everyone that sends me pictures and videos of your flying creations! -Nick
Nick Drehm for the creation of DrehmFlight which vastly sped up the creation of QuickDrehm

*/

// REQUIRED LIBRARIES (included with download in main sketch folder)

#include <Wire.h>     // I2c communication
#include <SPI.h>      // SPI communication
#include <PWMServo.h> // Commanding any extra actuators, installed with teensyduino installer
#include "global_defines.h"

//========================================================================================================================//
//                                                USER-SPECIFIED STRUCTS                                                  //                           
//========================================================================================================================//

typedef struct attitudePid_s {
  float kp; // pterm constant
  // no need for ki term in the attitude controller
  float kd; // dterm constant

  float previous_error_or_measurement[AXIS_COUNT]; // last error or measurement, used for calculating dterm
} attitudePid_t;

typedef struct ratePid_s {
  float kp[AXIS_COUNT]; // pterm constant
  float ki[AXIS_COUNT]; // iterm constant
  float kd[AXIS_COUNT]; // dterm constant
  float kff[AXIS_COUNT]; // ffterm constant, should only be used in fixed wing flight

  pt2Filter_t dterm_lowpass[AXIS_COUNT];
  float previous_error_or_measurement[AXIS_COUNT]; // last error or measurement, used for calculating dterm
  float integral[AXIS_COUNT]; // the integral of error used for calculating iterm

  float max_iterm_windup;
} ratePid_t;

typedef struct gyroFilters_s {
  dynNotch_t dynNotch;
  rpmFilter_t rpmFilter;
  pt1Filter_t lowpassFilter[AXIS_COUNT];
} gyroFilters_t;

typedef struct accFilters_s {
  pt2Filter_t lowpassFilter[AXIS_COUNT];
} accFilters_t;

typedef struct rcFilters_s {
  pt3Filter_t lowpassFilter[4];
} rcFilters_t;

//======================================================ATTITUDE PID=======================================================//

// TODO tune the attitude pid controller
void attitudePidInit(attitudePid_t *pid) {
  pid->kp = 275.0f;
  pid->kd = 1.0f;

  pid->previous_error_or_measurement[0] = 0.0f;
  pid->previous_error_or_measurement[1] = 0.0f;
  pid->previous_error_or_measurement[2] = 0.0f;

}

// finds the cross product between the attitude_setpoint and the measured gravity_vector
// don't worry about how this works, its pretty nifty, ask me if you are curious about it
void attitudeError(float attitude_setpoint[], float gravity_vector[], float error[]) {
  float temp_error[3];
  temp_error[0] = gravity_vector[AXIS_Z] * attitude_setpoint[AXIS_Y] - gravity_vector[AXIS_Y] * attitude_setpoint[AXIS_Z];
  temp_error[1] = -gravity_vector[AXIS_Z] * attitude_setpoint[AXIS_X] + gravity_vector[AXIS_X] * attitude_setpoint[AXIS_Z];
  temp_error[2] = gravity_vector[AXIS_Y] * attitude_setpoint[AXIS_X] - gravity_vector[AXIS_X] * attitude_setpoint[AXIS_Y];

  float dot_product = gravity_vector[AXIS_X] * attitude_setpoint[AXIS_X] + gravity_vector[AXIS_Y] * attitude_setpoint[AXIS_Y] + gravity_vector[AXIS_Z] * attitude_setpoint[AXIS_Z];

  // if the dot product is negative our error is greater than 90 degrees
  // increase correction on the more wrong axis and correct to upside down on the other
  // aka if pitch was 130 deg and roll was 5, rather than flipping over on roll axis, make roll 0 deg and push hard on pitch
  if (dot_product < 0.0) {
    float x = abs(temp_error[0]);
    float y = abs(temp_error[1]);

    if (x > y) {
      float sign_x = sign(temp_error[0]);
      error[0] = 2.0f * sign_x - temp_error[0];
      error[1] = -temp_error[1];
      error[2] = temp_error[2];
    } else {
      float sign_y = sign(temp_error[1]);
      error[0] = -temp_error[0];
      error[1] = 2.0f * sign_y - temp_error[1];
      error[2] = temp_error[2];
    }
  } else {
    error[0] = temp_error[0];
    error[1] = temp_error[1];
    error[2] = temp_error[2];
  }
}

// outputs setpoint for the rate pid controller
// TODO finish this function
void attitudePidApply(attitudePid_t *pid, float setpoint_angles[], float gravity_vector[], float setpoint_rpy[]) {
  float setpoint_xyz[AXIS_COUNT]; // gravity setpoint
  attitudeAnglesToAttitudeSetpoint(setpoint_angles[AXIS_ROLL], setpoint_angles[AXIS_PITCH], setpoint_xyz); // create the attitude setpoint

  float error_array[AXIS_COUNT]; 
  attitudeError(setpoint_xyz, gravity_vector, error_array);

  float yaw_setpoint = setpoint_rpy[AXIS_YAW];

  for (int axis = 0; axis < AXIS_COUNT; axis++) {
    float error = error_array[axis];

    // TODO calculate Pterm which is a scaler of error
    float pterm = pid->kp * 0.0;
    
    // TODO calculate Dterm which is the derivative of error (faster) or the negative derivative of gyro (smoother) times a scaler
    float derivative = 0.0f;
    // only keep one previous_error_or_measurement, remove the one you are not using
    pid->previous_error_or_measurement[axis] = gravity_vector[axis];
    pid->previous_error_or_measurement[axis] = error;

    float dterm = pid->kd * derivative;

    // pidsum is the addition of all the pid terms
    float pidsum = pterm + dterm;
    setpoint_rpy[axis] = pidsum; // attitude pid pidsum becomes the setpoint into to the rate pid controller
  }

  // this is how we add yaw rotation
  setpoint_rpy[AXIS_ROLL] += gravity_vector[AXIS_X] * yaw_setpoint;
  setpoint_rpy[AXIS_PITCH] += gravity_vector[AXIS_Y] * yaw_setpoint;
  setpoint_rpy[AXIS_YAW] += gravity_vector[AXIS_Z] * yaw_setpoint;
}

//======================================================RATE PID=======================================================//

// TODO tune the rate pid controller
void ratePidInit(ratePid_t *pid) {
// pid scaling used in BF/inav
#define PTERM_SCALE (0.032029f / 1000.0f)
#define ITERM_SCALE (0.244381f / 1000.0f)
#define DTERM_SCALE (0.000529f / 1000.0f)
#define FFTERM_SCALE (0.032029f / 1000.0f)

  // Roll PID's
  pid->kp[AXIS_ROLL] = PTERM_SCALE * 10.0f;
  pid->ki[AXIS_ROLL] = ITERM_SCALE * 10.0f;
  pid->kd[AXIS_ROLL] = DTERM_SCALE * 10.0f;
  pid->kff[AXIS_ROLL] = FFTERM_SCALE * 0.0f;

  // Pitch PID's
  pid->kp[AXIS_PITCH] = PTERM_SCALE * 10.0f;
  pid->ki[AXIS_PITCH] = ITERM_SCALE * 10.0f;
  pid->kd[AXIS_PITCH] = DTERM_SCALE * 10.0f;
  pid->kff[AXIS_PITCH] = FFTERM_SCALE * 0.0f;

  // Yaw PID's
  pid->kp[AXIS_YAW] = PTERM_SCALE * 15.0f;
  pid->ki[AXIS_YAW] = ITERM_SCALE * 10.0f;
  pid->kd[AXIS_YAW] = DTERM_SCALE * 5.0f;
  pid->kff[AXIS_YAW] = FFTERM_SCALE * 0.0f;

  for (int axis = 0; axis < AXIS_COUNT; axis++) {
    pid->previous_error_or_measurement[axis] = 0.0f;
    pid->integral[axis] = 0.0f;

    pt2FilterInit(
      &pid->dterm_lowpass[axis], 
      90.0f,  // cutoff
      DT
    );
  }

  pid->max_iterm_windup = 0.15f; // iterm can grow at most to 15% motor output
}

void updatePids(
    ratePid_t *pid,
    float roll_p,
    float roll_i,
    float roll_d,
    float roll_ff,
    float pitch_p,
    float pitch_i,
    float pitch_d,
    float pitch_ff,
    float yaw_p,
    float yaw_i,
    float yaw_d,
    float yaw_ff
  ) {
  #define PTERM_SCALE (0.032029f / 1000.0f)
  #define ITERM_SCALE (0.244381f / 1000.0f)
  #define DTERM_SCALE (0.000529f / 1000.0f)
  #define FFTERM_SCALE (0.032029f / 1000.0f)
  // Roll PID's
  pid->kp[AXIS_ROLL] = PTERM_SCALE * roll_p;
  pid->ki[AXIS_ROLL] = ITERM_SCALE * roll_i;
  pid->kd[AXIS_ROLL] = DTERM_SCALE * roll_d;
  pid->kff[AXIS_ROLL] = FFTERM_SCALE * roll_ff;

  // Pitch PID's
  pid->kp[AXIS_PITCH] = PTERM_SCALE * pitch_p;
  pid->ki[AXIS_PITCH] = ITERM_SCALE * pitch_i;
  pid->kd[AXIS_PITCH] = DTERM_SCALE * pitch_d;
  pid->kff[AXIS_PITCH] = FFTERM_SCALE * pitch_ff;

  // Yaw PID's
  pid->kp[AXIS_YAW] = PTERM_SCALE * roll_p;
  pid->ki[AXIS_YAW] = ITERM_SCALE * roll_i;
  pid->kd[AXIS_YAW] = DTERM_SCALE * roll_d;
  pid->kff[AXIS_YAW] = FFTERM_SCALE * roll_ff;
}

// TODO finish this function
void ratePidApply(ratePid_t *pid, float setpoint[], float gyro[], float pidSums[]) {
  for (int axis = 0; axis < AXIS_COUNT; axis++) {
    // TODO calculate error
    float error = 0.0f;

    // TODO calculate Pterm which is a scaler of error
    float pterm = pid->kp[axis] * 0.0f;

    // TODO calculate Iterm which is the integral of error times a scaler
    pid->integral[axis] += pid->ki[axis] * 0.0f;

    pid->integral[axis] = constrain(pid->integral[axis], -pid->max_iterm_windup, pid->max_iterm_windup);
    float iterm = pid->integral[axis];

    // TODO calculate Dterm which is the derivative of error (faster) or the negative derivative of gyro (smoother) times a scaler
    float derivative = 0.0f;
    // only keep one previous_error_or_measurement, remove the one you are not using
    pid->previous_error_or_measurement[axis] = gyro[axis];
    pid->previous_error_or_measurement[axis] = error;

    derivative = pt2FilterApply(&pid->dterm_lowpass[axis], derivative); // filter the dterm, helps with noise
    float dterm = pid->kd[axis] * derivative;

    // TODO calculate FFterm which is a scaler of the setpoint (Should be really only be used in fixed wing flight modes)
    float ffterm = pid->kff[axis] * 0.0f;
    
    // pidsum is the output of the PID controller, simply add all pid terms together
    pidSums[axis] = pterm + iterm + dterm + ffterm;
  }
}

void ratePidReset(ratePid_t *pid) {
  for (int axis = 0; axis < AXIS_COUNT; axis++) {
    pid->integral[axis] = 0.0f;
  }
}

//======================================================GYRO FILTERS=======================================================//

// initialize gyro filters
void initGyroFilters(gyroFilters_t *gyroFilters) {
  dynNotchInit( // you likely don't need to mess with this
    &gyroFilters->dynNotch, 
    100.0f, // min freq
    600.0f, // max_freq
    3, // number of peaks tracked, max allowed is five
    2.5f, // notch q
    DT // dT
  );

  rpmFilterInit( // You likely don't need to mess with this
    &gyroFilters->rpmFilter, 
    75.0f, // min freq
    50.0f, // fade range
    3.0f, // notch q
    DT // dT
  );
  
  for (int axis = 0; axis < AXIS_COUNT; axis++) { // initialize all lowpass filters for each axis
    pt1FilterInit(
      &gyroFilters->lowpassFilter[axis], 
      100.0f, // Filter cutoff 
      DT // dT
    );
  }
}

// applies gyro filters to all axis of the gyro
void gyroFiltersApply(gyroFilters_t *gyroFilters, float gyro[]) {
  for (int axis = 0; axis < AXIS_COUNT; axis++) {
    for (int motor = 0; motor < MOTOR_COUNT; motor++) { // rpm filter
      gyro[axis] = rpmFilterApply(&gyroFilters->rpmFilter, axis, gyro[axis]);
    }
    gyro[axis] = pt1FilterApply(&gyroFilters->lowpassFilter[axis], gyro[axis]); // lowpass filter
  }

  dynNotchUpdate(&gyroFilters->dynNotch, gyro, DT); // update the dyn notch
  
  for (int axis = 0; axis < AXIS_COUNT; axis++) {
    gyro[axis] = dynNotchFilter(&gyroFilters->dynNotch, axis, gyro[axis]);
  }
}

//======================================================ACC FILTERS=======================================================//

// initialize acc filters
void initAccFilters(accFilters_t *accFilters) {
  for (int axis = 0; axis < AXIS_COUNT; axis++) { // initialize all lowpass filters for each axis
    pt2FilterInit(
      &accFilters->lowpassFilter[axis], 
      20.0f, // Filter cutoff 
      DT // dT
    );
  }
}

// applies ac filters to all axis of the acc
void accFiltersApply(accFilters_t *accFilters, float acc[]) {
  for (int axis = 0; axis < AXIS_COUNT; axis++) {
    acc[axis] = pt2FilterApply(&accFilters->lowpassFilter[axis], acc[axis]); // lowpass filter
  }
}

//======================================================RC FILTERS=======================================================//

// initialize rc filters
void initRCFilters(rcFilters_t *rcFilters) {
  for (int channel = 0; channel < 4; channel++) { // initialize all lowpass filters for each axis
    pt3FilterInit(
      &rcFilters->lowpassFilter[channel], 
      20.0f, // Filter cutoff 
      DT // dT
    );
  }
}

// applies gyro filters to all axis of the gyro
void rcFiltersApply(rcFilters_t *rcFilters, float rc[]) {
  for (int channel = 0; channel < 4; channel++) {
    rc[channel] = pt3FilterApply(&rcFilters->lowpassFilter[channel], rc[channel]); // lowpass filter
  }
}

//========================================================================================================================//
//                                                      VOID SETUP                                                        //
//========================================================================================================================//

//==================================================VALUES TO INITIALIZE==================================================//

midpointRangeScaler_t rcScalers[RC_CHANNEL_COUNT];
boundedRangeScaler_t servoScales[MAX_SERVO_COUNT];
attitudePid_t attitudePid;
ratePid_t ratePid;
gyroFilters_t gyroFilters;
accFilters_t accFilters;
rcFilters_t rcFilters;

// All the code that is only run once
void setup() {
  Serial.begin(500000); // USB serial
  delay(500);
  
  initMotors();

  // Initilize the rcScalers
  initRcScalers(rcScalers);

  // Initilize the servoScales
  initServos(servoScales);
  
  // Initilize the attitudePid
  attitudePidInit(&attitudePid);

  // Initilize the ratePid
  ratePidInit(&ratePid);

  // Initilize the gyroFilters
  initGyroFilters(&gyroFilters);

  // Initilize the accFilters
  initAccFilters(&accFilters);

  // Initilize the rcFilters
  initRCFilters(&rcFilters);

  // Initialize all pins
  pinMode(13, OUTPUT); // Pin 13 LED blinker on board, do not modify 

  // Set built in LED to turn on to signal startup
  digitalWrite(13, HIGH);

  delay(5);

  // Initialize radio communication
  radioSetup();

  //
  // delay(1000); // Add extra delay so that we can get a radio connection first. Increase value if things aren't working.
  // findRcChannelLimits(RC_ARM); // RC limits printed to serial monitor. Paste these in radio.ino, then comment this out forever.
  //

  // Initialize IMU communication
  IMUinit();

  delay(5);

  // Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  // calculateGyroBias(); // Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.
  // 

  // Indicate entering main loop with 3 quick blinks
  setupBlink(3, 160, 70); // numBlinks, upTime (ms), downTime (ms)
}


//========================================================================================================================//
//                                                         DEBUG                                                          //
//========================================================================================================================//
// TODO easily debug any part of your code when needed
// Example of how to print values copy and paste this into the function loop() wherever you want debugging
/*
  bool should_print = shouldPrint(current_time, 10.0f); // Print data at 50hz
  if (should_print) {
    printDebug("gyro roll", raw_gyro[AXIS_ROLL]);
    printDebug(", pitch", raw_gyro[AXIS_PITCH]);
    printDebug(", yaw", raw_gyro[AXIS_YAW]);
    printNewLine();
  }
*/

//========================================================================================================================//
//                                                       MAIN LOOP                                                        //
//========================================================================================================================//
unsigned long current_time, previous_time;
void loop() {
  previous_time = current_time;      
  current_time = micros();      
  float measured_dt = (current_time - previous_time)/1000000.0;

  loopBlink(current_time, 0.5f); // Indicate we are in main loop with a blink every x seconds


//===============================================GET IMU DATA AND FILTER===================================================//

  float raw_gyro[AXIS_COUNT];
  float raw_acc[AXIS_COUNT]; // static means it is not reset every loop

  // Get IMU sensor data
  bool new_acc = getIMUdata( // Pulls raw gyro, and accelerometer from IMU returns true if there was new acc data
    gyro_bias, acc_bias, // is used to fix bias, but is not updated
    raw_gyro, raw_acc, // will be updated with imu data
    imuRotation // the rotation of the imu
  );

  float gyro_filtered[AXIS_COUNT] = { raw_gyro[AXIS_ROLL], raw_gyro[AXIS_PITCH], raw_gyro[AXIS_YAW] };
  gyroFiltersApply(&gyroFilters, gyro_filtered);

  float acc_filtered[AXIS_COUNT] = { raw_acc[AXIS_X], raw_acc[AXIS_Y], raw_acc[AXIS_Z] };
  accFiltersApply(&accFilters, acc_filtered);

  float attitude_euler[AXIS_COUNT]; // This is the attitude in euler angles, AKA roll, pitch, and yaw
  float gravity_vector[AXIS_COUNT]; // This is a 3d vector that points towards gravity. Using this is more accurate for angle mode

  // Updates estimated atittude (degrees) Think of this like AHRS in an aircraft
  Madgwick6DOF(
    gyro_filtered, acc_filtered, new_acc, DT, ACC_DT, // Is used to calculate attitude changes, but is not updated
    attitude_euler, gravity_vector // Will be updated with attitude data
  );

//===============================================GET RC DATA AND FILTER===================================================//

  // Get rc commands
  uint16_t raw_rc_channels[RC_CHANNEL_COUNT]; // We will scale and set this to a float later
  bool failsafe = getRcChannels( // Pulls current available radio commands
    raw_rc_channels // Gets updated to have raw rc channel data
  );
  // scale rc_channels
  float rc_channels[RC_CHANNEL_COUNT];

  // scale all the rc_channels
  for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
    rc_channels[i] = midpointRangeScalerApply( // rc_channels now contains the scaled rc channel
      rcScalers[i], // The scale being used
      raw_rc_channels[i] // The input being scaled
    );
  }

  /* 
  // EXAMPLE rotating your rc roll pitch and yaw
  // used when your aircraft changes its control scheme mid flight
  // example aircraft found here https://www.youtube.com/watch?v=RabFZzRyZo8&list=PLTSCOv-lGtMax-oA4Pnq8OTxd4fTucrjQ&index=7&ab_channel=NicholasRehm
  // dont use the method Nick does, instead rotate your controls
  float rc_rpy[AXIS_COUNT] = { rc_channels[RC_ROLL], rc_channels[RC_PITCH], rc_channels[RC_YAW] };
  axisRotation rc_rotation[AXIS_COUNT] = {ROT_0_DEG, ROT_90_DEG, ROT_180_DEG}; // roll, pitch, yaw rotation set as needed
  vectorRotation(rc_rpy, rc_rotation); // apply rotation to rc_rpy
  // update rc_channels to have the rotated rc
  rc_channels[RC_ROLL] = rc_rpy[0];
  rc_channels[RC_PITCH] = rc_rpy[1];
  rc_channels[RC_YAW] = rc_rpy[2];
  */

  // will only filter the first 4 channels and not switch channels
  rcFiltersApply(&rcFilters, rc_channels);

//===============================================CREATE SETPOINTS FOR PID CONTROLLER===================================================//

  // TODO finish this
  float setpoints_rpy[AXIS_COUNT]; // these are the desired angles or rotation

  // TODO add extra modes for fixedwing flight modes with different setpoints
  if (rc_channels[RC_AUX1] > 0.55) { // lets call aux1 angle mode for now, you can rename it later
    // These setpoints are in deg, in other words what angle you want to be at, except for yaw which is in deg/s
    // keep the max angle below about 60

    float max_angle = 45.0f;
    setpoints_rpy[AXIS_ROLL] = rcCurve(rc_channels[RC_ROLL], 0.5f, max_angle); // scaled value, expo, max angle deg
    setpoints_rpy[AXIS_PITCH] = rcCurve(rc_channels[RC_PITCH], 0.5f, max_angle); // scaled value, expo, max angle deg

    float max_rotation = 300.0f;
    // yaw is set to negative rc_channels positive gyro yaw is to the left we want stick movements to the right to be positive
    setpoints_rpy[AXIS_YAW] = -rcCurve(rc_channels[RC_YAW], 0.5f, max_rotation); // scaled value, expo, max rotation deg/sec
  } else { // acro mode
    // These setpoints are in deg/sec, in otherwords how fast you want to rotate
    // be careful setting the max setpoint above 60 deg for yaw on fixed wings as they have trouble yawing that fast
    // with thrust vectoring fixed wing setpoint above 80 deg for ayw is possible
    // be careful setting the max setpoint above 300 for pitch and roll on fixed wing as they have trouble rotating that fast

    float max_rotation = 300.0f;

    setpoints_rpy[AXIS_ROLL] = rcCurve(rc_channels[RC_ROLL], 0.5f, max_rotation); // scaled value, expo, max rotation deg/sec
    setpoints_rpy[AXIS_PITCH] = rcCurve(rc_channels[RC_PITCH], 0.5f, max_rotation); // scaled value, expo, max rotation deg/sec

    // yaw is set to negative rc_channels positive gyro yaw is to the left we want stick movements to the right to be positive
    setpoints_rpy[AXIS_YAW] = -rcCurve(rc_channels[RC_YAW], 0.5f, max_rotation); // scaled value, expo, max rotation deg/sec
  }

//===============================================MODIFY SETPOINTS DURING FAILSAFE===================================================//

  // deal with failsafe case
  if (failsafe) { // Set rc_channels/acro to values you want after failsafe
    // You may want to set rc_channels to allow for a plane to do slow turns in angle mode.
    rc_channels[RC_THROTTLE] = 0.0f; // Unless you really know what you are doing set throttle to 0
    rc_channels[RC_ARM] = 0.0f; // Unless you really know what you are doing disarm

    // put your fixed wing into angle mode and slowly turn it to the right while failsafed
    // really only works for fixed wing aircraft
    rc_channels[RC_AUX1] = 1.0f; // set the aircraft to angle mode
    setpoints_rpy[AXIS_ROLL] = 25.0; // tilt right slightly to help turn
    setpoints_rpy[AXIS_PITCH] = 5.0; // pitch down to help keep some airspeed and prevent stalling

    // yaw is set to negative rc_channels positive gyro yaw is to the left we want to turn right
    setpoints_rpy[AXIS_YAW] = -25.0; // coordinate the turn
  };

//=========================================================PID CONTROLLERS=========================================================//

// TODO enable and finish the below code when getting ready for transition flights
/*
  // update pid values based on flight mode
  float roll_kp = ???;
  float roll_ki = ???;
  float roll_kd = ???;
  float roll_kff = ???;

  float pitch_kp = ???;
  float pitch_ki = ???;
  float pitch_kd = ???;
  float pitch_kff = ???;

  float yaw_kp = ???;
  float yaw_ki = ???;
  float yaw_kd = ???;
  float yaw_kff = ???;
  updatePids(
    &ratePid,
    roll_kp,
    roll_ki,
    roll_kd,
    roll_kff,
    pitch_kp,
    pitch_ki,
    pitch_kd,
    pitch_kff,
    yaw_kp,
    yaw_ki,
    yaw_kd,
    yaw_kff
  );
*/
  float pidSums[AXIS_COUNT] = {0.0f, 0.0f, 0.0f}; // will be used in the mixer
  if (rc_channels[RC_AUX1] > 0.55) { // lets call aux1 angle mode for now, you can rename it later

    // will modify setpoints_rpy to be used as the setpoint input to ratePidApply
    attitudePidApply(
      &attitudePid,
      setpoints_rpy, // the angles you to roll/pitch the craft to
      gravity_vector, // used to find how far off the desired angle we are
      setpoints_rpy // output of the attitude pid controller, will modify your setpoint roll pitch and yaw
    );

    // uses the setpoint modified by attitudePidApply as the input setpoint
    ratePidApply(
      &ratePid, 
      setpoints_rpy, // how fast you want to rotate
      gyro_filtered, // filtered gyro data
      pidSums // pidSums gets updated, will be used in the mixer later 
    );
  } else { // acro mode
    ratePidApply(
      &ratePid, 
      setpoints_rpy, // how fast you want to rotate
      gyro_filtered, // filtered gyro data
      pidSums // pidSums gets updated, will be used in the mixer later 
    );
  }

//==========================================================CONTROL MIXER==========================================================//
  float servo_commands[SERVO_COUNT];
  float motor_commands[MOTOR_COUNT];

  for (int servo = 0; servo < SERVO_COUNT; servo++) {
    servo_commands[servo] = 0.0f;
  }

  for (int motor = 0; motor < MOTOR_COUNT; motor++) {
    motor_commands[motor] = 0.0f;
  }

  // Actuator mixing and scaling to PWM values
  // Mixes PID outputs and motor/servo commands -- custom mixing assignments done here
  controlMixer(
    rc_channels, // input used for flight modes and rc passthrough
    pidSums, // input PID output used for stabilization
    motor_commands, // output motor values
    servo_commands // output servo values
  );

  // Throttle cut check
  bool motor_cut = motorCutStatus(rc_channels[RC_THROTTLE]); // Return if we should turn motors off by default motors are turned off when throttle is low edit this function to your liking

//============================================CHECK ARMING/SEND MOTOR AND SERVO SIGNALS=============================================//

  // Get arming status
  bool armed = armedStatus(rc_channels[RC_THROTTLE], rc_channels[RC_ARM]); // Check if you are armed

  if (armed == false) { // DISARM Set all your motor and servo commands to disarm values.
    ratePidReset(&ratePid); // reset PID controller to prevent iterm windup while not flying

    // Set all motors to 0.0
    for (int i = 0; i < MOTOR_COUNT; i++) {
      motor_commands[i] = 0.0f;
    }
    // Normally you don't want to mess with servos as they are safe to have moving during disarm.
    // Allowing servos to move can also help verify and debug that things are working as they should.
  } else if (motor_cut) { // MOTOR CUT Set all motors to 0
    // Set all motors to 0.0
    for (int i = 0; i < MOTOR_COUNT; i++) {
      motor_commands[i] = 0.0f;
    }
  }

  sendServoCommands(
    servoScales, // scales servo commands from degrees to servo units
    servo_commands // servo commands in degrees
  );

  float motor_rpms[MOTOR_COUNT];
  // Command motors
  bool new_rpm = sendMotorCommands( // Sends command pulses to each motor pin using dshot300 protocol
    motor_rpms, // will be updated with RPM info if there is new RPM data, happens when new_rpm = true
    motor_commands // values we want to send to the motors, should be between 0.0 and 1.0
  );

  rpmFilterUpdate(&gyroFilters.rpmFilter, motor_rpms, new_rpm, DT); // Update the RPM filter using the newest RPM measured

  // Regulate loop rate
  maxLoopRate(LOOPRATE); // Will not exceed LOOPRATE
}


//========================================================================================================================//
//                                                      FUNCTIONS                                                         //                           
//========================================================================================================================//


  // DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes pidSums[AXIS_ROLL], pidSums[AXIS_PITCH], and pidSums[AXIS_YAW] computed from the rate PID controller and appropriately mixes them for the desired
   * vehicle configuration. rc_channels can also be used for inputs mixer or switch logic.
   */
void controlMixer(float rc_channels[], float pidSums[], float motor_commands[], float servo_commands[]) {

  float throttle = rc_channels[RC_THROTTLE];

  // Positive roll = roll right
  // Positive pitch = pitch down
  // Positive yaw = yaw left

  // TODO mix inputs to motor commands
  // motor commands should be between 0 and 1
  motor_commands[MOTOR_0] = 0.0f;
  motor_commands[MOTOR_1] = 0.0f;
  motor_commands[MOTOR_2] = 0.0f;
  motor_commands[MOTOR_3] = 0.0f;
  
  // TODO mix inputs to servo commands
  // servos need to be scaled to work properly with the servo scaling that was set earlier
  servo_commands[SERVO_0] = 0.0f;
  servo_commands[SERVO_1] = 0.0f;
  servo_commands[SERVO_2] = 0.0f;
  servo_commands[SERVO_3] = 0.0f;
  servo_commands[SERVO_4] = 0.0f;
  servo_commands[SERVO_5] = 0.0f;
  servo_commands[SERVO_6] = 0.0f;
  servo_commands[SERVO_7] = 0.0f;
  servo_commands[SERVO_8] = 0.0f;
}

// DESCRIPTION: Arming occurs when arm switch is switched from low to high twice in the span of a second.
bool armedStatus(float throttle, float arm_channel) {
  static bool armed = false;
  static int transition_count = 0;
  static float previous_arm = 0.0f;
  
  static elapsedMicros switchingTime = 0;

  if (armed == false) {
    if (switchingTime > SEC_TO_MICROSEC) {
      transition_count = 0;
    }

    switch (transition_count) {
      default:
      case 0:
        if ((arm_channel > 0.95f) && (throttle < 0.01f) && (previous_arm < 0.05)) {
          transition_count = 1;
          switchingTime = 0;
        } else if (throttle > 0.01f) {
          transition_count = 0;
        }
        break;
      case 1:
        if ((arm_channel < 0.05f) && (throttle < 0.01f)) {
          transition_count = 2;
        } else if (throttle > 0.01f) {
          transition_count = 0;
        }
        break;
      case 2:
        if ((arm_channel > 0.95f) && (throttle < 0.01f)) {
          transition_count = 0;
          armed = true;
        } else if (throttle > 0.01f) {
          transition_count = 0;
        }
        break;
    }

  } else {
    if (arm_channel > 0.95f) {
      armed = true;
    } else {
      armed = false;
    }
  }
  previous_arm = arm_channel;

  return armed;
}

float motorCutStatus(float throttle) {
  // DESCRIPTION: Return true if we should set motors to 0.0
  /*
      Monitors the state of throttle. If throttle is low return true.

      Modify this to your liking. Potentially add a switch instead of throttle to do this.
  */
  if (throttle < 0.01f) {
    return true;
  } else {
    return false;
  }
}

// DESCRIPTION: Regulate main loop rate to specified frequency in Hz
/*
 * It's good to operate at a constant loop rate for optimal control and filtering. Interrupt routines running in the
 * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
 * the correct time has passed since the start of the current loop for the desired loop rate in Hz.
 */
void maxLoopRate(int freq) {
  static elapsedMicros loopTime = 0;

  unsigned long waitTimeMicroseconds = 1.0f / freq * SEC_TO_MICROSEC;
  
  // Sit in loop until appropriate time has passed
  while (waitTimeMicroseconds > loopTime) {}
  loopTime = 0;
}

// DESCRIPTION: Blink LED on board to indicate main loop is running
/*
 * It looks cool.
 */
void loopBlink(unsigned long current_time, float blink_seconds) {
  static unsigned long blink_counter = 0;
  static unsigned long blink_delay = 0;
  static bool blinkAlternate = 1;

  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate); // Pin 13 is built in LED
    
    unsigned long blink_microseconds = blink_seconds * SEC_TO_MICROSEC;

    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = blink_microseconds / 3; // on for 1/3rd of the time
    } else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = blink_microseconds * 2 / 3; // off for 2/3rd of the time
    }
  }
}

void setupBlink(int numBlinks,int upTime, int downTime) {
  // DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j <= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}
