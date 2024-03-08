// Teensy Flight Controller - QuickDrehm
// Authors: Kevin Plaizer
// Version: Alpha 1.0

//========================================================================================================================//
//                                                     PID CODE                                                           //
//========================================================================================================================//

// Struct defines live in global_defines.h

/*
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
*/

//======================================================ATTITUDE PID=======================================================//

// TODO tune the attitude pid controller
void attitudePidInit(attitudePid_t *pid) {
  pid->kp = 275.0f;
  pid->kd = 1.0f;

  pid->previous_error_or_measurement[0] = 0.0f;
  pid->previous_error_or_measurement[1] = 0.0f;
  pid->previous_error_or_measurement[2] = 0.0f;

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

// finds the 3d cross product between the attitude_setpoint and the measured gravity_vector
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
  pid->kp[AXIS_YAW] = PTERM_SCALE * yaw_p;
  pid->ki[AXIS_YAW] = ITERM_SCALE * yaw_i;
  pid->kd[AXIS_YAW] = DTERM_SCALE * yaw_d;
  pid->kff[AXIS_YAW] = FFTERM_SCALE * yaw_ff;
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
