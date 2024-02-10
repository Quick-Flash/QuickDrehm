// Teensy Flight Controller - QuickDrehm
// Authors: Kevin Plaizer, Nicholas Rehm
// Version: Alpha 1.0

//========================================================================================================================//

// HELPER FUNCTIONS
// These include math functions and other helpful functions

// Creates a curve great for adding expo to your sticks in angle mode or rate mode
// Link to a graph of this curve https://www.desmos.com/calculator/82mrlgysot
float rcCurve(float input, float expo, float max_value) {
  return (expo * input * input * input * input * sign(input) + (1.0f - expo) * input) * max_value;
}

float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq) {
  // DESCRIPTION: Linearly fades a float type variable between min and max bounds based on desired high or low state and time
  /*  
   *  Takes in a float variable, desired minimum and maximum bounds, fade time, high or low desired state, and the loop frequency 
   *  and linearly interpolates that param variable between the maximum and minimum bounds. This function can be called in controlMixer()
   *  and high/low states can be determined by monitoring the state of an auxillarly radio channel. For example, if channel_6_pwm is being 
   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical 
   *  statements in order to fade controller gains, for example between the two dynamic configurations. The 'state' (1 or 0) can be used
   *  to designate the two final options for that control gain based on the dynamic configuration assignment to the auxillary radio channel.
   *  
   */
  float diffParam = (param_max - param_min)/(fadeTime*loopFreq); //Difference to add or subtract from param for each loop iteration for desired fadeTime

  if (state == 1) { // Maximum param bound desired, increase param by diffParam for each loop iteration
    param = param + diffParam;
  }
  else if (state == 0) { // Minimum param bound desired, decrease param by diffParam for each loop iteration
    param = param - diffParam;
  }

  param = constrain(param, param_min, param_max); // Constrain param within max bounds
  
  return param;
}

float floatFaderLinear2(float param, float param_des, float param_lower, float param_upper, float fadeTime_up, float fadeTime_down, int loopFreq) {
  // DESCRIPTION: Linearly fades a float type variable from its current value to the desired value, up or down
  /*  
   *  Takes in a float variable to be modified, desired new position, upper value, lower value, fade time, and the loop frequency 
   *  and linearly fades that param variable up or down to the desired value. This function can be called in controlMixer()
   *  to fade up or down between flight modes monitored by an auxillary radio channel. For example, if channel_6_pwm is being 
   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical 
   *  statements in order to fade controller gains, for example between the two dynamic configurations. 
   *  
   */
  if (param > param_des) { // Need to fade down to get to desired
    float diffParam = (param_upper - param_des)/(fadeTime_down*loopFreq);
    param = param - diffParam;
  }
  else if (param < param_des) { // Need to fade up to get to desired
    float diffParam = (param_des - param_lower)/(fadeTime_up*loopFreq);
    param = param + diffParam;
  }

  param = constrain(param, param_lower, param_upper); //Constrain param within max bounds
  
  return param;
}

// Initializes the rangeScaler struct
void rangeScalerInit(rangeScaler_t *scaler, float input_min, float input_max, float desired_min, float desired_max) {
  float range_desired = desired_max - desired_min;
  float range_input = input_max - input_min;

  scaler->scaleFactor = range_desired / range_input;
  scaler->offset = desired_min - input_min * scaler->scaleFactor;
}

// Scales an input
float rangeScalerApply(rangeScaler_t scaler, float input) {
  return scaler.scaleFactor * input + scaler.offset;
}

void boundedRangeScalerInit(boundedRangeScaler_t *scaler, float input_min, float input_mid, float input_max, float desired_min, float desired_mid, float desired_max) {
  midpointRangeScalerInit(&scaler->scaler, input_min, input_mid, input_max, desired_min, desired_mid, desired_max, 0.0f);
  scaler->min = desired_min;
  scaler->max = desired_max;
}

float boundedRangeScalerApply(boundedRangeScaler_t scaler, float input) {
  float output = midpointRangeScalerApply(scaler.scaler, input);
  return constrain(output, scaler.min, scaler.max);
}

// Initializes the midpointRangeScaler struct
void midpointRangeScalerInit(
  midpointRangeScaler_t *scaler, 
  float input_min, float input_mid, float input_max, 
  float desired_min, float desired_mid, float desired_max, 
  float deadband
  ) {
  rangeScalerInit(&scaler->low_range, input_min, input_mid - deadband, desired_min, desired_mid);
  rangeScalerInit(&scaler->high_range, input_mid + deadband, input_max, desired_mid, desired_max);
  scaler->deadband_low = input_mid - deadband;
  scaler->deadband_high = input_mid + deadband;
  scaler->desired_mid = desired_mid;
}

// Scales an input with a midpoint
float midpointRangeScalerApply(midpointRangeScaler_t scaler, float input) {
  if ((input > scaler.deadband_low) && (input < scaler.deadband_high)) {
    return scaler.desired_mid;
  } else if (input < scaler.deadband_low) {
    return rangeScalerApply(scaler.low_range, input);
  } else {
    return rangeScalerApply(scaler.high_range, input);
  }
}

// returns a 1.0 for positive numbers and a -1.0 for negative numbers
float sign(float input) {
  if (input > 0.0f) {
    return 1.0f;
  } else {
    return -1.0f;
  }
}

// DESCRIPTION: Takes roll and pitch and returns a desired acc 
/*  
 *  Takes in a float variable, setpoint roll and pitch angles, these must be between -90 to 90 and in units of degrees. This function creates the setpoint that should be used in the attitude 
 *  PID controller. This setpoint is a normalized 3d vector that is the desired acc value we want to acheive.
 */
void attitudeAnglesToAttitudeSetpoint(float roll_angle, float pitch_angle, float attitude_setpoint[]) {
  float roll_angle_rad = roll_angle * DEG_TO_RAD;
  float pitch_angle_rad = pitch_angle * DEG_TO_RAD;

  float vector_roll = sinf(roll_angle_rad);
  float vector_pitch = sinf(pitch_angle_rad);
  
  float vector_z = cosf(roll_angle_rad) * cosf(pitch_angle_rad);

  float magnitude = vector_roll * vector_roll + vector_pitch * vector_pitch;
  float scaler = 1.0f;
  if (magnitude != 0.0f) {
    scaler = invSqrt(magnitude / (1.0f - vector_z * vector_z));
  }

  float vector_x = -vector_pitch * scaler;
  float vector_y = vector_roll * scaler;

  attitude_setpoint[AXIS_X] = vector_x;
  attitude_setpoint[AXIS_Y] = vector_y;
  attitude_setpoint[AXIS_Z] = vector_z; 
}

// returns 1.0 / squareRoot(x)
float invSqrt(float x) {
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  /*
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
  */
  return 1.0f / sqrtf(x); // Teensy can run sqrtf() just as fast as it divides, so no real speed penalty
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
