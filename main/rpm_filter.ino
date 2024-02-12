// Teensy Flight Controller - QuickDrehm
// Authors: Kevin Plaizer
// Version: Alpha 1.0

//========================================================================================================================//
//                                                  RPM FILTER CODE                                                       //
//========================================================================================================================//

// Struct defines live in global_defines.h

void rpmFilterInit(rpmFilter_t *rpmFilter, float minHz, float fadeRange, float q, float dT) {
  rpmFilter->q = q;
  rpmFilter->minHz = minHz;
  rpmFilter->maxHz = (0.5f / dT) * 0.9f; // only allowed to set a bit below nyquist
  rpmFilter->fadeRange = fadeRange;
  
  for (int axis = 0; axis < AXIS_COUNT; axis++) {
    for (int motor = 0; motor < MOTOR_COUNT; motor++) {
      notchFilterInit(&rpmFilter->notch[axis][motor], rpmFilter->maxHz / 2.0f, q, dT);
    }
  }

  for (int motor = 0; motor < MOTOR_COUNT; motor++) {
    pt1FilterInit(&rpmFilter->cutoffFilter[motor], 135.0,  dT);
  }
}

void rpmFilterUpdate(rpmFilter_t *rpmFilter, float motor_rpms[], bool new_rpm, float dT) {
  if (new_rpm == false) {
    return;
  }


  for (int motor = 0; motor < MOTOR_COUNT; motor++) { // update all 4 motors
    float motor_hz = motor_rpms[motor] / 60.0f; // rpm is 60 times greater than its frequency
    if (motor_hz > 0.0f) {
      motor_hz = pt1FilterApply(&rpmFilter->cutoffFilter[motor], motor_hz);
      motor_hz = constrain(motor_hz, rpmFilter->minHz, rpmFilter->maxHz);

      float weight = 1.0f;
      float margin = motor_hz - rpmFilter->minHz;
      if (margin < rpmFilter->fadeRange) {
        weight = margin / rpmFilter->fadeRange;
      }

      notchFilterUpdate(&rpmFilter->notch[AXIS_ROLL][motor], motor_hz, rpmFilter->q, weight, dT); // update roll
      notchFilterUpdate(&rpmFilter->notch[AXIS_PITCH][motor], motor_hz, rpmFilter->q, weight, dT); // update pitch
      notchFilterUpdate(&rpmFilter->notch[AXIS_YAW][motor], motor_hz, rpmFilter->q, weight, dT); // update yaw

      // for (int axis = 1; axis < AXIS_COUNT; axis++) { // update pitch and yaw
      //   rpmFilter->notch[axis][motor].b0 = rpmFilter->notch[AXIS_ROLL][motor].b0;
      //   rpmFilter->notch[axis][motor].b1 = rpmFilter->notch[AXIS_ROLL][motor].b1;
      //   rpmFilter->notch[axis][motor].b2 = rpmFilter->notch[AXIS_ROLL][motor].b2;
      //   rpmFilter->notch[axis][motor].a1 = rpmFilter->notch[AXIS_ROLL][motor].a1;
      //   rpmFilter->notch[axis][motor].a2 = rpmFilter->notch[AXIS_ROLL][motor].a2;
      //   rpmFilter->notch[axis][motor].weight = rpmFilter->notch[AXIS_ROLL][motor].weight;
      // }
    }
  }
}

float rpmFilterApply(rpmFilter_t *rpmFilter, int axis, float input) {
  for (int motor = 0; motor < MOTOR_COUNT; motor++) { // apply filters, one for each motor
    input = notchFilterApply(&rpmFilter->notch[axis][motor], input);
  }

  return input;
}
