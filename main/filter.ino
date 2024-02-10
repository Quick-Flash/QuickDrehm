// Teensy Flight Controller - QuickDrehm
// Authors: Kevin Plaizer
// Version: Alpha 1.0

//========================================================================================================================//
//                                                      FILTER CODE                                                       //
//========================================================================================================================//

// Struct defines live in global_defines.h

// TODO use slew filter/s to smoothly transition flight modes
void slewFilterInit(slewFilter_t *filter, float change_per_second, float dT) {
  filter->state = 0.0;
  filter->max_delta = change_per_second * dT;
}

float slewFilterApply(slewFilter_t *filter, float input) {
  float delta = input - filter->state;
  if (abs(delta) > filter->max_delta) {
    filter->state += filter->max_delta * sign(delta);
  } else {
    filter->state = input;
  }

  return filter->state;
}

// PT1 Low Pass filter

float pt1FilterGain(float f_cut, float dT) {
  float omega = 2.0f * M_PI * f_cut * dT;
  float cos = 1.0f - cosf(omega);
  return sqrtf(cos * (cos + 2.0f)) - cos;
}

void pt1FilterInit(pt1Filter_t *filter, float cutoff, float dT) {
  filter->state = 0.0f;
  filter->k = pt1FilterGain(cutoff, dT);
}

float pt1FilterApply(pt1Filter_t *filter, float input) {
  filter->state = filter->state + filter->k * (input - filter->state);
  return filter->state;
}

// PT2 Low Pass filter

float pt2FilterGain(float f_cut, float dT) {
  // PTn cutoff correction = 1 / sqrt(2^(1/n) - 1)
  #define CUTOFF_CORRECTION_PT2 1.553773974f

  // shift f_cut to satisfy -3dB cutoff condition
  return pt1FilterGain(f_cut * CUTOFF_CORRECTION_PT2, dT);
}

void pt2FilterInit(pt2Filter_t *filter, float cutoff, float dT) {
  filter->state = 0.0f;
  filter->state1 = 0.0f;
  filter->k = pt2FilterGain(cutoff, dT);
}

float pt2FilterApply(pt2Filter_t *filter, float input) {
  filter->state1 = filter->state1 + filter->k * (input - filter->state1);
  filter->state = filter->state + filter->k * (filter->state1 - filter->state);
  return filter->state;
}

// PT3 Low Pass filter

float pt3FilterGain(float f_cut, float dT) {
  // PTn cutoff correction = 1 / sqrt(2^(1/n) - 1)
  #define CUTOFF_CORRECTION_PT3 1.961459177f

  // shift f_cut to satisfy -3dB cutoff condition
  return pt1FilterGain(f_cut * CUTOFF_CORRECTION_PT3, dT);
}

void pt3FilterInit(pt3Filter_t *filter, float cutoff, float dT) {
  filter->state = 0.0f;
  filter->state1 = 0.0f;
  filter->state2 = 0.0f;
  filter->k = pt3FilterGain(cutoff, dT);
}

float pt3FilterApply(pt3Filter_t *filter, float input) {
  filter->state1 = filter->state1 + filter->k * (input - filter->state1);
  filter->state2 = filter->state2 + filter->k * (filter->state1 - filter->state2);
  filter->state = filter->state + filter->k * (filter->state2 - filter->state);
  return filter->state;
}

// Notch filter

void notchFilterUpdate(notchFilter_t *filter, float filterFreq, float q, float weight, float dT) {
  // setup variables
  const float omega = 2.0f * M_PI * filterFreq * dT;
  const float sn = sinf(omega);
  const float cs = cosf(omega);
  const float alpha = sn / (2.0f * q);
  const float a0_inv = 1.0 / (1.0f + alpha);

  filter->b0 = 1.0f * a0_inv;
  filter->b1 = -2.0f * cs * a0_inv;
  filter->b2 = filter->b0;
  filter->a1 = filter->b1;
  filter->a2 = (1.0f - alpha) * a0_inv;

  filter->weight = weight;
}

void notchFilterInit(notchFilter_t *filter, float filterFreq, float q, float dT) {
  notchFilterUpdate(filter, filterFreq, q, 1.0, dT);

  // zero initial samples
  filter->x1 = filter->x2 = 0.0f;
  filter->y1 = filter->y2 = 0.0f;
}

float notchFilterApply(notchFilter_t *filter, float input) {
  /* compute result */
  const float result = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 - filter->a2 * filter->y2;

  /* shift x1 to x2, input to x1 */
  filter->x2 = filter->x1;
  filter->x1 = input;

  /* shift y1 to y2, result to y1 */
  filter->y2 = filter->y1;
  filter->y1 = result;

  return filter->weight * result + (1.0 - filter->weight) * input;
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
