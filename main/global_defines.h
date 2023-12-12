// Teensy Flight Controller - QuickDrehm
// Authors: Kevin Plaizer
// Version: Alpha 1.0

//========================================================================================================================//
// DEFINITIONS that need to be shared between files
// These are meant to be constant values that are set once to setup things and never touched again

//========================================================================================================================//
//                                                    HELPFUL CONSTANTS                                                   //
//========================================================================================================================//
// don't modify these

#define SEC_TO_MICROSEC 1000000

#define LOOPRATE 4000.0f // leave at 4k unless you measure looprate as less than 4k, then try 2k
#define DT (1.0f / LOOPRATE)
#define ACC_DIVIDER 4 // this is how many times slower than looprate the ACC runs
#define ACC_LOOPRATE (LOOPRATE / ACC_DIVIDER)
#define ACC_DT (1.0f / ACC_LOOPRATE)

#define AXIS_COUNT 3

#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2

#define AXIS_ROLL 0
#define AXIS_PITCH 1
#define AXIS_YAW 2

//========================================================================================================================//
//                                                 HARDWARE SETUP DEFINES                                                 //
//========================================================================================================================//


//======================================================RADIO SETUP=======================================================//

#define RC_CHANNEL_COUNT 12 // 12 is the default channel number for ELRS I guess setting this number higher will cause failsafes on ELRS 12 is plenty though
// Uncomment only one receiver type
#define USE_SBUS_RX // if using ELRS set SBUS Failsafe to Last Position otherwise there will be failsafe problems
// #define USE_DSM_RX

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


//=======================================================IMU SETUP========================================================//

typedef enum {
  ROT_0_DEG = 0,
  ROT_90_DEG = 1,
  ROT_180_DEG = 2,
  ROT_270_DEG = 3,
} axisRotation;

// TODO change this depending on your imu rotation
axisRotation imuRotation[AXIS_COUNT] = {ROT_0_DEG, ROT_0_DEG, ROT_180_DEG}; // roll, pitch, yaw rotation

// TODO Run the function calculateGyroBias() in setup() to find these values.
float gyro_bias[AXIS_COUNT] = {
  0.0f, // roll
  0.0f, // pitch
  0.0f, // yaw
};
float acc_bias[AXIS_COUNT] = {
  0.0f, // x
  0.0f, // y
  0.0f, // z
};


//======================================================SERVO SETUP=======================================================//

#define MAX_SERVO_COUNT 9 // don't change this
#define SERVO_COUNT 9 // no real need to change

// TODO rename to match servo function, IE SERVO_FRONT_LEFT
// pin 0
#define SERVO_0 0 // rename to match what the servo does
// pin 1
#define SERVO_1 1 // rename to match what the servo does
// pin 5
#define SERVO_2 2 // rename to match what the servo does
// pin 6
#define SERVO_3 3 // rename to match what the servo does
// pin 10
#define SERVO_4 4 // rename to match what the servo does
// pin 11
#define SERVO_5 5 // rename to match what the servo does
// pin 12
#define SERVO_6 6 // rename to match what the servo does
// pin 14
#define SERVO_7 7 // rename to match what the servo does
// pin 15
#define SERVO_8 8 // rename to match what the servo does

//======================================================MOTOR SETUP=======================================================//

#define MAX_MOTOR_COUNT 6 // don't change this
#define MOTOR_COUNT 4 // set to your motor count

#define POLE_COUNT 12 // the number of magnets in the motor, must be accurate to get accurate RPM data

// TODO rename to match motor function, IE MOTOR_FRONT_LEFT
#define MOTOR_0 0 // rename to match where the motor is or its function
#define MOTOR_1 1 // rename to match where the motor is or its function
#define MOTOR_2 2 // rename to match where the motor is or its function
#define MOTOR_3 3 // rename to match where the motor is or its function
#define MOTOR_4 4 // rename to match where the motor is or its function
#define MOTOR_5 5 // rename to match where the motor is or its function

//========================================================================================================================//
//                                                     DEFINED STRUCTS                                                    //
//========================================================================================================================//
// Used in helper.ino to scale ranges between an input range and an output range
typedef struct rangeScaler_s {
  float scaleFactor;
  float offset;
} rangeScaler_t;

typedef struct boundedRangeScaler_s {
  rangeScaler_t scaler;
  float min;
  float max;
} boundedRangeScaler_t;

// Used in helper.ino to scale ranges between an input range with midpoint and an output range with midpoint
typedef struct midpointRangeScaler_s {
  rangeScaler_t low_range;
  rangeScaler_t high_range;
  float deadband_low;
  float deadband_high;
  float desired_mid;
} midpointRangeScaler_t;

// filter structs
typedef struct pt1Filter_s {
  float state;
  float k;
} pt1Filter_t;

typedef struct pt2Filter_s {
  float state;
  float state1;
  float k;
} pt2Filter_t;

typedef struct pt3Filter_s {
  float state;
  float state1;
  float state2;
  float k;
} pt3Filter_t;

// should be used to slowly move from one value to another or limit change of an input
typedef struct slewFilter_s {
  float state;
  float max_delta;
} slewFilter_t;

typedef struct notchFilter_s {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
    float weight;
} notchFilter_t;

typedef struct rpmFilter_s {
  notchFilter_t notch[AXIS_COUNT][MOTOR_COUNT];
  pt1Filter_t cutoffFilter[MOTOR_COUNT];
  float minHz;
  float maxHz;
  float fadeRange;
  float q;
} rpmFilter_t;

//============================================DYNAMIC NOTCH STUFF CAN BE IGNORED=======================================================//
// conversion from time to frequency domain and filtering wizardry happens here
#define DYN_NOTCH_COUNT_MAX 5
#define SDFT_SAMPLE_SIZE 72
#define SDFT_BIN_COUNT   (SDFT_SAMPLE_SIZE / 2)

typedef struct complex_s {
  float re;
  float im;
} complex_t;

typedef struct sdft_s {
  int idx;                           // circular buffer index
  int startBin;
  int endBin;
  int batchSize;
  int numBatches;
  float rPowerN;  // SDFT_R to the power of SDFT_SAMPLE_SIZE

  float samples[SDFT_SAMPLE_SIZE];   // circular buffer
  complex_t data[SDFT_BIN_COUNT];    // complex frequency spectrum
  complex_t twiddle[SDFT_BIN_COUNT]; // how we "twiddle" the samples
} sdft_t;

typedef enum {
  STEP_WINDOW = 0,
  STEP_DETECT_PEAKS = 1,
  STEP_CALC_FREQUENCIES = 2,
  STEP_UPDATE_FILTERS = 3,
  STEP_COUNT = 4
} sdftStep_e;

typedef struct sdftPeak_s {
  int bin;
  float value;
} sdftPeak_t;

typedef struct dynNotchState_s {
  // state machine step information
  int tick;
  int step;
  int axis;
} dynNotchState_t;

typedef struct dynNotch_s {
  // notch settings and count
  float q;
  float minHz;
  float maxHz;
  int count;

  float centerFreq[AXIS_COUNT][DYN_NOTCH_COUNT_MAX];

  notchFilter_t notch[AXIS_COUNT][DYN_NOTCH_COUNT_MAX];

  // state stuff // some of these may not be needed, lol bad bf code
  int   sampleIndex;
  int   sampleCount;
  float sampleCountRcp;
  float sampleAccumulator[AXIS_COUNT];
  float sampleAvg[AXIS_COUNT];
  dynNotchState_t state;
  sdft_t  sdft[AXIS_COUNT];
  sdftPeak_t  peaks[DYN_NOTCH_COUNT_MAX];
  float sdftData[SDFT_BIN_COUNT];
  float   sdftResolutionHz;
  int     sdftStartBin;
  int     sdftEndBin;
  float   sdftNoiseThreshold;
  float   pt1LooptimeS;
} dynNotch_t;
