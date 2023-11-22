// Teensy Flight Controller - QuickDrehm
// Authors: Kevin Plaizer
// Version: Alpha 1.0

//========================================================================================================================//
//                                                      MOTOR SETUP                                                       //
//========================================================================================================================//

#include "src/BIDIRECTIONAL_DSHOT/bidirectional_dshot.h"

#define POLE_PAIRS (POLE_COUNT / 2) // used when calculating RPM

rangeScaler_t motor_scaling;
DshotManager dshot;

#define FEEDBACK_DIVISOR 2 // How many loops we should run before getting rpm data
int32_t loop_count = 0; // 32 bit as its faster to run but uses more memory, we have plenty of memory

// sends new commands to motors and updates the motor rpm
// returns true if there is new rpm data
bool sendMotorCommands(float motor_rpms[], float motor_commands[]) {
  bool new_rpm = false;
  loop_count += 1;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    int scaled_motor;
    if (motor_commands[i] <= 0.0f) {
      dshot.set_throttle_esc(i, 0); // This is the motor stop command
    } else {
      scaled_motor = rangeScalerApply(motor_scaling, motor_commands[i]);
      scaled_motor = constrain(scaled_motor, 48, 2047);
      dshot.set_throttle_esc(i, scaled_motor);
    }
  }

  if (imu_count > FEEDBACK_DIVISOR) {
    new_rpm = true;
    loop_count = 0;
    for (int i = 0; i < MOTOR_COUNT; i++) {
      float erpm = dshot.average_eRPM(i);
      if (erpm == 0xffffffff) { // means that there were no RPM packets to read
        motor_rpms[i] = 0; // use 0 to mean that we don't use the rpm value
      } else {
        motor_rpms[i] = erpm / POLE_PAIRS;
      }
    }
  }

  return new_rpm;
}

void initMotors() {
  initMotorScaling();
  delay(100); // small delay
  dshot.start_tx(); // Startup IntervalTimer to trigger tx continuously

  for (int i = 0; i < MOTOR_COUNT; i++) {
    dshot.set_throttle_esc(i, 0); // 0 is the stop motor command
  }

  for (int k = 0; k < 500; k++) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
      dshot.set_throttle_esc(i, 0); // 0 is the stop motor command
    }
    maxLoopRate(LOOPRATE); // Will not exceed LOOPRATE
  }
}

// TODO edit the 2047.0f to 120.0f or so when wanting to safely test motors, will limit max motor output
// once finished testing set to 2047.0f again to get full throttle from motors
void initMotorScaling() {
  rangeScalerInit(
    &motor_scaling,
    0.0f, 1.0f, // input min and max
    48.0f, 2047.0f // 48 is the slowest esc command, 2047 is the highest
  );
}
