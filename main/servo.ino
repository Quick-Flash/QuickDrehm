// Teensy Flight Controller - QuickDrehm
// Authors: Kevin Plaizer
// Version: Alpha 1.0

//========================================================================================================================//
//                                                      SERVO SETUP                                                       //
//========================================================================================================================//

const int servoPins[MAX_SERVO_COUNT] = {0, 1, 5, 6, 10, 11, 12, 14, 15}; // Weird servo ordering due to hardware requirements of dshot

PWMServo servo[SERVO_COUNT];  // Create servo objects to control a servo or ESC with PWM

// TODO set up the servo scaling as needed for each servo
void initServoScales(boundedRangeScaler_t servoScales[]) {
  servoScalerInitHelper(
    servoScales[SERVO_0], 
    90.0f, -90.0f, // Servo control surface angle min and max, use a higher then lower number to swap directions
    0.0f, 0.5f, 1.0f // change these if you want to limit servo movement, only takes a range of 0.0-1.0
  );

  servoScalerInitHelper(
    servoScales[SERVO_1], 
    90.0f, -90.0f, // Servo control surface angle min and max, use a higher then lower number to swap directions
    0.0f, 0.5f, 1.0f // Servo output min, mid, and max. Modify is servo isn't centering or moving to far. Only takes a range of 0.0-1.0
  );

  servoScalerInitHelper(
    servoScales[SERVO_2], 
    90.0f, -90.0f, // Servo control surface angle min and max, use a higher then lower number to swap directions
    0.0f, 0.5f, 1.0f // Servo output min, mid, and max. Modify is servo isn't centering or moving to far. Only takes a range of 0.0-1.0
  );

  servoScalerInitHelper(
    servoScales[SERVO_3],
    90.0f, -90.0f, // Servo control surface angle min and max, use a higher then lower number to swap directions
    0.0f, 0.5f, 1.0f // Servo output min, mid, and max. Modify is servo isn't centering or moving to far. Only takes a range of 0.0-1.0
  );
  
  servoScalerInitHelper(
    servoScales[SERVO_4], 
    90.0f, -90.0f, // Servo control surface angle min and max, use a higher then lower number to swap directions
    0.0f, 0.5f, 1.0f // Servo output min, mid, and max. Modify is servo isn't centering or moving to far. Only takes a range of 0.0-1.0
  ); 

  servoScalerInitHelper(
    servoScales[SERVO_5], 
    90.0f, -90.0f, // Servo control surface angle min and max, use a higher then lower number to swap directions
    0.0f, 0.5f, 1.0f // Servo output min, mid, and max. Modify is servo isn't centering or moving to far. Only takes a range of 0.0-1.0
  );

  servoScalerInitHelper(
    servoScales[SERVO_6], 
    90.0f, -90.0f, // Servo control surface angle min and max, use a higher then lower number to swap directions
    0.0f, 0.5f, 1.0f // Servo output min, mid, and max. Modify is servo isn't centering or moving to far. Only takes a range of 0.0-1.0
  );

  servoScalerInitHelper(
    servoScales[SERVO_7], 
    90.0f, -90.0f, // Servo control surface angle min and max, use a higher then lower number to swap directions
    0.0f, 0.5f, 1.0f // Servo output min, mid, and max. Modify is servo isn't centering or moving to far. Only takes a range of 0.0-1.0
  ); 

  servoScalerInitHelper(
    servoScales[SERVO_8], 
    90.0f, -90.0f, // Servo control surface angle min and max, use a higher then lower number to swap directions
    0.0f, 0.5f, 1.0f // Servo output min, mid, and max. Modify is servo isn't centering or moving to far. Only takes a range of 0.0-1.0
  );
}

void initServos(boundedRangeScaler_t servoScales[]) {
  for (int i = 0; i < SERVO_COUNT; i++) {
    // these values should work for most servos
    servo[i].attach(servoPins[i], 500, 2500); // Pin, min PWM value, max PWM value
  }

  initServoScales(servoScales);
}

void servoScalerInitHelper(boundedRangeScaler_t &servoScales, float angle_min, float angle_max, float output_min, float output_mid, float output_max) {
    boundedRangeScalerInit(
    &servoScales, 
    angle_min, 0.0f, angle_max, // input min and max set to servo/control surface angles
    output_min * 180.0f, output_mid * 180.0f, output_max * 180.0f // output min and max don't change as this relates to servo library code
  );
}

void sendServoCommands(boundedRangeScaler_t servoScales[], float servo_commands[]) {
  for (int i = 0; i < SERVO_COUNT; i++) {
    float scaled_command = boundedRangeScalerApply(servoScales[i], servo_commands[i]);

    int command = constrain(scaled_command, 0, 180);
    servo[i].write(command); // Writes PWM value to servo object takes 0-180
  }
}
