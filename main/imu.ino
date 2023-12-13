// Teensy Flight Controller - QuickDrehm
// Authors: Kevin Plaizer
// Version: Alpha 1.0

//========================================================================================================================//
//                                                      IMU DEFINES                                                       //
//========================================================================================================================//
#include "src/MPU6050/MPU6050.h"
MPU6050 mpu6050;


// Uncomment only one full scale gyro range (deg/sec)
// #define GYRO_250DPS // not recommended
// #define GYRO_500DPS // only recommended if not flying acro or agressive
// #define GYRO_1000DPS // recommended only if mild acro
#define GYRO_2000DPS // Default

// Uncomment only one full scale accelerometer range (G's)
// #define ACCEL_2G
// #define ACCEL_4G
// #define ACCEL_8G
#define ACCEL_16G // Default

// Setup gyro and accel full scale value selection and scale factor
#define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
#define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16

  
#ifdef GYRO_250DPS
  #define GYRO_SCALE GYRO_FS_SEL_250
  #define GYRO_SCALE_FACTOR 131.0f
#elif defined GYRO_500DPS
  #define GYRO_SCALE GYRO_FS_SEL_500
  #define GYRO_SCALE_FACTOR 65.5f
#elif defined GYRO_1000DPS
  #define GYRO_SCALE GYRO_FS_SEL_1000
  #define GYRO_SCALE_FACTOR 32.8f
#elif defined GYRO_2000DPS
  #define GYRO_SCALE GYRO_FS_SEL_2000
  #define GYRO_SCALE_FACTOR 16.4f
#endif

#ifdef ACCEL_2G
  #define ACCEL_SCALE ACCEL_FS_SEL_2
  #define ACCEL_SCALE_FACTOR 16384.0f
#elif defined ACCEL_4G
  #define ACCEL_SCALE ACCEL_FS_SEL_4
  #define ACCEL_SCALE_FACTOR 8192.0f
#elif defined ACCEL_8G
  #define ACCEL_SCALE ACCEL_FS_SEL_8
  #define ACCEL_SCALE_FACTOR 4096.0f
#elif defined ACCEL_16G
  #define ACCEL_SCALE ACCEL_FS_SEL_16
  #define ACCEL_SCALE_FACTOR 2048.0f
#endif

//========================================================================================================================//
//                                                    SENSOR FUNCTIONS                                                    //
//========================================================================================================================//

// SENSOR FUNCTIONS
// These include functions for using and setup of sensors

// imu sensor rotation
void vectorRotation(float dest[], axisRotation rotation[]) {
  const float x = dest[AXIS_X];
  const float y = dest[AXIS_Y];
  const float z = dest[AXIS_Z];

  int cos[AXIS_COUNT];
  int sin[AXIS_COUNT];

  for (int i = 0; i < AXIS_COUNT; i++) {
  switch (rotation[i]) {
    default:
    case ROT_0_DEG:
      cos[i] = 1;
      sin[i] = 0;
      break;
    case ROT_90_DEG:
      cos[i] = 0;
      sin[i] = 1;
      break;
    case ROT_180_DEG:
      cos[i] = -1;
      sin[i] = 0;
      break;
    case ROT_270_DEG:
      cos[i] = 0;
      sin[i] = -1;
      break;
    }
  }

  // Auxiliary variables to avoid repeated arithmetic
  int cosZcosX = cos[AXIS_Z] * cos[AXIS_X];
  int sinZcosX = sin[AXIS_Z] * cos[AXIS_X];
  int cosZsinX = cos[AXIS_Z] * sin[AXIS_X];
  int sinZsinX = sin[AXIS_Z] * sin[AXIS_X];

  int rotMatrix [AXIS_COUNT][AXIS_COUNT];
  rotMatrix[0][AXIS_X] = cos[AXIS_Z] * cos[AXIS_Y];
  rotMatrix[0][AXIS_Y] = -cos[AXIS_Y] * sin[AXIS_Z];
  rotMatrix[0][AXIS_Z] = sin[AXIS_Y];
  rotMatrix[1][AXIS_X] = sinZcosX + (cosZsinX * sin[AXIS_Y]);
  rotMatrix[1][AXIS_Y] = cosZcosX - (sinZsinX * sin[AXIS_Y]);
  rotMatrix[1][AXIS_Z] = -sin[AXIS_X] * cos[AXIS_Y];
  rotMatrix[2][AXIS_X] = sinZsinX - (cosZcosX * sin[AXIS_Y]);
  rotMatrix[2][AXIS_Y] = cosZsinX + (sinZcosX * sin[AXIS_Y]);
  rotMatrix[2][AXIS_Z] = cos[AXIS_Y] * cos[AXIS_X];

  dest[AXIS_X] = rotMatrix[0][AXIS_X] * x + rotMatrix[1][AXIS_X] * y + rotMatrix[2][AXIS_X] * z;
  dest[AXIS_Y] = rotMatrix[0][AXIS_Y] * x + rotMatrix[1][AXIS_Y] * y + rotMatrix[2][AXIS_Y] * z;
  dest[AXIS_Z] = rotMatrix[0][AXIS_Z] * x + rotMatrix[1][AXIS_Z] * y + rotMatrix[2][AXIS_Z] * z;

}

void calculateGyroBias() {
  // DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This helps eliminate drift in the
   * measurement. 
   */
  int16_t unscaled_acc[AXIS_COUNT];
  int16_t unscaled_gyro[AXIS_COUNT];

  float gyro_bias[AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
  float acc_bias[AXIS_COUNT] = {0.0f, 0.0f, 0.0f};

  //Read IMU values 12000 times
  int c = 0;
  while (c < LOOPRATE * 10) { // 10 seconds of averaging
    mpu6050.getMotion6(
      &unscaled_acc[AXIS_X], &unscaled_acc[AXIS_Y], &unscaled_acc[AXIS_Z], 
      &unscaled_gyro[AXIS_ROLL], &unscaled_gyro[AXIS_PITCH], &unscaled_gyro[AXIS_YAW]
    );

    
    for (int i = 0; i < AXIS_COUNT; i++) {
      float acc  = unscaled_acc[i] / ACCEL_SCALE_FACTOR;
      float gyro = unscaled_gyro[i] / GYRO_SCALE_FACTOR;
          
      // Sum all readings
      acc_bias[i] += acc;
      
      gyro_bias[i] += gyro;
    }
    c++;
    maxLoopRate(LOOPRATE);
  }

  for (int i = 0; i < AXIS_COUNT; i++) {
    // Divide the sum by number of iterations to get the error value
    acc_bias[i] /= c;
    gyro_bias[i] /= c;
  }
  acc_bias[AXIS_Z] -= 1.0f; // Subtract one to remove the effect that gravity has on the sensor

  Serial.println("float acc_bias[AXIS_COUNT] = {");
  Serial.print("  ");
  Serial.print(acc_bias[AXIS_X], 5);
  Serial.println("f, // x");
  Serial.print("  ");
  Serial.print(acc_bias[AXIS_Y], 5);
  Serial.println("f, // y");
  Serial.print("  ");
  Serial.print(acc_bias[AXIS_Z], 5);
  Serial.println("f, // z");
  Serial.println("};");
  Serial.println("");
  
  Serial.println("float gyro_bias[AXIS_COUNT] = {");
  Serial.print("  ");
  Serial.print(gyro_bias[AXIS_ROLL], 5);
  Serial.println("f, // roll");
  Serial.print("  ");
  Serial.print(gyro_bias[AXIS_PITCH], 5);
  Serial.println("f, // pitch");
  Serial.print("  ");
  Serial.print(gyro_bias[AXIS_YAW], 5);
  Serial.println("f, // yaw");
  Serial.println("};");
  Serial.println("");

  Serial.println("Paste these values in user specified variables section and comment out calculateGyroBias() in void setup.");
  Serial.println("Code will not run past the function calculateGyroBias until it is commented out!");
  while(1); // Halt code so it won't enter main loop until this function commented out
}


int32_t imu_count = 0; // 32 bit as its faster to run but uses more memory, we have plenty of memory

bool getIMUdata(float gyro_bias[], float acc_bias[], float gyro[], float acc[], axisRotation rotation[]) {
  // DESCRIPTION: Request full dataset from IMU gyro, accelerometer, and magnetometer data
  /*
   * Reads accelerometer, gyro, and magnetometer data from IMU.
   * These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. 
   * The constant errors found in calculateGyroBias() on startup are subtracted from the accelerometer and gyro readings.
   * If there is new acc data it will return true, otherwise it returns false.
   */
  int16_t unscaled_acc[AXIS_COUNT];
  int16_t unscaled_gyro[AXIS_COUNT];
  bool new_acc = false;

  imu_count += 1;
  if (imu_count > ACC_DIVIDER) {
    new_acc = true;
    imu_count = 0;
    mpu6050.getAcceleration(
      &unscaled_acc[AXIS_X],
      &unscaled_acc[AXIS_Y],
      &unscaled_acc[AXIS_Z]
    );
  }

  mpu6050.getRotation(
    &unscaled_gyro[AXIS_ROLL],
    &unscaled_gyro[AXIS_PITCH],
    &unscaled_gyro[AXIS_YAW]
  );

  // mpu6050.getMotion6(&unscaled_acc[AXIS_X], &unscaled_acc[AXIS_Y], &unscaled_acc[AXIS_Z], &unscaled_gyro[AXIS_ROLL], &unscaled_gyro[AXIS_PITCH], &unscaled_gyro[AXIS_YAW]);

  // loop through all acc, gyro and mag
  for (int i = 0; i < AXIS_COUNT; i++) {
    if (new_acc) {
      // Scale accelerometer
      acc[i] = unscaled_acc[i] / ACCEL_SCALE_FACTOR; //G's

      // Bias correct the acc
      acc[i] -= acc_bias[i];
    }
    // Scale gyro
    gyro[i] = unscaled_gyro[i] / GYRO_SCALE_FACTOR; //deg/sec

    // Bias correct the gyro
    gyro[i] -= gyro_bias[i];
  }

  vectorRotation(acc, rotation);
  vectorRotation(gyro, rotation);
  return new_acc;
}

void IMUinit() {
  // DESCRIPTION: Initialize IMU
  /*
   * Don't worry about how this works.
   */
  Wire.begin();
  Wire.setClock(1000000); // Note this is 2.5 times the spec sheet 400 kHz max...
    
  mpu6050.initialize();
    
  if (mpu6050.testConnection() == false) {
    Serial.println("MPU6050 initialization unsuccessful");
    Serial.println("Check MPU6050 wiring or try cycling power");
    while(1) {}
  }

  // From the reset state all registers should be 0x00, so we should be at
  // max sample rate with digital low pass filter(s) off.  All we need to
  // do is set the desired fullscale ranges
  mpu6050.setFullScaleGyroRange(GYRO_SCALE);
  mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
}

//===================================================MADGWICK AKA AHRS====================================================//
float B_madgwick = 0.025f;  // Hand tuned value, higher values mean that the accelerometer corrects faster

float q0 = 1.0f; //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

void Madgwick6DOF(float gyro[], float acc[], bool new_acc, float gyro_dt, float acc_dt, float attitude_euler[], float gravity_vector[]) {
  // DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */

  // give gyro and acc shorter names
  float gx = gyro[AXIS_X];
  float gy = gyro[AXIS_Y];
  float gz = gyro[AXIS_Z];

  float ax = acc[AXIS_X];
  float ay = acc[AXIS_Y];
  float az = acc[AXIS_Z];

  // Convert gyro degrees/sec to radians/sec
  gx *= DEG_TO_RAD;
  gy *= DEG_TO_RAD;
  gz *= DEG_TO_RAD;

  // Gyro to quaternion rate of change
  float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  // Look into improving this part of the algorithm
  // TODO for Kevin Look at disabling ACC unless its near a 1.0 magnitude
  if (!((ax < 0.01f && ax > -0.01f) && (ay < 0.01f && ay > -0.01f) && (az < 0.01f && az > -0.01f)) && new_acc) {
    // Normalise accelerometer measurement
    float accNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= accNorm;
    ay *= accNorm;
    az *= accNorm;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;
    
    // Gradient decent algorithm corrective step for acc
    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    // unoptimized version
    // float s0 = 4.0f * (q0 * q2 * q2 + q0 * q1 * q1) + 2.0f * (q2 * ax - q1 * ay);
    // float s1 = 4.0f * (q1 * q3 * q3 + q0 * q0 * q1 - q1 + q1 * az) - 2.0f * (q3 * ax + q0 * ay) + 8.0f * (q1 * q1 * q1 + q1 * q2 * q2);
    // float s2 = 4.0f * (q0 * q0 * q2 + q2 * q3 * q3 - q2 + q2 * az) + 2.0f * (q0 * ax - q3 * ay) + 8.0f * (q2 * q1 * q1 + q2 * q2 * q2);
    // float s3 = 4.0f * (q1 * q1 * q3 + q2 * q2 * q3) - (2.0f * q1 * ax + q2 * ay);

    float sNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    
    s0 *= sNorm;
    s1 *= sNorm;
    s2 *= sNorm;
    s3 *= sNorm;

    // Apply feedback step
    q0 -= B_madgwick * s0 * acc_dt;
    q1 -= B_madgwick * s1 * acc_dt;
    q2 -= B_madgwick * s2 * acc_dt;
    q3 -= B_madgwick * s3 * acc_dt;
  }

  // Add gyro rotation to quaternion
  q0 += qDot1 * gyro_dt;
  q1 += qDot2 * gyro_dt;
  q2 += qDot3 * gyro_dt;
  q3 += qDot4 * gyro_dt;

  // Normalise quaternion
  float quatNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= quatNorm;
  q1 *= quatNorm;
  q2 *= quatNorm;
  q3 *= quatNorm;

  // Compute angles, North West Up
  attitude_euler[AXIS_ROLL] = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
  attitude_euler[AXIS_PITCH] = -90.0f + acos(2.0f * (q1 * q3 - q0 * q2)) * RAD_TO_DEG;
  attitude_euler[AXIS_YAW] = atan2(2.0f * (q1 * q2 + q0 * q3), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;

  gravity_vector[AXIS_X] = 2.0f * (q1 * q3 - q0 * q2);
  gravity_vector[AXIS_Y] = 2.0f * (q0 * q1 + q2 * q3);
  gravity_vector[AXIS_Z] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}
