// Teensy Flight Controller - QuickDrehm
// Authors: Kevin Plaizer
// Version: Alpha 1.0

//========================================================================================================================//
//                                                      IMU DEFINES                                                       //
//========================================================================================================================//
#ifdef USE_MPU6050_I2C
  #include "src/MPU6050/MPU6050.h"
  MPU6050 mpu6050;
#elif defined USE_MPU9250_SPI
  #include "src/MPU9250/MPU9250.h"
  MPU9250 mpu9250(SPI2,36);
#else
  #error No MPU defined... 
#endif

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
#ifdef USE_MPU6050_I2C
  #define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
  #define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
  #define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
  #define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
  #define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
  #define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
  #define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
  #define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16
#elif defined USE_MPU9250_SPI
  #define GYRO_FS_SEL_250    mpu9250.GYRO_RANGE_250DPS
  #define GYRO_FS_SEL_500    mpu9250.GYRO_RANGE_500DPS
  #define GYRO_FS_SEL_1000   mpu9250.GYRO_RANGE_1000DPS                                                        
  #define GYRO_FS_SEL_2000   mpu9250.GYRO_RANGE_2000DPS
  #define ACCEL_FS_SEL_2     mpu9250.ACCEL_RANGE_2G
  #define ACCEL_FS_SEL_4     mpu9250.ACCEL_RANGE_4G
  #define ACCEL_FS_SEL_8     mpu9250.ACCEL_RANGE_8G
  #define ACCEL_FS_SEL_16    mpu9250.ACCEL_RANGE_16G
#endif
  
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

// magnetometer calibration
void calibrateMagnetometer() {
#ifdef USE_MPU9250_SPI 
  Serial.println("Beginning magnetometer calibration in");
  Serial.println("3...");
  delay(1000);
  Serial.println("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);
  Serial.println("Rotate the IMU about all axes until complete.");
  Serial.println(" ");
  float success = mpu9250.calibrateMag();
  if(success) {
    Serial.println("Calibration Successful!");
    Serial.println("Please comment out the calibrateMagnetometer() function and copy these values into the code:");
    Serial.print("float mag_scale[AXIS_COUNT] = {");
    Serial.print(mpu9250.getMagBiasX_uT());
    Serial.print("f, ");
    Serial.print(mpu9250.getMagBiasY_uT());
    Serial.print("f, ");
    Serial.print(mpu9250.getMagBiasZ_uT());
    Serial.println("f};");

    Serial.print("float mag_scale[AXIS_COUNT] = {");
    Serial.print(mpu9250.getMagScaleFactorX());
    Serial.print("f, ");
    Serial.print(mpu9250.getMagScaleFactorY());
    Serial.print("f, ");
    Serial.print(mpu9250.getMagScaleFactorZ());
    Serial.println("f};");
    Serial.println(" ");
    Serial.println("If you are having trouble with your attitude estimate at a new flying location, repeat this process as needed.");
  } else {
    Serial.println("Calibration Unsuccessful. Please reset the board and try again.");
  }
  
    while(1); // Halt code so it won't enter main loop until this function commented out
#endif
  Serial.println("Error: MPU9250 not selected. Cannot calibrate non-existent magnetometer.");
  while(1); // Halt code so it won't enter main loop until this function commented out
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
#ifdef USE_MPU6050_I2C
    mpu6050.getMotion6(
      &unscaled_acc[AXIS_X], &unscaled_acc[AXIS_Y], &unscaled_acc[AXIS_Z], 
      &unscaled_gyro[AXIS_ROLL], &unscaled_gyro[AXIS_PITCH], &unscaled_gyro[AXIS_YAW]
    );
#elif defined USE_MPU9250_SPI
    int16_t unscaled_mag[AXIS_COUNT]; // only MPU9250 has mag
    mpu9250.getMotion9(
      &unscaled_acc[AXIS_X], &unscaled_acc[AXIS_Y], &unscaled_acc[AXIS_Z], 
      &unscaled_gyro[AXIS_ROLL], &unscaled_gyro[AXIS_PITCH], &unscaled_gyro[AXIS_YAW], 
      &unscaled_mag[AXIS_X], &unscaled_mag[AXIS_Y], &unscaled_mag[AXIS_Z]
    );
#endif
    
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
  Serial.print(acc_bias[AXIS_X]);
  Serial.println("f, // x");
  Serial.print("  ");
  Serial.print(acc_bias[AXIS_Y]);
  Serial.println("f, // y");
  Serial.print("  ");
  Serial.print(acc_bias[AXIS_Z]);
  Serial.println("f, // z");
  Serial.println("};");
  Serial.println("");
  
  Serial.println("float gyro_bias[AXIS_COUNT] = {");
  Serial.print("  ");
  Serial.print(gyro_bias[AXIS_ROLL]);
  Serial.println("f, // roll");
  Serial.print("  ");
  Serial.print(gyro_bias[AXIS_PITCH]);
  Serial.println("f, // pitch");
  Serial.print("  ");
  Serial.print(gyro_bias[AXIS_YAW]);
  Serial.println("f, // yaw");
  Serial.println("};");
  Serial.println("");

  Serial.println("Paste these values in user specified variables section and comment out calculateGyroBias() in void setup.");
  Serial.println("Code will not run past the function calculateGyroBias until it is commented out!");
  while(1); // Halt code so it won't enter main loop until this function commented out
}


#ifdef USE_MPU6050_I2C // no mag
int32_t imu_count = 0; // 32 bit as its faster to run but uses more memory, we have plenty of memory

bool getIMUdata(float gyro_bias[], float acc_bias[], float gyro[], float acc[], axisRotation rotation[]) {
#elif USE_MPU9250_SPI // has mag
bool getIMUdata(float gyro_bias[], float acc_bias[], float mag_bias[], float gyro[], float acc[], float mag[], axisRotation rotation[]) {
#endif
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

#ifdef USE_MPU6050_I2C
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
#elif defined USE_MPU9250_SPI
  int16_t unscaled_mag[AXIS_COUNT]; // only MPU9250 has mag
  mpu9250.getMotion9(
    &unscaled_acc[AXIS_X], &unscaled_acc[AXIS_Y], &unscaled_acc[AXIS_Z], 
    &unscaled_gyro[AXIS_ROLL], &unscaled_gyro[AXIS_PITCH], &unscaled_gyro[AXIS_YAW], 
    &unscaled_mag[AXIS_X], &unscaled_mag[AXIS_Y], &unscaled_mag[AXIS_Z]
  );
#endif

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

#ifdef USE_MPU9250_SPI
    // Scale magnetometer
    mag[i] = unscaled_mag[i] / 6.0f; // uT

    // Bias correct the mag
    mag[i] = (Mag[i] - mag_bias[i]) * mag_scale[i];
#endif
  }

  vectorRotation(acc, rotation);
  vectorRotation(gyro, rotation);
#ifdef USE_MPU9250_SPI
  vectorRotation(mag, rotation);
#endif
  return new_acc;
}

void IMUinit() {
  // DESCRIPTION: Initialize IMU
  /*
   * Don't worry about how this works.
   */
#ifdef USE_MPU6050_I2C
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
    
#elif defined USE_MPU9250_SPI
  int status = mpu9250.begin();    

  if (status < 0) {
    Serial.println("MPU9250 initialization unsuccessful");
    Serial.println("Check MPU9250 wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  // From the reset state all registers should be 0x00, so we should be at
  // max sample rate with digital low pass filter(s) off.  All we need to
  // do is set the desired fullscale ranges
  mpu9250.setGyroRange(GYRO_SCALE);
  mpu9250.setAccelRange(ACCEL_SCALE);
  mpu9250.setMagCalX(mag_bias[X], mag_scale[X]);
  mpu9250.setMagCalY(mag_bias[Y], mag_scale[Y]);
  mpu9250.setMagCalZ(mag_bias[Z], mag_scale[Z]);
  mpu9250.setSrd(0); // sets gyro and accel read to 1khz, magnetometer read to 100hz
#endif
}

//===================================================MADGWICK AKA AHRS====================================================//
float B_madgwick = 0.025f;  // Hand tuned value, higher values mean that the accelerometer corrects faster

float q0 = 1.0f; //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

#ifdef USE_MPU9250_SPI
void Madgwick9DOF(float gyro[], float acc[], float mag[], float dt) {
  // DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
  /*
   * This function fuses the accelerometer gyro, and magnetometer readings Acc[], Gyro[] and, Mag[] for attitude estimation.
   * Don't worry about the math. There is a tunable parameter B_madgwick in the user specified variable section which basically
   * adjusts the weight of gyro data in the state estimate. Higher beta leads to noisier estimate, lower 
   * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the roll_IMU,
   * pitch_IMU, and yaw_IMU variables which are in degrees. If magnetometer data is not available, this function calls Madgwick6DOF() instead.
   */

  // give mag a shorter name
  float mx = mag[AXIS_X];
  float my = mag[AXIS_Y];
  float mz = mag[AXIS_Z];

  // Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    Madgwick6DOF(gyro, acc, dt, dt);
    return;
  }

  // give gyro and acc shorter names
  float gx = gyro[AXIS_X];
  float gy = gyro[AXIS_Y];
  float gz = gyro[AXIS_Z];

  float ax = acc[AXIS_X];
  float ay = acc[AXIS_X];
  float az = acc[AXIS_X];

  // Convert gyroscope degrees/sec to radians/sec
  gx *= DEG_TO_RAD;
  gy *= DEG_TO_RAD;
  gz *= DEG_TO_RAD;

  // Rate of change of quaternion from gyroscope
  float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    float accNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= accNorm;
    ay *= accNorm;
    az *= accNorm;

    // Normalise magnetometer measurement
    float magNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= magNorm;
    my *= magNorm;
    mz *= magNorm;

    // Auxiliary variables to avoid repeated arithmetic
    // Unused for now
    // TODO look at how to optimize this part of the code again
    /*
    float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
    */

    // Reference direction of Earth's magnetic field

    float _2q0mx = 2.0f * q0 * mx;
    float _2q0my = 2.0f * q0 * my;
    float _2q0mz = 2.0f * q0 * mz;
    float _2q1mx = 2.0f * q1 * mx;

    float hx = mx * q0 * q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1 * q1 + 2.0f * q1 * my * q2 + 2.0f * q1 * mz * q3 - mx * q2 * q2 - mx * q3 * q3;
    float hy = _2q0mx * q3 + my * q0 * q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1 * q1 + my * q2 * q2 + 2.0f * q2 * mz * q3 - my * q3 * q3;
    float _2bx = sqrtf(hx * hx + hy * hy);
    float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0 * q0 + _2q1mx * q3 - mz * q1 * q1 + 2.0f * q2 * my * q3 - mz * q2 * q2 + mz * q3 * q3;
    float _4bx = 2.0f * _2bx;
    float _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    float s0 = -2.0f * q2 * (2.0f * q1 * q3 - 2.0f * q0 * q2 - ax) + 2.0f * q1 * (2.0f * q0 * q1 + 2.0f * q2 * q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2 * q2 - q3 * q3) + _2bz * (q1 * q3 - q0 * q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q0 * q1 + q2 * q3) - my) + _2bx * q2 * (_2bx * (q0 * q2 + q1 * q3) + _2bz * (0.5f - q1 * q1 - q2 * q2) - mz);
    float s1 = 2.0f * q3 * (2.0f * q1 * q3 - 2.0f * q0 * q2 - ax) + 2.0f * q0 * (2.0f * q0 * q1 + 2.0f * q2 * q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1 * q1 - 2.0f * q2 * q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2 * q2 - q3 * q3) + _2bz * (q1 * q3 - q0 * q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q0 * q1 + q2 * q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0 * q2 + q1 * q3) + _2bz * (0.5f - q1 * q1 - q2 * q2) - mz);
    float s2 = -2.0f * q0 * (2.0f * q1 * q3 - 2.0f * q0 * q2 - ax) + 2.0f * q3 * (2.0f * q0 * q1 + 2.0f * q2 * q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1 * q1 - 2.0f * q2 * q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2 * q2 - q3 * q3) + _2bz * (q1 * q3 - q0 * q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q0 * q1 + q2 * q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0 * q2 + q1 * q3) + _2bz * (0.5f - q1 * q1 - q2 * q2) - mz);
    float s3 = 2.0f * q1 * (2.0f * q1 * q3 - 2.0f * q0 * q2 - ax) + 2.0f * q2 * (2.0f * q0 * q1 + 2.0f * q2 * q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2 * q2 - q3 * q3) + _2bz * (q1 * q3 - q0 * q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q0 * q1 + q2 * q3) - my) + _2bx * q1 * (_2bx * (q0 * q2 + q1 * q3) + _2bz * (0.5f - q1 * q1 - q2 * q2) - mz);
    float sNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= sNorm;
    s1 *= sNorm;
    s2 *= sNorm;
    s3 *= sNorm;

    // Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalize quaternion
  float quatNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= quatNorm;
  q1 *= quatNorm;
  q2 *= quatNorm;
  q3 *= quatNorm;
  
  attitude_euler[AXIS_ROLL] = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
  attitude_euler[AXIS_PITCH] = -90.0f + acos(2.0f * (q1 * q3 - q0 * q2)) * RAD_TO_DEG;
  attitude_euler[AXIS_YAW] = atan2(2.0f * (q1 * q2 + q0 * q3), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;

  gravity_vector[AXIS_X] = 2.0f * (q1 * q3 - q0 * q2);
  gravity_vector[AXIS_Y] = 2.0f * (q0 * q1 + q2 * q3);
  gravity_vector[AXIS_Z] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}
#endif


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
