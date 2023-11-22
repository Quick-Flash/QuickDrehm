// Teensy Flight Controller - QuickDrehm
// Authors: Kevin Plaizer, Nicholas Rehm
// Version: Alpha 1.0

//========================================================================================================================//
// This file contains all necessary functions and code used for radio communication to avoid cluttering the main code
#ifdef USE_SBUS_RX
  #include "src/SBUS/SBUS.h"   //sBus interface
#endif

#ifdef USE_DSM_RX
  #include "src/DSMRX/DSMRX.h"  
#endif

#ifdef USE_SBUS_RX
  SBUS sbus(Serial5);
  uint16_t sbusChannels[16];
  bool sbusFailSafe;
  bool sbusLostFrame;
#endif
#ifdef USE_DSM_RX
  DSM1024 DSM;
#endif

// TODO set this function up with settings found from findRcChannelLimits(RC_ARM)
// comment channels you are not using or add extra channels you are using
void initRcScalers(midpointRangeScaler_t rc_scalers[]) {
  midpointRangeScalerInit(
    &rc_scalers[RC_ROLL], // update roll rc_scaler
    174.0f, 992.0f, 1811.0f, // input min, mid and max
    -1.0f, 0.0f, 1.0f, // output min, mid and max don't typically change from a -1.0f to 1.0f range
    2.0f); // deadband

  midpointRangeScalerInit(
    &rc_scalers[RC_PITCH], // update pitch rc_scaler
    174.0f, 992.0f, 1811.0f, // input min, mid and max
    -1.0f, 0.0f, 1.0f, // output min, mid and max don't typically change from a -1.0f to 1.0f range
    2.0f); // deadband

  midpointRangeScalerInit(
    &rc_scalers[RC_THROTTLE], // update throttle rc_scaler
    174.0f, 992.0f, 1811.0f, // input min, mid and max
    0.0f, 0.5, 1.0f, // output min, mid and max don't typically change from a 0.5 to 1.0f range for throttle
    2.0f); // deadband

  midpointRangeScalerInit(
    &rc_scalers[RC_YAW], // update yaw rc_scaler
    174.0f, 992.0f, 1811.0f, // input min, mid and max
    -1.0f, 0.0f, 1.0f, // output min, mid and max don't typically change from a -1.0f to 1.0f range
    2.0f); // deadband

  midpointRangeScalerInit(
    &rc_scalers[RC_ARM], // update arm rc_scaler
    191.0f, 992.0f, 1792.0f, // input min, mid and max
    0.0f, 0.5, 1.0f, // output min, mid and max for switches keeping to a 0.0f to 1.0f range is normally best, for pots set it up as desired
    0.0f); // switches and aux channels typically need no deadband

  midpointRangeScalerInit(
    &rc_scalers[RC_AUX1], // update aux1 rc_scaler
    191.0f, 992.0f, 1792.0f, // input min, mid and max
    0.0f, 0.5, 1.0f, // output min, mid and max for switches keeping to a 0.0f to 1.0f range is normally best, for pots set it up as desired
    0.0f); // deadband

  midpointRangeScalerInit(
    &rc_scalers[RC_AUX2], // update aux2 rc_scaler
    191.0f, 992.0f, 1792.0f, // input min, mid and max
    0.0f, 0.5, 1.0f, // output min, mid and max for switches keeping to a 0.0f to 1.0f range is normally best, for pots set it up as desired
    0.0f); // switches and aux channels typically need no deadband

  midpointRangeScalerInit(
    &rc_scalers[RC_AUX3], // update aux3 rc_scaler
    191.0f, 992.0f, 1792.0f, // input min, mid and max
    0.0f, 0.5, 1.0f, // output min, mid and max for switches keeping to a 0.0f to 1.0f range is normally best, for pots set it up as desired
    0.0f); // switches and aux channels typically need no deadband

  midpointRangeScalerInit(
    &rc_scalers[RC_AUX4], // update aux4 rc_scaler
    191.0f, 992.0f, 1792.0f, // input min, mid and max
    0.0f, 0.5, 1.0f, // output min, mid and max for switches keeping to a 0.0f to 1.0f range is normally best, for pots set it up as desired
    0.0f); // switches and aux channels typically need no deadband

  midpointRangeScalerInit(
    &rc_scalers[RC_AUX5], // update aux5 rc_scaler
    191.0f, 992.0f, 1792.0f, // input min, mid and max
    0.0f, 0.5, 1.0f, // output min, mid and max for switches keeping to a 0.0f to 1.0f range is normally best, for pots set it up as desired
    0.0f); // switches and aux channels typically need no deadband

  midpointRangeScalerInit(
    &rc_scalers[RC_AUX6], // update aux6 rc_scaler
    191.0f, 992.0f, 1792.0f, // input min, mid and max
    0.0f, 0.5, 1.0f, // output min, mid and max for switches keeping to a 0.0f to 1.0f range is normally best, for pots set it up as desired
    0.0f); // switches and aux channels typically need no deadband

  midpointRangeScalerInit(
    &rc_scalers[RC_AUX7], // update aux7 rc_scaler
    191.0f, 992.0f, 1792.0f, // input min, mid and max
    0.0f, 0.5, 1.0f, // output min, mid and max for switches keeping to a 0.0f to 1.0f range is normally best, for pots set it up as desired
    0.0f); // switches and aux channels typically need no deadband
}

bool getRcChannels(uint16_t rc_channels[]) {
  // DESCRIPTION: Get scaled PWM values for every channel from the radio, returns if there is a failsafe
  /*
   * If using an SBUS receiver, the alues are pulled from the SBUS library directly.
   * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise. 
   */

  bool failsafe = false;

#ifdef USE_SBUS_RX
  sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame); // update sbusChannels to have data

  if (sbusFailSafe) {
    failsafe = true;
  }
  
  int failsafe_min_value = 150;
  int failsafe_max_value = 1850;
  for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
    rc_channels[i] = sbusChannels[i];
    if (rc_channels[i] > failsafe_max_value || rc_channels[i] < failsafe_min_value) {
      failsafe = true;
    }
  }
#elif defined USE_DSM_RX
  if (DSM.timedOut(micros())) {
    Serial.println("*** DSM RX TIMED OUT ***");
  }

  unsigned failsafe_min_value = 800;
  unsigned failsafe_max_value = 2200;
    
  uint16_t values[RC_CHANNEL_COUNT];
  DSM.getChannelValues(values, RC_CHANNEL_COUNT);
  for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
    rc_channels[i] = values[i];

    // check for failsafed values
    if (rc_channels[i] > failsafe_max_value || rc_channels[i] < failsafe_min_value) {
      failsafe = true;
    }
  }
#endif

  return failsafe;
}


void findRcChannelLimits(int arm_channel) {
  uint16_t rc_channels[RC_CHANNEL_COUNT];

  int min[RC_CHANNEL_COUNT];
  int max[RC_CHANNEL_COUNT];
  int min_digits[RC_CHANNEL_COUNT];
  int max_digits[RC_CHANNEL_COUNT];
  for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
    max[i] = 0;
    min[i] = 2147483647;
    min_digits[i] = 0;
    max_digits[i] = 0;
  } 

  while(1) {
    getRcChannels(rc_channels);
    if (shouldPrint(micros(), 20.0f)) {
      
      // find min and max
      for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
        if (max[i] < rc_channels[i]) {
          max[i] = rc_channels[i];
        }
        if (min[i] > rc_channels[i]) {
          min[i] = rc_channels[i];
        }

        max_digits[i] = 0;
        for (int k = max[i]; k > 0; k /= 10) {
          max_digits[i] += 1;
        }

        min_digits[i] = 0;
        for (int k = min[i]; k > 0; k /= 10) {
          min_digits[i] += 1;
        }
      }
      Serial.println(F(""));
      Serial.println(F(""));

      // Print max line
      Serial.print(F("Max: "));
      for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
        Serial.print(F("Channel "));
        Serial.print(i);
        Serial.print(F(": "));
        Serial.print(max[i]);
        Serial.print(F(".0f"));
        Serial.print(F(", "));
        for (int k = 4; k > max_digits[i]; k--) {
          Serial.print(F(" "));
        }
      }
      Serial.println(F(""));

      // Print current line
      Serial.print(F("Cur: "));
      for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
        int current_digits = 0;
        for (int k = rc_channels[i]; k > 0; k /= 10) {
          current_digits += 1;
        }

        Serial.print(F("Channel "));
        Serial.print(i);
        Serial.print(F(": "));
        Serial.print(rc_channels[i]);
        Serial.print(F(".0f"));
        Serial.print(F(", "));
        for (int k = 4; k > current_digits; k--) {
          Serial.print(F(" "));
        }
      }
      Serial.println(F(""));

      // Print low line
      Serial.print(F("Min: "));
      for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
        Serial.print(F("Channel "));
        Serial.print(i);
        Serial.print(F(": "));
        Serial.print(min[i]);
        Serial.print(F(".0f"));
        Serial.print(F(", "));
        for (int k = 4; k > min_digits[i]; k--) {
          Serial.print(F(" "));
        }
      }
    }
  }
  maxLoopRate(LOOPRATE); // Going too fast seems to cause failsafes, same as going to slow, wtf.
}


void radioSetup() {
  //SBUS Recevier
#ifdef USE_SBUS_RX
  sbus.begin();

  //DSM receiver
#elif defined USE_DSM_RX
  Serial5.begin(115000);
#else
  #error No RX type defined...
#endif
}

//For DSM type receivers
void serialEvent3(void)
{
#if defined USE_DSM_RX
  while (Serial5.available()) {
    DSM.handleSerialEvent(Serial5.read(), micros());
  }
#endif
}
