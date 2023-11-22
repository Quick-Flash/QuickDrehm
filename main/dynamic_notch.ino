// Teensy Flight Controller - QuickDrehm
// Authors: Kevin Plaizer
// Version: Alpha 1.0

//========================================================================================================================//
//                                                 DYNAMIC NOTCH CODE                                                     //
//========================================================================================================================//

// conversion from time to frequency domain and filtering wizardry happens here

// Struct defines live in global_defines.h
#define DYN_NOTCH_SMOOTH_HZ        4
#define DYN_NOTCH_CALC_TICKS       (AXIS_COUNT * STEP_COUNT) // 3 axes and 4 steps per axis

#define MAX(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a > _b ? _a : _b; })

#define MIN(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a < _b ? _a : _b; })

void dynNotchInit(dynNotch_t *dynNotch, float min_hz, float max_hz, float notch_count, float q, float dT) {
  // dynNotchUpdate() is running at looprateHz (which is the PID looprate aka. 1e6f / gyro.targetLooptime)
  const float looprateHz = 1.0f / dT;
  const float nyquistHz = looprateHz / 2.0f;

  // If dynamic notch is available, initialise so it can be activated at any time
  dynNotch->q = q;
  dynNotch->minHz = min_hz;
  dynNotch->maxHz = MAX(dynNotch->minHz, max_hz);
  dynNotch->maxHz = MIN(dynNotch->maxHz, nyquistHz); // Ensure to not go above the nyquist limit
  dynNotch->count = notch_count;
  dynNotch->state.tick = DYN_NOTCH_CALC_TICKS;
  dynNotch->state.axis = AXIS_ROLL;
  dynNotch->state.step = STEP_WINDOW;
  dynNotch->sampleIndex = 0;

  dynNotch->sampleCount = MAX(1, nyquistHz / dynNotch->maxHz);
  dynNotch->sampleCountRcp = 1.0f / dynNotch->sampleCount;

  float sdftSampleRateHz = looprateHz / dynNotch->sampleCount;

  dynNotch->sdftResolutionHz = sdftSampleRateHz / SDFT_SAMPLE_SIZE;
  dynNotch->sdftStartBin = MAX(1, lrintf(dynNotch->minHz / dynNotch->sdftResolutionHz)); // can't use bin 0 because it is DC.
  dynNotch->sdftEndBin = MIN(SDFT_BIN_COUNT - 1, lrintf(dynNotch->maxHz / dynNotch->sdftResolutionHz)); // can't use more than SDFT_BIN_COUNT bins.
  dynNotch->pt1LooptimeS = DYN_NOTCH_CALC_TICKS / looprateHz;

  for (int axis = 0; axis < AXIS_COUNT; axis++) {
    sdftInit(&dynNotch->sdft[axis], dynNotch->sdftStartBin, dynNotch->sdftEndBin, dynNotch->sampleCount);
    dynNotch->sampleAccumulator[axis] = 0.0f; // zero the array
    dynNotch->sampleAvg[axis] = 0.0f; // zero the array
  }

  for (int p = 0; p < dynNotch->count; p++) {
    dynNotch->peaks[p].bin = 0;
    dynNotch->peaks[p].value = 0.0f;
  }

  for (int axis = 0; axis < AXIS_COUNT; axis++) {
    for (int p = 0; p < dynNotch->count; p++) {
      // any init value is fine, but evenly spreading centerFreqs across frequency range makes notches stick to peaks quicker
      dynNotch->centerFreq[axis][p] = (p + 0.5f) * (dynNotch->maxHz - dynNotch->minHz) / (float)dynNotch->count + dynNotch->minHz;
      notchFilterInit(&dynNotch->notch[axis][p], dynNotch->centerFreq[axis][p], dynNotch->q, dT);
    }
  }
}


// Downsample and analyse gyro data
void dynNotchUpdate(dynNotch_t *dynNotch, float sample[], float dT) {
  for (int axis = 0; axis < AXIS_COUNT; axis++) {
    dynNotch->sampleAccumulator[axis] += sample[axis];
  }

  // samples should have been pushed by `dynNotchPush`
  // if gyro sampling is > 1kHz, accumulate and average multiple gyro samples
  if (dynNotch->sampleIndex == dynNotch->sampleCount) {

    dynNotch->sampleIndex = 0;

    // calculate mean value of accumulated samples
    for (int axis = 0; axis < AXIS_COUNT; axis++) {
      dynNotch->sampleAvg[axis] = dynNotch->sampleAccumulator[axis] * dynNotch->sampleCountRcp;
      dynNotch->sampleAccumulator[axis] = 0;
    }

    dynNotch->state.tick = DYN_NOTCH_CALC_TICKS;
  }

  // 2us @ F722
  // SDFT processing in batches to synchronize with incoming downsampled data
  for (int axis = 0; axis < AXIS_COUNT; axis++) {
    sdftPushBatch(&dynNotch->sdft[axis], dynNotch->sampleAvg[axis], dynNotch->sampleIndex);
  }
  dynNotch->sampleIndex++;

  // Find frequency peaks and update filters
  if (dynNotch->state.tick > 0) {
    dynNotchProcess(dynNotch, dT);
    --dynNotch->state.tick;
  }
}


// Find frequency peaks and update filters
void dynNotchProcess(dynNotch_t *dynNotch, float dT) {
  switch (dynNotch->state.step) {

    case STEP_WINDOW: // 4.1us (3-6us) @ F722
    {
      sdftWinSq(&dynNotch->sdft[dynNotch->state.axis], dynNotch->sdftData);

      // Get total vibrational power in dyn notch range for noise floor estimate in STEP_CALC_FREQUENCIES
      dynNotch->sdftNoiseThreshold = 0.0f;
      for (int bin = dynNotch->sdftStartBin; bin <= dynNotch->sdftEndBin; bin++) {
        dynNotch->sdftNoiseThreshold += dynNotch->sdftData[bin];  // sdft contains power spectral density
      }

      break;
    }
    case STEP_DETECT_PEAKS: // 5.5us (4-7us) @ F722
    {
      // Get memory ready for new peak data on current axis
      for (int p = 0; p < dynNotch->count; p++) {
        dynNotch->peaks[p].bin = 0;
        dynNotch->peaks[p].value = 0.0f;
      }

      // Search for N biggest peaks in frequency spectrum
      for (int bin = (dynNotch->sdftStartBin + 1); bin < dynNotch->sdftEndBin; bin++) {
        // Check if bin is peak
        if ((dynNotch->sdftData[bin] > dynNotch->sdftData[bin - 1]) && (dynNotch->sdftData[bin] > dynNotch->sdftData[bin + 1])) {
          // sorts the peaks in order of amplitude
          for (int p = 0; p < dynNotch->count; p++) {
            if (dynNotch->sdftData[bin] > dynNotch->peaks[p].value) {
              for (int k = dynNotch->count - 1; k > p; k--) {
                dynNotch->peaks[k] = dynNotch->peaks[k - 1];
              }
              dynNotch->peaks[p].bin = bin;
              dynNotch->peaks[p].value = dynNotch->sdftData[bin];
              break;
            }
          }
          bin++; // If bin is peak, next bin can't be peak => skip it
        }
      }

/*
      // Sort N biggest peaks in ascending bin order (example: 3, 8, 25, 0, 0, ..., 0)
      for (int p = dynNotch->count - 1; p > 0; p--) {
        for (int k = 0; k < p; k++) {
          // Swap peaks but ignore swapping void peaks (bin = 0). This leaves
          // void peaks at the end of peaks array without moving them
          if (dynNotch->peaks[k].bin > dynNotch->peaks[k + 1].bin && dynNotch->peaks[k + 1].bin != 0) {
            sdftPeak_t temp = dynNotch->peaks[k];
            dynNotch->peaks[k] = dynNotch->peaks[k + 1];
            dynNotch->peaks[k + 1] = temp;
          }
        }
      }
*/
      break;
    }
    case STEP_CALC_FREQUENCIES: // 4.0us (2-7us) @ F722
    {
      // Approximate noise floor (= average power spectral density in dyn notch range, excluding peaks)
      int peakCount = 0;
      for (int p = 0; p < dynNotch->count; p++) {
        if (dynNotch->peaks[p].bin != 0) {
          dynNotch->sdftNoiseThreshold -= 0.75f * dynNotch->sdftData[dynNotch->peaks[p].bin - 1];
          dynNotch->sdftNoiseThreshold -= dynNotch->sdftData[dynNotch->peaks[p].bin];
          dynNotch->sdftNoiseThreshold -= 0.75f * dynNotch->sdftData[dynNotch->peaks[p].bin + 1];
          peakCount++;
        }
      }
      dynNotch->sdftNoiseThreshold /= dynNotch->sdftEndBin - dynNotch->sdftStartBin - peakCount + 1;

      // A noise threshold 2 times the noise floor prevents peak tracking being too sensitive to noise
      dynNotch->sdftNoiseThreshold *= 2.0f;

      float peakFreqs[DYN_NOTCH_COUNT_MAX];
      int peaks_detected = 0;

      for (int p = 0; p < dynNotch->count; p++) {

        // Only update dynNotch.centerFreq if there is a peak (ignore void peaks) and if peak is above noise floor
        if (dynNotch->peaks[p].bin != 0 && dynNotch->peaks[p].value > dynNotch->sdftNoiseThreshold) {

          float meanBin = dynNotch->peaks[p].bin;

          // Height of peak bin (y1) and shoulder bins (y0, y2)
          const float y0 = dynNotch->sdftData[dynNotch->peaks[p].bin - 1];
          const float y1 = dynNotch->sdftData[dynNotch->peaks[p].bin];
          const float y2 = dynNotch->sdftData[dynNotch->peaks[p].bin + 1];

          float denominator = y0 + y1 + y2;
          if (denominator != 0.0) {
            meanBin += (y2 - y0) / denominator;
          }

          // Convert bin to frequency: freq = bin * binResoultion (bin 0 is 0Hz)
          peakFreqs[p] = meanBin * dynNotch->sdftResolutionHz;

          peaks_detected += 1; // this will limit how many loops we do later

        } else {
          break; // no need to keep running loop as once one peak fails the rest will
        }
      }

      bool matched_notches[DYN_NOTCH_COUNT_MAX];
      bool matched_peaks[DYN_NOTCH_COUNT_MAX];
      for (int i = 0; i < DYN_NOTCH_COUNT_MAX; i++) {
        matched_notches[i] = false;
        matched_peaks[i] = false;
      }
      float peak_distance[DYN_NOTCH_COUNT_MAX][DYN_NOTCH_COUNT_MAX];

      float minimum_distance_away = 10000.0f; // maximum f32 value
      int matched_notch = -1;
      int matched_peak = -1;

      for (int peak = 0; peak < peaks_detected; peak++) { // find how far away each notch is from each peak and store that for later use
        float peak_frequency = peakFreqs[peak];
        for (int notch = 0; notch < dynNotch->count; notch++) {
          float notch_frequency = dynNotch->centerFreq[dynNotch->state.axis][notch];
          float distance_away = abs(notch_frequency - peak_frequency);
          peak_distance[peak][notch] = distance_away;
        }
      }


      for (int i = 0; i < peaks_detected; i++) { // match peaks to notches in order of closest to furthest
        minimum_distance_away = 10000.0f; // reset each loop
        matched_notch = -1;
        matched_peak = -1;

        for (int peak = 0; peak < peaks_detected; peak++) {
          if (matched_peaks[peak] == false) { // only continue if this peak isn't matched yet
            for (int notch = 0; notch < dynNotch->count; notch++) {
              if (matched_notches[notch] == false) { // only continue if this notch isn't matched yet
                float distance_away = peak_distance[peak][notch];

                if (distance_away < minimum_distance_away) {
                  minimum_distance_away = distance_away;
                  matched_notch = notch;
                  matched_peak = peak;
                }
              }
            }
          }
        }


        if ((matched_notch != -1) && (matched_peak != -1)) {
          matched_peaks[matched_peak] = true;
          matched_notches[matched_notch] = true;
          // PT1 style smoothing moves notch center freqs rapidly towards big peaks and slowly away, up to 10x higher cutoff
          const float cutoffMult = constrain(dynNotch->peaks[matched_peak].value / (dynNotch->sdftNoiseThreshold * 2.0f), 1.0f, 10.0f);
          const float gain = pt1FilterGain(DYN_NOTCH_SMOOTH_HZ * cutoffMult, dynNotch->pt1LooptimeS); // dynamic PT1 k value

          // Filter notch center frequency of matched notch towards the matched peak value
          dynNotch->centerFreq[dynNotch->state.axis][matched_notch] += gain * (peakFreqs[matched_peak] - dynNotch->centerFreq[dynNotch->state.axis][matched_notch]);
        }
      }

      break;
    }
    case STEP_UPDATE_FILTERS: // 5.4us (2-9us) @ F722
    {
      for (int p = 0; p < dynNotch->count; p++) {
        notchFilterUpdate(&dynNotch->notch[dynNotch->state.axis][p], dynNotch->centerFreq[dynNotch->state.axis][p], dynNotch->q, 1.0f, dT);
      }

      dynNotch->state.axis = (dynNotch->state.axis + 1) % AXIS_COUNT;
    }
  }

  dynNotch->state.step = (dynNotch->state.step + 1) % STEP_COUNT;
}


float dynNotchFilter(dynNotch_t *dynNotch, const int axis, float value)
{
  for (int i = 0; i < dynNotch->count; i++) {
    value = notchFilterApply(&dynNotch->notch[axis][i], value);
  }

  return value;
}

//======================================================SDFT STUFF=======================================================//
#define SDFT_R 0.9999f  // damping factor for guaranteed SDFT stability (r < 1.0f) 

void sdftInit(sdft_t *sdft, const int startBin, const int endBin, const int numBatches) {
  sdft->rPowerN = powf(SDFT_R, SDFT_SAMPLE_SIZE);
  const float c = 2.0f * M_PI / (float)SDFT_SAMPLE_SIZE;
  float phi = 0.0f;
  for (int i = 0; i < SDFT_BIN_COUNT; i++) {
    phi = c * i;
    sdft->twiddle[i].re = SDFT_R * cosf(phi);
    sdft->twiddle[i].im = SDFT_R * sinf(phi);
  }

  sdft->idx = 0;

  sdft->startBin = constrain(startBin, 0, SDFT_BIN_COUNT - 1);
  sdft->endBin = constrain(endBin, sdft->startBin, SDFT_BIN_COUNT - 1);

  sdft->numBatches = MAX(numBatches, 1);
  sdft->batchSize = (sdft->endBin - sdft->startBin + 1) / sdft->numBatches;

  for (int i = 0; i < SDFT_SAMPLE_SIZE; i++) {
    sdft->samples[i] = 0.0f;
  }

  for (int i = 0; i < SDFT_BIN_COUNT; i++) {
    sdft->data[i].re = 0.0f;
    sdft->data[i].im = 0.0f;
  }
}


// Add new sample to frequency spectrum in parts
void sdftPushBatch(sdft_t *sdft, const float sample, const int batchIdx) {
  const int batchStart = sdft->batchSize * batchIdx + sdft->startBin;
  int batchEnd = batchStart;

  float delta = sample - sdft->rPowerN * sdft->samples[sdft->idx];

  if (batchIdx == sdft->numBatches - 1) {
    sdft->samples[sdft->idx] = sample;
    sdft->idx = (sdft->idx + 1) % SDFT_SAMPLE_SIZE;
    batchEnd += sdft->endBin - batchStart + 1;
  } else {
    batchEnd += sdft->batchSize;
  }

  for (int i = batchStart; i < batchEnd; i++) {
    float re = sdft->twiddle[i].re * (sdft->data[i].re + delta) - sdft->twiddle[i].im * sdft->data[i].im;
    float im = sdft->twiddle[i].re * sdft->data[i].im + sdft->twiddle[i].im * (sdft->data[i].re + delta);
    sdft->data[i].re = re;
    sdft->data[i].im = im;
  }

  updateEdges(sdft, delta, batchIdx);
}


// Get squared magnitude of frequency spectrum
void sdftMagSq(const sdft_t *sdft, float *output) {
  float re;
  float im;

  for (int i = sdft->startBin; i <= sdft->endBin; i++) {
    re = sdft->data[i].re;
    im = sdft->data[i].im;
    output[i] = re * re + im * im;
  }
}


// Get squared magnitude of frequency spectrum with Hann window applied
// Hann window in frequency domain: X[k] = -0.25 * X[k-1] +0.5 * X[k] -0.25 * X[k+1]
void sdftWinSq(const sdft_t *sdft, float *output) {
  complex_t val;
  float re;
  float im;

  // Apply window at the lower edge of active range
  if (sdft->startBin == 0) {
    val.re = sdft->data[sdft->startBin].re - sdft->data[sdft->startBin + 1].re;
    val.im = sdft->data[sdft->startBin].im - sdft->data[sdft->startBin + 1].im;
  } else {
    val.re = sdft->data[sdft->startBin].re - 0.5f * (sdft->data[sdft->startBin - 1].re + sdft->data[sdft->startBin + 1].re);
    val.im = sdft->data[sdft->startBin].im - 0.5f * (sdft->data[sdft->startBin - 1].im + sdft->data[sdft->startBin + 1].im);
  }
  re = val.re;
  im = val.im;
  output[sdft->startBin] = re * re + im * im;

  for (int i = (sdft->startBin + 1); i < sdft->endBin; i++) {
    val.re = sdft->data[i].re - 0.5f * (sdft->data[i - 1].re + sdft->data[i + 1].re); // multiply by 2 to save one multiplication
    val.im = sdft->data[i].im - 0.5f * (sdft->data[i - 1].im + sdft->data[i + 1].im); // multiply by 2 to save one multiplication

    re = val.re;
    im = val.im;
    output[i] = re * re + im * im;
  }

  // Apply window at the upper edge of active range
  if (sdft->endBin == SDFT_BIN_COUNT - 1) {
    val.re = sdft->data[sdft->endBin].re - sdft->data[sdft->endBin - 1].re;
    val.im = sdft->data[sdft->endBin].im - sdft->data[sdft->endBin - 1].im;
  } else {
    val.re = sdft->data[sdft->endBin].re - 0.5f * (sdft->data[sdft->endBin - 1].re + sdft->data[sdft->endBin + 1].re);
    val.im = sdft->data[sdft->endBin].im - 0.5f * (sdft->data[sdft->endBin - 1].im + sdft->data[sdft->endBin + 1].im);
  }
  re = val.re;
  im = val.im;
  output[sdft->endBin] = re * re + im * im;
}


// Needed for proper windowing at the edges of active range
void updateEdges(sdft_t *sdft, const float value, const int batchIdx) {
  // First bin outside of lower range
  if (sdft->startBin > 0 && batchIdx == 0) {
    const unsigned idx = sdft->startBin - 1;
    float re = sdft->twiddle[idx].re * (sdft->data[idx].re + value) - sdft->twiddle[idx].im * sdft->data[idx].im;
    float im = sdft->twiddle[idx].re * sdft->data[idx].im + sdft->twiddle[idx].im * (sdft->data[idx].re + value);
    sdft->data[idx].re = re;
    sdft->data[idx].im = im;
  }

  // First bin outside of upper range
  if (sdft->endBin < SDFT_BIN_COUNT - 1 && batchIdx == sdft->numBatches - 1) {
    const unsigned idx = sdft->endBin + 1;
    float re = sdft->twiddle[idx].re * (sdft->data[idx].re + value) - sdft->twiddle[idx].im * sdft->data[idx].im;
    float im = sdft->twiddle[idx].re * sdft->data[idx].im + sdft->twiddle[idx].im * (sdft->data[idx].re + value);
    sdft->data[idx].re = re;
    sdft->data[idx].im = im;
  }
}
