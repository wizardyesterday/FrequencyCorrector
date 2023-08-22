//**************************************************************************
// file name: FrequencyErrorEstimator.h
//**************************************************************************
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
// This class implements a signal processing block that performs
// estimation of frequency error of a signal using autocorrelation
// methods.
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

#ifndef __FREQUENCYERRORESTIMATOR__
#define __FREQUENCYERRORESTIMATOR__

#include <stdint.h>

class FrequencyErrorEstimator
{
  //***************************** operations **************************

  public:

  FrequencyErrorEstimator(float sampleRate,
                            int32_t lag,
                            uint32_t numberOfAccumulations,
                            void (*callbackPtr)(int16_t frequencyError,
                                                void *contextPtr),
                                                void *callbackContextPtr);

  ~FrequencyErrorEstimator(void);

  void reset(void);
  void run(int16_t *inPhasePtr,int16_t *quadrturePtr,uint32_t bufferLength);
  int16_t retrieveFrequencyError(void);

  //***************************** attributes **************************
  private:

  // This is the sampling rate in S/s.
  float sampleRate;

  // This is the lag that will be used for crosscorrelation.
  int16_t lag;

  // This is the number of accumulations before the callback is invoked.
  uint32_t accumulationCountThreshold;

  // When this equations the numberOfAccumulations, the callback is invoked.
  uint32_t accumulationCount;

  // This is used if asynchronous notification is used.
  void (*callbackPtr)(int16_t frequencyError,void *contextPtr);
  void *callbackContextPtr;

  // This represents the distribution of frequency error estimates.
  uint32_t *histogramPtr;
};

#endif //__FREQUENCYERRORESTIMATOR__

