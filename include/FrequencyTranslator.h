//**************************************************************************
// file name: FrequencyTranslator.h
//**************************************************************************
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
// This class implements a signal processing block known as a wideband  FM
// demodulator.  This class has a configurable demodulator gain.
///_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

#ifndef __FREQUENCYTRANSLATOR__
#define __FREQUENCYTRANSLATOR__

#include <stdint.h>

#include "Nco.h"

class FrequencyTranslator
{
  //***************************** operations **************************

  public:

  FrequencyTranslator(float sampleRate,uint32_t numberOfAverages);
  ~FrequencyTranslator(void);

  void resetDemodulator(void);
  void setDemodulatorGain(float gain);
  void acceptIqData(int8_t *bufferPtr,uint32_t bufferLength);

  void computeFrequencyError(uint32_t sampleCount);
  float getFrequencyError(void);

  private:

  //*******************************************************************
  // Utility functions.
  //*******************************************************************
  uint32_t demodulateSignal(int8_t *bufferPtr,uint32_t bufferLength);
  void derotateSignal(int8_t *bufferPtr,uint32_t bufferLength);

  //*******************************************************************
  // Attributes.
  //*******************************************************************
  // This gain maps d(phi)/dt to an information signal.
  float demodulatorGain;

  // Storage for last phase angle.
  float previousTheta;

  // This is used for averaging.
  uint32_t frequencyMeasurementCount;
  uint32_t numberOfAverages;
  float accumulatedFrequencyError;

  // This is the average offset from the center frequency.
  float frequencyError;

  // Demodulated data is the demodulator gain times delta theta.
  float demodulatedData[16384];

  // Frequency translation support.
  Nco *ncoPtr;
};

#endif // _FREQUENCYTRANSLATOR__
