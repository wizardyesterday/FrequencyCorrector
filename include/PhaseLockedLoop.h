//**************************************************************************
// file name: PhaseLockedLoop.h
//**************************************************************************
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
// This class implements a signal processing block known as a phase-locked
// loop.  Given 8-bit IQ samples from an SDR, the system will lock to the
// phase of the incoming signal by performing appropriate frequency
// adjustments of an NCO.
///_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

#ifndef __PHASELOCKEDLOOP__
#define __PHASELOCKEDLOOP__

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>

#include "PhaseDetector.h"
#include "LoopFilter.h"
#include "Nco.h"

class PhaseLockedLoop
{
  //***************************** operations **************************

  public:

  PhaseLockedLoop(float sampleRate,
                  float maxNcoFrequency,
                  float initialNcoFrequency);

 ~PhaseLockedLoop(void);

  void run(int8_t *bufferPtr,uint32_t bufferLength);
  void reset(void);

  private:

  //*******************************************************************
  // Utility functions.
  //*******************************************************************
  void updateNcoFrequency(int8_t iData,int8_t qData);
  void derotateSignal(int8_t *iDataPtr,int8_t *qDataPtr);

  //*******************************************************************
  // Attributes.
  //*******************************************************************
  
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // Loop parameters. 
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
 // Noise equivalent bandwidth.
  float Bn;

  // Damping factor.
  float zeta;

  // NCO gain.
  float K0;

  // Phase detector gain.
  float Kd;

  // Proportional gain of loop filter.
  float Kp;

  // Integrator gain of loop filter.
  float Ki;
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

  // This is, after all, a sampled system.
  float sampleRate;

  // This introduces saturation nonlinearity.
  float maxNcoFrequency;

  // This frequency is then initial value of the NCO at startup.
  float initialNcoFrequency;

  // Have it here to make debugging easier.
  float ncoFrequency;

  // We need to supply these to the phase detector.
  float iNco;
  float qNco;

  PhaseDetector *detectorPtr;
  LoopFilter *filterPtr;
  Nco *ncoPtr;
};

#endif // __PHASELOCKEDLOOP__
