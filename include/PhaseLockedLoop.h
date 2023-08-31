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

  PhaseLockedLoop(float sampleRate);
 ~PhaseLockedLoop(void);

  float run(float iValue,float qValue);
  bool derotateSignal(int8_t iData,int8_t qData);
  bool isLocked(void);

  private:

  //*******************************************************************
  // Utility functions.
  //*******************************************************************
  // This is the phase detector.
  float computePhaseError(float iReference,float qReference,
                          float *iNcoOutputPtr,float *qNcoOutputPtr);

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

  // Current output values of NCO.
  float iNcoOutput;
  float qNcoOutput;

  // Deadband for lock.
  float lockErrorThreshold;

  PhaseDetector *detectorPtr;
  LoopFilter *filterPtr;
  Nco *ncoPtr;
};

#endif // __PHASELOCKEDLOOP__
