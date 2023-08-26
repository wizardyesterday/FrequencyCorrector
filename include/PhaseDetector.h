//**************************************************************************
// file name: PhaseDetector.h
//**************************************************************************
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
// This class implements a signal processing block that performs a
// phase detector function, that is needed by a phase-locked loop.  The
// system is implemented using the polar discriminant method.  The
// advantage to this method is that the phase angle is confined to
// -PI < theta < PI.  That is, no phase wrapping is necessary.
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

#ifndef __PHASEDETECTOR__
#define __PHASEDETECTOR__

#include <stdint.h>

class PhaseDetector
{
  //***************************** operations **************************

  public:

  PhaseDetector(float gain);

  ~PhaseDetector(void);

  float computePhaseError(float iRef, float qRef,float iNco,float qNco);

  //***************************** attributes **************************
  private:

  // The gain is needed when computing the phase error.
  float gain;

};

#endif // __PHASEDETECTOR__
