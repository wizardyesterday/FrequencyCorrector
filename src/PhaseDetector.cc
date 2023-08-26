//************************************************************************
// file name: PhaseDetector.cc
//************************************************************************
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#include "PhaseDetector.h"

using namespace std;

/*****************************************************************************

  Name: PhaseDetector

  Purpose: The purpose of this function is to serve as the constructor for
  an instance of a PhaseDetector.

  Calling Sequence: PhaseDetector(gain);

  Inputs:

    gain - The gain of the phase detector.

  Outputs:

    None.

*****************************************************************************/
PhaseDetector::PhaseDetector(float gain)
{

  // Save for phase error computations.
  this->gain = gain;

  return;

} // PhaseDetector

/*****************************************************************************

  Name: ~PhaseDetector

  Purpose: The purpose of this function is to serve as the destructor for
  an instance of a PhaseDetector.

  Calling Sequence: ~PhaseDetector()

  Inputs:

    None.

  Outputs:

    None.

*****************************************************************************/
PhaseDetector::~PhaseDetector(void)
{

  return;

} // ~PhaseDetector

/*****************************************************************************

  Name: computePhaseError

  Purpose: The purpose of this function is to compute the phase difference
  between the reference signal and an NCO signal.  These signals are
  complex quantities.

  Calling Sequence: phaseError = computePaseError(iRef,qRef,iNco,qNco)

  Inputs:

    iRef - The in-phase component of the reference (input) signal.

    qRef - The quadrature component of the reference (input) signal.

    iNco - The in-phase component of the NCO signal.

    qNco - The quadrature component of the NCO signal.


  Outputs:

    phaseError - The phase difference between the reference (input)
    signal and the NCO signal, scaled by the gain.

*****************************************************************************/
float PhaseDetector::computePhaseError(
  float iRef,
  float qRef,
  float iNco,
  float qNco)
{
  float iOut;
  float qOut;
  float phaseError;

  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // Multiply the reference signal by the complex conjugate
  // of the NCO signal.  The form of the multiply is,
  // z = I + jQ = (a + jb)(c - jd) = (ac + bd) + j(bc - ad)
  // The roles of the input parameters appear below.
  //  a = iRef
  //  b = qRef
  //  c = iNco
  //  d = qNco
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // Compute in-phase part.
  iOut = (iRef * iNco) + (qRef * qNco);

  // Compute quadrature part.
  qOut = (qRef * iNco) - (iRef * qNco);
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

  // Compute the phase error multiplied by the gain.
  phaseError = atan2(qOut,iOut);
  phaseError *= gain;

  return (phaseError);

} // computePhaseError

