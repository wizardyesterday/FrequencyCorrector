//************************************************************************
// file name: PhaseLockedLoop.cc
//************************************************************************
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

#include "PhaseLockedLoop.h"

using namespace std;

/*****************************************************************************

  Name: PhaseLockedLoop

  Purpose: The purpose of this function is to serve as the contructor for
  an instance of an PhaseLockedLoop.

  Note: I decided to break up the computations for the some of the loop
  parameters,  I'm not a fan of dealing with nasty/complicated
  expressions, hence, I de-uglified the code.

  Calling Sequence: PhaseLockedLoop(sampleRate,numberOfAverages)

  Inputs:

    pcmCallbackPtr - The sample rate of the IQ data in S/s.

    numberOfAverages - The number of frequency errors to average.

 Outputs:

    None.

*****************************************************************************/
PhaseLockedLoop::PhaseLockedLoop(float sampleRate)
{
  float factor1;
  float factor2;
  float factor3;
  float factor4;

  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // Set loop parameters. 
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // Set noise equivalent bandwidth: 0.01*Fs <= Bn <= 0.05*Fs
  Bn = 0.01 * sampleRate;

  // Set damping factor for a critically damped system.
  zeta = 1 / sqrt(2);

  // set the NCO gain.
  K0 = 1;

  // Set the phase detector gain.
  Kd = 0.5;

  //************************************************************
  // Set the proportional gain of loop (PI) filter.
  //************************************************************
  // Let's break up the computations for,
  // Kp = 1/(Kd*K0) * 4*zeta/(zeta + 1/(4*zeta)) * Bn/Fs
  //************************************************************
  factor1 = 1 / (Kd * K0);

  factor2 = 4 * zeta;

  factor3 = zeta + (1/(4 * zeta));
  factor3 = 1 / factor3;

  factor4 = (Bn / sampleRate);

  // Put it all together.
  Kp = factor1 * factor2 * factor3 * factor4;
  //************************************************************

  //************************************************************
  // Set the integrator gain for the (PI) loop filter.
  //************************************************************
  // Let's break up the computations for,
  // Ki = 1/(Kd*K0) * 4*zeta/(zeta + 1/(4*zeta))^2  * (Bn/Fs)^2
  //************************************************************
  factor1 = 1 / (Kd * K0);

  factor2 = 4 * zeta;

  factor3 = zeta + (1/(4 * zeta));
  factor3 = factor3 * factor3;
  factor3 = 1 / factor3;

  factor4 = (Bn / sampleRate);
  factor4 = factor4 * factor4;

  // Put it all together.
  Ki = factor1 * factor2 * factor3 * factor4;
  //************************************************************

 //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

  // Instantiate the loop filter.
  filterPtr = new LoopFilter(Kp,Ki);

  // Instantiate the phase detector.
  detectorPtr = new PhaseDetector(Kd);

  // Instatiate the NCO at a free-running frequency of 200Hz.
  ncoPtr = new Nco(sampleRate,200);

  return;

} // PhaseLockedLoop

/*****************************************************************************

  Name: ~PhaseLockedLoop

  Purpose: The purpose of this function is to serve as the destructor for
  an instance of an PhaseLockedLoop.

  Calling Sequence: ~PhaseLockedLoop()

  Inputs:

    None.

  Outputs:

    None.

*****************************************************************************/
PhaseLockedLoop::~PhaseLockedLoop(void)
{

  // Release resources.
  if (detectorPtr != NULL)
  {
    delete detectorPtr;
  } // if

  if (filterPtr != NULL)
  {
    delete filterPtr;
  } // if

  if (ncoPtr != NULL)
  {
    delete ncoPtr;
  } // if

  return;

} // ~PhaseLockedLoop

/*****************************************************************************

  Name: reset

  Purpose: The purpose of this function is to reset the PLL to its
  initial condition.

  Calling Sequence: reset()

  Inputs:

    None.

  Outputs:

    None.

*****************************************************************************/
void PhaseLockedLoop::reset(void)
{

  // Reset the dynamic components of the system.
  filterPtr->reset();
  ncoPtr->reset();

  // Set the free-running frequency to 200Hz.
  ncoPtr->setFrequency(200);

  return;

} // reset

/*****************************************************************************

  Name: acceptIqData

  Purpose: The purpose of this function is to perform the high level
  processing that is associated with the processing of a signal.

  Calling Sequence: acceptIqData(bufferPtr,bufferLength)

  Inputs:

    bufferPtr - A pointer to IQ data.

    bufferLength - The number of bytes contained in the buffer that is
    in the buffer.

  Outputs:

    None.

*****************************************************************************/
void PhaseLockedLoop::acceptIqData(
  int8_t *bufferPtr,
  uint32_t bufferLength)
{
  uint32_t i;
  float frequencyError;

  for (i = 0; i < bufferLength; i+= 2)
  {
    // First, the frequency offset, from baseband is computed.
    frequencyError = computeFrequencyError(bufferPtr[i],bufferPtr[i+1]); 

    // Set the NCO to the negative of the frequency error.
    ncoPtr->setFrequency(-frequencyError);

    // Perform the frequency correction.
    derotateSignal(&bufferPtr[i],&bufferPtr[i+1]);
  } // for

  return;

} // acceptIqData

/*****************************************************************************

  Name: computeFrequencyError

  Purpose: The purpose of this function is to compute the frequency
  error in a block of IQ samples.

  Calling Sequence: computeFrequencyError(sampleCount)

  Inputs:

    sampleCount - The number of samples in the demodulated data buffer.

  Outputs:

    None.

*****************************************************************************/
float PhaseLockedLoop::computeFrequencyError(int8_t iData,int8_t qData)
{
  float phaseError;
  float frequencyError;

  // Compute phase error.
  phaseError = detectorPtr->computePhaseError((float)iData,
                                             (float)qData,
                                             iNco,
                                             qNco);
  // Filter the phase error.
  phaseError = filterPtr->filterData(phaseError);

  // Convert to a frequency error.
  frequencyError = K0 * phaseError * sampleRate / (2 * M_PI);

  return (frequencyError);

} // computeFrequencyError

/*****************************************************************************

  Name: derotateSignal

  Purpose: The purpose of this function is to derotate a block of IQ data.

  Calling Sequence: derotateSignal(bufferPtr,bufferLength)

  Inputs:

    bufferPtr - A pointer to IQ data.

    bufferLength - The number of bytes contained in the buffer that is
    in the buffer.

  Outputs:

    None.

*****************************************************************************/
void PhaseLockedLoop::derotateSignal(int8_t *iDataPtr,int8_t *qDataPtr)
{
  float iIn;
  float qIn;
  float iOut;
  float qOut;

  // Retrieve in-phase component.
  iIn = (float)(*iDataPtr);

  // Retrieve quadrature component.
  qIn = (float)(*qDataPtr);

  // Retrieve the complex NCO output.
  ncoPtr->run(&iNco,&qNco);

  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // Perform the frequency translation.  From complex analysis,
  // (a + jb)(c + jd) = (ac - bd) + j(bc + ad).
  // In our case,
  // iIn is a.
  // qIn is b.
  // iNco is c.
  // qNco is d.
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // Compute in-phase component of output.
  iOut = (iIn * iNco) - (qIn * qNco);

  // Compute quadrature component of output.
  qOut = (qIn * iNco) + (iIn * qNco);    
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

  // Store the outputs in place.
  *iDataPtr = (int8_t)iOut;
  *qDataPtr = (int8_t)qOut;

  return;

} // derotateSignal
