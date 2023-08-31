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

#if 0
/*****************************************************************************

  Name: resetDemodulator

  Purpose: The purpose of this function is to reset the demodulator to its
  initial condition.

  Calling Sequence: resetDemodulator()

  Inputs:

    None.

  Outputs:

    None.

*****************************************************************************/
void PhaseLockedLoop::resetDemodulator(void)
{

  // Initial phase angle for d(theta)/dt computation.
  previousTheta = 0;

  return;

} // resetDemodulator

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
  uint32_t sampleCount;

  // Demodulate the signal.
  sampleCount = demodulateSignal(bufferPtr,bufferLength);

  // We need this for derotating the signal.
  computeFrequencyError(sampleCount);

  // Get the signal to baseband.
  derotateSignal(bufferPtr,bufferLength);

  return;

} // acceptIqData

/*****************************************************************************

  Name: demodulateSignal

  Purpose: The purpose of this function is to demodulate an FM signal that
  is stored in the iData[] and qData[] arrays.

  Calling Sequence: demodulateSignal(bufferPtr,bufferLength)

  Inputs:

    bufferPtr - A pointer to IQ data.

    bufferLength - The number of bytes contained in the buffer that is
    in the buffer.

  Outputs:

    sampleCount - The number of samples stored in the demodulatedData[]
    array.

*****************************************************************************/
uint32_t PhaseLockedLoop::demodulateSignal(int8_t *bufferPtr,
                                       uint32_t bufferLength)
{
  uint32_t sampleCount;
  uint32_t i;
  uint8_t iData, qData;
  float theta;
  float deltaTheta;

  // We're mapping interleaved data into two separate buffers.
  sampleCount = bufferLength / 2;

  for (i = 0; i < sampleCount; i++)
  {
    // Compute lookup table indices.
    iData = (uint8_t)bufferPtr[2 * i] + 128;
    qData = (uint8_t)bufferPtr[(2 * i) + 1] + 128;

    // Compute phase angle.
    theta = atan2LookupTable[qData][iData];

    //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
    // Compute d(theta)/dt.
    //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
    deltaTheta = theta - previousTheta;

    //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
    // We want -M_PI <= deltaTheta <= M_PI.
    //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
    while (deltaTheta > M_PI)
    {
      deltaTheta -= (2 * M_PI);
    } // while

    while (deltaTheta < (-M_PI))
    {
      deltaTheta += (2 * M_PI);
    } // while
    //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

    // Store the normalized demodulated data.
    demodulatedData[i] = deltaTheta;
 
    // Update our last phase angle for the next iteration.
    previousTheta = theta;

  } // for

  return (sampleCount);

} // demodulateSignal

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
void PhaseLockedLoop::computeFrequencyError(uint32_t sampleCount)
{
  uint32_t i;
  float sum;
  float *dataPtr;

  // Reference the begining of the demodulated data.
  dataPtr = demodulatedData;

  // This represents a frequency error over a block of IQ data.
  sum = 0;

  for (i = 0; i < sampleCount; i++)
  {
    sum += *dataPtr;

    // Reference next item.
    dataPtr++;
  } // for

  // Convert to an average frequency error for this block of data.
  sum /= sampleCount;

  // Accumulate the next average.
  accumulatedFrequencyError += sum;

  // One more measurement has been made.
  frequencyMeasurementCount++;

  if (frequencyMeasurementCount == numberOfAverages)
  {
    // Convert to average value of the averages.
    accumulatedFrequencyError /= numberOfAverages;

    // Map to actual frequency.
    frequencyError = accumulatedFrequencyError * demodulatorGain;

    // Set the NCO to the negative of the frequency error.
    ncoPtr->setFrequency(-frequencyError);

    // Reset for the next set of measurements.
    accumulatedFrequencyError = 0;
    frequencyMeasurementCount = 0;
  } // if

  return;

} // computeFrequencyError

/*****************************************************************************

  Name: getFrequencyError

  Purpose: The purpose of this function is to return the frequency error
  from the last processed block of IQ data.

  Calling Sequence: frequency = getFrequencyError()

  Inputs:

    None.

  Outputs:

    frequency - The frequency offset from the center frequency.  Ideally,
    it should be zero.

*****************************************************************************/
float PhaseLockedLoop::getFrequencyError(void)
{

  return (frequencyError);

} // getFrequencyError

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
void PhaseLockedLoop::derotateSignal(
  int8_t *bufferPtr,
  uint32_t bufferLength)
{
  uint32_t i;
  float iIn;
  float qIn;
  float iNco;
  float qNco;
  float iOut;
  float qOut;

  for (i = 0; i < bufferLength; i += 2)
  {
    // Retrieve in-phase component.
    iIn = bufferPtr[i];

    // Retrieve quadrature component.
    qIn = bufferPtr[i+1];

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
    bufferPtr[i] = (int8_t)iOut;
    bufferPtr[i+1] = (int8_t)qOut;
  } // for  

} // derotateSignal
#endif
