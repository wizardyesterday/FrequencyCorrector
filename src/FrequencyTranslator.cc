//************************************************************************
// file name: FrequencyTranslator.cc
//************************************************************************
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

#include "FrequencyTranslator.h"

using namespace std;

// This provides a quick method for computing the atan2() function.
static float atan2LookupTable[256][256];

/*****************************************************************************

  Name: FrequencyTranslator

  Purpose: The purpose of this function is to serve as the contructor for
  an instance of an FrequencyTranslator.

  Calling Sequence: FrequencyTranslator(sampleRate,numberOfAverages)

  Inputs:

    pcmCallbackPtr - The sample rate of the IQ data in S/s.

    numberOfAverages - The number of frequency errors to average.

 Outputs:

    None.

*****************************************************************************/
FrequencyTranslator::FrequencyTranslator(
  float sampleRate,
  uint32_t numberOfAverages)
{
  int x, y;
  double xArg, yArg;

  // Save for later processing.
  this->numberOfAverages = numberOfAverages;

  // No computations have been performed yet.
  frequencyMeasurementCount = 0;
  accumulatedFrequencyError = 0;

  // Instatiate with a frequency of zero.
  ncoPtr = new Nco(sampleRate,0);

  // Construct lookup table.
  for (x = 0; x < 256; x++)
  {
    for (y = 0; y < 256; y++)
    {
      // Set arguments into the proper range.
      xArg = (double)x - 128;
      yArg = (double)y - 128;

      // Save entry into the table.
      atan2LookupTable[y][x] = atan2(yArg,xArg);
    } // for
  } // for

  // Set to a nominal gain.
  demodulatorGain = sampleRate / (2 * M_PI);

  // Initial phase angle for d(theta)/dt computation.
  previousTheta = 0;

  return;

} // FrequencyTranslator

/*****************************************************************************

  Name: ~FrequencyTranslator

  Purpose: The purpose of this function is to serve as the destructor for
  an instance of an FrequencyTranslator.

  Calling Sequence: ~FrequencyTranslator()

  Inputs:

    None.

  Outputs:

    None.

*****************************************************************************/
FrequencyTranslator::~FrequencyTranslator(void)
{

  // Release resources.
  if (ncoPtr != NULL)
  {
    delete ncoPtr;
  } // if

  return;

} // ~FrequencyTranslator

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
void FrequencyTranslator::resetDemodulator(void)
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
void FrequencyTranslator::acceptIqData(
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
uint32_t FrequencyTranslator::demodulateSignal(int8_t *bufferPtr,
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
void FrequencyTranslator::computeFrequencyError(uint32_t sampleCount)
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
float FrequencyTranslator::getFrequencyError(void)
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
void FrequencyTranslator::derotateSignal(
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
