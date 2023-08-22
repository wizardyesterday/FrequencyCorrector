//************************************************************************
// file name: FrequencyErrorEstimator.cc
//************************************************************************
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#include "FrequencyErrorEstimator.h"

using namespace std;

// This define represents a reasonable maximum lag for autocorrelation.
#define MAXIMUM_LAG (4)

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
// The history array is dimensioned so that a symmetric spectral
// distribution can be accomodated.  The index that represents
// zero frequency error is at the midpoint of the array.  Both
// positive and negative frequency errors (about the center
// frequency) are handled.
// Make sure that division of the maximum frequency error by the
// frequency error resolution results in an integer value since
// the histogram size is a function of this integer value.
// The oscillator accuracy of a handheld radio is 1.5ppm, and it
// is noted that the oscillator accuracy of the M4C8 is 2ppm.
// That would determine that the maximum frequency error is
// (1.5 + 2)*850MHz = 2975Hz.  Now assuming that the relative
// speed between the source and observer is 600mph, the doppler
// shift at 850MHz is 760Hz.  That translates to a frequency
// acceptance gate of 2975Hz + 760Hz = 3735Hz.  In order to
// have a frequency acceptance gate that is a multiple of 50Hz,
// a value of 3750Hz is chosen.
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
#define MAXIMUM_FREQUENCY_ERROR_IN_HZ (3750)
//#define FREQUENCY_ERROR_RESOLUTION_IN_HZ (50)
#define FREQUENCY_ERROR_RESOLUTION_IN_HZ (2)

#define HISTOGRAM_SIZE ((2 * MAXIMUM_FREQUENCY_ERROR_IN_HZ /\
                        FREQUENCY_ERROR_RESOLUTION_IN_HZ) + 1)
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

/*****************************************************************************

  Name: FrequencyErrorEstimator

  Purpose: The purpose of this function is to serve as the constructor for
  an instance of an FrequencyErrorEstimator.

  Calling Sequence: FrequencyErrorEstimator(Fs,
                                              lag,
                                              accumulationCountThreshold,
                                              callbackPtr,
                                              callbackContextPtr);

  Inputs:

    Fs - The sample rate in S/s.

    lag - The lag that will be used for crosscorrelation purposes.

    accumulationCountThreshold - The number of accumulations of histogram
    data before the client callback is invoked.  For example, if the
    value of this parameter were 16, the accumulateFrequencyError()
    method would invoke the callback function after 16 invocations of the
    accumulateFrequencyError() method under the constraint that a usable
    signal was detected.  For example if one invocation determined that
    no signal was present, then 17 invocations would be invoked.  If 2
    invocations occurred for which no useful signal was detected, 18
    invocations would occur before the callback function was invoked.

    callbackPtr - A pointer to a static callback function that is provided
    by the client.  If the callback pointer has a value of NULL, the client
    will be expected to retrieve the estimated value of the frequency
    error via invocation of the retrieveFrequencyError() method followed
    by an invocation of the reset() method.  In this latter case, it is
    the responsibility of the client to manage any sort of averaging
    that needs to be performed.

    callbackContextPtr - A pointer to private data supplied by the client.

  Outputs:

    None.

*****************************************************************************/
FrequencyErrorEstimator::FrequencyErrorEstimator(
  float Fs,
  int32_t lag,
  uint32_t accumulationCountThreshold,
  void (*callbackPtr)(int16_t frequencyError,void *contextPtr),
  void *callbackContextPtr)
{

  // Retrieve the sample rate.
  this->Fs = Fs;

  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // Ensure that no insane values are used.
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // Keep the lag small to ensure proper autocorrelation.
  if (lag > MAXIMUM_LAG)
  {
    // Clip it.
    lag = MAXIMUM_LAG;
  } // if

  // This has to be greater than zero.
  if (accumulationCountThreshold == 0)
  {
    // Default to no averaging being performed.
    accumulationCountThreshold = 1;
  } // if
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

  // The lag is used for autocorrelation computation.
  this->lag = lag;

  // Save for averaging.
  this->accumulationCountThreshold = accumulationCountThreshold;

  // Save for the case of asynchronous notification.
  this->callbackPtr = callbackPtr;
  this->callbackContextPtr = callbackContextPtr;

  // The histogram represents the distribution of estimated frequency errors.
  histogramPtr = new uint32_t[HISTOGRAM_SIZE];

  // Clear all runtime data.
  reset();

  return;

} // FrequencyErrorEstimator

/*****************************************************************************

  Name: ~FrequencyErrorEstimator

  Purpose: The purpose of this function is to serve as the destructor for
  an instance of an FrequencyErrorEstimator.

  Calling Sequence: ~FrequencyErrorEstimator()

  Inputs:

    None.

  Outputs:

    None.

*****************************************************************************/
FrequencyErrorEstimator::~FrequencyErrorEstimator(void)
{

  // Release resources.
  delete[] histogramPtr;

  return;

} // ~FrequencyErrorEstimator

/*****************************************************************************

  Name: reset

  Purpose: The purpose of this function is to reset all runtime values to.
  initial values.  This includes negating the signal detection flag and
  clearing the histogram memory.  This method is invoked by the
  accumulateFrequencyError() method if a callback function was provided
  by the client, otherwise, it is the responsibility of the client
  to invoke this method.

  Calling Sequence: reset()

  Inputs:

    None.

  Outputs:

    None.

*****************************************************************************/
void FrequencyErrorEstimator::reset(void)
{
  uint32_t i;

  // Set to no accumulations.
  accumulationCount = 0;

  // Clear the histogram for the next accumulation of data..
  for (i = 0; i < HISTOGRAM_SIZE; i++)
  {
    histogramPtr[i] = 0;
  } // for

  return;

} // reset

/*****************************************************************************

  Name: run 

  Purpose: The purpose of this function is to perform an autocorrelation
  of the phase values in order to estimate the error in carrier frequency.
  The equation for the autocorrelation function is,

    sum = summation[lag,bufferLength](x(n)x*(n-lag)],

  where "*" represents the complex conjugate of x(n).

  After the sum is computed, the argument of the sum, arg() function,
  also known as atan2() is computed.  Additionally, it is ensured that
  the result of the atan2() function is bounded by its principal value
  (-PI < atan2(Im{sum),Re{sum}) < PI).  The actual name for such a
  function is Arg() (notice the capitol A which indicates principal
  value (from complex variable theory).

  Now, these computations are only carried out if the incoming signal
  exceeds a threshold.  Given that the signal does exceed the threshold,
  a detection counter is incremented that determines the number of times
  that the signal had exceeded the threshold.

  Given that the detection counter exceeds the number of times that the
  threshold is equal to, the following steps are carried out.

  The error frequency is next computed as ,

    frequencyError = (Fs * Arg(sum))/(2 * PI * lag)

  Finally, given that the signal had exceed the threshold, a histogram,
  that represents the distribution of estimated error frequencies, is
  updated.  Let it be noted that the signal must exceed the threshold
  a number of times that is equal to the value of bufferLength minus the
  lag.  For this reason, the number of samples presented to this function
  should be short enough such that it is associated with the minimum time
  that a signal might be available. Noting that the sample rate is 24000
  Samples/second, a bufferLength value of 2400 would be 0.1 seconds.
  Generally a voice transmission would exist longer than 0.1 seconds, so
  it is reasonable to expect the number of signal detections to be equal
  to the bufferLength parameter minus the lag. 

  Now, if enough collections have occurred (this function has been invoked
  a configurable number of times), if an asynchronous model is being used
  (a callback function has been registered), the client will be
  notified of the estimated frequency, and all data associated with the
  collected data will be cleared.  The next accumulation of data can now
  occur, thus, repeating the cycle.

  As a side note, the paper that prompted me to design this system is:
  "Digital Carrier Frequency Estimation For Multilevel CPM Signals" by
  A. N. D'Andrea, A. Ginesi and U. Mengali.

  Calling Sequence: run(magnitudePtr,phasePtr,ufferLength)

  Inputs:

    magnitudePtr - A pointer to signal envelope data.

    phasePtr - A pointer to signal phase values.  The phase values are
    signed fractional values with a sign bit, 2 mantissa bits, and 13
    fractional bits yielding a range of -PI < phase < PI.

    bufferLength - The number of samples in each buffer referenced by
    magnitudePtr and phasePtr.

  Outputs:

    None.

*****************************************************************************/
void FrequencyErrorEstimator::run(
  int16_t *inPhasePtr,
  int16_t *quadrturePtr,
  uint32_t bufferLength)
{
  uint32_t i;
  float sumRe, sumIm;
  float frequencyError;
  float deltaTheta;
  float a, b, c, d;
  uint32_t histogramIndex;
  int16_t quantizedFrequencyError;
  bool dummy;

  // Ensure that intermediate results are cleared.
  sumRe = 0;
  sumIm = 0;

  // Perform the autocorrelation computation (and other housekeeping).
  for (i = lag; i < bufferLength; i++)
  {
      //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
      // Convert to our complex quantities.
      //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
      a = (float)inPhasePtr[i];
      b = (float)quadrturePtr[i];
      c = (float)inPhasePtr[i - lag];
      d = (float)quadrturePtr[i - lag];
      //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

      //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
      // Carry out the complex conjugate multiplication, that is,
      // sumRe + j*sumIm += (a +jb)(c - jb).
      //
      // Remember (c + jb)* = (c - jb), where "*" indicates the
      // complex conjugate of a number.
      //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
      sumRe = sumRe + ((a * c) + (b * d));
      sumIm = sumIm + ((b * c) - (a * d));
      //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  } // for

  if (sumIm == 0)
  {
    // Avoid passing Im{sum} value of 0 to the atan2() function.
    sumIm = 0.000001;
  } // if

  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // Compute the Arg() function.  Note that Arg() represents the
  // principal value of arg().  The principal value is enforced
  // by the while loops that follow the computation of deltaTheta.
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // Start out with the arg() computation.
  deltaTheta = atan2(sumIm,sumRe);

  //-------------------------------------------------------------------
  // Handle the branch cut. That is, force it to be the Arg() function
  // so that the principal value results.
  //-------------------------------------------------------------------
  while (deltaTheta > M_PI)
  {
    deltaTheta -= (2 * M_PI);
  } // while

  while (deltaTheta < (-M_PI))
  {
    deltaTheta += (2 * M_PI);
  } // while
  //-------------------------------------------------------------------
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

  // The frequency error is the scaled value of Arg(sum).
  frequencyError = (Fs * deltaTheta) / (2 * M_PI * lag);

  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // Make sure that the frequency error is within tolerance.
  // Anything outside of the frequency limits is considered to be
  // an outlier and will be rejected.
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  if ((frequencyError > - MAXIMUM_FREQUENCY_ERROR_IN_HZ) &&
      (frequencyError < MAXIMUM_FREQUENCY_ERROR_IN_HZ))
  {
    // Indicate that one more accumulation has occurred.
    accumulationCount++;

    histogramIndex =
      (int32_t)roundf(frequencyError / FREQUENCY_ERROR_RESOLUTION_IN_HZ);

    // The dc index must be accounted for, hence, the (-1).
    histogramIndex += ((HISTOGRAM_SIZE - 1) / 2);

    histogramPtr[histogramIndex]++;

    // We have enough data for a good statistical average.
    if (accumulationCount >= accumulationCountThreshold)
    {
      // An asynchronous model is being used.
      if (callbackPtr != NULL)
      {
         // We know we have signal detection, hence, ignore the result.
         retrieveFrequencyError(&quantizedFrequencyError);

        // Notify the client of new data.
        callbackPtr(quantizedFrequencyError,callbackContextPtr);
      } // if

      // Reset runtime data for the next collection.
      reset();
    } // if
  } // if

  return;

} // run

/*****************************************************************************

  Name: retrieveFrequencyError 

  Purpose: The purpose of this function is to search the histogram
  for the maximum value, and map the index of the maximum to a
  quantized frequency error.  This frequency error will be considered
  as valid if a signal has been detected, otherwise, the value is not
  usable by the caller.

  Note that the caller the accumulateFrequencyError() method if a
  callback function was provided by the client.  If a callback
  function was not provided by the client, it is the responsibility of
  the client to invoke this function and invoke the reset() method to
  start with a clean slate for the next collection of data.

  Calling Sequence: retrieveFrequencyError(frequencyErrorPtr)

  Inputs:

    frequencyErrorPtr - A pointer to storage of the frequency error
    result.  The resolution is FREQUENCY_ERROR_RESOLUTION_IN_HZ.

  Outputs:

    None.

*****************************************************************************/
void FrequencyErrorEstimator::retrieveFrequencyError(
  int16_t *frequencyErrorPtr)
{
  uint32_t i;
  uint16_t indexOfMaximum;

  // Avoid compiler warnings.
  indexOfMaximum = 0;

  // Search for the maximum value in the histogram.
  for (i = 1; i < HISTOGRAM_SIZE; i++)
  {
    if (histogramPtr[i] > histogramPtr[indexOfMaximum])
    {
      indexOfMaximum = i;
    } // if
  } // for

  // Convert index into frequency with the appropriate resolution.
  // The dc index must be accounted for, hence, the (-1).
  *frequencyErrorPtr = indexOfMaximum - ((HISTOGRAM_SIZE - 1) / 2);
  *frequencyErrorPtr *= FREQUENCY_ERROR_RESOLUTION_IN_HZ;

  return;

} // retrieveFrequencyError

