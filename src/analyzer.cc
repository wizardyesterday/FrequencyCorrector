//*************************************************************************
// File name: analyzer.cc
//*************************************************************************

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

#include "CpmFrequencyErrorEstimator.h"

int16_t inPhaseBuffer[16384];
int16_t quadratureBuffer[16384];

/*****************************************************************************

  Name: processFrequencyError

  Purpose: The purpose of this function is to serve as the callback
  function for the frequency error estimator.

  Calling Sequence: processFrequencyError(frequencyError,contextPtr).

  Inputs:

    frequencyError - The estimated frequency error provided by the
    frequency error estimator subsystem.

    contextPtr - A pointer to private data that is supplied by the client
    that supplied this callback the FrequencyErrorEstimator class.

  Outputs:

    None.

*****************************************************************************/
void processFrequencyError(int16_t frequencyError,void *contextPtr)
{

  fprintf(stderr,"Frequency Error: %d\n",frequencyError);

  // Update the frequency of the phase corrector.
//  mePtr->phaseCorrectorPtr->setFrequency(filteredFrequencyError);

  return;

} // processFrequencyError

//*************************************************************************
// Mainline code.
//*************************************************************************
int main(int argc,char **argv)
{
  bool done;
  uint32_t i, j;
  uint32_t count;
  int8_t inputBuffer[16384];
  float sampleRate;
  CpmFrequencyErrorEstimator *estimatorPtr;

  // Instantiate a frequency error estimator.
  estimatorPtr = new CpmFrequencyErrorEstimator(256000,2,100,
                                                processFrequencyError,NULL);

  // Set up for loop entry.
  done = false;

  while (!done)
  {
    // Read a block of input samples (2 * complex FFT length).
    count = fread(inputBuffer,sizeof(int8_t),16384,stdin);

    if (count == 0)
    {
      // We're done.
      done = true;
    } // if
    else
    {
      // Reference the first item in the target buffers.
      j = 0;

      for (i = 0; i < count; i += 2)
      {
        // Deinterleave the data.
        inPhaseBuffer[j] = (int16_t)inputBuffer[i];
        quadratureBuffer[j] = (int16_t)inputBuffer[i+1];

        // Reference the next storage location.
        j++;
      } // for

      // Compute the frequency error.
      estimatorPtr->run(inPhaseBuffer,quadratureBuffer,(count / 2));
    } // else
  } // while

  // Release resources.
  delete estimatorPtr;

  return (0);

} // main
