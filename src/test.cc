#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "FrequencyTranslator.h"
#include "PhaseLockedLoop.h"

PhaseDetector *detPtr;
LoopFilter *filterPtr;
FrequencyTranslator *translatorPtr;
PhaseLockedLoop *pllPtr;

int main(int argc,char **argv)
{
  bool done;
  uint32_t count;
  float frequencyError;
  int8_t inputBuffer[16384];

  // Create a frequency translator.
  translatorPtr = new FrequencyTranslator(256000,64);

  // Create a phase-locked loop.
  pllPtr = new PhaseLockedLoop(256000);

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
      // Run the phase-locked loop.
      pllPtr->run(inputBuffer,16384);

//      translatorPtr->acceptIqData(inputBuffer,count);
//      frequencyError = translatorPtr->getFrequencyError();

//      fprintf(stderr,"Frequency Error: %f\n",frequencyError);

        fwrite(inputBuffer,sizeof(int8_t),count,stdout);
    } // else
  } // while

  // Release resources.
  delete pllPtr;
  delete translatorPtr;

  return (0);

} // main
