//*************************************************************************
// File name: pll.cc
//*************************************************************************

//*************************************************************************
// This program provides the ability to phase-lock and frequency-lock to
// IQ (In-phase or Quadrature) signals that are provided by stdin.  The
// data is 8-bit signed 2's complement, and is formatted as
// I1,Q1; I2,Q2; ...
// The phase-derotated samples will be written to stdout.
//
// To run this program type,
// 
//    ./pll > -d <displaytype> -r <sampleRate> -f <maxNcoFrequency>
//            -i <initialNcoFrequency> <inputFile
//
// where,
//
//    sampleRate - The sample rate of the IQ data in S/s.
//
//    maxNcoFrequency - The maximum frequency (magnitude) that the NCO
//    can operate at.
//
//    initialNcoFrequency - The frequency of the NCO when the system
//    first starts up.
///*************************************************************************
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "PhaseLockedLoop.h"

// This structure is used to consolidate user parameters.
struct MyParameters
{
  float *sampleRatePtr;
  float *maxNcoFrequencyPtr;
  float *initialNcoFrequencyPtr;
};

/*****************************************************************************

  Name: getUserArguments

  Purpose: The purpose of this function is to retrieve the user arguments
  that were passed to the program.  Any arguments that are specified are
  set to reasonable default values.

  Calling Sequence: exitProgram = getUserArguments(parameters)

  Inputs:

    parameters - A structure that contains pointers to the user parameters.

  Outputs:

    exitProgram - A flag that indicates whether or not the program should
    be exited.  A value of true indicates to exit the program, and a value
    of false indicates that the program should not be exited..

*****************************************************************************/
bool getUserArguments(int argc,char **argv,struct MyParameters parameters)
{
  bool exitProgram;
  bool done;
  int opt;

  // Default not to exit program.
  exitProgram = false;

  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // Default parameters.
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // Default to 256000S/s.
  *parameters.sampleRatePtr = 256000;

  // Default to 2000Hz.
  *parameters.maxNcoFrequencyPtr = 2000;

  // Default to 0Hz.
  *parameters.initialNcoFrequencyPtr = 0;
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

  // Set up for loop entry.
  done = false;

  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // Retrieve the command line arguments.
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/ 
  while (!done)
  {
    // Retrieve the next option.
    opt = getopt(argc,argv,"r:f:i:-Dh");

    switch (opt)
    {
      case 'r':
      {
        *parameters.sampleRatePtr = atof(optarg);
        break;
      } // case

      case 'f':
      {
        *parameters.maxNcoFrequencyPtr = atof(optarg);
        break;
      } // case

      case 'i':
      {
        *parameters.initialNcoFrequencyPtr = atof(optarg);
        break;
      } // case

      case 'h':
      {
        // Display usage.
        fprintf(stderr,"./pll -r sampleRate (S/s) -f maxNcoFrequency "
                "-i initialNcoFrequency\n");

        // Indicate that program must be exited.
        exitProgram = true;
        break;
      } // case

      case -1:
      {
        // All options consumed, so bail out.
        done = true;
      } // case
    } // switch

  } // while
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

  return (exitProgram);

} // getUserArguments

//*************************************************************************
// Mainline code.
//*************************************************************************
int main(int argc,char **argv)
{
  bool done;
  bool exitProgram;
  uint32_t count;
  int8_t inputBuffer[16384];
  float sampleRate;
  float maxNcoFrequency;
  float initialNcoFrequency;
  PhaseLockedLoop *pllPtr;
  struct MyParameters parameters;

   //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // Default parameters.
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  parameters.sampleRatePtr = &sampleRate;
  parameters.maxNcoFrequencyPtr = &maxNcoFrequency;
  parameters.initialNcoFrequencyPtr = &initialNcoFrequency;
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

  // Retrieve the system parameters.
  exitProgram = getUserArguments(argc,argv,parameters);

  if (exitProgram)
  {
    // Bail out.
    return (0);
  } // if

  // Create a phase-locked loop.
  pllPtr = new PhaseLockedLoop(sampleRate,maxNcoFrequency,initialNcoFrequency);

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

      // Write the phase-derotated data to stdout.
      fwrite(inputBuffer,sizeof(int8_t),count,stdout);
    } // else
  } // while

  // Release resources.
  delete pllPtr;

  return (0);

} // main
