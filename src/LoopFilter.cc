//************************************************************************
// file name: LoopFilter.cc
//************************************************************************
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#include "LoopFilter.h"

using namespace std;

/*****************************************************************************

  Name: LoopFilter

  Purpose: The purpose of this function is to serve as the constructor for
  an instance of a LoopFilter.

  Calling Sequence: LoopFilter(proportionalGain,integratorGain);

  Inputs:

    proportinalGain - The gain of the proportional portion of the PI
    filter.

    integratorGain - The gain of the integrator portion of the PI filter.

  Outputs:

    None.

*****************************************************************************/
LoopFilter::LoopFilter(float proportionalGain,
                       float integratorGain,
                       float maximumPhase)
{

  // Save for filtering computations.
  this->proportionalGain = proportionalGain;
  this->integratorGain = integratorGain;
  this->maximumPhase = maximumPhase;

  // Reset filter state.
  reset();

  return;

} // LoopFilter

/*****************************************************************************

  Name: ~LoopFilter

  Purpose: The purpose of this function is to serve as the destructor for
  an instance of a LoopFilter.

  Calling Sequence: ~LoopFilter()

  Inputs:

    None.

  Outputs:

    None.

*****************************************************************************/
LoopFilter::~LoopFilter(void)
{

  return;

} // ~LoopFilter

/*****************************************************************************

  Name: reset

  Purpose: The purpose of this function is to reset the filter to an initial
  state.

  Calling Sequence: reset()

  Inputs:

    None.

  Outputs:

    None.

*****************************************************************************/
void LoopFilter::reset(void)
{

  // Clear.
  recursiveState = 0;

  return;

} // reset

/*****************************************************************************

  Name: filterData

  Purpose: The purpose of this function is to filter data using a
  proportional-integral method.  This is a first-order filter.
  This filter is used specifically for a phase-locked loop that uses a
  phase detector that outputs a phase error between -PI and PI.  Because
  of this, the filter ensures that the output has magnitude no greater
  than PI.

  Calling Sequence: y = filterData(x)

  Inputs:

    x - The input sample.

  Outputs:

    y - The filtered value.

*****************************************************************************/
float LoopFilter::filterData(float x)
{
  float y;
  float y1;
  float y2;

  // Compute proportional portion.
  y1 = proportionalGain * x;

  // Compute integral portion.
  y2 = recursiveState + (integratorGain * x);

  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // Make sure that y2 doesn't grow without bound.
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  if (y2 > maximumPhase)
  {
    // Clip it.
    y2 = maximumPhase;
  } // if
  else
  {
    if (y2 < (-maximumPhase))
    {
      // Clip it.
      y2 = -maximumPhase;
    } // if
  } // else
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

  // Update the state memory.
  recursiveState = y2;
 
  // Compute output.
  y = y1 + y2;

  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // Make sure that y doesn't grow without bound.
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  if (y > maximumPhase)
  {
    // Clip it.
    y = maximumPhase;
  } // if
  else
  {
    if (y < (-maximumPhase))
    {
      // Clip it.
      y = -maximumPhase;
    } // if
  } // else
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
 
  return (y);

} // filterData

