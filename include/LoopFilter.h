//**************************************************************************
// file name: LoopFilter.h
//**************************************************************************
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
// This class implements a signal processing block that performs a
// loop filter function, that is needed by a phase-locked loop.  The
// filter is implemented as a PI (proportional-integral) structure.
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

#ifndef __LOOPFILTER__
#define __LOOPFILTER__

#include <stdint.h>

class LoopFilter
{
  //***************************** operations **************************

  public:

  LoopFilter(float proportionalGain,float integratorGain);
  ~LoopFilter(void);

  void reset(void);
  float filterData(float x);

  //***************************** attributes **************************
  private:

  // These are the gains for the PI filter.
  float proportionalGain;
  float integratorGain;

  // State memory for recursive portion of the filter.
  float recursiveState;
};

#endif // __LOOPFILTER__
