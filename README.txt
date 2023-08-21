When using an SDR (software defined radio), there is generally an error
between the receiver frequency and the transmitter frequency.  This results
in a frequency beat that is not necessarily audible, but it will be quite
apparent when viewing the IQ data on a Lissajous display.

I noticed this when I added a Lissajous scope to my signal analyzer.  It
never dawned on me until I mentioned this on IRC (##rtlsdr), and tejeez
indicated that the big circle is the result of a frequency error.

This prompted me to come up with a frequency error estimator so that I can
use the result (the frequency error) to drive an NCO plus phase corrector, so
that I could derotate the phase velocity of the signal.

And so it begins...

Chris G. 08/21/2023

