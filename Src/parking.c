//*user code for parking task 3.1*//
//imports
#include "main.h"
#include "stm32f0xx_it.h"

/*general approach:

- move straight while continously measuring front distance (US_F)
- once a spec. threshold at US_F has been reached: Stop straight movement
- Move wheels in oposite direction to complete 180° turn, while still beeing centred
- move straight back for a distance of d_backoff = d_threshhol - 1.5 cm (+/- diffenerence of distance between front and back of the mouse after turning)
- Stop

*/