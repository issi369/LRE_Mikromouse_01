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

void parking ()
{
    //  US 1
    US_Select = 0;
    uint8_t distance_str_front[200];
    uint8_t len_front = 0;

    trig_front();
    HAL_Delay(200);

    US_Select = 1;
}

void turn_left ()
{

}