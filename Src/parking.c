//*user code for parking task 3.1*//
//imports
#include "main.h"
#include "stm32f0xx_it.h"
#include "parking.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
//#include "main.h"

/*general approach:

- move straight while continously measuring front distance (US_F)
- once a spec. threshold at US_F has been reached: Stop straight movement
- Move wheels in oposite direction to complete 180° turn, while still beeing centred
- move straight back for a distance of d_backoff = d_threshhol - 1.5 cm (+/- diffenerence of distance between front and back of the mouse after turning)
- Stop

*/

void parking (void)
{
    //  US 1
    US_Select = 0;
    uint8_t distance_str_front[200];
    uint8_t len_front = 0;

    trig_front();
    HAL_Delay(200);

    US_Select = 1;
}

void mv_turn (int left_turn, uint16_t rot)
{
    if (left_turn == 1)
        {
            l = 0;
            r = 0;
            rotation = rot;
            //mv_direction = 2;

        }

    else if (left_turn == 0)
        {
            l = 0;
            r = 0;
            //mv_direction = 3; //right turn
        }
}