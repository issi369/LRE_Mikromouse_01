#include "lre_stepper.h"

/*
    Stepper connections to STM32F072 expected to be:
    BLUE   - 1: PC6 B4 
    PINK   - 2: PC7 B5
    YELLOW - 3: PC8 B6
    ORANGE - 4: PC9 B7

    BLUE   - 1: PB4 B4 
    PINK   - 2: PB5 B5
    YELLOW - 3: PB6 B6
    ORANGE - 4: PB7 B7

    BLUE   - 1: PB3 B4 
    PINK   - 2: PD2 B5
    YELLOW - 3: PC12 B6
    ORANGE - 4: PC11 B7
*/

void lre_stepper_setStep(uint8_t step){
    switch (step)
    {
        case 0:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_5, GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11 , GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12 | GPIO_PIN_6, GPIO_PIN_RESET);
            break;
        case 1:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7 | GPIO_PIN_6, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_4, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12 | GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11 , GPIO_PIN_RESET);

            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 , GPIO_PIN_SET);
            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12 | GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11 , GPIO_PIN_RESET);

            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_SET);
            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
            break;
        case 3:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_SET); 
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7 | GPIO_PIN_4, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12 | GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11 , GPIO_PIN_RESET);

            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 |GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
            break;
        case 4:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7 | GPIO_PIN_4 | GPIO_PIN_6, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12 | GPIO_PIN_6, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11 , GPIO_PIN_RESET);

            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_SET);
            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_7, GPIO_PIN_RESET);
            break;
        case 5:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_4, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7 | GPIO_PIN_6, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12 | GPIO_PIN_6, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11 , GPIO_PIN_RESET);

            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 , GPIO_PIN_SET);
            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7, GPIO_PIN_RESET);
            break;
        case 6:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12 | GPIO_PIN_6, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11 , GPIO_PIN_SET);

            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 |  GPIO_PIN_7, GPIO_PIN_SET);
            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
            break;
        case 7:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_7, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12 | GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11 , GPIO_PIN_SET);

            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 |GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);
            break;

        default:
            break;
    }
}