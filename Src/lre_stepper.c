#include "lre_stepper.h"
#include "main.h"

/*
    Stepper connections to STM32F072 expected to be:
//left motor 
    YELLOW      - 1: PB4  
    ORANGE      - 2: PB5 
    RED         - 3: PB6 
    BROWN       - 4: PB7 

//right motor 
    BLUE        - 1: PB3  
    GREEN       - 2: PD2 
    YELLOW      - 3: PC12 
    ORANGE      - 4: PC11 
*/

void lre_stepper_setStep (uint8_t step, int left, int right)
{
    switch (step)
    {
        case 0:
            if (left == 1)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_5, GPIO_PIN_RESET);
                }

            if (right == 1)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11 , GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12 | GPIO_PIN_6, GPIO_PIN_RESET);
                }            
            break;

        case 1:
            if (left == 1)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7 | GPIO_PIN_6, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_4, GPIO_PIN_RESET);
                }

            if (right == 1)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12 | GPIO_PIN_6, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11 , GPIO_PIN_RESET);
                }            
            break;

        case 2:
            if (left == 1)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
                }

            if (right == 1)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12 | GPIO_PIN_6, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11 , GPIO_PIN_RESET);
                }
            break;

        case 3:
            if (left == 1)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_SET); 
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7 | GPIO_PIN_4, GPIO_PIN_RESET);
                }

            if (right == 1)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12 | GPIO_PIN_6, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11 , GPIO_PIN_RESET);
                }
            break;

        case 4:
            if (left == 1)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7 | GPIO_PIN_4 | GPIO_PIN_6, GPIO_PIN_RESET);
                }

            if (right == 1)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12 | GPIO_PIN_6, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11 , GPIO_PIN_RESET);
                }
            break;

        case 5:
            if (left == 1)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_4, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7 | GPIO_PIN_6, GPIO_PIN_RESET);
                }

            if (right == 1)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12 | GPIO_PIN_6, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11 , GPIO_PIN_RESET);
                }
            break;

        case 6:
            if (left == 1)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
                }

            if (right == 1)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12 | GPIO_PIN_6, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11 , GPIO_PIN_SET);
                }
            break;

        case 7:
            if (left == 1)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_7, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);
                }

            if (right == 1)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12 | GPIO_PIN_6, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11 , GPIO_PIN_SET);
                }
            break;

        default:
            break;
    }
}