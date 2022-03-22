/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

TSC_HandleTypeDef htsc;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TSC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//init UART module
uint8_t UART1_rxBuffer[12] = {0};
uint8_t Buffer[64] = {0};
uint8_t Text[12] = {0};
uint8_t length;

//init buffer for UART transmit of US-Sensor
uint8_t distance_str_front[200];
uint8_t len_front = 0;
uint8_t distance_str_left[200];
uint8_t len_left = 0;
uint8_t distance_str_right[200];
uint8_t len_right = 0;

//init buffer for follow wall initial distances
uint8_t wall_len_left = 35; // Set desired distance to left wall
uint8_t wall_len_right = 35; // Set desired distance to right wall

//PLEASE ADD DESCRIPTION
uint8_t distance_str_od[200];
uint8_t len_od = 0;

int x;
int y;

// Let's write the callback function
void delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < time);  // wait for the counter to reach the us input in the parameter
}

void trig_front ()
{
  __HAL_TIM_SET_COUNTER(&htim1,0);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  delay(2);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
}

void trig_left ()
{
  __HAL_TIM_SET_COUNTER(&htim1,0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
  delay(2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

void trig_right ()
{
  __HAL_TIM_SET_COUNTER(&htim1,0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  delay(2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  
}
 

uint16_t dist_calc (uint16_t echo_time)
{
  // distance in [mm]
  uint16_t distance = 0; 
  distance = round((343. * echo_time/1.e3)/2.);
  return distance;
}

//start of UART Callback Checker
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{    
  HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 1);
  x = strlen(Buffer);

  if (x == 0)
  {
    strcpy (Buffer, UART1_rxBuffer);
  }
  else
  {
    strcat (Buffer, UART1_rxBuffer);
  }

  //give out total driven distance
  if (strcmp (Buffer, "tm od\r\n") == 0)
  {
    tmod_trig = 1;
    return;
  }

  //give out current distance in all three directions
  else if (strcmp (Buffer, "tm ds\r\n") == 0)
  {
    tmds_trig = 1;
    return;
  }

  //command straight movement with fixed driving-distance
  else if (strncmp (Buffer, "mv ds", 5) == 0 && strncmp (&Buffer[10], "\r\n", 2) == 0)
  {
    strcpy(Text, &Buffer[6]);
    mvds_trig = 1;
    return;
  }

  //command change in motor-speed (for both motors)
  else if (strncmp (Buffer, "mv sp", 5) == 0 && strncmp (&Buffer[10], "\r\n", 2) == 0)
  {
    strcpy(Text, &Buffer[6]);
    set_sp_trig = 1;
    return;
  }

  else if (strcmp (Buffer, "tr wa\r\n") == 0)
  {
    set_wa_trig = 1;
    return;
  }

  else if (strncmp (Buffer, "mv lt", 5) == 0 && strncmp (&Buffer[10], "\r\n", 2) == 0)
  {
    strcpy(Text, &Buffer[6]);
    mvleft_trig = 1;
    return;
  }

  else if (strncmp (Buffer, "mv rt", 5) == 0 && strncmp (&Buffer[10], "\r\n", 2) == 0)
  {
    strcpy(Text, &Buffer[6]);
    mvright_trig = 1;
    return;
  }

  else if (strcmp (Buffer, "tr pk\r\n") == 0) 
  {
    parking_trig = 1;
    return;
  }

  else if (strcmp (Buffer, "tr cn\r\n") == 0) 
  {
    corner_trig = 1;
    return;
  }

  else if (strncmp(Buffer, "tr 4.1.1", 8) == 0 && strncmp (&Buffer[13], "\r\n", 2) == 0)  // trigger task 4.1.1 "drive straight for XXXX cells"
  {
    strcpy(Text, &Buffer[9]);
    lab_drv_trig = 1;
    return;
  }

  else if (strncmp(Buffer, "tr 4.1.2", 8) == 0 && strncmp (&Buffer[13], "\r\n", 2) == 0)  // trigger task 4.1.2 "turn 90° right and drive straight for XXXX cells"
  {
    strcpy(Text, &Buffer[9]);
    lab_turn_trig = 1;
    return;
  }

  else if (strncmp(Buffer, "tr 4.1.3", 8) == 0)  // trigger task 4.1.3 "turn around and drive back to right turn location"
  {
    lab_turnaround_trig = 1;
    return;
  }

   else if (strncmp(Buffer, "tr 4.1.4", 8) == 0)  // trigger task 4.1.4 "turn left and drive back to the start position of 4.1.1"
  {
    lab_turnleft_trig = 1;
    return;
  }

  else if (strcmp (Buffer, "mz explore\r\n") == 0) // trigger labyrinth explore to middle
  {
    maze_explore_trig = 1;
    return;
  }

/*
    else if (strcmp (Buffer, "tr 4.1\r\n") == 0) 
  {
    lab_drv_trig = 1;
    return;
  }*/
}
//end of UART Callback Checker

//void fnct to drive straight for a given distance
void mv_straight (uint16_t ds, uint8_t reverse)
  {
    mv_direction = reverse; //set driving direction
    current_dis = 0; //reset currently driven distance
    dis_val = ds; //Solution for Bug: VS code tends to optimize ds which ends up deleting its content
    target_dis = dis_val/0.061; //set target distance
    HAL_TIM_Base_Start_IT(&htim3);
    length = sprintf(Buffer, "My predicted driving distance in mm is: %d\r\n", ds);
    HAL_UART_Transmit(&huart1, Buffer, length, 100);
    memset(Buffer, 0, strlen(Buffer)); //clear buffer
  }

void mv_labyrinth (uint16_t ds, uint8_t reverse)
  {
    mv_direction_lab = reverse; //set driving direction
    current_dis_lab = 0; //reset currently driven distance
    dis_val_lab = ds; //Solution for Bug: VS code tends to optimize ds which ends up deleting its content
    target_dis_lab = dis_val_lab/0.061; //set target distance
    HAL_TIM_Base_Start_IT(&htim3);
    memset(Buffer, 0, strlen(Buffer)); //clear buffer
  }

void turn (uint16_t rot_dis, uint8_t rot_dir)
{
      l = 0;//reset left iterator
      r = 0;//reset right iterator
      rotation = rot_dis * rot_norm;//set desired rotation into ticks
      mv_direction = rot_dir; //set turning mode
      cur_rotation = 0; //reset currently rotation
      in_rot = 1;
      HAL_TIM_Base_Start_IT(&htim3);
      while (in_rot == 1)
      {
          // wait for rotation to complete
      }
      HAL_Delay(300); //small delay
}

// follow wall/corner Function
void follow_wall (uint16_t ds)
{
      mv_straight(ds, 0);  // Set distance 2 meters , drive foreward

      //  Move along wall function which enbales the mv_straight until deviation is detected or od > dist_val
      while (current_dis < target_dis)
      {
        // check distance to left wall
        US_Select = 1;
        trig_left();
        HAL_Delay(500);
        cur_dis_left = dist_calc(echo_duration_left);
        len_left = sprintf(distance_str_left, "Distance left is: %02d \r\n", cur_dis_left);
        HAL_UART_Transmit(&huart1, distance_str_left, len_left, 100);

        // check distance to right wall
        US_Select = 2;
        trig_right();
        HAL_Delay(500);
        cur_dis_right = dist_calc(echo_duration_right);
        len_right = sprintf(distance_str_right, "Distance right is: %02d \r\n--------- \r\n", cur_dis_right);
        HAL_UART_Transmit(&huart1, distance_str_right, len_right, 100);

        if (check_for_corner == 1) // active corner-checking by enabling front-sensor trigger
        {
          US_Select = 0; //select front sensor
          trig_front();
          len_front_parking = sprintf(distance_str_parking, "Distance front is: %02d \r\n", dist_calc(echo_duration_front));
          HAL_UART_Transmit(&huart1, distance_str_parking, len_front_parking, 100);
          HAL_Delay(200);
          if (dist_calc(echo_duration_front) < 70)
          {
            front_wall_trig = 1;
            HAL_Delay(300); //small delay

            //turn right 90°
            turn(90, 3);//right = 3

          }
        }

        if (cur_dis_left > wall_len_left + 30 && cur_dis_left < 100)
        {
          // left turn if left wall too far
          uint8_t* message2 = "LEFT FOLLOW\r\n";
          HAL_UART_Transmit(&huart1, message2, strlen(message2),100);
          turn(3, 2);//left = 2

          mv_direction = 0; //resume straight drive
          HAL_TIM_Base_Start_IT(&htim3);
          HAL_Delay(500);
        }

        else if (cur_dis_right > wall_len_right + 30 && cur_dis_right < 100) 
        {
          // right turn if left wall too close
          uint8_t* message2 = "RIGHT FOLLOW\r\n";
          HAL_UART_Transmit(&huart1, message2, strlen(message2),100);
          turn(3, 3);//right = 3

          mv_direction = 0; //resume straight drive
          HAL_TIM_Base_Start_IT(&htim3);
          HAL_Delay(500);
        }

/* REMOVED as sensors read wrong values close to a wall
        else if (cur_dis_left < wall_len_left - 10) //len_right > wall_len_right + 10 || 
        {
          // right turn if left wall too close
          turn(5, 3);//right = 3

          mv_direction = 0; //resume straight drive
          HAL_TIM_Base_Start_IT(&htim3);
          HAL_Delay(300);
        }*/
      }
}

void explore_labyrinth (uint16_t ds)
{
      mv_straight(ds, 0);  // Set distance 2 meters , drive foreward

      //  Move along wall function which enbales the mv_straight until deviation is detected or od > dist_val
      while (current_dis < target_dis)
      { 
        US_Select = 0; //select front sensor
        trig_front();
        len_front_parking = sprintf(distance_str_parking, "Distance front is: %02d \r\n", dist_calc(echo_duration_front));
        HAL_UART_Transmit(&huart1, distance_str_parking, len_front_parking, 100);
        HAL_Delay(200);

        // check distance to left wall
        US_Select = 1;
        trig_left();
        HAL_Delay(500);
        cur_dis_left = dist_calc(echo_duration_left);
        len_left = sprintf(distance_str_left, "Distance left is: %02d \r\n", cur_dis_left);
        HAL_UART_Transmit(&huart1, distance_str_left, len_left, 100);

        // check distance to right wall
        US_Select = 2;
        trig_right();
        HAL_Delay(500);
        cur_dis_right = dist_calc(echo_duration_right);
        len_right = sprintf(distance_str_right, "Distance right is: %02d \r\n--------- \r\n", cur_dis_right);
        HAL_UART_Transmit(&huart1, distance_str_right, len_right, 100);

          //if (dist_calc(echo_duration_front) < 70)
          //{
            //front_wall_trig = 1;
            //HAL_Delay(300); //small delay

            //turn right 90°
            //turn(90, 3);//right = 3

          //}
        

        if (cur_dis_left > wall_len_left + 30 && cur_dis_left < 100)
        {
          // left turn if left wall too far
          uint8_t* message2 = "follow left\r\n";
          HAL_UART_Transmit(&huart1, message2, strlen(message2),100);
          turn(3, 2);//left = 2

          mv_direction = 0; //resume straight drive
          HAL_TIM_Base_Start_IT(&htim3);
          HAL_Delay(500);
        }

        else if (cur_dis_right > wall_len_right + 30 && cur_dis_right < 100) 
        {
          // right turn if left wall too close
          uint8_t* message2 = "follow right\r\n";
          HAL_UART_Transmit(&huart1, message2, strlen(message2),100);
          turn(3, 3);//right = 3

          mv_direction = 0; //resume straight drive
          HAL_TIM_Base_Start_IT(&htim3);
          HAL_Delay(500);
        }

        else if (cur_dis_right > 100) 
        {
          // right turn if left wall too close
          uint8_t* message2 = "RIGHT TURN\r\n";
          HAL_UART_Transmit(&huart1, message2, strlen(message2),100);
          while (dist_calc(echo_duration_front > 80))
          {
            follow_wall(110);
          }
          turn(90, 3);//right = 3
          mv_direction = 0; //resume straight drive
          follow_wall(200);
          HAL_TIM_Base_Start_IT(&htim3);
          target_dis = 63000;
          HAL_Delay(500);
        }
        
        else if (cur_dis_left > 100 && cur_dis_right < 100 && dist_calc(echo_duration_front < 75))
        {
          // left turn if left wall too far
          uint8_t* message2 = "LEFT TURN\r\n";
          HAL_UART_Transmit(&huart1, message2, strlen(message2),100);
          turn(90, 2);//left = 2
          mv_direction = 0; //resume straight drive
          follow_wall(200);
          HAL_TIM_Base_Start_IT(&htim3);
          target_dis = 63000;
          HAL_Delay(500);
        }

        else if (cur_dis_left < 100 && cur_dis_right < 100 && dist_calc(echo_duration_front < 75))
        {
          // left turn if left wall too far
          uint8_t* message2 = "TURN AROUND\r\n";
          HAL_UART_Transmit(&huart1, message2, strlen(message2),100);
          turn(180, 3);//right = 3

          mv_direction = 0; //resume straight drive
          HAL_TIM_Base_Start_IT(&htim3);
          HAL_Delay(500);
        }


/* REMOVED as sensors read wrong values close to a wall
        else if (cur_dis_left < wall_len_left - 10) //len_right > wall_len_right + 10 || 
        {
          // right turn if left wall too close
          turn(5, 3);//right = 3

          mv_direction = 0; //resume straight drive
          HAL_TIM_Base_Start_IT(&htim3);
          HAL_Delay(300);
        }*/
      }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TSC_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

//define properties of trigger timer (TIM_1)
  __HAL_TIM_SET_PRESCALER(&htim1, SystemCoreClock/1e6-1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

//initialize sensors
  echo_duration_front = 0;
  echo_trig_front = 0; 

  echo_duration_left = 0;
  echo_trig_left = 0; 

  echo_duration_right = 0;
  echo_trig_right = 0; 

  t_0_front = 0;
  t_1_front = 0;

  t_0_left = 0;
  t_1_left = 0;

  t_0_right = 0;
  t_1_right = 0;

  US_Select = 3;

//init drive straight trigger
  mvds_trig = 0;
  dis_val = 0;

//init send tm ds distance trigger
  tmds_trig = 0;

//init send tm od distance trigger
  tmod_trig = 0;

//set default speed
  sp = 55;
  p = 62500/sp; //update speed variable
__HAL_TIM_SET_PRESCALER(&htim3, SystemCoreClock / 1000000-1); //update tim3 prescaler
__HAL_TIM_SET_AUTORELOAD(&htim3, p - 1);

//set default move direction for both wheels (1-reverse, 0-foreward)
  i = 0;

//set default turning radius
  rotation = 0; //default turn: corresponds to 90° turn
  rot_norm = 16.2;
  mvleft_trig = 0;
  mvright_trig = 0;

//init parking variables
  parking_trig = 0;
  front_wall_trig = 0;

//init labyrinth variables
  lab_drv_trig = 0;
  maze_explore_trig = 0;
//ADD COMMENT
  od_buf = 0;
  od = 0;


  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  HAL_UART_Receive_IT (&huart1, UART1_rxBuffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
//Currently the main code triggers every US-Sensor every 400ms -> distance data can be obtained continiously

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(mvds_trig == 1) //drive straight
    {
      dis_val = atoi(Text); //read out buffer info
      mv_straight(dis_val, 0); //call move function
      mvds_trig = 0; //reset move command trigger
    }

    else if (mvleft_trig == 1) //drive a left turn
    {
      rotation = atoi(Text); //read out buffer info
      turn(rotation, 2);//left = 2
      mvleft_trig = 0; //reset trigger
    }

    else if (mvright_trig == 1) //drive a right turn
    {
      rotation = atoi(Text); //read out buffer info
      turn(rotation, 3);//right = 3
      mvright_trig = 0; //reset trigger
    }

    else if (parking_trig == 1) //parking mode routine
    {
      mv_straight(2000, 0);      //drive straight

      while (front_wall_trig == 0)
        {
          US_Select = 0; //select front sensor
          trig_front();
          len_front_parking = sprintf(distance_str_parking, "Distance front is: %02d \r\n", dist_calc(echo_duration_front));
          HAL_UART_Transmit(&huart1, distance_str_parking, len_front_parking, 100);
          HAL_Delay(200);
          if (dist_calc(echo_duration_front) < 60)
          {
            front_wall_trig = 1;
          }
        }

      front_wall_trig = 0;

      //turn left 180°
      turn(180, 2);//left = 2 

      //reverse for x cm
      mv_straight(50, 1); //drive reverse
      parking_trig = 0; //reset trigger
    }

    else if (tmds_trig == 1) //read out all sensor data
    {
      //trigger all sensors
      US_Select = 0;

      //  US 1
      uint8_t distance_str_front[200];
      uint8_t len_front = 0;

      trig_front();
      HAL_Delay(200);

      US_Select = 1;

      //  US 2 
      uint8_t distance_str_left[200];
      uint8_t len_left = 0;

      trig_left();
      HAL_Delay(200);

      US_Select = 2;

      //  US 3 
      uint8_t distance_str_right[200];
      uint8_t len_right = 0;

      trig_right();
      HAL_Delay(200);

      // sent data via UART

      len_front = sprintf(distance_str_front, "Distance front is: %02d \r\n", dist_calc(echo_duration_front));
      HAL_UART_Transmit(&huart1, distance_str_front, len_front, 100); 

      len_left = sprintf(distance_str_left, "Distance left is: %02d \r\n", dist_calc(echo_duration_left));
      HAL_UART_Transmit(&huart1, distance_str_left, len_left, 100); 

      len_right = sprintf(distance_str_right, "Distance right is: %02d \r\n", dist_calc(echo_duration_right));
      HAL_UART_Transmit(&huart1, distance_str_right, len_right, 100); 

      //  Draw vertical line
      uint8_t* message2 = "-------------------\r\n";
      HAL_UART_Transmit(&huart1, message2, strlen(message2),100);

      tmds_trig = 0; //reset tmds command trigger
    }

    else if (tmod_trig == 1) // sent currently driven distance
    {
      len_od = sprintf(distance_str_od, "traveled distance approx.: %02d \r\n", od+od_buf);
      HAL_UART_Transmit(&huart1, distance_str_od, len_od, 100); 
      tmod_trig = 0; //reset tmod command trigger
    }

    else if (set_sp_trig == 1) //change motor speed
    {
      sp = atoi(Text); //copy commandes speed
      p = 62500/sp; //update speed variable
      __HAL_TIM_SET_PRESCALER(&htim3, SystemCoreClock / 1000000-1); //update tim3 prescaler
      __HAL_TIM_SET_AUTORELOAD(&htim3, p - 1);
      length = sprintf(Buffer, "My predicted speed in mm/s is: %d\r\n", sp);
      HAL_UART_Transmit(&huart1, Buffer, length, 100);
      set_sp_trig = 0; //reset speed command trigger
    }


    else if (set_wa_trig == 1) // follow wall function /w corner function disabled
    {
      check_for_corner = 0;
      follow_wall(2000);
      set_wa_trig = 0; // reset follow wa command trigger
    }

    
    else if (corner_trig == 1) // follow wall function /w corner function enabled
    {
      check_for_corner = 1;
      follow_wall(2000);
      corner_trig = 0;
    }

    else if (lab_drv_trig == 1) // drive certain amount of cells
    {
      cells = atoi(Text);
      follow_wall(cells * 200);
      lab_drv_trig = 0;
      current_cell = current_cell + cells;
    }

    else if (maze_explore_trig == 1) //drive to the middle of the labyrinth
    {
      explore_labyrinth(63000);
      maze_explore_trig = 0;
    }

    else if (lab_turn_trig) // turn right and drive certain amount of cells
    {
      cells = atoi(Text);
      turn(90, 3);
      follow_wall(cells * 200);
      lab_turn_trig = 0;
      current_cell = current_cell + cells;
    }

    else if (lab_turnaround_trig) // turn around and return to the right turn position
    {
      turn(180, 3);
      follow_wall(cells * 200);
      lab_turnaround_trig = 0;
      current_cell = current_cell - cells;
    }

    else if (lab_turnleft_trig) // turn left at the right turn position and return to the start of 4.1.1
    {
      turn(90, 2);
      follow_wall(current_cell * 200); // current cell is used here, since it is the distance left to the start point
      lab_turnleft_trig = 0;
      current_cell = 0; // start point reached 
    }

    HAL_Delay(600);
    memset(Buffer, 0, strlen(Buffer)); //reset buffer
  } 
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI4_15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TSC Initialization Function
  * @param None
  * @retval None
  */
static void MX_TSC_Init(void)
{

  /* USER CODE BEGIN TSC_Init 0 */

  /* USER CODE END TSC_Init 0 */

  /* USER CODE BEGIN TSC_Init 1 */

  /* USER CODE END TSC_Init 1 */
  /** Configure the TSC peripheral
  */
  htsc.Instance = TSC;
  htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
  htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
  htsc.Init.SpreadSpectrum = DISABLE;
  htsc.Init.SpreadSpectrumDeviation = 1;
  htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
  htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
  htsc.Init.MaxCountValue = TSC_MCV_8191;
  htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
  htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
  htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
  htsc.Init.MaxCountInterrupt = DISABLE;
  htsc.Init.ChannelIOs = TSC_GROUP2_IO3|TSC_GROUP3_IO2;
  htsc.Init.ShieldIOs = 0;
  htsc.Init.SamplingIOs = TSC_GROUP1_IO4|TSC_GROUP2_IO4|TSC_GROUP3_IO3;
  if (HAL_TSC_Init(&htsc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TSC_Init 2 */

  /* USER CODE END TSC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|EXT_RESET_Pin|Trig_US1_Pin|M2_4_Pin
                          |M2_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Trig_US2_Pin|Trig_US3_Pin|Trig_US3_Test_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M2_2_GPIO_Port, M2_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M2_1_Pin|M1_1_Pin|M1_2_Pin|M1_3_Pin
                          |M1_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin EXT_RESET_Pin Trig_US1_Pin M2_4_Pin
                           M2_3_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|EXT_RESET_Pin|Trig_US1_Pin|M2_4_Pin
                          |M2_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT1_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Trig_US2_Pin Trig_US3_Pin Trig_US3_Test_Pin */
  GPIO_InitStruct.Pin = Trig_US2_Pin|Trig_US3_Pin|Trig_US3_Test_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : US3_Test_Pin */
  GPIO_InitStruct.Pin = US3_Test_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(US3_Test_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : US2_Pin_Pin US1_Pin_Pin */
  GPIO_InitStruct.Pin = US2_Pin_Pin|US1_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : M2_2_Pin */
  GPIO_InitStruct.Pin = M2_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(M2_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M2_1_Pin M1_1_Pin M1_2_Pin M1_3_Pin
                           M1_4_Pin */
  GPIO_InitStruct.Pin = M2_1_Pin|M1_1_Pin|M1_2_Pin|M1_3_Pin
                          |M1_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
