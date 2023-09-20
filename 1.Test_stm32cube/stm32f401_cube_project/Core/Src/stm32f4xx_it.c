/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint32_t ii=0;
int16_t pnew=0;
int16_t pold=0;
int16_t flag=0;
int16_t flag2=0;
float buffer[200];
float theta;
float vel;		// rev/s
float velold=0.0f;
float u = 0.0f; // Accion de Control
float u_old = 0.0f;
float r = 0.0f;	// Referencia rev/s
float e = 0.0f; // Error rev/s
float e_old = 0.0f;
float Kp = 0.66821f;	// Constante proporcional
float Ki = 70.4747f;
float Ts = 0.005; 	//5ms


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	pnew = TIM3->CNT;
	theta = 360*(float)((int)pnew / 28800.0f);	//degrees
	vel = (int)(pnew - pold)*200.0f / 28800.0f;	//rev/s
	buffer[ii]=vel;
	flag2=1;
	if ((vel >= 50.0f) || (vel <= -50.0f)) {
		flag=1;
		buffer[ii]=velold;
		vel = velold;
	}

	// System identification
//	if ( ii < 39){  		// 0-98 Initial velocity stable state
//		u = 0.5;
//		ii++;
//	}else if(ii == 39){ 	// 99 Start signal flag
//		buffer[ii] = 10;
//		u = 0.5;
//		ii++;
//	}else if(ii <= 119){	// Step in max values
//		u = 0.9;
//		ii++;
//	}else if(ii < 199){	// Step to min value
//		u = 0.5;
//		ii++;
//	}else if(ii ==200){ // ii = 199
//		ii++;
//		u = 0;
//		for (int j = 0; j<200;j++){
//			buffer[j] = 0;
//		}
//
//	}else{
//		u = 0;
//	}


	// Reference Creation

	if ( (ii >= 0) && (ii <= 99) ) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
//		TIM2->CCR1=300;
//		TIM2->CCR2=0;
		r = 1;

	}
	if ( (ii >= 100) && (ii <= 199) ) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
//		TIM2->CCR1=700;
//		TIM2->CCR2=0;
		r = 2;
	}
	 // Comparator (Error)
	 e = r - vel;
	 //Control Law (Proportional)

	 u = ((Kp + (Ki*Ts))*e) - (Kp * e_old) + u_old;

	// Control action execution
	if (u>1){
		u=1;
	}else if(u<-1) {
		u=-1;
	}
	if (u < 0){
		TIM2->CCR1=0;
		TIM2->CCR2=(-u)*1000;
	}else{
		TIM2->CCR1=u*1000;
		TIM2->CCR2=0;
	}
	ii++;
	if (ii == 200) {
		ii=0;
	}

	//  update variables
	pold = pnew;
	velold = vel;
	e_old = e;
	u_old = u;
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
