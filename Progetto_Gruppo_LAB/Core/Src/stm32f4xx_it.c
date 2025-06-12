/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
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
char buffer[200]= "";
float timeout = 1;
float CP = 8399;	//variabili per gestire il timer casuale
float PSC = 9999;
uint8_t cont = 0;
uint32_t last_interrupt_time = 0;	//variabili per gestire il debounce
uint32_t current_time = 0;
uint8_t led_on = 0;
uint32_t t = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim4;
extern uint8_t a;
extern uint32_t risultati[5];
extern uint8_t vittoria;
extern double sogliams;
extern double punteggio;
extern double somma;
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

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Button_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

	current_time = __HAL_TIM_GET_COUNTER(&htim4);

	if(current_time - last_interrupt_time > 1000){// debounce di circa 120 ms, gestisci il pulsante solo se si tratta di pressione intenzionale
		if (led_on == 1) // gestiscici il caso in cui il led si sia effettivamente acceso
		{
			last_interrupt_time = current_time;


			sprintf(buffer, "Successo! Pulsante premuto per la %u° volta.\r\n", cont+1);
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);

			// Spegni LED
			HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);
			led_on = 0;

			t = __HAL_TIM_GET_COUNTER(&htim3);
			sprintf(buffer, "Tempo di risposta = %lu ms.\r\n", (t*1000)/8400);
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
			risultati[cont] = (t*1000)/8400;  // tempo di reazione in millisecondi

			cont++;

			// Ferma Timer3
			__HAL_TIM_DISABLE(&htim3);
			__HAL_TIM_SET_COUNTER(&htim3, 0);

			timeout = 1.0f + ((float)rand() / RAND_MAX) * 3.0f;
			sprintf(buffer, "Il led si accenderà fra: %2.2f secondi.\r\n", timeout);
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
			CP = (uint32_t)(timeout * (84000000.0f / (9999 + 1)));  // calcolo il valore di timeout ed il corrispettivo counter period per generare un segnale alla frequenza mediafreq

			if(cont<5){

				__HAL_TIM_DISABLE(&htim2);
				__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
				__HAL_TIM_SET_AUTORELOAD(&htim2, CP);
				__HAL_TIM_SET_COUNTER(&htim2, 0);
				__HAL_TIM_ENABLE(&htim2);

			}
			else{
				vittoria = 1;
				for(int i = 0; i<5; i++){
					somma += risultati[i];
				}

				punteggio=100*(exp(-pow((((somma/5)-sogliams)/60.2168),2))-1)*((somma/5)-sogliams);
				HAL_PWR_DisableSleepOnExit (); // caso in cui si è completata la prova con successo
			}
		}
		else{

			sprintf(buffer, "Hai premuto il pulsante troppo presto! La sessione è andata persa. \r\n");
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
			HAL_PWR_DisableSleepOnExit ();
		}
	}

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	led_on = 1;
	sprintf(buffer, "LED acceso!\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);

	// Ferma Timer2 per non far scattare ripetutamente l'IRQ
	__HAL_TIM_DISABLE(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2, 0);

	// faccio partire la base dei tempi di tim3 solo alla prima run
	if(a == 0){
		a = 1;
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		HAL_TIM_Base_Start_IT(&htim3);}
	else{
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		__HAL_TIM_ENABLE(&htim3);
	}
	// Accendi LED
	HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET);
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	sprintf(buffer, "Sono passati 3 secondi, la sessione è andata persa.\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
	// Spegni LED
	HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);
	led_on = 0;

	// Ferma Timer3
	__HAL_TIM_DISABLE(&htim3);
	__HAL_TIM_SET_COUNTER(&htim3, 0);

	HAL_PWR_DisableSleepOnExit ();
  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
