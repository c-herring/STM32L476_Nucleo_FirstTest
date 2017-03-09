/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @author  Ac6
  * @version V1.0
  * @date    02-Feb-2015
  * @brief   Default Interrupt Service Routines.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"

#include <string.h>
#ifdef USE_RTOS_SYSTICK
#include <cmsis_os.h>
#endif

#include "globals.h"
#include "stm32l4xx_it.h"




/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();/*
#ifdef USE_RTOS_SYSTICK
	osSystickHandler();
#endif*/
}



/**
  * @brief  Handle EXTI interrupt request on the encoder pins 10-15.
  * @param  None
  * @retval None
  */
// Temp stuff
int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
uint8_t encA_val = 0;
uint8_t encB_val = 0;
int32_t motorA_pos = 0;

void EXTI15_10_IRQHandler(void)
{
	static uint32_t trig1;
	static uint32_t trig2;
	// Check which interrupt was triggered, clear the interrupt before we handle it
	if (__HAL_GPIO_EXTI_GET_IT(Encoder1_A_Pin) != RESET)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(Encoder1_A_Pin);
		trig1 = 1;
	}
	if (__HAL_GPIO_EXTI_GET_IT(Encoder1_B_Pin) != RESET)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(Encoder1_B_Pin);
		trig2 = 1;
	}
	// Shift old encoder state left two bits
	encA_val = encA_val << 2;
	// Or in the two encoder channels
	encA_val |= HAL_GPIO_ReadPin(Encoder1_A_Port, Encoder1_A_Pin) | HAL_GPIO_ReadPin(Encoder1_B_Port, Encoder1_B_Pin) << 1;
	// Update encoder position
	motorA_pos += lookup_table[encA_val & 0b1111];


	sprintf(txbuff, "Interrupt pin state: %d(%lu)\t%d(%lu)\tEncoder = %d\n\r", HAL_GPIO_ReadPin(Encoder1_A_Port, Encoder1_A_Pin), trig1, HAL_GPIO_ReadPin(Encoder1_B_Port, Encoder1_B_Pin), trig2, motorA_pos);
	HAL_UART_Transmit_IT(&huart2, (uint8_t*)txbuff, strlen(txbuff));

	trig1 = 0;
	trig2 = 0;
/*
	sprintf(txbuff, "Saw an interrupt!");
	multiplier = 2;
	HAL_UART_Transmit(&huart2, (uint8_t*)txbuff, strlen(txbuff), 0xFFFF);
	HAL_NVIC_ClearPendingIRQ(EXTI4_IRQn);*/
}



void DMA1_Channel6_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(DMA1_Channel6_IRQn);
    rxB2 = 'a';
    HAL_DMA_IRQHandler(&hdma_usart2_rx);
}
