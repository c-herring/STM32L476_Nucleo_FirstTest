/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32l4xx.h"
#include "stm32l4xx_nucleo.h"
#include <string.h>
#include "globals.h"

// -------- Stuff that can be moved to header file later --------

// Default LED flashing speed
#define CORE_LED_DELAY 500
// UART transmit rate
#define UART_TRANSMIT_RATE 1000

// TEST CODE
volatile int multiplier = 1;

// -------- END OF WOULD - BE HEADER --------

int main(void)
{
	// Init the HAL
	HAL_Init();

	// Init the GPIO
	GPIO_Init();

	// Init UART2
	USART2_UART_Init();


	// Delay to flash LED
	uint32_t LED_Delay = CORE_LED_DELAY; //ms
	// Start the stopwatch
	uint32_t LEDstopwatch = HAL_GetTick();
	uint32_t UART2stopwatch = HAL_GetTick();

	int btn_pressed = 0;

	// Infinite loop
	for(;;)
	{
		// If more than the delay has elapsed, toggle the LED.
		// Note that this is an overflow safe delay check. If HAL_GetTick() rolls over during the delay, then the
		// subtraction of stopwatch will underflow and wrap back around. This essentially calculates the elapsed time
		// and compares it to delay. Of course 2^32 seconds (49.7days) is the limitation.
		// This is compared to a non roll over safe comparison (HAL_GetTick() > stopwatch + delay). This method calculates
		// a time point that the delay will elapse and checks if HAL_GetTick() has passed that. But if adding delay to stopwatch
		// causes an overflow, then HAL_GetTick() will immediately be greater than the finish time and cause a false positive in the
		// comparison.
		if (HAL_GetTick() - LEDstopwatch > LED_Delay*multiplier)
		{
			// Toggle the LED
			BSP_LED_Toggle(LED2);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
			// Reset the stopwatch
			LEDstopwatch = HAL_GetTick();
		}

		// Periodically transmit UART message
		if (HAL_GetTick() - UART2stopwatch > UART_TRANSMIT_RATE)
		{
			//sprintf(txbuff, "Hello, batman -- %d\t%lu\n\r", btn_pressed, BSP_PB_GetState(BUTTON_USER));
			//HAL_UART_Transmit_IT(&huart2, (uint8_t*)txbuff, strlen(txbuff));
			UART2stopwatch = HAL_GetTick();

		}

		// Check if the user button is pressed, if it is then change the led flashing speed
		if (BSP_PB_GetState(BUTTON_USER) == 0)
		{
			LED_Delay = CORE_LED_DELAY/2;
		}
		// Check if the button on PC0 is pressed
		else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == 1)
		{
			LED_Delay = CORE_LED_DELAY*2;
		}
		else
		{
			LED_Delay = CORE_LED_DELAY;
		}

		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == 1)
		{
			btn_pressed = 1;
		} else {
			btn_pressed = 0;
		}


	}
}



/*
 * This function is called from inside HAL_UART_Init
 * Initialize the UART gpio
 */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
	// Create init struct to pass to the GPIO init function
	GPIO_InitTypeDef GPIO_InitStruct;

	// This function is called for all UART initialization.
	if(huart->Instance==USART2)
	{
		// Enable the peripheral clock for PORTA GPIO
		__GPIOA_CLK_ENABLE();
		// Enable the clock associated with the UART2 peripheral
		//__USART2_CLK_ENABLE();
		__HAL_RCC_USART2_CLK_ENABLE();

		// GPIO pins to be configured are 2 and 3. We want pullup pulldown mode
		GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		// Low speed is more than sufficient for our serial comms (<5MHz)
		GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
		// Set the alternate function to AF7, USART2
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;

		// Init
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USART2_IRQn);
	}
}

void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
}

void EXTI4_IRQHandler(void)
{
	/*
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
	sprintf(txbuff, "Saw an interrupt..\n");
	multiplier = 2;
	HAL_UART_Transmit(&huart2, (uint8_t*)txbuff, strlen(txbuff), 0xFFFF);
	*/
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
/*
	sprintf(txbuff, "Saw an interrupt!");
	multiplier = 2;
	HAL_UART_Transmit(&huart2, (uint8_t*)txbuff, strlen(txbuff), 0xFFFF);
	HAL_NVIC_ClearPendingIRQ(EXTI4_IRQn);*/
}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//uint8_t M1_encAPin =
	sprintf(txbuff, "Saw an interrupt in EXTI Callback..GPIOA_%d\n\r", GPIO_Pin);
	multiplier = 2;
	HAL_UART_Transmit(&huart2, (uint8_t*)txbuff, strlen(txbuff), 0xFFFF);
}

