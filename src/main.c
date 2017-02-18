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

// -------- Stuff that can be moved to header file later --------

// Default LED flashing speed
#define CORE_LED_DELAY 500
// UART transmit rate
#define UART_TRANSMIT_RATE 1000

// Allocate space for a transmit buffer
char txbuff[50];

// Create the UART2 handle
UART_HandleTypeDef huart2;

// Function Declarations
void USART2_UART_Init(void);

// TEST CODE
volatile int multiplier = 1;

// -------- END OF WOULD - BE HEADER --------

int main(void)
{
	// Init the HAL
	HAL_Init();

	// Initialize the on-board LED (On GPIOA Pin5)
	BSP_LED_Init(LED2);

	// Set the on-board user button as a GPIO input
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

	// Set the PC0 Pin as a gpio input
	GPIO_InitTypeDef gpioinitstruct = {0};
	gpioinitstruct.Pin = GPIO_PIN_0;
	gpioinitstruct.Mode = GPIO_MODE_INPUT;
	gpioinitstruct.Pull   = GPIO_NOPULL;
	gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &gpioinitstruct);

	// Set the PA4 Pin as a gpio input
	gpioinitstruct.Pin = GPIO_PIN_4;
	gpioinitstruct.Mode = GPIO_MODE_IT_RISING;
	gpioinitstruct.Pull   = GPIO_NOPULL;
	gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &gpioinitstruct);

	// Set PC10 pin as an output
	gpioinitstruct.Pin = GPIO_PIN_10;
	gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
	gpioinitstruct.Pull   = GPIO_NOPULL;
	gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &gpioinitstruct);


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
			sprintf(txbuff, "Hello, batman -- %d\t%lu\n\r", btn_pressed, BSP_PB_GetState(BUTTON_USER));
			HAL_UART_Transmit(&huart2, (uint8_t*)txbuff, strlen(txbuff), 0xFFFF);
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
 * Initialize UART
 */
void USART2_UART_Init(void)
{
	// Enable interrupts
	//__HAL_UART_ENABLE_IT(&huart2, UART_IT_TXE);
	// Enable the USART2 global interrupt
	//NVIC_EnableIRQ(USART2_IRQn);
	//NVIC_InitTypeDef nvicStructure;
	//NVIC_Init();

	// This is all pretty self explaining.
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	HAL_UART_Init(&huart2);
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
		__USART2_CLK_ENABLE();

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
	}
}

void UART2_IRQHandler(void)
{
	multiplier = 2;
	return;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	sprintf(txbuff, "Saw an interrupt");
	multiplier = 2;
	HAL_UART_Transmit(&huart2, (uint8_t*)txbuff, strlen(txbuff), 0xFFFF);
}
