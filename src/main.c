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
			

int main(void)
{
	// Init the HAL
	HAL_Init();

	// Initialize the on-board LED (On GPIOA Pin5)
	BSP_LED_Init(LED2);

	// Set the onboard User Switch to be a

	// Delay to flash LED
	uint32_t LED_Delay = 200; //ms
	// Start the stopwatch
	uint32_t stopwatch = HAL_GetTick();

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
		if (HAL_GetTick() - stopwatch > LED_Delay)
		{
			// Toggle the LED
			BSP_LED_Toggle(LED2);
			// Reset the stopwatch
			stopwatch = HAL_GetTick();
		}

	}
}
