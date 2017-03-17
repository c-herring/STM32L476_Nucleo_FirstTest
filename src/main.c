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
#include "PIDMotor.h"

// -------- Stuff that can be moved to header file later --------

// Default LED flashing speed
#define CORE_LED_DELAY 1000
// UART transmit rate
#define UART_TRANSMIT_RATE 1000
#define PWM_PERIOD 7999
// TEST CODE
volatile int multiplier = 1;

char somebuff[50];



// -------- END OF WOULD - BE HEADER --------

int main(void)
{

	velSet = 0;

	// Init the HAL

	HAL_Init();

	SystemClock_Config();
	//HAL_RCC_DeInit();
	// Init the GPIO
	GPIO_Init();

	// Init UART2
	USART2_UART_Init();

	// Init timer 4
	TIM4_Init();
	// Start the PWM
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

	// Start the uart2 DMA2 circular RX buffer
	rxB = 'o';
	rxB2 = '_';
	rxB3 = '_';
	__HAL_UART_FLUSH_DRREGISTER(&huart2); // First flush buffer to prevent an overrun.

	// Create the PID moor structs
	PIDParams_TypeDef motorA_params;
	motorA_params.Kp = 0.15f;
	motorA_params.Ki = 0.001f;
	motorA_params.Kd = 0.0f;

	//PIDMotor_TypeDef MotorA;
	Motor_Init(&MotorA, motorA_params, PID_TD, PWM_PERIOD, 5000.0);

	uint32_t PIDStopwatch = HAL_GetTick();

	// Delay to flash LED
	uint32_t LED_Delay = CORE_LED_DELAY; //ms
	// Start the stopwatch
	uint32_t LEDstopwatch = HAL_GetTick();
	uint32_t UART2stopwatch = HAL_GetTick();
	pwm = 0;
	uint32_t pwm_dir = 1;
	int btn_pressed = 0;

	rxbuff[0] = '\0';
	cmdbuff[0] = '\0';
	cmdBuffIndex = 0;
	executeCmd = 0;
	HAL_UART_Receive_DMA(&huart2, &rxB, 1);
	// Infinite loop
	for(;;)
	{

		//HAL_UART_Receive_IT(&huart2, rxbuff, 10);
		//if (strlen(somebuff) > 0)
		//{
		//	HAL_UART_Transmit_IT(&huart2, somebuff, strlen(rxbuff));
		//	somebuff[0] = '\0';
		//}

		// If more than the delay has elapsed, toggle the LED.
		// Note that this is an overflow safe delay check. If HAL_GetTick() rolls over during the delay, then the
		// subtraction of stopwatch will underflow and wrap back around. This essentially calculates the elapsed time
		// and compares it to delay. Of course 2^32 seconds (49.7days) is the limitation.
		// This is compared to a non roll over safe comparison (HAL_GetTick() > stopwatch + delay). This method calculates
		// a time point that the delay will elapse and checks if HAL_GetTick() has passed that. But if adding delay to stopwatch
		// causes an overflow, then HAL_GetTick() will immediately be greater than the finish time and cause a false positive in the
		// comparison.
		if (HAL_GetTick() - LEDstopwatch > 100)
		{
			// Toggle the LED
			BSP_LED_Toggle(LED2);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
			// Reset the stopwatch
			LEDstopwatch = HAL_GetTick();
			//if (pwm < 1000) pwm_dir = 1;
			//if (pwm > 7000) pwm_dir = -1;
			//pwm += pwm_dir*1000;
			//__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwm);
			sprintf(txbuff, "%c%c%c..PWM = %d\tvelSet = %0.2f\tenc = %d\tvel = %f\tRawOut = %f \n\r", rxB, rxB2, rxB3, pwm, velSet, MotorA.encPos, MotorA.vel, MotorA.pid.rawOut);
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)txbuff, strlen(txbuff));
		}

		// Periodically transmit UART message
		if (HAL_GetTick() - UART2stopwatch > UART_TRANSMIT_RATE)
		{
			//sprintf(txbuff, "Hello, batman -- %d\t%lu\n\r", btn_pressed, BSP_PB_GetState(BUTTON_USER));
			//HAL_UART_Transmit_IT(&huart2, (uint8_t*)txbuff, strlen(txbuff));
			UART2stopwatch = HAL_GetTick();

		}

		// Periodically compute PID
		if (HAL_GetTick() - PIDStopwatch > PID_TD)
		{
			Motor_Vel_Set(&MotorA, velSet);
			PID_Compute(&MotorA);
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, (uint32_t) MotorA.pid.rawOut);
			//sprintf(txbuff, "%c%c%c..PWM = %d\tenc = %d\tvel = %f \n\r", rxB, rxB2, rxB3, pwm, MotorA.encPos, MotorA.vel);
			//HAL_UART_Transmit_IT(&huart2, (uint8_t*)txbuff, strlen(txbuff));
			PIDStopwatch = HAL_GetTick();
		}
/*
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
*/

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


		// Get the DMA2 clock chooching
		__HAL_RCC_DMA1_CLK_ENABLE();

		// Set up DMA
		hdma_usart2_rx.Instance = DMA1_Channel6;
		hdma_usart2_rx.Init.Request = DMA_REQUEST_2;
		hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;// The direction id from peripheral to memory.
		hdma_usart2_rx.Init.PeriphInc = DMA_PINC_ENABLE; // Do not increment the peripheral memory.
		hdma_usart2_rx.Init.MemInc = DMA_MINC_DISABLE; // Do not increment the memory address.
		hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; // DMA peripheral data size only needs to be one byte
		hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE; // As does the memory
		hdma_usart2_rx.Init.Mode = 	DMA_CIRCULAR; // Circular will reload DMA_SxNDTR register after each transfer (DMA_SxNDTR is number of remaining data to be transfered)
		hdma_usart2_rx.Init.Priority = 	DMA_PRIORITY_LOW; // Low priority
		HAL_DMA_Init(&hdma_usart2_rx);

		__HAL_LINKDMA(huart, hdmarx, hdma_usart2_rx);

		// Enable DMA interrupt
		HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, UART_PRIORITY, UART_RX_SUBPRIORITY);
		HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
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
	//multiplier = 2;
	HAL_UART_Transmit(&huart2, (uint8_t*)txbuff, strlen(txbuff), 0xFFFF);
	sprintf(txbuff, "System core clock = %d Hz\n\r", SystemCoreClock);
	HAL_UART_Transmit(&huart2, (uint8_t*)txbuff, strlen(txbuff), 0xFFFF);

	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	uint32_t pfLatecy;
	HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pfLatecy);
	sprintf(txbuff, "Clock Type = %lx\n\rSYSCLK Src = %lx\n\rAHB Div = %lx\n\rAPB1 Div = %lx\n\rAPB2 Div = %lx\n\r", RCC_ClkInitStruct.ClockType, RCC_ClkInitStruct.SYSCLKSource, RCC_ClkInitStruct.AHBCLKDivider, RCC_ClkInitStruct.APB1CLKDivider, RCC_ClkInitStruct.APB2CLKDivider);
	HAL_UART_Transmit(&huart2, (uint8_t*)txbuff, strlen(txbuff), 0xFFFF);
	sprintf(txbuff, "(clk src: RCC_CFGR_SW_MSI = %lx\tRCC_CFGR_SW_HSI = %lx\tRCC_CFGR_SW_HSE = %lx\tRCC_CFGR_SW_PLL = %lx\n\rRCC_CFGR_HPRE_DIV1 = %lx\tRCC_CFGR_HPRE_DIV8 = %lx\n\r", RCC_CFGR_SW_MSI, RCC_CFGR_SW_HSI, RCC_CFGR_SW_HSE, RCC_CFGR_SW_PLL, RCC_CFGR_HPRE_DIV1, RCC_CFGR_HPRE_DIV8);
	HAL_UART_Transmit(&huart2, (uint8_t*)txbuff, strlen(txbuff), 0xFFFF);
}

/*
 * Initialize timer4. Put PWM mode on pin PB6
 */
void TIM4_Init(void)
{
	// Create the configuration structures
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	// First init the timer. The RCC cloxk for timer 4 is done inside the HAL_TIM_PWM_MspInit callback. Why can't we just do that here?
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0; //TODO timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1)
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	// PWM_frequency = timer_tick_frequency / (TIM_Period + 1)
	// TIM_Period = (timer_tick_frequency / PWM_frequency) - 1
	// Assuming APB1 clock is running at 84MHZ, we have no prescaler. If this is true then we are targeting 10khz
	// TIM_Period = 84000000 / 10000 - 1 = 8399
	htim4.Init.Period = PWM_PERIOD;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	// Timer base is set in here:
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
	{
		//Error_Handler();
	}



	// TODO: What does all this do!?
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		//Error_Handler();
	}



	// TODO: What does all this do?!
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 2000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		//Error_Handler();
	}

	// Now set the pin to use the timer
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);
}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    //Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    //Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    //Error_Handler();
  }

    /**Configure the main internal regulator output voltage
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    //Error_Handler();
  }

    /**Configure the Systick interrupt time
    */
  //HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  //HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}


void parseCommand()
{
	char invalidCmd = 0;
	switch (cmdbuff[0])
	{
		// CMD is targeted towards motor A
		case '1':
			break;
		// CMD is targeted towards motor B
		case '2':
			break;
		// No Motor is specified, CMD is invalid
		default:
			invalidCmd = 1;
	}
	float tempfloat;
	switch (cmdbuff[1])
	{
		case 'V':
			// Set the velocity
			if (sscanf(cmdbuff+2, "%f", &tempfloat) != EOF)
			{
				velSet = tempfloat;
				sprintf(txbuff, "I saw V = %f\n", tempfloat);
				HAL_UART_Transmit_IT(&huart2, (uint8_t*)txbuff, strlen(txbuff));
			}
			break;
		case 'v':
			// Return the velocity
			break;
		// COmmand is not recognised
		default:
			invalidCmd = 1;
	}

	if (invalidCmd == 1)
	{
		sprintf(txbuff, "?%s\n", cmdbuff);
		HAL_UART_Transmit_IT(&huart2, (uint8_t*)txbuff, strlen(txbuff));
	}

	//sprintf(somebuff, "saw: %c \n\r", buff);
	//HAL_UART_Transmit_IT(&huart2, txbuff, strlen(rxbuff));
}

// DMA RX callback for UART
void HAL_UART_RxCptlCallback(UART_HandleTypeDef *huart)
{
	__HAL_UART_FLUSH_DRREGISTER(&huart2); // Clear the buffer to prevent overrun

	rxB3 = '!';
	//sprintf(somebuff, "saw: %c \n\r", rxB);
	//HAL_UART_Transmit_IT(&huart2, somebuff, strlen(somebuff));

}

// DMA RX callback for UART
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	__HAL_UART_FLUSH_DRREGISTER(&huart2); // Clear the buffer to prevent overrun

	if (rxB == 'u')
	{
		velSet += 500;
		pwm += 100;
		//__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwm);
	}
	if (rxB == 'd')
	{
		velSet -= 500;
		pwm -= 100;
		//__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwm);
	}
	if (rxB == '\r' | cmdBuffIndex > MAX_CMD_BUFFER_LEN-2)
	{
		cmdbuff[cmdBuffIndex] = '\0';
		cmdBuffIndex = 0;
		//sprintf(txbuff, "cmd = %s\n", cmdbuff);
		//HAL_UART_Transmit_IT(&huart2, (uint8_t*)txbuff, strlen(txbuff));
		parseCommand();
	} else
	{
		cmdbuff[cmdBuffIndex++] = rxB;
	}
	//sprintf(somebuff, "saw: %c \n\r", rxB);
	//HAL_UART_Transmit_IT(&huart2, somebuff, strlen(somebuff));

}



