/**
  ******************************************************************************
  * File Name          : stm32l4xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "globals.h"
#include "stm32l4xx_nucleo.h"

extern void Error_Handler(void);

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}


/*
 * Initialize our GPIO
 */
void GPIO_Init(void)
{
	// Enable GPIO clocks
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// Initialize the on-board LED (On GPIOA Pin5)
	BSP_LED_Init(LED2);

	// Set the on-board user button as a GPIO input
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

	// Set the PC0 Pin as a gpio input. This is our breadboard GPIO button
	GPIO_InitTypeDef gpioinitstruct = {0};
	gpioinitstruct.Pin = GPIO_PIN_0;
	gpioinitstruct.Mode = GPIO_MODE_INPUT;
	gpioinitstruct.Pull   = GPIO_NOPULL;
	gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &gpioinitstruct);

	// Set the PA4 Pin as a gpio input. This is out breadboard interrupt button

	gpioinitstruct.Pin = GPIO_PIN_4;
	gpioinitstruct.Mode = GPIO_MODE_IT_FALLING;
	gpioinitstruct.Pull   = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &gpioinitstruct);
	HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	// Set encoder 1 A signal as an interrupt for both rising and falling edge
	gpioinitstruct.Pin = Encoder1_A_Pin;
	gpioinitstruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	gpioinitstruct.Pull   = GPIO_PULLDOWN;
	HAL_GPIO_Init(Encoder1_A_Port, &gpioinitstruct);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	// Set encoder 1 B signal as an interrupt for both rising and falling edge
	gpioinitstruct.Pin = Encoder1_B_Pin;
	gpioinitstruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	gpioinitstruct.Pull   = GPIO_PULLDOWN;
	HAL_GPIO_Init(Encoder1_B_Port, &gpioinitstruct);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	// Set PC10 pin as an output. This is out breadboard LED
	gpioinitstruct.Pin = GPIO_PIN_10;
	gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
	gpioinitstruct.Pull   = GPIO_NOPULL;
	gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &gpioinitstruct);
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

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
	// Why is this done in here. Can't I just do this at the start of my timer init function?
	if (htim_pwm->Instance == TIM4)
	{
		// Enable the TIM4 clock
		__HAL_RCC_TIM4_CLK_ENABLE();
	}
}


