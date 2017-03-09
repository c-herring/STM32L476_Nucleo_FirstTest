/*
 * globalsp.hpp
 *
 *  Created on: Mar 9, 2017
 *      Author: HeZ
 */


#include "PIDMotor.h"

#ifndef GLOBALSP_HPP_
#define GLOBALSP_HPP_

extern PIDMotor MotorA;


// Pin and port for encoder signals. Note these are handled in the 10-15 ISR, so must be pins 10-15
#define Encoder1_A_Port	GPIOA
#define Encoder1_A_Pin  GPIO_PIN_11
#define Encoder1_B_Port  GPIOA
#define Encoder1_B_Pin  GPIO_PIN_12

// Create the UART2 handle
UART_HandleTypeDef huart2;
// UART2 DMA handle
DMA_HandleTypeDef hdma_usart2_rx;

#define UART_PRIORITY         6
#define UART_RX_SUBPRIORITY   0

// PWM timer handle
TIM_HandleTypeDef htim4;


#define MAX_TX_BUFFER_LEN 500
#define MAX_RX_BUFFER_LEN 50
#define MAX_CMD_BUFFER_LEN 50
// Allocate space for a transmit buffer
char txbuff[MAX_TX_BUFFER_LEN];
// Allocate space for the receive buffer
char rxbuff[MAX_RX_BUFFER_LEN];
// Allocate space for compiling a command string buffer
char cmdbuff[MAX_CMD_BUFFER_LEN];
int32_t cmdBuffIndex;
char executeCmd;
// Create the single byte rx buffer for circular DMA buffer generation
uint8_t rxB;
volatile char rxB2;
volatile char rxB3;



// Function Declarations
void USART2_UART_Init(void);
void parseCommand(uint8_t *buff);
void TIM4_Init(void);
void SystemClock_Config(void);
void GPIO_Init(void);


#endif /* GLOBALSP_HPP_ */
