/* Define to prevent recursive inclusion -------------------------------------*/

#include "PIDMotor.h"
#ifndef __MY_GLOBALS_H
#define __MY_GLOBALS_H


#ifdef __cplusplus
extern "C" {
#endif



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
uint32_t pwm;
float velSet;

PIDMotor_TypeDef MotorA;
#define PID_TD 25 // ms PID loop time

// Function Declarations
void USART2_UART_Init(void);
void parseCommand(uint8_t *buff);
void TIM4_Init(void);
void SystemClock_Config(void);
void GPIO_Init(void);

#ifdef __cplusplus
}
#endif


#endif
