/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MY_GLOBALS_H
#define __MY_GLOBALS_H


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
uint8_t txbuff[MAX_TX_BUFFER_LEN];
// Allocate space for the receive buffer
uint8_t rxbuff[MAX_RX_BUFFER_LEN];
// Allocate space for cpmpiling a command string buffer
uint8_t cmdbuff[MAX_CMD_BUFFER_LEN];
uint32_t cmdBuffIndex;
uint8_t executeCmd;
// Create the single byte rx buffer for circular DMA buffer generation
volatile uint8_t rxB;
volatile uint8_t rxB2;
volatile uint8_t rxB3;

// Function Declarations
void USART2_UART_Init(void);
void parseCommand(uint8_t ch);


#endif
