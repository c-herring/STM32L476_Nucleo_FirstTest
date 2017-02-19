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


// Allocate space for a transmit buffer
char txbuff[50];

// Function Declarations
void USART2_UART_Init(void);


#endif