/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Platform config -----------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include <stdio.h>
#include "string.h"

/* Define config -------------------------------------------------------------*/
#define TRUE			1
#define FALSE           0
#define 	ON				1
#define 	OFF				0
typedef unsigned char   BOOL;
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

#define AudioControl_USART                         	USART1
#define AudioControl_USART_CLK                     	RCC_APB2Periph_USART1
#define AudioControl_USART_GPIO_PORT          		GPIOA
#define AudioControl_USART_GPIO_CLK         		RCC_AHBPeriph_GPIOA

#define AudioControl_USART_TX_PIN                	GPIO_Pin_9
#define AudioControl_USART_TX_SOURCE            	GPIO_PinSource9
#define AudioControl_USART_TX_AF                  	GPIO_AF_1

#define AudioControl_USART_RX_PIN                 	GPIO_Pin_10
#define AudioControl_USART_RX_SOURCE             	GPIO_PinSource10
#define AudioControl_USART_RX_AF                  	GPIO_AF_1

#define AudioControl_USART_IRQn                   	USART1_IRQn

#define NCS_Low()       							GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define NCS_High()      							GPIO_SetBits(GPIOA, GPIO_Pin_4)  
/* Macro ---------------------------------------------------------------------*/
/*
#define UartTxPutChar(x)		\
{	\
     UART1_SendData8(x);	\
     while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);	\
}*/

/* Exported types ------------------------------------------------------------*/
void SPI_CMD_EXAMPLE(void);
//void SPI_Device_Init(void);
void SPI_Config(void);

void USART_TEST(void);
void USART_Config(void);  

void SysTickTimer_Config(void);

void Delay_ms(__IO uint32_t uTime);
void Delay_s(__IO uint32_t mTime);

void TimingDelay_Decrement(void);
void UART_SendByte(uint8_t Data);
void UART_SendString(uint8_t* Data,uint16_t len);
void SystemClkDelay(uint32_t u32Delay);
/* Exported constants --------------------------------------------------------*/

#endif  /* __HW_CONFIG_H */

