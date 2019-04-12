/* Includes ------------------------------------------------------------------*/
#include "Hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(AudioControl_USART, (uint8_t) ch);

  /* Loop until transmit data register is empty */
  while (USART_GetFlagStatus(AudioControl_USART, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef 	RCC_ClockFreq;

/*SPI variable*/
uint8_t sDummy=0x5A;
//uint8_t Rx_Buffer1 = 0;

void SPI_DMABufferStart(uint8_t pBuffer, uint16_t NumByteToWrite)
{
	DMA_InitTypeDef DMA_InitStructure;

	/* DMA configuration -------------------------------------------------------*/
	/* Deinitialize DMA Streams */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);

	DMA_DeInit(DMA1_Channel3);
	DMA_DeInit(DMA1_Channel2);

	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	
	/* Configure DMA Initialization Structure */
	DMA_InitStructure.DMA_BufferSize = NumByteToWrite ;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI1->DR)) ;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	/* Configure TX DMA */
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST ;
	DMA_InitStructure.DMA_MemoryBaseAddr =(uint32_t) &pBuffer ;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);

	/* Configure DMA Initialization Structure */
	DMA_InitStructure.DMA_BufferSize = NumByteToWrite ;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI1->DR)) ;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	/* Configure RX DMA */
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC ;
	DMA_InitStructure.DMA_MemoryBaseAddr =(uint32_t)&sDummy ; 
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);

	/* Enable DMA SPI TX Stream */
	DMA_Cmd(DMA1_Channel3,ENABLE);
	/* Enable DMA SPI RX Stream */
	DMA_Cmd(DMA1_Channel2,ENABLE);
	/* Enable SPI DMA TX Requsts */
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
	/* Enable SPI DMA RX Requsts */
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
	/* The Data transfer is performed in the SPI using Direct Memory Access */

}

void SPI_DMABufferWait(void)
{
	/* Waiting the end of Data transfer */
	while (DMA_GetFlagStatus(DMA1_FLAG_TC3)==RESET);
	while (DMA_GetFlagStatus(DMA1_FLAG_TC2)==RESET);
	
	/* Clear DMA Transfer Complete Flags */
	DMA_ClearFlag(DMA1_FLAG_GL3);//DMA1_FLAG_TC3
	DMA_ClearFlag(DMA1_FLAG_GL2);//DMA1_FLAG_TC2  
	
	/* Disable DMA SPI TX Stream */
	DMA_Cmd(DMA1_Channel3,DISABLE);
	/* Disable DMA SPI RX Stream */
	DMA_Cmd(DMA1_Channel2,DISABLE);  
	
	/* Disable SPI DMA TX Requsts */
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);
	/* Disable SPI DMA RX Requsts */
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);

}

void SPI_WriteByte(uint8_t Data)
{
#if 1	//use DMA to transfer data
//	Delay_ms(1);
	NCS_Low();	
	
	SPI_DMABufferStart(~(Data),1);
	SPI_DMABufferWait();	

//	Delay_ms(1);	
	NCS_High();	
	
#else	//use regular SPI method to transfer data
//	Delay_ms(1);
	NCS_Low();	
	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
	{};
//	SPI_SendData8(SPI1,Data);
	SPI_SendData8(SPI1, ~(Data));
	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
	{};
	SPI_ReceiveData8(SPI1);

//	Delay_ms(1);	
	NCS_High();
#endif
}

void SPI_CMD_EXAMPLE(void)
{
#if 0	//example 1 , modify delay to check led display
	uint8_t i = 0;
	uint8_t j = 0;

    j=1;
	for (i=0;i<8;i++)
	{
		SPI_WriteByte(j);
		j<<=1;
//		printf(" CMD = 0x%2X ,0x%8X\r\n" ,i,j);
		Delay_ms(50);
	}	
#else	//example 2 , modify delay to check led display
	SPI_WriteByte(0xC0);
	Delay_ms(1000);
	SPI_WriteByte(0x30);
	Delay_ms(1000);
	SPI_WriteByte(0x0C);
	Delay_ms(1000);
	SPI_WriteByte(0x03);
	Delay_ms(1000);	

#endif
}

//void SPI_Device_Init(void)
//{
//	SPI_WriteByte(0xAA);
//	Delay_ms(1);
//	SPI_WriteByte(0x55);
//	Delay_ms(1);
//}

/*====================================================================
PCB					74AHC74595							MCU
----------------------------------------------------------------------			
*			MR#   : master reset (active LOW)			=>VCC
SCLOCK		SHCP  : shift register clock input			=>SPI1_CLK , PA5
LATCH		STCP  : storage register clock input		=>SPI1_NSS , PA4
DATA		DS    : serial data input					=>SPI1_MOSI , PA7
OE			OE#   : output enable input (active LOW)	=>GND
====================================================================*/

void SPI_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	/* Enable the SPI periph */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* Enable SCK, MOSI, MISO and NSS GPIO clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_0);
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_0);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_0);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* SPI  MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* SPI MISO pin configuration */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	NCS_High();
	
	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPI1);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_Init(SPI1, &SPI_InitStructure);

	/* Initialize the FIFO threshold */
	SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);

	/* Enable the SPI peripheral */
	SPI_Cmd(SPI1, ENABLE);

}

void USART_TEST(void)
{
	__IO uint8_t temp;
	
	if(USART_GetFlagStatus(AudioControl_USART, USART_FLAG_RXNE) == SET)
	{
			temp = USART_ReceiveData(AudioControl_USART);
			printf("Press KEY : %c \n\r",temp);

			switch (temp)
			{
				case '1': 

					break;

				case '2': 

					break;	

				case '3':

					break;
					
				default : 
					printf("INPUT CMD not support !\r\n");
					break;
			}
	}
}

void USART_Config(void)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Clock configuration ---------------------------------------------------*/
	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(AudioControl_USART_GPIO_CLK, ENABLE);

	/* Enable USART clock */
	RCC_APB2PeriphClockCmd(AudioControl_USART_CLK, ENABLE);

	/* GPIO configuration ----------------------------------------------------*/
	GPIO_DeInit(AudioControl_USART_GPIO_PORT);

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(AudioControl_USART_GPIO_PORT, AudioControl_USART_TX_SOURCE, AudioControl_USART_TX_AF);
	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig(AudioControl_USART_GPIO_PORT, AudioControl_USART_RX_SOURCE, AudioControl_USART_RX_AF);

	/* Configure USARTx_Tx,USARTx_Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = AudioControl_USART_TX_PIN|AudioControl_USART_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(AudioControl_USART_GPIO_PORT, &GPIO_InitStructure);

	/* USART configuration ---------------------------------------------------*/
	USART_DeInit(AudioControl_USART);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(AudioControl_USART, &USART_InitStructure);

	/* Enable USARTy Receive and Transmit interrupts */
	USART_ITConfig(AudioControl_USART, USART_IT_RXNE, ENABLE); 
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

	/* The software must wait until TC=1. The TC flag remains cleared during all data
	transfers and it is set by hardware at the last frame’s end of transmission*/	
	while (USART_GetFlagStatus(AudioControl_USART, USART_FLAG_TC) == RESET)
	{}

	/* NVIC configuration */
	/* Enable the USARRx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = AudioControl_USART_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPriority = 3; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure); 

	/* Enable the USARRx */
	USART_Cmd(AudioControl_USART, ENABLE);
}

void SysTickTimer_Config(void)
{
	RCC_GetClocksFreq(&RCC_ClockFreq);
	
	#if 1 //debug
	printf("===========================\r\n");
	printf("SYSCLK_Frequency = %d \r\n" , 		RCC_ClockFreq.SYSCLK_Frequency);
	printf("HCLK_Frequency = %d \r\n" , 			RCC_ClockFreq.HCLK_Frequency);
	printf("PCLK_Frequency = %d \r\n" , 			RCC_ClockFreq.PCLK_Frequency);
	printf("ADCCLK_Frequency= %d \r\n" , 		RCC_ClockFreq.ADCCLK_Frequency);
	printf("CECCLK_Frequency = %d \r\n" , 		RCC_ClockFreq.CECCLK_Frequency);
	printf("I2C1CLK_Frequency = %d \r\n" , 		RCC_ClockFreq.I2C1CLK_Frequency);
	printf("USART1CLK_Frequency = %d \r\n" , 	RCC_ClockFreq.USART1CLK_Frequency); 
	#endif /*debug*/
	
	/* Setup SysTick Timer for 1ms interrupts  */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		/* Capture error */
		while (1);
	}
	
	/* Configure the SysTick handler priority */
	NVIC_SetPriority(SysTick_IRQn, 0x01);
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay_ms(__IO uint32_t uTime)
{ 
	uwTimingDelay = uTime;
	while(uwTimingDelay != 0);
}

void Delay_s(__IO uint32_t mTime)
{ 
	uint32_t i;
	for(i=0;i<mTime;i++)
		Delay_ms(1000);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}

//currently not use
/*

void SystemClkDelay(void)
{
	uint32_t i;
	i = 0xffff;
	while(i--);
}

void wtPutChar(uint8_t ccc)
{
	UART1_SendData8(ccc);
	// Loop until the end of transmission 
	while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);	
}

u16 GetAbsTime(u16 a,u16 b)
{
	u16 c;
	if(a>=b) c=(a-b);
	else c=65535-(b-a);	
	
	return c;
}
*/

void UART_SendByte(uint8_t Data)
{
	USART_SendData(AudioControl_USART , (unsigned char)Data);
	while (USART_GetFlagStatus(AudioControl_USART , USART_FLAG_TXE)==RESET);
	{
	}
}

void UART_SendString(uint8_t* Data,uint16_t len)
{
	uint16_t i=0;
	for(i=0;i<len;i++ )
	{
		UART_SendByte(Data[i]);
	}
}

void SystemClkDelay(uint32_t u32Delay)
{
	//uint32_t i;
	//i = 0xffff;
	while(u32Delay--);
}


