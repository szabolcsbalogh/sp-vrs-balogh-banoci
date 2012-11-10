#include "usart.h"
void (* gCallback2)(unsigned char) = 0;


/**
  * @brief  Configures the USART1 Peripheral.
  * @param  None
  * @retval None
  */
void initUSART2(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock */	//turning on the needed peripherals
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  //choosing peripherals for selected pins
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

  /* Configure USART Tx and Rx pins */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//alternating function
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;		//not very important, do not change
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//output is push pull type
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		//do not use pull ups or pull downs
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;	//choosing the pins
  GPIO_Init(GPIOA, &GPIO_InitStructure);				//applying settings


  //usart configuration
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);				//applying settings

  //configuring interrupts
  /* NVIC configuration */
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			//choose interrupt source
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);			//choosing which event should cause interrupt
  /* Enable USART */
  USART_Cmd(USART2, ENABLE);								//turning USART on.
}
/**
 * @brief Called by system when an interrupt occurs
  * @param  None
  * @retval None
 */

void USART2_IRQHandler(void)
{
	uint8_t pom = 0;
	//some error handling - check reference manual for more details
	uint32_t aSr = USART2->SR;
	if((aSr&0b1000) == 0b1000)
	{
		uint32_t aSr = USART2->SR;
		uint8_t aDr = USART2->DR;
	}

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)		//checking if a character was received
																//it might be a different interrupt
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);			//clearing interrupt flag
		pom = USART_ReceiveData(USART2);						//taking the received character
		if (gCallback2)											//checking if a callback function is registrated
		{
			gCallback2(pom);									//calling the registrated function
		}
	}
}


/**
  * @brief  Sends character to usart1, but first checks if usart is free
  * @param  ch - character to send
  * @retval None
  */
void PutcUART2(char ch)
{
	USART_SendData(USART2, (uint8_t) ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
	{}
}

/**
  * @brief  Sends string to usart
  * @param  c - buffer to send
  * @retval None
  */
void PutsUART2(char* c)
{
	int i = 0;
	while(c[i] != '\0')
	{
		PutcUART2(c[i++]);
	}
}

/**
  * @brief  Sends buffer of specified length to usart
  * @param  c - buffer to send
  * @param  length - length of the buffer
  * @retval None
  */
void PutbUART2(char* c, unsigned short length)
{
	int i = 0;
	for (i = 0; i < length; i++)
	{
		PutcUART2(c[i]);
	}
}


/**
 * ****************************************************************
 * ****************************************************************
 *
 */

void RegisterCallbackUART2(void *callback)
{
	 gCallback2 = callback;
}
