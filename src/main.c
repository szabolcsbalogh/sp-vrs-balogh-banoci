/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : main function.
**
**  Functions   : main
**
**  Environment : Atollic TrueSTUDIO/STM32
**                STMicroelectronics STM32Lxx Standard Peripherals Library
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
**
*****************************************************************************
*/
/* Includes */
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include "mcu.h"
#include "usart.h"
#include "eeprom.h"
#include "main.h"

#define MCP_ADDR 0x30
#define CAL_NUM 30

#define DEBUG_SWD_PIN
#define FACTORY_CALIB_BASE        ((uint32_t)0x1FF80078)
#define FACTORY_CALIB_DATA        ((CALIB_TypeDef *) FACTORY_CALIB_BASE)
#define USER_CALIB_BASE           ((uint32_t)0x08080000)
#define USER_CALIB_DATA           ((CALIB_TypeDef *) USER_CALIB_BASE)
#define TEST_CALIB_DIFF           (int32_t) 50

#define HOT_CAL_TEMP 110
#define COLD_CAL_TEMP  25

#define DEFAULT_HOT_VAL 0x362
#define DEFAULT_COLD_VAL 0x2A8

#define MAX_TEMP_CHNL 16

#define TS_110

#define AVG_SLOPE 	1620
#define V90		597000
#define V_REF		3300000

#define bool _Bool
#define FALSE 0
#define TRUE !FALSE

#define ADC_CONV_BUFF_SIZE 20

#define USERBUTTON_GPIO_PORT	GPIOA
#define USERBUTTON_GPIO_PIN     GPIO_Pin_0
#define USERBUTTON_GPIO_CLK     RCC_AHBPeriph_GPIOA

#define LD_GPIO_PORT 		GPIOB
#define LD_GREEN_GPIO_PIN 		GPIO_Pin_7
#define LD_BLUE_GPIO_PIN             GPIO_Pin_6
#define LD_GPIO_PORT_CLK             RCC_AHBPeriph_GPIOB

#define GPIO_HIGH(a,b) 		a->BSRRL = b
#define GPIO_LOW(a,b)		a->BSRRH = b
#define GPIO_TOGGLE(a,b) 	a->ODR ^= b

typedef struct
{
    uint16_t VREF;
    uint16_t TS_CAL_COLD;
    uint16_t reserved;
    uint16_t TS_CAL_HOT;
} CALIB_TypeDef;

int32_t temperature_C;
ADC_InitTypeDef ADC_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;
DMA_InitTypeDef DMA_InitStructure;
__IO uint16_t 	ADC_ConvertedValue, T_StartupTimeDelay;
__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;
uint16_t ADC_ConvertedValueBuff[ADC_CONV_BUFF_SIZE];
CALIB_TypeDef calibdata;    /* field storing temp sensor calibration data */
uint32_t ADC_Result, INTemperature, refAVG, tempAVG, Address = 0;
volatile bool flag_ADCDMA_TransferComplete;
volatile bool flag_UserButton;
bool isLowPWR = TRUE;
static volatile uint32_t TimingDelay;

void TimingDelay_Decrement(void)
{

  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }

}

void setADCDMA_TransferComplete(void)
{
  flag_ADCDMA_TransferComplete = TRUE;
}

void insertionSort(uint16_t *numbers, uint32_t array_size)
{

	uint32_t i, j;
	uint32_t index;

  for (i=1; i < array_size; i++) {
    index = numbers[i];
    j = i;
    while ((j > 0) && (numbers[j-1] > index)) {
      numbers[j] = numbers[j-1];
      j = j - 1;
    }
    numbers[j] = index;
  }
}

uint32_t interquartileMean(uint16_t *array, uint32_t numOfSamples)
{
    uint32_t sum=0;
    uint32_t  index, maxindex;
    /* discard  the lowest and the highest data samples */
	maxindex = 3 * numOfSamples / 4;
    for (index = (numOfSamples / 4); index < maxindex; index++){
            sum += array[index];
    }
	/* return the mean value of the remaining samples value*/
    return ( sum / (numOfSamples / 2) );
}

void RCC_Configuration(void)
{

  /* Enable HSI Clock */
  RCC_HSICmd(ENABLE);

  /*!< Wait till HSI is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)
  {}

  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

  RCC_MSIRangeConfig(RCC_MSIRange_6);

  RCC_HSEConfig(RCC_HSE_OFF);
  if(RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET )
  {
    while(1);
  }

  /* Enable  comparator clock for PWR mngt */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Enable ADC clock & SYSCFG */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_SYSCFG, ENABLE);

}


void RTC_Configuration(void)
{

  /* Allow access to the RTC */
  PWR_RTCAccessCmd(ENABLE);

  /* Reset Backup Domain */
  RCC_RTCResetCmd(ENABLE);
  RCC_RTCResetCmd(DISABLE);

  /* LSE Enable */
  RCC_LSEConfig(RCC_LSE_ON);

  /* Wait till LSE is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {}

  RCC_RTCCLKCmd(ENABLE);

  /* LCD Clock Source Selection */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

}

FunctionalState  testUserCalibData(void)
{
  int32_t testdiff;
  FunctionalState retval = DISABLE;

  testdiff = USER_CALIB_DATA->TS_CAL_HOT - USER_CALIB_DATA->TS_CAL_COLD;

  if ( testdiff > TEST_CALIB_DIFF )    retval = ENABLE;

  return retval;
}

FunctionalState  testFactoryCalibData(void)
{
  int32_t testdiff;
  FunctionalState retval = DISABLE;

  testdiff = FACTORY_CALIB_DATA->TS_CAL_HOT - FACTORY_CALIB_DATA->TS_CAL_COLD;

  if ( testdiff > TEST_CALIB_DIFF )    retval = ENABLE;

  return retval;
}

void clearADCDMA_TransferComplete(void)
{
  flag_ADCDMA_TransferComplete = FALSE;
}

void  writeCalibData(CALIB_TypeDef* calibStruct)
{

  uint32_t  Address = 0;
  uint32_t  dataToWrite;

  /* Unlock the FLASH PECR register and Data EEPROM memory */
  DATA_EEPROM_Unlock();

  /* Clear all pending flags */
  FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR);

  /*  Data EEPROM Fast Word program of FAST_DATA_32 at addresses defined by
      DATA_EEPROM_START_ADDR and DATA_EEPROM_END_ADDR */

  Address = (uint32_t) USER_CALIB_DATA;


  dataToWrite = 0x00;
  dataToWrite = (uint32_t)(calibStruct->TS_CAL_COLD) << 16;

  FLASHStatus = DATA_EEPROM_ProgramWord(Address, dataToWrite);

  if(FLASHStatus != FLASH_COMPLETE)
  {
    while(1); /* stay in loop in case of crittical programming error */
  }

  Address += 4;

  dataToWrite = 0x00;
  dataToWrite = (uint32_t)(calibStruct->TS_CAL_HOT) << 16;

  FLASHStatus = DATA_EEPROM_ProgramWord(Address, dataToWrite);

}

void configureWakeup (void)
{
  /* Declare initialisation structures for (NVIC) and external interupt (EXTI) */
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Clear IT pending bit from external interrupt Line 20 */
  EXTI_ClearITPendingBit(EXTI_Line20);

  /* Initialise EXTI using its init structure */
  EXTI_InitStructure.EXTI_Line = EXTI_Line20;			 // interrupt generated on RTC Wakeup event (Line 20)
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;    // Use EXTI line as interrupt
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; // Trigg interrupt on rising edge detection
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;				 // Enable EXTI line
  EXTI_Init(&EXTI_InitStructure);

  /* Initialise the NVIC interrupts (IRQ) using its init structure */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;        // set IRQ channel to RTC Wakeup Interrupt
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	 // set channel Preemption priority to 0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         // set channel sub priority to 0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	         // Enable channel
  NVIC_Init(&NVIC_InitStructure);

  /* Clear Wake-up flag */
  PWR->CR |= PWR_CR_CWUF;

  /* Enable PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Allow access to RTC */
  PWR_RTCAccessCmd(ENABLE);

  /* Enable Low Speed External clock */
  RCC_LSEConfig(RCC_LSE_ON);

  /* Wait till LSE is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);

  /* Select LSE clock as RCC Clock source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();

  /* Select 1Hz clock for RTC wake up*/
  RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);

  /* Set Wakeup auto-reload value to 2 sec */
  RTC_SetWakeUpCounter(1);

  /* Clear RTC Interrupt pending bit */
  RTC_ClearITPendingBit(RTC_IT_WUT);

  /* Clear EXTI line20 Interrupt pending bit */
  EXTI_ClearITPendingBit(EXTI_Line20);

  /* Enable the Wakeup Interrupt */
  RTC_ITConfig(RTC_IT_WUT, ENABLE);
}

void configureDMA(void)
{
  /* Declare NVIC init Structure */
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* De-initialise  DMA */
  DMA_DeInit(DMA1_Channel1);

  /* DMA1 channel1 configuration */
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);	     // Set DMA channel Peripheral base address to ADC Data register
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedValueBuff;  // Set DMA channel Memeory base addr to ADC_ConvertedValueBuff address
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                         // Set DMA channel direction to peripheral to memory
  DMA_InitStructure.DMA_BufferSize = ADC_CONV_BUFF_SIZE;                     // Set DMA channel buffersize to peripheral to ADC_CONV_BUFF_SIZE
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	     // Disable DMA channel Peripheral address auto increment
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                    // Enable Memeory increment (To be verified ....)
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;// set Peripheral data size to 8bit
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	     // set Memeory data size to 8bit
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                              // Set DMA in normal mode
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;	                     // Set DMA channel priority to High
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                               // Disable memory to memory option
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);								 // Use Init structure to initialise channel1 (channel linked to ADC)

  /* Enable Transmit Complete Interrup for DMA channel 1 */
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

  /* Setup NVIC for DMA channel 1 interrupt request */
  NVIC_InitStructure.NVIC_IRQChannel =   DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

void configureADC_Temp(void)
{
  uint32_t ch_index;

  /* Enable ADC clock & SYSCFG */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  /* Enable the internal connection of Temperature sensor and with the ADC channels*/
  ADC_TempSensorVrefintCmd(ENABLE);

  /* Wait until ADC + Temp sensor start */
  T_StartupTimeDelay = 1024;
  while (T_StartupTimeDelay--);

  /* Setup ADC common init struct */
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
  ADC_CommonInit(&ADC_CommonInitStructure);


  /* Initialise the ADC1 by using its init structure */
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;	          // Set conversion resolution to 12bit
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;	                          // Enable Scan mode (single conversion for each channel of the group)
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;			  // Disable Continuous conversion
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None; // Disable external conversion trigger
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                  // Set conversion data alignement to right
  ADC_InitStructure.ADC_NbrOfConversion = ADC_CONV_BUFF_SIZE;             // Set conversion data alignement to ADC_CONV_BUFF_SIZE
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular Temperature sensor channel16 and internal reference channel17 configuration */

    for (ch_index = 1; ch_index <= MAX_TEMP_CHNL; ch_index++){
      ADC_RegularChannelConfig(ADC1, ADC_Channel_16, ch_index,
                               ADC_SampleTime_384Cycles);
    }

  ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 17, ADC_SampleTime_384Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 18, ADC_SampleTime_384Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 19, ADC_SampleTime_384Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 20, ADC_SampleTime_384Cycles);
}

void acquireTemperatureData(void)
{
  /* Enable ADC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Wait until the ADC1 is ready */
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET);

  /* re-initialize DMA -- is it needed ?*/
  DMA_DeInit(DMA1_Channel1);
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  DMA_Cmd(DMA1_Channel1, ENABLE);

  /* Enable DMA channel 1 Transmit complete interrupt*/
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

  /* Disable DMA mode for ADC1 */
  ADC_DMACmd(ADC1, DISABLE);

   /* Enable DMA mode for ADC1 */
  ADC_DMACmd(ADC1, ENABLE);

  /* Clear global flag for DMA transfert complete */
  clearADCDMA_TransferComplete();

  /* Start ADC conversion */
  ADC_SoftwareStartConv(ADC1);
}

void powerDownADC_Temper(void)
{
  /* Disable DMA channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  /* Disable ADC1 */
  ADC_Cmd(ADC1, DISABLE);

  /* Disable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);
  /* Disable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, DISABLE);
}

void processTempData(void)
{
  uint32_t index,dataSum;
  dataSum = 0;

  /* sort received data in */
  insertionSort(ADC_ConvertedValueBuff, MAX_TEMP_CHNL);

  /* Calculate the Interquartile mean */
  tempAVG = interquartileMean(ADC_ConvertedValueBuff, MAX_TEMP_CHNL);

  /* Sum up all mesured data for reference temperature average calculation */
  for (index=16; index < ADC_CONV_BUFF_SIZE; index++){
    dataSum += ADC_ConvertedValueBuff[index];
  }
  /* Devide sum up result by 4 for the temperature average calculation*/
  refAVG = dataSum / 4 ;


  /* Calculate temperature in °C from Interquartile mean */
  temperature_C = ( (int32_t) tempAVG - (int32_t) calibdata.TS_CAL_COLD ) ;
  temperature_C = temperature_C * (int32_t)(110 - 25);
  temperature_C = temperature_C /
                  (int32_t)(calibdata.TS_CAL_HOT - calibdata.TS_CAL_COLD);
  temperature_C = temperature_C + 25;
}


/*************************************************************************************************************/
int follow_stream = 0;

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/

void get_csv() {
	uint32_t addr;
	char s[512];
	sprintf(s,"===CSV START===\n");
	PutsUART1(s);
	//find the start
	for(addr = EEPROM_LOG_START_ADDRESS; addr < EEPROM_END_ADDRESS && read_eeprom_32(addr) != (uint32_t)0xffffffff; addr += 4);
	//put data
	for(; addr < EEPROM_END_ADDRESS && read_eeprom_32(addr) != (uint32_t)0xffffffff; addr += 4) {
		sprintf(s,"%d\n", (uint32_t)read_eeprom_32(addr));
		PutsUART1(s);
	}
	for(addr = EEPROM_LOG_START_ADDRESS; addr < EEPROM_END_ADDRESS && read_eeprom_32(addr) != (uint32_t)0xffffffff; addr += 4) {
		sprintf(s,"%d\n", (uint32_t)read_eeprom_32(addr));
		PutsUART1(s);
	}
	sprintf(s,"===CSV END===\n");
	PutsUART1(s);
}

void handleReceivedChar(unsigned char data)
{
	char s[128];
	double temp;
	int i;
	switch(data) {
	case 'h':
		sprintf(s,"F - flow ON\nf - flow off\ng - get CSV\ne - erase data from EEPROM\n");
		PutsUART1(s);
		break;
	case 'F':
		follow_stream = 1;
		break;
	case 'f':
		follow_stream = 0;
		break;
	case 'g':
		get_csv();
		break;
	case 'e':
		sprintf(s,"===EEPROM Clear start====\n");
		PutsUART1(s);
		eeprom_clear();
		sprintf(s,"===EEPROM Clear finished====\n");
		PutsUART1(s);
		break;
	}
}

void conf_analog_all_GPIOS(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIOs clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |
                        RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD |
                        RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOH, ENABLE);

  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;

  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_Init(GPIOH, &GPIO_InitStructure);

#ifdef  DEBUG_SWD_PIN
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All & (~GPIO_Pin_13) & (~GPIO_Pin_14);
#endif

  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Disable GPIOs clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |
                        RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD |
                        RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOH, DISABLE);
}

void  Init_GPIOs (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  conf_analog_all_GPIOS();   /* configure all GPIOs as analog input */

  /* Enable GPIOs clock */
  RCC_AHBPeriphClockCmd(LD_GPIO_PORT_CLK | USERBUTTON_GPIO_CLK, ENABLE);

  /* USER button and WakeUP button init: GPIO set in input interrupt active mode */

  /* Configure User Button pin as input */
  GPIO_InitStructure.GPIO_Pin = USERBUTTON_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(USERBUTTON_GPIO_PORT, &GPIO_InitStructure);

  /* Connect Button EXTI Line to Button GPIO Pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);

  /* Configure User Button and IDD_WakeUP EXTI line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0 ;  // PA0 for User button AND IDD_WakeUP
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set User Button and IDD_WakeUP EXTI Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

/* Configure the GPIO_LED pins  LD3 & LD4*/
  GPIO_InitStructure.GPIO_Pin = LD_GREEN_GPIO_PIN | LD_BLUE_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(LD_GPIO_PORT, &GPIO_InitStructure);
  GPIO_LOW(LD_GPIO_PORT, LD_GREEN_GPIO_PIN);
  GPIO_LOW(LD_GPIO_PORT, LD_BLUE_GPIO_PIN);

/* Disable all GPIOs clock */
  // don't forget to enable clock for GPIOB for LED's
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |
                        RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD |
                        RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOH, DISABLE);

}

void setUserButtonFlag(void)
{
  flag_UserButton = TRUE;
}

void clearUserButtonFlag(void)
{
  flag_UserButton = FALSE;
}

void Delay(uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);

}

int main(void)
{
	char s[1024];
	uint16_t raw_temperature;
	uint64_t tmp1, tmp2;
	//new
	RCC_ClocksTypeDef RCC_Clocks;

	/**
	 * Connection
	 * UART2
	 * pa9
	 * pa10
	 *
	 * yellow scl
	 * blue sdl
	 */

	//new INIT
	//configure clocks
	RCC_Configuration();
	//configure RTC
	RTC_Configuration();
	//set internal voltage to 1.8V and wait for regulator
	PWR_VoltageScalingConfig(PWR_VoltageScaling_Range1);
	while (PWR_GetFlagStatus(PWR_FLAG_VOS) != RESET) ;
	//enable debug features if needed (can be turned of for lower power consumption)
#ifdef  DEBUG_SWD_PIN
    DBGMCU_Config(DBGMCU_SLEEP | DBGMCU_STOP | DBGMCU_STANDBY, ENABLE);
#endif
    RCC_GetClocksFreq(&RCC_Clocks);
    //init gpio
    Init_GPIOs();

    //load callibration data
    if ( testUserCalibData() == ENABLE ) calibdata = *USER_CALIB_DATA;
    else if ( testFactoryCalibData() == ENABLE ) calibdata = *FACTORY_CALIB_DATA;
    else {
      calibdata.TS_CAL_COLD = DEFAULT_COLD_VAL;
      calibdata.TS_CAL_HOT = DEFAULT_HOT_VAL;
      writeCalibData(&calibdata);
      calibdata = *USER_CALIB_DATA;
    }

    //configure wakeup
    configureWakeup();
    //configure DMA
    configureDMA();
    //condifure adc
    configureADC_Temp();
    //init eeprom for temperature logging
    eeprom_init();
    //configure USART
    initUSART1();
    RegisterCallbackUART1(&handleReceivedChar);
    PutsUART1(" Inited\n");

    while(1) {
        acquireTemperatureData();
        if(isLowPWR) {
        	__WFI();
        }else{
        	while (!flag_ADCDMA_TransferComplete);
        }
        //for debug use this instead of __WFI
        powerDownADC_Temper();
        processTempData();
        if(follow_stream && !isLowPWR) {
        	sprintf(s,"Measured %d\n",temperature_C);
        	PutsUART1(s);
        }

        if (flag_UserButton == TRUE){
               clearUserButtonFlag();
               isLowPWR = !isLowPWR;
               sprintf(s,"==> Low Power turned %s\n",(isLowPWR ? "ON":"OFF"));
               PutsUART1(s);
               //if(isLowPWR)
               //   GPIO_HIGH(LD_GPIO_PORT,LD_GREEN_GPIO_PIN);
               //else
               //   GPIO_LOW(LD_GPIO_PORT,LD_GREEN_GPIO_PIN);
               //GPIO_HIGH(LD_GPIO_PORT, LD_BLUE_GPIO_PIN);

               if(isLowPWR) follow_stream = 0;
        }

        eeprom_log_next(temperature_C);

        if(isLowPWR) {
			//enable wakup
			RTC_WakeUpCmd(ENABLE);
			//enter to sleep
			PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
			//reinit
			RCC_Configuration();
			//disable wakeup
			RTC_WakeUpCmd(DISABLE);
        }else{
            SysTick_Config(RCC_Clocks.HCLK_Frequency / 2000);
        	Delay(1000);
        	SysTick->CTRL  &= ~ ( SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk );
        }
    }


	return 0;
}

