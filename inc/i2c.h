#ifndef __I2C_H
#define __I2C_H

void initI2C2(void);

////////////////////////////////////////////////////////



/* Definition for SPI3 HW resources ********************************************/
#define I2C1_SCL_PIN                    GPIO_Pin_8	//PB
#define I2C1_SCL_PINSOURCE				GPIO_PinSource8

#define I2C1_SDA_PIN					GPIO_Pin_9	//PB
#define I2C1_SDA_PINSOURCE				GPIO_PinSource9

#define I2C1_CONTROL_PINS_PORT_CLK      RCC_AHBPeriph_GPIOB
#define I2C1_CONTROL_PINS_PORT			GPIOB

#define I2C_TIMEOUT 100

/* I2C SPE mask */
#define CR1_PE_Set              ((uint16_t)0x0001)
#define CR1_PE_Reset            ((uint16_t)0xFFFE)

/* I2C START mask */
#define CR1_START_Set           ((uint16_t)0x0100)
#define CR1_START_Reset         ((uint16_t)0xFEFF)

#define CR1_POS_Set           ((uint16_t)0x0800)
#define CR1_POS_Reset         ((uint16_t)0xF7FF)

/* I2C STOP mask */
#define CR1_STOP_Set            ((uint16_t)0x0200)
#define CR1_STOP_Reset          ((uint16_t)0xFDFF)

/* I2C ACK mask */
#define CR1_ACK_Set             ((uint16_t)0x0400)
#define CR1_ACK_Reset           ((uint16_t)0xFBFF)

/* I2C ENARP mask */
#define CR1_ENARP_Set           ((uint16_t)0x0010)
#define CR1_ENARP_Reset         ((uint16_t)0xFFEF)

/* I2C NOSTRETCH mask */
#define CR1_NOSTRETCH_Set       ((uint16_t)0x0080)
#define CR1_NOSTRETCH_Reset     ((uint16_t)0xFF7F)

/* I2C registers Masks */
#define CR1_CLEAR_Mask          ((uint16_t)0xFBF5)

/* I2C DMAEN mask */
#define CR2_DMAEN_Set           ((uint16_t)0x0800)
#define CR2_DMAEN_Reset         ((uint16_t)0xF7FF)

/* I2C LAST mask */
#define CR2_LAST_Set            ((uint16_t)0x1000)
#define CR2_LAST_Reset          ((uint16_t)0xEFFF)

/* I2C FREQ mask */
#define CR2_FREQ_Reset          ((uint16_t)0xFFC0)

/* I2C ADD0 mask */
#define OAR1_ADD0_Set           ((uint16_t)0x0001)
#define OAR1_ADD0_Reset         ((uint16_t)0xFFFE)

/* I2C ENDUAL mask */
#define OAR2_ENDUAL_Set         ((uint16_t)0x0001)
#define OAR2_ENDUAL_Reset       ((uint16_t)0xFFFE)

/* I2C ADD2 mask */
#define OAR2_ADD2_Reset         ((uint16_t)0xFF01)

/* I2C F/S mask */
#define CCR_FS_Set              ((uint16_t)0x8000)

/* I2C CCR mask */
#define CCR_CCR_Set             ((uint16_t)0x0FFF)

/* I2C FLAG mask */
#define FLAG_Mask               ((uint32_t)0x00FFFFFF)

/* I2C Interrupt Enable mask */
#define ITEN_Mask               ((uint32_t)0x07000000)


#define I2C_IT_BUF                      ((uint16_t)0x0400)
#define I2C_IT_EVT                      ((uint16_t)0x0200)
#define I2C_IT_ERR                      ((uint16_t)0x0100)


#define  ClockSpeed            400000

#define I2C_DIRECTION_TX 0
#define I2C_DIRECTION_RX 1

#define OwnAddress1 0x28
#define OwnAddress2 0x30


#define I2C1_DMA_CHANNEL_TX           DMA1_Channel6
#define I2C1_DMA_CHANNEL_RX           DMA1_Channel7

#define I2C2_DMA_CHANNEL_TX           DMA1_Channel4
#define I2C2_DMA_CHANNEL_RX           DMA1_Channel5

#define I2C1_DR_Address              0x40005410
#define I2C2_DR_Address              0x40005810

typedef enum
{
    GeneralError = 0,
    StartConditionError,
    AddressAckError,
    BussyTimeoutError,
    RestartConditionError,
    Success
}Status;


Status writeByteI2C1(unsigned char deviceAddress, unsigned char registerAddress, unsigned char data);
Status writeWordI2C1(unsigned char deviceAddress, unsigned char registerAddress, unsigned short data);
Status writeArrayI2C1(unsigned char deviceAddress, unsigned char registerAddress, unsigned char *data, unsigned short length);
Status readByteI2C1(unsigned char deviceAddress, unsigned char registerAddress, unsigned char *data);
Status readWordI2C1(unsigned char deviceAddress, unsigned char registerAddress, unsigned short *data);
Status readDWordI2C1(unsigned char deviceAddress, unsigned char registerAddress, unsigned long *data);
char readArrayI2C1(unsigned char slaveAddress, unsigned char* dataBuffer, unsigned char registerAddress, unsigned short bytesToRead);
char readArrayWithoutRegisterAddressI2C1(unsigned char slaveAddress, unsigned char* dataBuffer, unsigned short bytesToRead);

Status I2C_Master_BufferRead(unsigned char* pBuffer,  unsigned long NumByteToRead, unsigned char SlaveAddress, unsigned char registerAddress);
Status I2C_Master_BufferReadWithoutRegisterAddress(unsigned char* pBuffer,  unsigned long NumByteToRead, unsigned char SlaveAddress);
Status I2C_Master_BufferWrite(unsigned char* pBuffer,  unsigned long NumByteToWrite, unsigned char SlaveAddress, unsigned char registerAddress);
Status I2C_Master_BufferWriteWithoutRegisterAddress(unsigned char* pBuffer,  unsigned long NumByteToWrite, unsigned char SlaveAddress);
#endif
