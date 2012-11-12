#include "i2c.h"
#include "mcu.h"

unsigned short gI2C_Timeout;
unsigned char gDataBuffer[16];

void initI2C1(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	RCC_AHBPeriphClockCmd(I2C1_CONTROL_PINS_PORT_CLK, ENABLE);

	//GPIO port, PIN, AF Function
	GPIO_PinAFConfig(I2C1_CONTROL_PINS_PORT, I2C1_SDA_PINSOURCE, GPIO_AF_I2C1);
	GPIO_PinAFConfig(I2C1_CONTROL_PINS_PORT, I2C1_SCL_PINSOURCE, GPIO_AF_I2C1);

	/*!< GPIO configuration */
	/*!< Configure sEE_I2C pins: SCL */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;


	GPIO_InitStructure.GPIO_Pin = I2C1_SCL_PIN;
	GPIO_Init(I2C1_CONTROL_PINS_PORT, &GPIO_InitStructure);

	/*!< Configure sEE_I2C pins: SDA */
	GPIO_InitStructure.GPIO_Pin = I2C1_SDA_PIN;
	GPIO_Init(I2C1_CONTROL_PINS_PORT, &GPIO_InitStructure);

	I2C_InitTypeDef  I2C_InitStructure;

	/*!< I2C configuration */
	/* sEE_I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 200000;

	/* Apply sEE_I2C configuration after enabling it */
	I2C_Init(I2C1, &I2C_InitStructure);

    I2C_Cmd(I2C1, ENABLE);
}

Status writeByteI2C1(unsigned char deviceAddress, unsigned char registerAddress, unsigned char data)
{
    Status error = I2C_Master_BufferWrite(&data, 1, deviceAddress, registerAddress);

    return error;
}

Status writeArrayI2C1(unsigned char deviceAddress, unsigned char registerAddress, unsigned char *data, unsigned short length)
{
    Status error = I2C_Master_BufferWrite(data, length, deviceAddress, registerAddress);
    return error;
}

Status readByteI2C1(unsigned char deviceAddress, unsigned char registerAddress, unsigned char *data)
{
    gDataBuffer[0] = registerAddress;

    Status error = I2C_Master_BufferRead(gDataBuffer, 1, deviceAddress, registerAddress);

    *data = gDataBuffer[0];

    return error;
}

Status readWordI2C1(unsigned char deviceAddress, unsigned char registerAddress, unsigned short *data)
{
    Status error =  I2C_Master_BufferRead(gDataBuffer, 2, deviceAddress, registerAddress);

    *data = gDataBuffer[0];
    *data = ((*data)<<8) + gDataBuffer[1];

    return error;
}

Status readDWordI2C1(unsigned char deviceAddress, unsigned char registerAddress, unsigned long *data)
{
    Status error = I2C_Master_BufferRead(gDataBuffer, 4, deviceAddress, registerAddress);

    *data = gDataBuffer[0];
    *data = ((*data)<<8) + gDataBuffer[1];
    *data = ((*data)<<8) + gDataBuffer[2];
    *data = ((*data)<<8) + gDataBuffer[3];

    return error;
}

char readArrayI2C1(unsigned char slaveAddress, unsigned char* dataBuffer, unsigned char registerAddress, unsigned short bytesToRead)
{
    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    /* While the bus is busy */
    gI2C_Timeout = I2C_TIMEOUT;
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) if((gI2C_Timeout--) == 0) return -1;

    /* Send START condition */
    I2C_GenerateSTART(I2C1, ENABLE);

    /* Test on EV5 and clear it */
    gI2C_Timeout = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) if((gI2C_Timeout--) == 0) return -2;

    /* Send MPU6050 address for write */
    I2C_Send7bitAddress(I2C1, slaveAddress, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    gI2C_Timeout = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) if((gI2C_Timeout--) == 0) return -3;

    /* Clear EV6 by setting again the PE bit */
    I2C_Cmd(I2C1, ENABLE);

    /* Send the Device's internal address to read from */
    I2C_SendData(I2C1, registerAddress);

    /* Test on EV8 and clear it */
    gI2C_Timeout = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) if((gI2C_Timeout--) == 0) return -4;

    /* Send START condition a second time */
    I2C_GenerateSTART(I2C1, ENABLE);

    /* Test on EV5 and clear it */
    gI2C_Timeout = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) if((gI2C_Timeout--) == 0) return -5;

    /* Send device address for read */
    I2C_Send7bitAddress(I2C1, slaveAddress, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
    gI2C_Timeout = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) if((gI2C_Timeout--) == 0) return -6;

    /* While there is data to be read */
    while(bytesToRead)
    {
        if(bytesToRead == 1)
        {
            /* Disable Acknowledgement */
            I2C_AcknowledgeConfig(I2C1, DISABLE);

            /* Send STOP Condition */
            I2C_GenerateSTOP(I2C1, ENABLE);
            /* Test on stop */
            gI2C_Timeout = I2C_TIMEOUT;
            while(I2C1->CR1 & I2C_CR1_STOP) if((gI2C_Timeout--) == 0) return -8;
        }

        /* Test on EV7 and clear it */
        gI2C_Timeout = I2C_TIMEOUT;
        while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED | I2C_FLAG_BTF)) if((gI2C_Timeout--) == 0) return -7;
        /* Read a byte from the device */
        *dataBuffer = I2C_ReceiveData(I2C1);

        /* Point to the next location where the byte read will be saved */
        dataBuffer++;

        /* Decrement the read bytes counter */
        bytesToRead--;
  }

  return 0;
}

char readArrayWithoutRegisterAddressI2C1(unsigned char slaveAddress, unsigned char* dataBuffer, unsigned short bytesToRead)
{
    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    /* While the bus is busy */
    gI2C_Timeout = I2C_TIMEOUT;
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) if((gI2C_Timeout--) == 0) return -1;

    /* Send START condition a second time */
    I2C_GenerateSTART(I2C1, ENABLE);

    /* Test on EV5 and clear it */
    gI2C_Timeout = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) if((gI2C_Timeout--) == 0) return -5;

    /* Send device address for read */
    I2C_Send7bitAddress(I2C1, slaveAddress, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
    gI2C_Timeout = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) if((gI2C_Timeout--) == 0) return -6;

    /* While there is data to be read */
    while(bytesToRead)
    {
        if(bytesToRead == 1)
        {
            /* Disable Acknowledgement */
            I2C_AcknowledgeConfig(I2C1, DISABLE);

            /* Send STOP Condition */
            I2C_GenerateSTOP(I2C1, ENABLE);
            /* Test on stop */
            gI2C_Timeout = I2C_TIMEOUT;
            while(I2C1->CR1 & I2C_CR1_STOP) if((gI2C_Timeout--) == 0) return -8;
        }

        /* Test on EV7 and clear it */
        gI2C_Timeout = I2C_TIMEOUT;
        while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED | I2C_FLAG_BTF)) if((gI2C_Timeout--) == 0) return -7;
        /* Read a byte from the device */
        *dataBuffer = I2C_ReceiveData(I2C1);

        /* Point to the next location where the byte read will be saved */
        dataBuffer++;

        /* Decrement the read bytes counter */
        bytesToRead--;
  }

  return 0;
}

Status I2C_Master_BufferRead(unsigned char* pBuffer,  unsigned long NumByteToRead, unsigned char SlaveAddress, unsigned char registerAddress)
{
    //SwBreak();
    unsigned long temp = 0;
    unsigned long Timeout = 0;

    /* Enable Acknowledgement to be ready for another reception */
    I2C1->CR1  |= CR1_ACK_Set;
    /* Clear POS bit */
    I2C1->CR1  &= CR1_POS_Reset;
    /* Enable Error IT (used in all modes: DMA, Polling and Interrupts */
    //I2C1->CR2 |= I2C_IT_ERR;

    Timeout = 0xFFFF;
    /* Send START condition */
    I2C1->CR1 |= CR1_START_Set;
    /* Wait until SB flag is set: EV5 */
    while ((I2C1->SR1&0x0001) != 0x0001)
    {
        if (Timeout-- == 0)
        {
            return StartConditionError;
        }
    }

    /* Send slave address */
    /* Reset the address bit0 for write*/
    SlaveAddress &= 0xFFFE;
    /* Send the slave address */
    I2C1->DR = SlaveAddress;
    Timeout = I2C_TIMEOUT;
    /* Wait until ADDR is set: EV6 */
    while ((I2C1->SR1 &0x0002) != 0x0002)
    {
        if (Timeout-- == 0)
        {
            I2C1->CR1 |= CR1_STOP_Set;
            /* Make sure that the STOP bit is cleared by Hardware */
            while ((I2C1->CR1&0x200) == 0x200);
            return AddressAckError;
        }

    }
    /* Clear ADDR flag by reading SR2 register */
    temp = I2C1->SR2;

    I2C1->DR = registerAddress;
    /* Poll on BTF to receive data because in polling mode we can not guarantee the
      EV8 software sequence is managed before the current byte transfer completes */
    while ((I2C1->SR1 & 0x00004) != 0x000004);

    if (NumByteToRead == 1)
    {
        Timeout = I2C_TIMEOUT;
        /* Send repeated START condition */
        I2C1->CR1 |= CR1_START_Set;
        /* Wait until SB flag is set: EV5  */
        while ((I2C1->SR1&0x0001) != 0x0001)
        {
            if (Timeout-- == 0)
            {
                return RestartConditionError;
            }
        }
        /* Send slave address */
        /* Reset the address bit0 for read */
        SlaveAddress |= 0x0001;
        /* Send the slave address */
        I2C1->DR = SlaveAddress;
        /* Wait until ADDR is set: EV6_3, then program ACK = 0, clear ADDR
        and program the STOP just after ADDR is cleared. The EV6_3
        software sequence must complete before the current byte end of transfer.*/
        /* Wait until ADDR is set */
        Timeout = I2C_TIMEOUT;
        while ((I2C1->SR1&0x0002) != 0x0002)
        {
            if (Timeout-- == 0)
            {
                /* EV8_2: Wait until BTF is set before programming the STOP */
                while ((I2C1->SR1 & 0x00004) != 0x000004);
                /* Send STOP condition */
                I2C1->CR1 |= CR1_STOP_Set;
                /* Make sure that the STOP bit is cleared by Hardware */
                while ((I2C1->CR1&0x200) == 0x200);
                return AddressAckError;
            }
        }
        /* Clear ACK bit */
        I2C1->CR1 &= CR1_ACK_Reset;
        /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
        software sequence must complete before the current byte end of transfer */
        __disable_irq();
        /* Clear ADDR flag */
        temp = I2C1->SR2;
        /* Program the STOP */
        I2C1->CR1 |= CR1_STOP_Set;
        /* Re-enable IRQs */
        __enable_irq();
        /* Wait until a data is received in DR register (RXNE = 1) EV7 */
        while ((I2C1->SR1 & 0x00040) != 0x000040);
        /* Read the data */
        *pBuffer = I2C1->DR;
        /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
        while ((I2C1->CR1&0x200) == 0x200);
        /* Enable Acknowledgement to be ready for another reception */
        I2C1->CR1 |= CR1_ACK_Set;
    }

    else if (NumByteToRead == 2)
    {
        /* Set POS bit */
        I2C1->CR1 |= CR1_POS_Set;
        Timeout = I2C_TIMEOUT;
        /* Send START condition */
        I2C1->CR1 |= CR1_START_Set;
        /* Wait until SB flag is set: EV5 */
        while ((I2C1->SR1&0x0001) != 0x0001)
        {
            if (Timeout-- == 0)
            {
                return RestartConditionError;
            }
        }
        Timeout = I2C_TIMEOUT;
        /* Send slave address */
        /* Set the address bit0 for read */
        SlaveAddress |= 0x0001;
        /* Send the slave address */
        I2C1->DR = SlaveAddress;
        /* Wait until ADDR is set: EV6 */
        while ((I2C1->SR1&0x0002) != 0x0002)
        {
            if (Timeout-- == 0)
            {
                I2C1->CR1 |= CR1_STOP_Set;
                /* Make sure that the STOP bit is cleared by Hardware */
                while ((I2C1->CR1&0x200) == 0x200);
                return AddressAckError;
            }
        }
        /* EV6_1: The acknowledge disable should be done just after EV6,
        that is after ADDR is cleared, so disable all active IRQs around ADDR clearing and
        ACK clearing */
        __disable_irq();
        /* Clear ADDR by reading SR2 register  */
        temp = I2C1->SR2;
        /* Clear ACK */
        I2C1->CR1 &= CR1_ACK_Reset;
        /*Re-enable IRQs */
        __enable_irq();
        /* Wait until BTF is set */
        while ((I2C1->SR1 & 0x00004) != 0x000004);
        /* Disable IRQs around STOP programming and data reading because of the limitation ?*/
        __disable_irq();
        /* Program the STOP */
        I2C_GenerateSTOP(I2C1, ENABLE);
        /* Read first data */
        *pBuffer = I2C1->DR;
        /* Re-enable IRQs */
        __enable_irq();
        /**/
        pBuffer++;
        /* Read second data */
        *pBuffer = I2C1->DR;
        /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
        while ((I2C1->CR1&0x200) == 0x200);
        /* Enable Acknowledgement to be ready for another reception */
        I2C1->CR1  |= CR1_ACK_Set;
        /* Clear POS bit */
        I2C1->CR1  &= CR1_POS_Reset;
    }

    else
    {

        Timeout = I2C_TIMEOUT;
        /* Send START condition */
        I2C1->CR1 |= CR1_START_Set;
        /* Wait until SB flag is set: EV5 */
        while ((I2C1->SR1&0x0001) != 0x0001)
        {
            if (Timeout-- == 0)
            {
                return RestartConditionError;
            }
        }
        Timeout = I2C_TIMEOUT;
        /* Send slave address */
        /* Reset the address bit0 for write */
        SlaveAddress |= OAR1_ADD0_Set;
        /* Send the slave address */
        I2C1->DR = SlaveAddress;
        /* Wait until ADDR is set: EV6 */
        while ((I2C1->SR1&0x0002) != 0x0002)
        {
            if (Timeout-- == 0)
            {
                I2C1->CR1 |= CR1_STOP_Set;
                /* Make sure that the STOP bit is cleared by Hardware */
                while ((I2C1->CR1&0x200) == 0x200);
                return AddressAckError;
            }
        }
        /* Clear ADDR by reading SR2 status register */
        temp = I2C1->SR2;
        /* While there is data to be read */
        while (NumByteToRead)
        {
            /* Receive bytes from first byte until byte N-3 */
            if (NumByteToRead != 3)
            {
                /* Poll on BTF to receive data because in polling mode we can not guarantee the
                EV7 software sequence is managed before the current byte transfer completes */
                while ((I2C1->SR1 & 0x00004) != 0x000004);
                /* Read data */
                *pBuffer = I2C1->DR;
                /* */
                pBuffer++;
                /* Decrement the read bytes counter */
                NumByteToRead--;
            }

            /* it remains to read three data: data N-2, data N-1, Data N */
            if (NumByteToRead == 3)
            {

                /* Wait until BTF is set: Data N-2 in DR and data N -1 in shift register */
                while ((I2C1->SR1 & 0x00004) != 0x000004);
                /* Clear ACK */
                I2C1->CR1 &= CR1_ACK_Reset;

                /* Disable IRQs around data reading and STOP programming because of the
                limitation ? */
                __disable_irq();
                /* Read Data N-2 */
                *pBuffer = I2C1->DR;
                /* Increment */
                pBuffer++;
                /* Program the STOP */
                I2C1->CR1 |= CR1_STOP_Set;
                /* Read DataN-1 */
                *pBuffer = I2C1->DR;
                /* Re-enable IRQs */
                __enable_irq();
                /* Increment */
                pBuffer++;
                /* Wait until RXNE is set (DR contains the last data) */
                while ((I2C1->SR1 & 0x00040) != 0x000040);
                /* Read DataN */
                *pBuffer = I2C1->DR;
                /* Reset the number of bytes to be read by master */
                NumByteToRead = 0;

            }
        }
        /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
        while ((I2C1->CR1&0x200) == 0x200);
        /* Enable Acknowledgement to be ready for another reception */
        I2C1->CR1 |= CR1_ACK_Set;

    }

    Timeout = I2C_TIMEOUT;

    /* While the bus is busy */
    while ((I2C1->SR2 & 0x00002) != 0x000002)
    {
        if (Timeout-- == 0)
            return BussyTimeoutError;
    }

    return Success;
}

Status I2C_Master_BufferReadWithoutRegisterAddress(unsigned char* pBuffer,  unsigned long NumByteToRead, unsigned char SlaveAddress)
{
    //SwBreak();
    unsigned long temp = 0;
    unsigned long Timeout = 0;

    /* Enable Acknowledgement to be ready for another reception */
    I2C1->CR1  |= CR1_ACK_Set;
    /* Clear POS bit */
    I2C1->CR1  &= CR1_POS_Reset;
    /* Enable Error IT (used in all modes: DMA, Polling and Interrupts */
    //I2C1->CR2 |= I2C_IT_ERR;

    if (NumByteToRead == 1)
    {
        Timeout = I2C_TIMEOUT;
        /* Send repeated START condition */
        I2C1->CR1 |= CR1_START_Set;
        /* Wait until SB flag is set: EV5  */
        while ((I2C1->SR1&0x0001) != 0x0001)
        {
            if (Timeout-- == 0)
            {
                return RestartConditionError;
            }
        }
        /* Send slave address */
        /* Reset the address bit0 for read */
        SlaveAddress |= 0x0001;
        /* Send the slave address */
        I2C1->DR = SlaveAddress;
        /* Wait until ADDR is set: EV6_3, then program ACK = 0, clear ADDR
        and program the STOP just after ADDR is cleared. The EV6_3
        software sequence must complete before the current byte end of transfer.*/
        /* Wait until ADDR is set */
        Timeout = I2C_TIMEOUT;
        while ((I2C1->SR1&0x0002) != 0x0002)
        {
            if (Timeout-- == 0)
            {
                /* EV8_2: Wait until BTF is set before programming the STOP */
                while ((I2C1->SR1 & 0x00004) != 0x000004);
                /* Send STOP condition */
                I2C1->CR1 |= CR1_STOP_Set;
                /* Make sure that the STOP bit is cleared by Hardware */
                while ((I2C1->CR1&0x200) == 0x200);
                return AddressAckError;
            }
        }
        /* Clear ACK bit */
        I2C1->CR1 &= CR1_ACK_Reset;
        /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
        software sequence must complete before the current byte end of transfer */
        __disable_irq();
        /* Clear ADDR flag */
        temp = I2C1->SR2;
        /* Program the STOP */
        I2C1->CR1 |= CR1_STOP_Set;
        /* Re-enable IRQs */
        __enable_irq();
        /* Wait until a data is received in DR register (RXNE = 1) EV7 */
        while ((I2C1->SR1 & 0x00040) != 0x000040);
        /* Read the data */
        *pBuffer = I2C1->DR;
        /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
        while ((I2C1->CR1&0x200) == 0x200);
        /* Enable Acknowledgement to be ready for another reception */
        I2C1->CR1 |= CR1_ACK_Set;
    }

    else if (NumByteToRead == 2)
    {
        /* Set POS bit */
        I2C1->CR1 |= CR1_POS_Set;
        Timeout = I2C_TIMEOUT;
        /* Send START condition */
        I2C1->CR1 |= CR1_START_Set;
        /* Wait until SB flag is set: EV5 */
        while ((I2C1->SR1&0x0001) != 0x0001)
        {
            if (Timeout-- == 0)
            {
                return RestartConditionError;
            }
        }
        Timeout = I2C_TIMEOUT;
        /* Send slave address */
        /* Set the address bit0 for read */
        SlaveAddress |= 0x0001;
        /* Send the slave address */
        I2C1->DR = SlaveAddress;
        /* Wait until ADDR is set: EV6 */
        while ((I2C1->SR1&0x0002) != 0x0002)
        {
            if (Timeout-- == 0)
            {
                I2C1->CR1 |= CR1_STOP_Set;
                /* Make sure that the STOP bit is cleared by Hardware */
                while ((I2C1->CR1&0x200) == 0x200);
                return AddressAckError;
            }
        }
        /* EV6_1: The acknowledge disable should be done just after EV6,
        that is after ADDR is cleared, so disable all active IRQs around ADDR clearing and
        ACK clearing */
        __disable_irq();
        /* Clear ADDR by reading SR2 register  */
        temp = I2C1->SR2;
        /* Clear ACK */
        I2C1->CR1 &= CR1_ACK_Reset;
        /*Re-enable IRQs */
        __enable_irq();
        /* Wait until BTF is set */
        while ((I2C1->SR1 & 0x00004) != 0x000004);
        /* Disable IRQs around STOP programming and data reading because of the limitation ?*/
        __disable_irq();
        /* Program the STOP */
        I2C_GenerateSTOP(I2C1, ENABLE);
        /* Read first data */
        *pBuffer = I2C1->DR;
        /* Re-enable IRQs */
        __enable_irq();
        /**/
        pBuffer++;
        /* Read second data */
        *pBuffer = I2C1->DR;
        /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
        while ((I2C1->CR1&0x200) == 0x200);
        /* Enable Acknowledgement to be ready for another reception */
        I2C1->CR1  |= CR1_ACK_Set;
        /* Clear POS bit */
        I2C1->CR1  &= CR1_POS_Reset;
    }

    else
    {

        Timeout = I2C_TIMEOUT;
        /* Send START condition */
        I2C1->CR1 |= CR1_START_Set;
        /* Wait until SB flag is set: EV5 */
        while ((I2C1->SR1&0x0001) != 0x0001)
        {
            if (Timeout-- == 0)
            {
                return RestartConditionError;
            }
        }
        Timeout = I2C_TIMEOUT;
        /* Send slave address */
        /* Reset the address bit0 for write */
        SlaveAddress |= OAR1_ADD0_Set;
        /* Send the slave address */
        I2C1->DR = SlaveAddress;
        /* Wait until ADDR is set: EV6 */
        while ((I2C1->SR1&0x0002) != 0x0002)
        {
            if (Timeout-- == 0)
            {
                I2C1->CR1 |= CR1_STOP_Set;
                /* Make sure that the STOP bit is cleared by Hardware */
                while ((I2C1->CR1&0x200) == 0x200);
                return AddressAckError;
            }
        }
        /* Clear ADDR by reading SR2 status register */
        temp = I2C1->SR2;
        /* While there is data to be read */
        while (NumByteToRead)
        {
            /* Receive bytes from first byte until byte N-3 */
            if (NumByteToRead != 3)
            {
                /* Poll on BTF to receive data because in polling mode we can not guarantee the
                EV7 software sequence is managed before the current byte transfer completes */
                while ((I2C1->SR1 & 0x00004) != 0x000004);
                /* Read data */
                *pBuffer = I2C1->DR;
                /* */
                pBuffer++;
                /* Decrement the read bytes counter */
                NumByteToRead--;
            }

            /* it remains to read three data: data N-2, data N-1, Data N */
            if (NumByteToRead == 3)
            {

                /* Wait until BTF is set: Data N-2 in DR and data N -1 in shift register */
                while ((I2C1->SR1 & 0x00004) != 0x000004);
                /* Clear ACK */
                I2C1->CR1 &= CR1_ACK_Reset;

                /* Disable IRQs around data reading and STOP programming because of the
                limitation ? */
                __disable_irq();
                /* Read Data N-2 */
                *pBuffer = I2C1->DR;
                /* Increment */
                pBuffer++;
                /* Program the STOP */
                I2C1->CR1 |= CR1_STOP_Set;
                /* Read DataN-1 */
                *pBuffer = I2C1->DR;
                /* Re-enable IRQs */
                __enable_irq();
                /* Increment */
                pBuffer++;
                /* Wait until RXNE is set (DR contains the last data) */
                while ((I2C1->SR1 & 0x00040) != 0x000040);
                /* Read DataN */
                *pBuffer = I2C1->DR;
                /* Reset the number of bytes to be read by master */
                NumByteToRead = 0;

            }
        }
        /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
        while ((I2C1->CR1&0x200) == 0x200);
        /* Enable Acknowledgement to be ready for another reception */
        I2C1->CR1 |= CR1_ACK_Set;

    }

    Timeout = I2C_TIMEOUT;

    /* While the bus is busy */
    while ((I2C1->SR2 & 0x00002) != 0x000002)
    {
        if (Timeout-- == 0)
            return BussyTimeoutError;
    }

    return Success;
}


Status I2C_Master_BufferWrite(unsigned char* pBuffer,  unsigned long NumByteToWrite, unsigned char SlaveAddress, unsigned char registerAddress)
{

    unsigned long temp = 0;
    unsigned long Timeout = 0;

     //Enable Error IT (used in all modes: DMA, Polling and Interrupts
    //I2C1->CR2 |= I2C_IT_ERR;

    Timeout = I2C_TIMEOUT;
     //Send START condition
    I2C1->CR1 |= CR1_START_Set;
     //Wait until SB flag is set: EV5
    while ((I2C1->SR1&0x0001) != 0x0001)
    {
        if (Timeout-- == 0)
        {
            return StartConditionError;
        }
    }

     //Send slave address
     //Reset the address bit0 for write
    SlaveAddress &= 0xFFFE;
     //Send the slave address
    I2C1->DR = SlaveAddress;
    Timeout = I2C_TIMEOUT;
     //Wait until ADDR is set: EV6
    while ((I2C1->SR1 &0x0002) != 0x0002)
    {
        if (Timeout-- == 0)
        {
            I2C1->CR1 |= CR1_STOP_Set;
             //Make sure that the STOP bit is cleared by Hardware
            while ((I2C1->CR1&0x200) == 0x200);
            return AddressAckError;
        }
    }

     //Clear ADDR flag by reading SR2 register
    temp = I2C1->SR2;

    I2C1->DR = registerAddress;
     //Poll on BTF to receive data because in polling mode we can not guarantee the
      //EV8 software sequence is managed before the current byte transfer completes
    while ((I2C1->SR1 & 0x00004) != 0x000004);
     //Write the first data in DR register (EV8_1)
    I2C1->DR = *pBuffer;
     //Increment
    pBuffer++;
     //Decrement the number of bytes to be written
    NumByteToWrite--;
     //While there is data to be written
    while (NumByteToWrite--)
    {
         //Poll on BTF to receive data because in polling mode we can not guarantee the
          //EV8 software sequence is managed before the current byte transfer completes
        while ((I2C1->SR1 & 0x00004) != 0x000004);
         //Send the current byte
        I2C1->DR = *pBuffer;
         //Point to the next byte to be written
        pBuffer++;
    }
     //EV8_2: Wait until BTF is set before programming the STOP
    Timeout = I2C_TIMEOUT;
    while ((I2C1->SR1 & 0x00004) != 0x000004)
    {
		if (Timeout-- == 0)
			return BussyTimeoutError;
	}
     //Send STOP condition
    I2C1->CR1 |= CR1_STOP_Set;
     //Make sure that the STOP bit is cleared by Hardware
    Timeout = I2C_TIMEOUT;
    while ((I2C1->CR1&0x200) == 0x200)
    {
		if (Timeout-- == 0)
			return BussyTimeoutError;
	}

     //While the bus is busy
    Timeout = I2C_TIMEOUT;
    while ((I2C1->SR2 & 0x00002) != 0x000002)
    {
        if (Timeout-- == 0)
            return BussyTimeoutError;
    }
    return Success;
}

Status I2C_Master_BufferWriteWithoutRegisterAddress(unsigned char* pBuffer,  unsigned long NumByteToWrite, unsigned char SlaveAddress)
{

    unsigned long temp = 0;
    unsigned long Timeout = 0;

     //Enable Error IT (used in all modes: DMA, Polling and Interrupts
    //I2C1->CR2 |= I2C_IT_ERR;

    Timeout = I2C_TIMEOUT;
     //Send START condition
    I2C1->CR1 |= CR1_START_Set;
     //Wait until SB flag is set: EV5
    while ((I2C1->SR1&0x0001) != 0x0001)
    {
        if (Timeout-- == 0)
        {
            return StartConditionError;
        }
    }

     //Send slave address
     //Reset the address bit0 for write
    SlaveAddress &= 0xFFFE;
     //Send the slave address
    I2C1->DR = SlaveAddress;
    Timeout = I2C_TIMEOUT;
     //Wait until ADDR is set: EV6
    while ((I2C1->SR1 &0x0002) != 0x0002)
    {
        if (Timeout-- == 0)
        {
            I2C1->CR1 |= CR1_STOP_Set;
             //Make sure that the STOP bit is cleared by Hardware
            while ((I2C1->CR1&0x200) == 0x200);
            return AddressAckError;
        }
    }

     //Clear ADDR flag by reading SR2 register
    temp = I2C1->SR2;

    //Write the first data in DR register (EV8_1)
    I2C1->DR = *pBuffer;
     //Increment
    pBuffer++;
     //Decrement the number of bytes to be written
    NumByteToWrite--;
     //While there is data to be written
    while (NumByteToWrite--)
    {
         //Poll on BTF to receive data because in polling mode we can not guarantee the
          //EV8 software sequence is managed before the current byte transfer completes
        while ((I2C1->SR1 & 0x00004) != 0x000004);
         //Send the current byte
        I2C1->DR = *pBuffer;
         //Point to the next byte to be written
        pBuffer++;
    }
     //EV8_2: Wait until BTF is set before programming the STOP
    Timeout = I2C_TIMEOUT;
    while ((I2C1->SR1 & 0x00004) != 0x000004)
    {
		if (Timeout-- == 0)
			return BussyTimeoutError;
	}
     //Send STOP condition
    I2C1->CR1 |= CR1_STOP_Set;
     //Make sure that the STOP bit is cleared by Hardware
    Timeout = I2C_TIMEOUT;
    while ((I2C1->CR1&0x200) == 0x200)
    {
		if (Timeout-- == 0)
			return BussyTimeoutError;
	}

     //While the bus is busy
    Timeout = I2C_TIMEOUT;
    while ((I2C1->SR2 & 0x00002) != 0x000002)
    {
        if (Timeout-- == 0)
            return BussyTimeoutError;
    }
    return Success;
}


