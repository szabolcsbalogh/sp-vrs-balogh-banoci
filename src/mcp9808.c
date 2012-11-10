#include "mcu.h"
#include "mcp9808.h"
#include "i2c.h"

uint16_t mcp9808_reg_read(uint8_t addr, uint8_t reg) {
        uint8_t tmp1[2], tmp2[2];
        Status stat;
        //unsigned char b[3];
        //tmp1[0] = reg;
        //stat = I2C_Master_BufferRead(tmp2,2,addr,reg);
        PutsUART2("1\n");
        while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
    	I2C_GenerateSTART(I2C2, ENABLE);
    	PutsUART2("2\n");
    	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
    	I2C_Send7bitAddress(I2C2, 0x18, I2C_Direction_Transmitter);
    	PutsUART2("3\n");
    	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    	I2C_Cmd(I2C2, ENABLE);
    	I2C_SendData(I2C2, 0x05);
    	PutsUART2("4\n");
    	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    	I2C_GenerateSTART(I2C2, ENABLE);
    	PutsUART2("5\n");
    	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
    	I2C_Send7bitAddress(I2C2, 0x18, I2C_Direction_Receiver);
    	PutsUART2("6\n");
    	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    	int NumByteToReadN = 2;
    	PutsUART2("7\n");
    	while(NumByteToReadN)
    	{
    		if(NumByteToReadN == 1)
    		{
    			  I2C_AcknowledgeConfig(I2C2, DISABLE);
    			  I2C_GenerateSTOP(I2C2, ENABLE);
    		}
    		if(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))
    		{
    			tmp2[NumByteToReadN-2] = I2C_ReceiveData(I2C2);
    			NumByteToReadN--;
    		}
    	}
    	PutsUART2("8\n");
    	I2C_AcknowledgeConfig(I2C2, ENABLE);
        return tmp2[0] << 8 | tmp2[1];
}

int mcp9808_gettemp(uint8_t addr) {
        uint16_t temp;
        int v;

        temp = mcp9808_reg_read(addr, 0x05);

        temp &= 0x1fff;         //clear flag bits

        if ((temp & 0x1000) == 0x1000) {
                //temperature below 0C
                temp &= 0x0fff; //clear sign
                v = 25600 - temp * 100 / 16;
        } else {
                //temperature above 0C
                v = temp * 100 / 16;
        }

        return v;
}
