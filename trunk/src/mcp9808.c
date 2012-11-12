#include "mcu.h"
#include "mcp9808.h"
#include "i2c.h"
#include "usart.h"

uint16_t mcp9808_reg_read(uint8_t addr, uint8_t reg) {
        uint8_t tmp2[2];
        Status stat;
        stat = I2C_Master_BufferRead(tmp2,2,addr,reg);
        if(stat != Success) PutsUART1("NS\n");
        if(stat == StartConditionError) PutsUART1("SCE\n");
        if(stat == GeneralError) PutsUART1("GE\n");
        if(stat == AddressAckError) PutsUART1("AAE\n");
        if(stat == BussyTimeoutError) PutsUART1("BTE\n");
        if(stat == RestartConditionError) PutsUART1("RCE\n");
        if(stat != Success && stat != BussyTimeoutError) return 0xffff;
        return tmp2[0] << 8 | tmp2[1];
}

double mcp9808_gettemp(uint8_t addr) {
        uint16_t temp;
        uint64_t tmp = 0xffffffffffffffff;
        double v;

        v = data2double(tmp);

        temp = mcp9808_reg_read(addr, 0x05);
        if(temp == 0xffff) return v;
        temp &= 0x1fff;         //clear flag bits

        if ((temp & 0x1000) == 0x1000) {
                //temperature below 0C
                temp &= 0x0fff; //clear sign
                v = 256.0 - temp / 16.0;
        } else {
                //temperature above 0C
                v = temp / 16.0;
        }

        return v;
}
