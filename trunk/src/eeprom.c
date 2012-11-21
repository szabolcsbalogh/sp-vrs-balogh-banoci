#include "stm32l1xx_flash.h"
#include "eeprom.h"

uint32_t eeprom_log_at;

void eeprom_init() {
	for(eeprom_log_at = EEPROM_LOG_START_ADDRESS; eeprom_log_at < EEPROM_END_ADDRESS && read_eeprom_32(eeprom_log_at) != (uint32_t)0xffffffff; eeprom_log_at += 4);
}

void eeprom_clear() {
	for(eeprom_log_at = EEPROM_LOG_START_ADDRESS; eeprom_log_at < EEPROM_END_ADDRESS; eeprom_log_at += 8)
		write_eeprom_32(eeprom_log_at, (uint32_t)0xffffffff);
	eeprom_log_at = EEPROM_LOG_START_ADDRESS;
}

void eeprom_log_next(uint16_t time, uint16_t temperature) {
	//erase next as bookmark
	if(eeprom_log_at + 4 < EEPROM_END_ADDRESS)
		write_eeprom_32(eeprom_log_at + 4, (uint32_t)0xffffffff);
	else
		write_eeprom_32(EEPROM_LOG_START_ADDRESS, (uint32_t)0xffffffff);
	//write data
	write_eeprom_16(eeprom_log_at, time);
	write_eeprom_16(eeprom_log_at + 2, temperature);
	//set counter to next address
	if(eeprom_log_at + 4 < EEPROM_END_ADDRESS)
			eeprom_log_at += 4;
		else
			eeprom_log_at = EEPROM_LOG_START_ADDRESS;
}

void write_eeprom_8(uint32_t address,uint8_t data){
	int fs;
	if(IS_FLASH_DATA_ADDRESS(address)){
		DATA_EEPROM_Unlock();
		while(FLASH_GetStatus()!=FLASH_COMPLETE);
		fs = DATA_EEPROM_ProgramByte (address, data);
		if(fs==FLASH_COMPLETE)
		DATA_EEPROM_Lock();
	}
}

void write_eeprom_16(uint32_t address,uint16_t data){
	int fs;
	if(IS_FLASH_DATA_ADDRESS(address)){
		DATA_EEPROM_Unlock();
		while(FLASH_GetStatus()!=FLASH_COMPLETE);
		fs = DATA_EEPROM_ProgramHalfWord (address, data);
		if(fs==FLASH_COMPLETE)
		DATA_EEPROM_Lock();
	}
}

void write_eeprom_32(uint32_t address,uint32_t data){
	int fs;
	if(IS_FLASH_DATA_ADDRESS(address)){
		DATA_EEPROM_Unlock();
		while(FLASH_GetStatus()!=FLASH_COMPLETE);
		fs = DATA_EEPROM_ProgramWord (address, data);
		if(fs==FLASH_COMPLETE)
		DATA_EEPROM_Lock();
	}
}

uint32_t read_eeprom_32(uint32_t address){
	uint32_t tmp=0;
	if(IS_FLASH_DATA_ADDRESS(address)){
		DATA_EEPROM_Unlock();
		while(FLASH_GetStatus()==FLASH_BUSY);
		tmp = *(__IO uint32_t*)address;
		DATA_EEPROM_Lock();
	}
	return tmp;
}

uint16_t read_eeprom_16(uint32_t address){
	uint16_t tmp=0;
	if(IS_FLASH_DATA_ADDRESS(address)){
		DATA_EEPROM_Unlock();
		while(FLASH_GetStatus()==FLASH_BUSY);
		tmp = *(__IO uint16_t*)address;
		DATA_EEPROM_Lock();
	}
	return tmp;
}

uint8_t read_eeprom_8(uint32_t address){
	uint8_t tmp=0;
	if(IS_FLASH_DATA_ADDRESS(address)){
		DATA_EEPROM_Unlock();
		while(FLASH_GetStatus()==FLASH_BUSY);
		tmp = *(__IO uint8_t*)address;
		DATA_EEPROM_Lock();
	}
	return tmp;
}
