#define EEPROM_START_ADDRESS 0x08080000
#define EEPROM_END_ADDRESS 0x08080FFF
#define EEPROM_LOG_START_ADDRESS EEPROM_START_ADDRESS+0x400 //(reserve 1K for settings)

void eeprom_init();
void eeprom_clear();
void eeprom_log_next(uint16_t time, uint16_t temperature);

void write_eeprom_8(uint32_t address,uint8_t data);
void write_eeprom_16(uint32_t address,uint16_t data);
void write_eeprom_32(uint32_t address,uint32_t data);
uint8_t read_eeprom_8(uint32_t address);
uint16_t read_eeprom_16(uint32_t address);
uint32_t read_eeprom_32(uint32_t address);
