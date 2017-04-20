#ifndef __EEPROM_H
#define __EEPROM_H

struct EEPROMSettings {
    volatile uint32_t cookie;
    volatile uint8_t slide_count;
};

#define EEPROM_Settings ((struct EEPROMSettings *) FLASH_EEPROM_BASE)

#endif
