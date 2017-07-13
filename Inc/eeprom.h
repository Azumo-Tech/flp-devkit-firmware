#ifndef __EEPROM_H
#define __EEPROM_H

struct Slide {
    uint8_t img;
    uint8_t delay;
    uint16_t led_current;
    uint32_t reserved;
};

enum EEPROMSettingFlags {
    SETTING_NO_SLIDESHOW = 1<<0,
};

struct EEPROMSettings {
    volatile uint32_t version;
    volatile uint16_t default_led_current;
    volatile uint8_t default_delay;
    volatile uint8_t slide_count;
    volatile uint8_t lcd_model;
    volatile uint8_t flags;
    volatile uint8_t pad[2];
    volatile struct Slide slides[64];
};

#define EEPROM_Settings ((struct EEPROMSettings *) FLASH_EEPROM_BASE)

#endif
