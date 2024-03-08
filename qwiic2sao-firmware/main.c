#include "ch32v003fun.h"
#include "i2c_slave.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "color_utilities.h"

// Firmware version
#define FW_VERSION 1

// I2C peripheral configuration
#define I2C_ADDR 0x43

// Address selection jumper pins
#define PIN_AJ0 PA1
#define PIN_AJ1 PD7
#define PIN_AJ2 PD0

// LED pin
#define PIN_LED PA2

// I2C interface pins
#define PIN_SDA PC1
#define PIN_SCL PC2

// SAO GPIO pins
#define PIN_IO1 PC6
#define PIN_IO2 PC5

// GPIO pins on row of testpads
#define PIN_IOA PD2
#define PIN_IOB PD3
#define PIN_IOC PD4
#define PIN_IOD PD5
#define PIN_IOE PD6

// Other GPIO pins
#define PIN_EXT PC7
#define PIN_TP1 PC3
#define PIN_TP6 PC0

// I2C registers
#define I2C_REG_FW_VERSION_0      0  // LSB
#define I2C_REG_FW_VERSION_1      1  // MSB
#define I2C_REG_LED               2
#define I2C_REG_SAO_INPUTS        3
#define I2C_REG_SAO_OUTPUTS       4
#define I2C_REG_SAO_MODE          5
#define I2C_REG_GPIO_INPUTS       6
#define I2C_REG_GPIO_OUTPUTS      7
#define I2C_REG_GPIO_MODE         8
#define I2C_REG_ADDR_LED_LENGTH   9
#define I2C_REG_ADDR_LED_DATA     10

// Variables
volatile uint8_t i2c_registers[255] = {0};

const uint32_t input_poll_interval = 20 * DELAY_MS_TIME;
uint32_t input_poll_previous = 0;

bool addr_led_enabled = false;
uint8_t addr_led_length = 0;

// Functions: hardware control

uint8_t get_address() {
    uint8_t address = I2C_ADDR;
    if (!funDigitalRead(PIN_AJ0)) {
        address += 0x01;
    }
    if (!funDigitalRead(PIN_AJ1)) {
        address += 0x02;
    }
    if (!funDigitalRead(PIN_AJ2)) {
        address += 0x04;
    }
    return address;
}

bool get_standalone_mode() {
    return funDigitalRead(PIN_EXT);
}

// Addressable LEDs
void write_addressable_leds(uint8_t* data, uint8_t length) __attribute__((optimize("O0")));
void write_addressable_leds(uint8_t* data, uint8_t length) {
    I2C1->CTLR2 &= ~(I2C_CTLR2_ITEVTEN); // Disable I2C event interrupt
    for (uint8_t pos_byte = 0; pos_byte < length; pos_byte++) {
        for (int i = 7; i >= 0; i--) {
            if ((data[pos_byte] >> i) & 1) {
                // Send 1
                __asm__("nop");__asm__("nop");
                GPIOC->BSHR |= 1 << (6);
                __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
                __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
                __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
                __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
                __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
                __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
                __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
                __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
                __asm__("nop");__asm__("nop");
                __asm__("nop");__asm__("nop");
                GPIOC->BSHR |= 1 << (6 + 16);
            } else {
                // Send 0
                GPIOC->BSHR |= 1 << (6);
                __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
                __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
                __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
                __asm__("nop");__asm__("nop");
                GPIOC->BSHR |= 1 << (6 + 16);
                __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
                __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
                __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
                __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
                __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            }
        }
    }
    I2C1->CTLR2 |= I2C_CTLR2_ITEVTEN; // Enable I2C event interrupt
}

// Functions: I2C

void onRead(uint8_t reg) {
    // Empty
}

void onWrite(uint8_t reg, uint8_t length) {
    // LED register
    funDigitalWrite(PIN_LED, i2c_registers[I2C_REG_LED] & 1);

    // SAO outputs register
    funDigitalWrite(PIN_IO1, i2c_registers[I2C_REG_SAO_OUTPUTS] & (1 << 0));
    funDigitalWrite(PIN_IO2, i2c_registers[I2C_REG_SAO_OUTPUTS] & (1 << 1));

    // SAO mode register
    addr_led_enabled = !!(i2c_registers[I2C_REG_SAO_MODE] & (1 << 2));
    if (addr_led_enabled) {
        funPinMode(PIN_IO1, GPIO_CFGLR_OUT_10Mhz_PP);
    } else {
        funPinMode(PIN_IO1, i2c_registers[I2C_REG_SAO_MODE] & (1 << 0) ? GPIO_CFGLR_OUT_10Mhz_PP : GPIO_CFGLR_IN_PUPD);
    }
    funPinMode(PIN_IO2, i2c_registers[I2C_REG_SAO_MODE] & (1 << 1) ? GPIO_CFGLR_OUT_10Mhz_PP : GPIO_CFGLR_IN_PUPD);

    // Other GPIO outputs register
    funDigitalWrite(PIN_IOA, i2c_registers[I2C_REG_GPIO_OUTPUTS] & (1 << 0));
    funDigitalWrite(PIN_IOB, i2c_registers[I2C_REG_GPIO_OUTPUTS] & (1 << 1));
    funDigitalWrite(PIN_IOC, i2c_registers[I2C_REG_GPIO_OUTPUTS] & (1 << 2));
    funDigitalWrite(PIN_IOD, i2c_registers[I2C_REG_GPIO_OUTPUTS] & (1 << 3));
    funDigitalWrite(PIN_IOE, i2c_registers[I2C_REG_GPIO_OUTPUTS] & (1 << 4));
    funDigitalWrite(PIN_TP1, i2c_registers[I2C_REG_GPIO_OUTPUTS] & (1 << 5));
    funDigitalWrite(PIN_TP6, i2c_registers[I2C_REG_GPIO_OUTPUTS] & (1 << 6));

    // Addressable LED strip length register
    addr_led_length = i2c_registers[I2C_REG_ADDR_LED_LENGTH];
    if (addr_led_length > sizeof(i2c_registers) - I2C_REG_ADDR_LED_DATA) {
        addr_led_length = sizeof(i2c_registers) - I2C_REG_ADDR_LED_DATA;
    }
}

void standalone_main() {
    // Device is not connected to an I2C bus but instead used as a standalone
    // addressable LED driver. This mode outputs a rainbow pattern on the IO1
    // SAO pin.

    bool led = false;

    uint8_t hue = 0;

    funPinMode(PIN_IO1, GPIO_CFGLR_OUT_10Mhz_PP);

    while (1) {
        uint32_t now = SysTick->CNT;

        if (now - input_poll_previous >= input_poll_interval) {
            input_poll_previous = now;
            funDigitalWrite(PIN_LED, led);
            led = !led;

            for (uint8_t led = 0; led < 85; led++) {
                uint32_t color = EHSVtoHEX(hue + (led*3), 240, 128);
                i2c_registers[(led * 3) + 0] = (color >>  8) & 0xFF;
                i2c_registers[(led * 3) + 1] = (color >> 16) & 0xFF;
                i2c_registers[(led * 3) + 2] = (color >>  0) & 0xFF;
            }
            hue++;

            write_addressable_leds((uint8_t*) i2c_registers, 85*3);
        }
    }
}

uint8_t read_sao_inputs() {
    uint8_t value = 0;
    value |= funDigitalRead(PIN_IO1) << 0;
    value |= funDigitalRead(PIN_IO2) << 1;
    return value;
}

uint8_t read_other_inputs() {
    uint8_t value = 0;
    value |= funDigitalRead(PIN_IOA) << 0;
    value |= funDigitalRead(PIN_IOB) << 1;
    value |= funDigitalRead(PIN_IOC) << 2;
    value |= funDigitalRead(PIN_IOD) << 3;
    value |= funDigitalRead(PIN_IOE) << 4;
    value |= funDigitalRead(PIN_TP1) << 5;
    value |= funDigitalRead(PIN_TP6) << 6;
    return value;
}

int main() {
    SystemInit();
    funGpioInitAll();

    // Initialize GPIO for address select jumpers
    funPinMode(PIN_AJ0, GPIO_CFGLR_IN_PUPD);
    funDigitalWrite(PIN_AJ0, true); // Pull-up
    funPinMode(PIN_AJ1, GPIO_CFGLR_IN_PUPD);
    funDigitalWrite(PIN_AJ1, true); // Pull-up
    funPinMode(PIN_AJ2, GPIO_CFGLR_IN_PUPD);
    funDigitalWrite(PIN_AJ2, true); // Pull-up

    // Initialize GPIO for LED
    funPinMode(PIN_LED, GPIO_CFGLR_OUT_10Mhz_PP);

    // Initialize GPIO for I2C
    funPinMode(PIN_SDA, GPIO_CFGLR_OUT_10Mhz_AF_OD);
    funPinMode(PIN_SCL, GPIO_CFGLR_OUT_10Mhz_AF_OD);

    // Initialize SAO GPIO
    funPinMode(PIN_IO1, GPIO_CFGLR_IN_PUPD);
    funDigitalWrite(PIN_IO1, false);
    funPinMode(PIN_IO2, GPIO_CFGLR_IN_PUPD);
    funDigitalWrite(PIN_IO2, false);

    // Initialize other GPIO
    funPinMode(PIN_IOA, GPIO_CFGLR_IN_PUPD);
    funDigitalWrite(PIN_IOA, false);
    funPinMode(PIN_IOB, GPIO_CFGLR_IN_PUPD);
    funDigitalWrite(PIN_IOB, false);
    funPinMode(PIN_IOC, GPIO_CFGLR_IN_PUPD);
    funDigitalWrite(PIN_IOC, false);
    funPinMode(PIN_IOD, GPIO_CFGLR_IN_PUPD);
    funDigitalWrite(PIN_IOD, false);
    funPinMode(PIN_IOE, GPIO_CFGLR_IN_PUPD);
    funDigitalWrite(PIN_IOE, false);
    funPinMode(PIN_EXT, GPIO_CFGLR_IN_PUPD);
    funDigitalWrite(PIN_EXT, false);
    funPinMode(PIN_TP1, GPIO_CFGLR_IN_PUPD);
    funDigitalWrite(PIN_TP1, false);
    funPinMode(PIN_TP6, GPIO_CFGLR_IN_PUPD);
    funDigitalWrite(PIN_TP6, false);

    if (get_standalone_mode()) {
        funDigitalWrite(PIN_LED, true);
        standalone_main();
    }

    // Initialize I2C in peripheral mode
    SetupI2CSlave(get_address(), i2c_registers, sizeof(i2c_registers), onWrite, onRead, false);

    while (1) {
        // Update I2C registers
        i2c_registers[I2C_REG_FW_VERSION_0] = (FW_VERSION     ) & 0xFF;
        i2c_registers[I2C_REG_FW_VERSION_1] = (FW_VERSION >> 8) & 0xFF;

        uint32_t now = SysTick->CNT;
        if (now - input_poll_previous >= input_poll_interval) {
            input_poll_previous = now;
            i2c_registers[I2C_REG_SAO_INPUTS] = read_sao_inputs();
            i2c_registers[I2C_REG_GPIO_INPUTS] = read_other_inputs();

            if (addr_led_enabled) {
                write_addressable_leds((uint8_t*) &i2c_registers[I2C_REG_ADDR_LED_DATA], addr_led_length);
            }
        }
    }
}
