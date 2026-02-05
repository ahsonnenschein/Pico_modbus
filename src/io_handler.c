#include "io_handler.h"

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

// Current state of digital outputs
static uint8_t g_digital_outputs = 0;

void io_handler_init(void)
{
    // Initialize ADC
    adc_init();

    // Configure ADC pins (GP26-GP29)
    adc_gpio_init(26); // ADC0
    adc_gpio_init(27); // ADC1
    adc_gpio_init(28); // ADC2
    adc_gpio_init(29); // ADC3

    // Initialize digital inputs (GP0-GP7) with pull-ups
    for (uint8_t i = 0; i < MODBUS_NUM_DIGITAL_INPUTS; i++) {
        uint8_t pin = DIGITAL_INPUT_START_PIN + i;
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
    }

    // Initialize digital outputs (GP8-GP15)
    for (uint8_t i = 0; i < MODBUS_NUM_DIGITAL_OUTPUTS; i++) {
        uint8_t pin = DIGITAL_OUTPUT_START_PIN + i;
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 0);
    }

    g_digital_outputs = 0;
}

void io_read_adc(uint16_t* adc_values)
{
    for (uint8_t i = 0; i < MODBUS_NUM_ADC_CHANNELS; i++) {
        adc_values[i] = io_read_adc_channel(i);
    }
}

uint16_t io_read_adc_channel(uint8_t channel)
{
    if (channel >= MODBUS_NUM_ADC_CHANNELS) {
        return 0;
    }
    adc_select_input(channel);
    return adc_read();
}

uint8_t io_read_digital_inputs(void)
{
    uint8_t value = 0;
    for (uint8_t i = 0; i < MODBUS_NUM_DIGITAL_INPUTS; i++) {
        if (gpio_get(DIGITAL_INPUT_START_PIN + i)) {
            value |= (1 << i);
        }
    }
    return value;
}

bool io_read_digital_input(uint8_t pin)
{
    if (pin >= MODBUS_NUM_DIGITAL_INPUTS) {
        return false;
    }
    return gpio_get(DIGITAL_INPUT_START_PIN + pin);
}

uint8_t io_read_digital_outputs(void)
{
    return g_digital_outputs;
}

void io_write_digital_outputs(uint8_t value)
{
    g_digital_outputs = value;
    for (uint8_t i = 0; i < MODBUS_NUM_DIGITAL_OUTPUTS; i++) {
        gpio_put(DIGITAL_OUTPUT_START_PIN + i, (value >> i) & 0x01);
    }
}

void io_write_digital_output(uint8_t pin, bool value)
{
    if (pin >= MODBUS_NUM_DIGITAL_OUTPUTS) {
        return;
    }

    if (value) {
        g_digital_outputs |= (1 << pin);
    } else {
        g_digital_outputs &= ~(1 << pin);
    }

    gpio_put(DIGITAL_OUTPUT_START_PIN + pin, value);
}
