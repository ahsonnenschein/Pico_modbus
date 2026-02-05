#ifndef IO_HANDLER_H
#define IO_HANDLER_H

#include <stdint.h>
#include <stdbool.h>

// I/O Configuration
#define MODBUS_NUM_DIGITAL_INPUTS   8   // GP0-GP7
#define MODBUS_NUM_DIGITAL_OUTPUTS  8   // GP8-GP15
#define MODBUS_NUM_ADC_CHANNELS     4   // ADC0-ADC3 (GP26-GP29)

// GPIO pin ranges
#define DIGITAL_INPUT_START_PIN   0   // GP0
#define DIGITAL_OUTPUT_START_PIN  8   // GP8

// ADC channels
#define ADC_CHANNEL_0  0  // GP26
#define ADC_CHANNEL_1  1  // GP27
#define ADC_CHANNEL_2  2  // GP28
#define ADC_CHANNEL_3  3  // GP29

// Initialize all I/O (ADC and GPIO)
void io_handler_init(void);

// Read all ADC channels, store results in the provided array
// Returns 12-bit values (0-4095)
void io_read_adc(uint16_t* adc_values);

// Read single ADC channel
uint16_t io_read_adc_channel(uint8_t channel);

// Read digital inputs as a byte (bit 0 = GP0, bit 7 = GP7)
uint8_t io_read_digital_inputs(void);

// Read single digital input
bool io_read_digital_input(uint8_t pin);

// Read digital outputs current state as a byte (bit 0 = GP8, bit 7 = GP15)
uint8_t io_read_digital_outputs(void);

// Write digital outputs from a byte (bit 0 = GP8, bit 7 = GP15)
void io_write_digital_outputs(uint8_t value);

// Write single digital output
void io_write_digital_output(uint8_t pin, bool value);

#endif // IO_HANDLER_H
