/*
 * LTC2986.c
 *
 *  Created on: Jul 3, 2024
 *      Author: vsaona
 */

#include "LTC2986.h"

#define LTC2986_SPI_TIMEOUT 50U

//************************************
// -- INSTRUCTIONS --
//************************************
#define WRITE_TO_RAM            (uint8_t) 0x02
#define READ_FROM_RAM           (uint8_t) 0x03
//**********************************************************************************************************
// -- ADDRESSES --
//**********************************************************************************************************
#define LTC2986_COMMAND_STATUS_REGISTER          (uint16_t) 0x0000
#define LTC2986_STATUS_REGISTER                  (uint16_t) 0x0000
#define LTC2986_COMMAND_REGISTER                 (uint16_t) 0x0000
#define LTC2986_CH_ADDRESS_BASE                  (uint16_t) 0x0200
#define LTC2986_VOUT_CH_BASE                     (uint16_t) 0x0060
#define LTC2986_READ_CH_BASE                     (uint16_t) 0x0010
#define LTC2986_CONVERSION_RESULT_REGISTER    (uint16_t) 0x0010
#define LTC2986_GLOBAL_CONFIGURATION_REGISTER    (uint16_t) 0x00F0
#define LTC2986_DELAY_REGISTER                   (uint16_t) 0x00FF


void read_RAM(LTC2986_t *LTM, uint16_t address, int length, uint8_t *buffer) {
	uint8_t read_instruction = READ_FROM_RAM;
	uint8_t* address_8bit = (void*) &address;
	HAL_GPIO_WritePin(LTM->cs_pin.gpio_port, LTM->cs_pin.gpio_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(LTM->spi_handle, &read_instruction, 1, LTC2986_SPI_TIMEOUT); // Read instruction
	HAL_SPI_Transmit(LTM->spi_handle, address_8bit, 2, LTC2986_SPI_TIMEOUT); // Address
	HAL_SPI_Receive(LTM->spi_handle, buffer, length, LTC2986_SPI_TIMEOUT);
	HAL_GPIO_WritePin(LTM->cs_pin.gpio_port, LTM->cs_pin.gpio_pin, GPIO_PIN_SET);
	return;
}

void write_RAM(LTC2986_t *LTM, uint16_t address, int length, uint8_t *buffer) {
	uint8_t write_instruction = WRITE_TO_RAM;
	uint8_t* address_8bit = (void*) &address;
	HAL_GPIO_WritePin(LTM->cs_pin.gpio_port, LTM->cs_pin.gpio_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(LTM->spi_handle, &write_instruction, 1, LTC2986_SPI_TIMEOUT); // Read instruction
	HAL_SPI_Transmit(LTM->spi_handle, address_8bit, 2, LTC2986_SPI_TIMEOUT); // Address
	HAL_SPI_Transmit(LTM->spi_handle, buffer, length, LTC2986_SPI_TIMEOUT);
	HAL_GPIO_WritePin(LTM->cs_pin.gpio_port, LTM->cs_pin.gpio_pin, GPIO_PIN_SET);
	return;
}

void LTC2986_global_configure(LTC2986_t *LTM) {
	write_RAM(LTM, LTC2986_GLOBAL_CONFIGURATION_REGISTER, 1, 0x00); // Filter 55 Hz, Celsius, no excitation mode
}

// Esta configuracion siento que tiene demasiados parametros asi que por ahora la voy a dejar hardcodeada, pero si queremos hacer una biblioteca portable y general, se deberian dejar como paramteros.
void LTC2986_configure_rtd(LTC2986_t *LTM, LTC2986_sensor_t type, uint8_t channel_number, uint8_t sense_channel) {
	uint32_t configuration;
	configuration = type << 27;
	configuration |= (sense_channel) << 22;
	configuration |= 0x0 << 20; // 2-wire
	configuration |= 0x0 << 18; // no rotation, no sharing
	configuration |= 0x7 << 14; // 500 uA
	configuration |= 0x1 << 12; // American curve for the RTD.
	write_RAM(LTM, LTC2986_CH_ADDRESS_BASE + (4 * (channel_number - 1)), 4, (void*) &configuration);
	return;
}

void LTC2986_configure_thermocouple(LTC2986_t *LTM, LTC2986_sensor_t type, uint8_t channel_number, uint8_t cold_junction_channel) {
	uint32_t configuration;
	configuration = type << 27;
	configuration |= cold_junction_channel << 22;
	configuration |= 0b1 << 21; // Single-ended
	configuration |= 0b0 << 20; // TC_OPEN_CKT_DETECT__NO
	configuration |= 0x0 << 18; // TC_OPEN_CKT_DETECT_CURRENT__10UA
	write_RAM(LTM, LTC2986_CH_ADDRESS_BASE + (4 * (channel_number - 1)), 4, (void*) &configuration);
	return;
}

void LTC2986_configure_sense_resistor(LTC2986_t *LTM, uint8_t channel_number, float resistance) {
	uint32_t configuration = 0;
	if(resistance > 131.07) {
		// This is a programming error.
		resistance = 131;
	}
	configuration = (uint32_t) ((int) resistance* 1024);
	write_RAM(LTM, LTC2986_CH_ADDRESS_BASE + (4 * (channel_number - 1)), 4, (void*) &configuration);
	return;
}

uint8_t LTC2986_is_ready(LTC2986_t *LTM) {
	uint8_t status;
	read_RAM(LTM, LTC2986_STATUS_REGISTER, 1, &status);
	uint8_t started_convertion = status & 0b10000000;
	uint8_t done = status & 0b01000000;
	return(done && !started_convertion);
}

uint8_t LTC2986_read_status(LTC2986_t *LTM) {
	uint8_t status;
	read_RAM(LTM, LTC2986_STATUS_REGISTER, 1, &status);
	return(status);
}

float LTC2986_measure_channel(LTC2986_t *LTM, uint8_t channel_number) {
	write_RAM(LTM, LTC2986_COMMAND_REGISTER, 1, &channel_number); // We command to initiate the conversion
	while(!LTC2986_is_ready(LTM));
	uint32_t raw_reading;
	read_RAM(LTM, LTC2986_CONVERSION_RESULT_REGISTER, 4, (void*) &raw_reading);
	uint8_t fault = raw_reading >> 24;
	if(fault != LTC2986_VALID) {
		return(0xFFFFFF00 & fault); // Return a NaN with the fault encoded
	}
	uint32_t raw_result = raw_reading & 0xFFFFFF;
	float result = ((float) raw_result) / 1024; // Assuming it is a temperature channel (not voltage, for example)
	return(result);
}
