/*
 * I2C_BME280.h
 *
 *  Created on: 8 дек. 2024 г.
 *      Author: root
 */

#ifndef MAIN_HEADER_H_
#define MAIN_HEADER_H_

#include "freertos/FreeRTOS.h"

//================================================================================================
// Прототипы
// BME280
// Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
double BME280_compensate_T_double(int32_t adc_T);
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
double BME280_compensate_T_int32(int32_t adc_T);
// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
double BME280_compensate_P_double(int32_t adc_P);
// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
double BME280_compensate_P_int64(int32_t adc_P);
// Returns humidity in %rH as as double. Output value of “46.332” represents 46.332 %rH
double BME280_compensate_H_double(int32_t adc_H);
// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
double BME280_compensate_H_int32(int32_t adc_H);
uint8_t BME280_ReadStatus(void); // Функция чтения статуса
void BME280_init(const uint8_t mode, const uint8_t temperature, const uint8_t pressure, const uint8_t humidity, const uint8_t standby, const uint8_t filter);
void BME280_read_calib_Data(void); // Функция чтения калибровочных данных
double BME280_get_temperature(void);
double BME280_get_pressure(void);
double BME280_get_humidity(void);
void BME280_forsed_mode(void);

// I2C
void I2C_init(uint8_t rtc_addr);
void I2C_write(const uint8_t *p, const uint8_t write_size);
void I2C_read(uint8_t *p, uint8_t read_size);
void I2C_write_read(const uint8_t *write, const uint8_t size_write, uint8_t *read, const uint8_t size_read);

//================================================================================================



#endif /* MAIN_HEADER_H_ */
