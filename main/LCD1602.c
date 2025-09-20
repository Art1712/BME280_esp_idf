/*
 * LCD1602.c
 *
 *  Created on: 14 дек. 2024 г.
 *      Author: root
 */

#include "xtensa/hal.h"
#include <stdint.h>
#include <string.h>
#include "HEADER.h"

const uint8_t E_0 = 0b00000000;
const uint8_t E_1 = 0b00000100;
const uint8_t RS_0 = 0b00000000; // Записываем команду
const uint8_t RS_1 = 0b00000001; // Записываем данные
const uint8_t RW_0 = 0b00000000; // Записываем
const uint8_t RW_1 = 0b00000010; // Считываем
const uint8_t backlight_0 = 0b00000000; // 
const uint8_t backlight_1 = 0b00001000; // 
//const uint8_t backlight = 0; // Считываем
uint8_t backlight = 0;

// LCD PROTOTYPES
void LCD_init(void);
void LCD_write_data(uint8_t data);
void LCD_write_half_command(uint8_t command);
void LCD_write_command(uint8_t command);
void LCD_init(void);
void line(uint8_t line, uint8_t position);
void stroka(char *str);
void backlight_on(void);
void backlight_off(void);
uint8_t LCD_read_data(uint8_t line, uint8_t position);

void LCD_write_data(uint8_t data)
{
	uint8_t form, add;
	if (backlight == 0) add = RS_1 | RW_0 | E_1 | backlight_0; 
	else add = RS_1 | RW_0 | E_1 | backlight_1;
	form = (data & 0b11110000) | add;
	//ESP_ERROR_CHECK(i2c_master_transmit(dev1_handle, &form, 1, -1));
	I2C_write(&form, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	form &= ~(1 << 2); // Выставляем 2 бит в "0"
	//ESP_ERROR_CHECK(i2c_master_transmit(dev1_handle, &form, 1, -1));
	I2C_write(&form, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	
	form = (data << 4) | add;
	//ESP_ERROR_CHECK(i2c_master_transmit(dev1_handle, &form, 1, -1));
	I2C_write(&form, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	form &= ~(1 << 2);
	//ESP_ERROR_CHECK(i2c_master_transmit(dev1_handle, &form, 1, -1));
	I2C_write(&form, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
}
/*
uint8_t LCD_read_data(uint8_t data)
{
	uint8_t form, add = 0, res;
	data |= 0b10000000;
	if (backlight == 0) add = RS_1 | RW_1 | E_1 | backlight_0; 
	else add = RS_1 | RW_1 | E_1 | backlight_1;
	
	form = (data & 0b11110000) | add;
		
	ESP_ERROR_CHECK(i2c_master_transmit(dev1_handle, &form, 1, -1));
	vTaskDelay(1 / portTICK_PERIOD_MS);
	form &= ~((1 << 2)); // Выставляем 2 бит в "0"
	ESP_ERROR_CHECK(i2c_master_transmit(dev1_handle, &form, 1, -1));
	vTaskDelay(1 / portTICK_PERIOD_MS);
	
	//add &= ~();
	form = (data << 4) | add;
	ESP_ERROR_CHECK(i2c_master_transmit(dev1_handle, &form, 1, -1));
	vTaskDelay(1 / portTICK_PERIOD_MS);
	form &= ~((1 << 2));
	ESP_ERROR_CHECK(i2c_master_transmit(dev1_handle, &form, 1, -1));
	vTaskDelay(1 / portTICK_PERIOD_MS);
	
	i2c_master_receive(dev1_handle, &res, 1, -1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	//ESP_ERROR_CHECK(i2c_master_transmit_receive(dev1_handle, &form, 1, &res, 1, -1));
	return res;
}
*/
uint8_t LCD_read_data(uint8_t line, uint8_t position)
{
	uint8_t koordinata;
	if (line == 0)
	{
		koordinata = position | 0b10000000;
	}
	else if (line == 1)
	{
		koordinata = (position + 0x40) | 0b10000000;
	}
	
	else if (line == 2)
	{
		koordinata = (position + 0x14) | 0b10000000;
	}
	
	else 
	{
		koordinata = (position + 0x54) | 0b10000000;
	}
	
	uint8_t form, add = 0, res;
	if (backlight == 0) add = RS_1 | RW_1 | E_1 | backlight_0; 
	else add = RS_1 | RW_1 | E_1 | backlight_1;
	
	form = (koordinata & 0b11110000) | add;
		
	//ESP_ERROR_CHECK(i2c_master_transmit(dev1_handle, &form, 1, -1));
	I2C_write(&form, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	form &= ~((1 << 2)); // Выставляем 2 бит в "0"
	//ESP_ERROR_CHECK(i2c_master_transmit(dev1_handle, &form, 1, -1));
	I2C_write(&form, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	
	//add &= ~();
	form = (koordinata << 4) | add;
	//ESP_ERROR_CHECK(i2c_master_transmit(dev1_handle, &form, 1, -1));
	I2C_write(&form, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	form &= ~((1 << 2));
	//ESP_ERROR_CHECK(i2c_master_transmit(dev1_handle, &form, 1, -1));
	I2C_write(&form, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	
	//i2c_master_receive(dev1_handle, &res, 1, -1);
	I2C_write(&res, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	
	//i2c_master_receive(dev1_handle, &res, 1, -1);
	I2C_write(&res, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	//ESP_ERROR_CHECK(i2c_master_transmit_receive(dev1_handle, &form, 1, &res, 1, -1));
	I2C_write_read(&form, 1, &res, 1);
	return res;
}

void LCD_write_command(uint8_t command)
{
	uint8_t form, add;
	if (backlight == 0) add = RS_0 | RW_0 | E_1 | backlight_0; 
	else add = RS_0 | RW_0 | E_1 | backlight_1;
	form = (command & 0b11110000) | add;
	//ESP_ERROR_CHECK(i2c_master_transmit(dev1_handle, &form, 1, -1));
	I2C_write(&form, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	form &= ~(1 << 2);
	//ESP_ERROR_CHECK(i2c_master_transmit(dev1_handle, &form, 1, -1));
	I2C_write(&form, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	
	form = (command << 4) | add;
	//ESP_ERROR_CHECK(i2c_master_transmit(dev1_handle, &form, 1, -1));
	I2C_write(&form, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	form &= ~(1 << 2);
	//ESP_ERROR_CHECK(i2c_master_transmit(dev1_handle, &form, 1, -1));
	I2C_write(&form, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
}

void LCD_write_half_command(uint8_t command)
{
	uint8_t form;
			
	form = (command & 0b11110000) | RS_0 | RW_0 | E_1;
	//ESP_ERROR_CHECK(i2c_master_transmit(dev1_handle, &form, 1, -1));
	I2C_write(&form, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	form &= ~(1 << 2);
	//ESP_ERROR_CHECK(i2c_master_transmit(dev1_handle, &form, 1, -1));
	I2C_write(&form, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
}

void LCD_init(void)
{
	vTaskDelay(20 / portTICK_PERIOD_MS);
	LCD_write_half_command(0b00110000);
	vTaskDelay(5 / portTICK_PERIOD_MS);
	LCD_write_half_command(0b00110000);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	LCD_write_half_command(0b00110000);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	LCD_write_half_command(0b00100000);
	vTaskDelay(2 / portTICK_PERIOD_MS);
	// Function set
	LCD_write_command(0b00101000); // 0x28
	vTaskDelay(1 / portTICK_PERIOD_MS);
	// Display off
	LCD_write_command(0b00001000); // 0x08
	vTaskDelay(1 / portTICK_PERIOD_MS);
	// Display clear
	LCD_write_command(0b00000001); // 0x01
	vTaskDelay(2 / portTICK_PERIOD_MS);
	// Entry mode set
	LCD_write_command(0b00000110); // 0x06
	vTaskDelay(1 / portTICK_PERIOD_MS);
	// Включаем подсветку
	LCD_write_command(0b00001000); // 0x06
	vTaskDelay(1 / portTICK_PERIOD_MS);
	/*
	LCD_write_command(0x0C);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	
	LCD_write_command(0b00000010);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	
	LCD_write_command(0b10000000);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	*/
	LCD_write_command(0b00001100);
	backlight_off();

}

void line(uint8_t line, uint8_t position) // Определяем позицию вывода символа в 1 строке
{
	uint8_t koordinata;
	if (line == 0)
	{
		koordinata = position | 0b10000000;
	}
	else if (line == 1)
	{
		koordinata = (position + 0x40) | 0b10000000;
	}
	
	else if (line == 2)
	{
		koordinata = (position + 0x14) | 0b10000000;
	}
	
	else 
	{
		koordinata = (position + 0x54) | 0b10000000;
	}
	
	LCD_write_command(koordinata);
	vTaskDelay(2 / portTICK_PERIOD_MS);
}

void stroka(char *str)
{
	for (uint8_t i = 0; i < strlen(str); ++i)
	{
		LCD_write_data(str[i]);
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

void backlight_on(void)
{
	backlight = 1;
}

void backlight_off(void)
{
	backlight = 0;
}
