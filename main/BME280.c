/*
 * Funk.c
 *
 *  Created on: 8 дек. 2024 г.
 *      Author: root
 */

#include "HEADER.h"


const uint8_t ID		= 0xD0;
/*
The ID register contains the chip identification number chip_id[7:0], which is 0x60. This number can
be read as soon as the device finished the power-on-reset.

-------------------------------------------------------------------------------------------------
|	BIT 7	|	BIT 6	|	BIT 5	|	BIT 4	|	BIT 3	|	BIT 2	|	BIT 1	|	BIT 0	|
-------------------------------------------------------------------------------------------------
|	-		|	-		|	-		|	-		|	-		|	-		|	-		|	-		|
-------------------------------------------------------------------------------------------------
|	0		|	1		|	1		|	0		|	0		|	0		|	0		|	0		|
-------------------------------------------------------------------------------------------------
const ID meaning = 0x60
*/
const uint8_t RESET		= 0xE0;
/*
The “reset” register contains the soft reset word reset[7:0]. If the value 0xB6 is written to the register,
the device is reset using the complete power-on-reset procedure. Writing other values than 0xB6 has
no effect. The readout value is always 0x00.

-------------------------------------------------------------------------------------------------
|	BIT 7	|	BIT 6	|	BIT 5	|	BIT 4	|	BIT 3	|	BIT 2	|	BIT 1	|	BIT 0	|
-------------------------------------------------------------------------------------------------
|	-		|	-		|	-		|	-		|	-		|	-		|	-		|	-		|
-------------------------------------------------------------------------------------------------
|	0		|	1		|	1		|	0		|	0		|	0		|	0		|	0		|
-------------------------------------------------------------------------------------------------
const RESET meaning = 0x00
*/
const uint8_t CTRL_HUM	= 0xF2;
#define osrs_h_2		2
#define osrs_h_1		1
#define osrs_h_0		0
/*
The CTRL_HUM register sets the humidity data acquisition options of the device. Changes to this
register only become effective after a write operation to “ctrl_meas”.

------------------------------------------------------------------------------------------------------------
|	BIT 7	|	BIT 6	|	BIT 5	|	BIT 4	|	BIT 3	|	BIT 2		|	BIT 1		|	BIT 0		|
------------------------------------------------------------------------------------------------------------
|		-	|		-	|		-	|	-		|	-		|	osrs_h_2	|	osrs_h_1	|	osrs_h_0	|
------------------------------------------------------------------------------------------------------------
|		-	|		-	|		-	|	-		|	-		|		X		|		X		|		X		|
------------------------------------------------------------------------------------------------------------
Bit 2, 1, 0		-	Controls oversampling of humidity data.
(osrs_h[2:0])		

------------------------------------------------------------------------------------
|	osrs_h[2]	|	osrs_h[1]	|	osrs_h[0]	|		Humidity oversampling		|
------------------------------------------------------------------------------------
|		0		|		0		|		0		|	Skipped (output set to 0x8000)	|
------------------------------------------------------------------------------------
|		0		|		0		|		1		|		oversampling ×1				|
------------------------------------------------------------------------------------
|		0		|		1		|		0		|		oversampling ×2				|
------------------------------------------------------------------------------------
|		0		|		1		|		1		|		oversampling ×4				|
------------------------------------------------------------------------------------
|		1		|		0		|		0		|		versampling ×8				|
------------------------------------------------------------------------------------
|		1		|		0		|		1		|		oversampling ×16			| and others
------------------------------------------------------------------------------------
*/
const uint8_t STATUS	= 0xF3;
#define measuring		3
#define im_update		0
/*
The STATUS register contains two bits which indicate the status of the device.
---------------------------------------------------------------------------------------------------------
|	BIT 7	|	BIT 6	|	BIT 5	|	BIT 4	|		BIT 3	|	BIT 2	|	BIT 1	|	BIT 0		|
---------------------------------------------------------------------------------------------------------
|		-	|		-	|		-	|	-		|	measuring	|	-		|	-		|	im_update	|
---------------------------------------------------------------------------------------------------------
|		-	|		-	|		-	|	-		|		X		|	-		|	-		|		X		|
---------------------------------------------------------------------------------------------------------
Bit 3		-	Automatically set to ‘1’ whenever a conversion is
(measuring[0])	running and back to ‘0’ when the results have been
				transferred to the data registers.
Bit 0		-	Automatically set to ‘1’ when the NVM data are being
(im_update[0])	copied to image registers and back to ‘0’ when the
				copying is done. The data are copied at power-on-
				reset and before every conversion.z
*/
const uint8_t CTRL_MEAS	= 0xF4;
#define osrs_t_2		7
#define osrs_t_1		6
#define osrs_t_0		5
#define osrs_p_2		4
#define osrs_p_1		3
#define osrs_p_0		2
#define mode_1			1
#define mode_0			0
/*
The CTRL_MEAS register sets the pressure and temperature data acquisition options of the device. The
register needs to be written after changing “ctrl_hum” for the changes to become effective.
-------------------------------------------------------------------------------------------------------------------------
|		BIT 7	|		BIT 6	|		BIT 5	|		BIT 4	|		BIT 3	|		BIT 2	|	BIT 1	|	BIT 0	|
-------------------------------------------------------------------------------------------------------------------------
|	osrs_t_2	|	osrs_t_1	|	osrs_t_0	|	osrs_p_2	|	osrs_p_1	|	osrs_p_0	|	mode_1	|	mode_0	|
-------------------------------------------------------------------------------------------------------------------------
|		X		|		X		|		X		|		X		|		X		|		X		|	X		|	X		|
-------------------------------------------------------------------------------------------------------------------------
Bits 7, 6, 5	-	Controls oversampling of temperature data.
(osrs_t[2:0])	
------------------------------------------------------------------------------------
|	osrs_t_2	|	osrs_t_1	|	osrs_t_0	|	Temperature oversampling		|
------------------------------------------------------------------------------------
|		0		|		0		|		0		|	Skipped (output set to 0x8000)	|
------------------------------------------------------------------------------------
|		0		|		0		|		1		|		oversampling ×1				|
------------------------------------------------------------------------------------
|		0		|		1		|		0		|		oversampling ×2				|
------------------------------------------------------------------------------------
|		0		|		1		|		1		|		oversampling ×4				|
------------------------------------------------------------------------------------
|		1		|		0		|		0		|		versampling ×8				|
------------------------------------------------------------------------------------
|		1		|		0		|		1		|		oversampling ×16			|
------------------------------------------------------------------------------------
Bits 4, 3, 2	-	Controls oversampling of pressure data.
(osrs_p[2:0])	
------------------------------------------------------------------------------------
|	osrs_p_2	|	osrs_p_1	|	osrs_p_0	|		Pressure oversampling		|
------------------------------------------------------------------------------------
|		0		|		0		|		0		|	Skipped (output set to 0x8000)	|
------------------------------------------------------------------------------------
|		0		|		0		|		1		|		oversampling ×1				|
------------------------------------------------------------------------------------
|		0		|		1		|		0		|		oversampling ×2				|
------------------------------------------------------------------------------------
|		0		|		1		|		1		|		oversampling ×4				|
------------------------------------------------------------------------------------
|		1		|		0		|		0		|		versampling ×8				|
------------------------------------------------------------------------------------
|		1		|		0		|		1		|		oversampling ×16			|
------------------------------------------------------------------------------------
Bits 1, 0	-	Controls the sensor mode of the device.
(mode[1:0])		
---------------------------------------------
|	mode_1	|	mode_0	|		Mode		|
---------------------------------------------
|	0		|	0		|	Sleep mode		|
---------------------------------------------
|	0(1)	|	1(0)	|	Forced mode		|
---------------------------------------------
|	1		|	1		|	Normal mode		|
---------------------------------------------
*/

const uint8_t CONFIG = 0xF5;
#define t_sb_2			7
#define t_sb_1			6
#define t_sb_0			5
#define filter_2		4
#define filter_1		3
#define filter_0		2
#define spi3w_en_0		0
/*
The CONFIG register sets the rate, filter and interface options of the device. Writes to the “config”
register in normal mode may be ignored. In sleep mode writes are not ignored.
-----------------------------------------------------------------------------------------------------------------
|	BIT 7	|	BIT 6	|	BIT 5	|	BIT 4		|	BIT 3		|	BIT 2		|	BIT 1	|	BIT 0		|
-----------------------------------------------------------------------------------------------------------------
|	t_sb_2	|	t_sb_1	|	t_sb_0	|	filter_2	|	filter_1	|	filter_0	|	-		|	spi3w_en_0	|
-----------------------------------------------------------------------------------------------------------------
|	X		|	X		|	X		|		X		|		X		|		X		|	-		|		X		|
-----------------------------------------------------------------------------------------------------------------
Bits 7, 6, 5	-	Controls inactive duration t_standby in normal mode.
(t_sb[2:0])	
---------------------------------------------------------
|	t_sb_2	|	t_sb_1	|	t_sb_0	|	t_standby [ms]	|
---------------------------------------------------------
|	0		|	0		|	0		|		0.5			|
---------------------------------------------------------
|	0		|	0		|	1		|		62.5		|
---------------------------------------------------------
|	0		|	1		|	0		|		125			|
---------------------------------------------------------
|	0		|	1		|	1		|		250			|
---------------------------------------------------------
|	1		|	0		|	0		|		500			|
---------------------------------------------------------
|	1		|	0		|	1		|		1000		|
---------------------------------------------------------
|	1		|	1		|	0		|		10			|
---------------------------------------------------------
|	1		|	1		|	1		|		20			|
---------------------------------------------------------
Bits 4, 3, 2	-	Controls the time constant of the IIR filter.
(filter[2:0])	
-------------------------------------------------------------------------
|	filter_2	|	filter_1	|	filter_0	|	Filter coefficient	|
-------------------------------------------------------------------------
|		0		|		0		|		0		|		Filter off		|
-------------------------------------------------------------------------
|		0		|		0		|		1		|			2			|
-------------------------------------------------------------------------
|		0		|		1		|		0		|			4			|
-------------------------------------------------------------------------
|		0		|		1		|		1		|			8			|
-------------------------------------------------------------------------
|		1		|		0		|		0		|			16			|
-------------------------------------------------------------------------
Bit 0	-	Enables 3-wire SPI interface when set to ‘1’.
(spi3w_en[0])
*/

// Переменные поправочных коэффициентов
	const uint16_t t_fine_adjust = 12288; // Поправка температуры если не совпадает с реальными показателями
	int32_t t_fine; // Поправочный коэффициент
	int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2, dig_H4, dig_H5;
	uint16_t dig_T1, dig_P1;
	uint8_t dig_H1, dig_H3;
	int8_t dig_H6;

// Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
double BME280_compensate_T_double(int32_t adc_T)
{
	double var1, var2, T;
	var1 = (((double)adc_T)/16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) * (((double)adc_T)/131072.0 - ((double) dig_T1) / 8192.0)) * ((double)dig_T3);
	t_fine = (int32_t)(var1 + var2);
	T = (var1 + var2) / 5120.0;
	return T;
}
//================================================================================================
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
double BME280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2 - t_fine_adjust;
	T = (t_fine * 5 + 128) >> 8;
	return (double)T/100;
}
//================================================================================================
// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
double BME280_compensate_P_double(int32_t adc_P)
{
	double var1, var2, p;
	var1 = ((double)t_fine/2.0) - 64000.0;
	var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)dig_P5) * 2.0;
	var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
	var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
	if (var1 == 0.0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
	return p;
}
//================================================================================================
// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
double BME280_compensate_P_int64(int32_t adc_P)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)*131072/*<<17*/);
	var2 = var2 + (((int64_t)dig_P4)*34359738368/*<<35*/);
	var1 = ((var1 * var1 * (int64_t)dig_P3)/256/*>>8*/) + ((var1 * (int64_t)dig_P2)*4096/*<<12*/);
	var1 = (((((int64_t)1)*140737488355328/*<<47*/)+var1))*((int64_t)dig_P1)/8589934592/*>>33*/;
	if (var1 == 0)
		{
			return 0; // avoid exception caused by division by zero
		}
	p = 1048576 - adc_P;
	p = (((p*2147483648/*<<31*/) - var2) * 3125) / var1;
	var1 = (((int64_t)dig_P9) * (p / 8192/*>>13*/) * (p / 8192/*>>13*/)) / 33554432 /*>> 25*/;
	var2 = (((int64_t)dig_P8) * p) / 524288 /*>> 19*/;
	p = ((p + var1 + var2) / 256 /*>> 8*/) + (((int64_t)dig_P7) * 16/*<<4*/);
	return ((double)p *0.01 / 256.0)/1.333;
}
//================================================================================================
// Returns humidity in %rH as as double. Output value of “46.332” represents 46.332 %rH
double BME280_compensate_H_double(int32_t adc_H)
{
	double var_H;
	var_H = (((double)t_fine) - 76800.0);
	var_H = (adc_H - (((double)dig_H4) * 64.0 + ((double)dig_H5) / 16384.0 * var_H)) * (((double)dig_H2) / 65536.0 * (1.0 + ((double)dig_H6) / 67108864.0 * var_H * (1.0 + ((double)dig_H3) / 67108864.0 * var_H)));
	var_H = var_H * (1.0 - ((double)dig_H1) * var_H / 524288.0);
	if (var_H > 100.0)
	var_H = 100.0;
	else if (var_H < 0.0)
	var_H = 0.0;
	return var_H;
}
//================================================================================================
// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
double BME280_compensate_H_int32(int32_t adc_H)
{
	int32_t v_x1_u32r;
	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) +
	((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (double)(v_x1_u32r >> 12) / 1024;
}
//================================================================================================
void BME280_init(const uint8_t mode, const uint8_t temperature, const uint8_t pressure, const uint8_t humidity, const uint8_t standby, const uint8_t filter)
{
	uint8_t ID_read; // for read ID
	uint8_t RES[2] = {RESET, 0xB6}; // for restart device
	uint8_t hum[2] = {CTRL_HUM, 0};

	uint8_t meas[2] = {CTRL_MEAS, 0};

	uint8_t con[2] = {CONFIG, 0};

	I2C_write_read(&ID, 1, &ID_read, 1);
	if (ID_read == 0x60)
	printf("Devise BME280 is found\n");
	else 
	{
		printf("Error, devise BME280 is not found\n");
		printf("%x \n", ID_read);
		I2C_write(RES, 2); // restarting device
	}
	// Restart BME280
	I2C_write(RES, 2); // restarting device
	// Check if NVM data already copied
	while ((BME280_ReadStatus() & 0b00000001) == 1);
	BME280_read_calib_Data();
	// Check if NVM data already copied
	while ((BME280_ReadStatus() & 0b00000001) == 1);
	
	// BME280 mode-----------------------------------------------
	if (mode == 0) meas[1] &= ~((1 << mode_1) | (1 << mode_0));
	if (mode == 1 || mode == 2) // Forced mode 
	{
		meas[1] |= (1 << mode_0);
		meas[1] &= ~(1 << mode_1);
	}
	if (mode == 3) // Normal mode
	{
		meas[1] |= ((1 << mode_1) | (1 << mode_0));
	}
	// BME280 temp-----------------------------------------------
	switch (temperature)
	{
		case 0: // Skipped
			meas[1] &= ~((1 << osrs_t_2) | (1 << osrs_t_1) | (1 << osrs_t_0));
		break;
		case 1: // oversampling ×1
			meas[1] &= ~((1 << osrs_t_2) | (1 << osrs_t_1));
			meas[1] |= (1 << osrs_t_0);
		break;
		case 2: // oversampling ×2
			meas[1] &= ~((1 << osrs_t_2) | (1 << osrs_t_0));
			meas[1] |= (1 << osrs_t_1);
		break;
		case 3: // oversampling ×4
			meas[1] &= ~(1 << osrs_t_2);
			meas[1] |= ((1 << osrs_t_1) | (1 << osrs_t_0));
		break;
		case 4: // oversampling ×8
			meas[1] &= ~((1 << osrs_t_1) | (1 << osrs_t_0));
			meas[1] |= (1 << osrs_t_2);
		break;
		case 5: // oversampling ×16
			meas[1] &= ~(1 << osrs_t_1);
			meas[1] |= ((1 << osrs_t_2) | (1 << osrs_t_0));
		break;
	}
	
	// BME280 press-----------------------------------------------
	switch (pressure)
	{
		case 0: // Skipped
			meas[1] &= ~((1 << osrs_p_2) | (1 << osrs_p_1) | (1 << osrs_p_0));
		break;
		case 1: // oversampling ×1
			meas[1] &= ~((1 << osrs_p_2) | (1 << osrs_p_1));
			meas[1] |= (1 << osrs_p_0);
		break;
		case 2: // oversampling ×2
			meas[1] &= ~((1 << osrs_p_2) | (1 << osrs_p_0));
			meas[1] |= (1 << osrs_p_1);
		break;
		case 3: // oversampling ×4
			meas[1] &= ~(1 << osrs_p_2);
			meas[1] |= ((1 << osrs_p_1) | (1 << osrs_p_0));
		break;
		case 4: // oversampling ×8
			meas[1] &= ~((1 << osrs_p_1) | (1 << osrs_p_0));
			meas[1] |= (1 << osrs_p_2);
		break;
		case 5: // oversampling ×16
			meas[1] &= ~(1 << osrs_p_1);
			meas[1] |= ((1 << osrs_p_2) | (1 << osrs_p_0));
		break;
	}
	// BME280 humidity-----------------------------------------------
	switch (humidity)
		{
			case 0: // Skipped
				hum[1] &= ~((1 << osrs_h_2) | (1 << osrs_h_1) | (1 << osrs_h_0));
			break;
			case 1: // oversampling ×1
				hum[1] &= ~((1 << osrs_h_2) | (1 << osrs_h_1));
				hum[1] |= (1 << osrs_h_0);
			break;
			case 2: // oversampling ×2
				hum[1] &= ~((1 << osrs_h_2) | (1 << osrs_h_0));
				hum[1] |= (1 << osrs_h_1);
			break;
			case 3: // oversampling ×4
				hum[1] &= ~(1 << osrs_h_2);
				hum[1] |= ((1 << osrs_h_1) | (1 << osrs_h_0));
			break;
			case 4: // oversampling ×8
				hum[1] &= ~((1 << osrs_h_1) | (1 << osrs_h_0));
				hum[1] |= (1 << osrs_h_2);
			break;
			case 5: // oversampling ×16
				hum[1] &= ~(1 << osrs_h_1);
				hum[1] |= ((1 << osrs_h_2) | (1 << osrs_h_0));
			break;
		}
	// BME280 standby-----------------------------------------------
	switch (standby)
		{
			case 0: // 0.5 ms
				con[1] &= ~((1 << t_sb_2) | (1 << t_sb_1) | (1 << t_sb_0));
			break;
			case 1: // 62.5 ms
				con[1] &= ~((1 << t_sb_2) | (1 << t_sb_1));
				con[1] |= (1 << t_sb_0);
			break;
			case 2: // 125 ms
				con[1] &= ~((1 << t_sb_2) | (1 << t_sb_0));
				con[1] |= (1 << t_sb_1);
			break;
			case 3: // 250 ms
				con[1] &= ~(1 << t_sb_2);
				con[1] |= ((1 << t_sb_1) | (1 << t_sb_0));
			break;
			case 4: // 500 ms
				con[1] &= ~((1 << t_sb_1) | (1 << t_sb_0));
				con[1] |= (1 << t_sb_2);
			break;
			case 5: // 1000 ms
				con[1] &= ~(1 << t_sb_1);
				con[1] |= ((1 << t_sb_2) | (1 << t_sb_0));
			break;
			case 6: // 10 ms
				con[1] &= ~(1 << t_sb_0);
				con[1] |= ((1 << t_sb_2) | (1 << t_sb_1));
			break;
			case 7: // 20 ms
				con[1] |= ((1 << t_sb_2) | (1 << t_sb_1) | (1 << t_sb_0));
			break;
		}
	// BME280 standby-----------------------------------------------
	switch (filter)
		{
			case 0: // Filter off
				con[1] &= ~((1 << filter_2) | (1 << filter_1) | (1 << filter_0));
			break;
			case 1: // 2
				con[1] &= ~((1 << filter_2) | (1 << filter_1));
				con[1] |= (1 << filter_0);
			break;
			case 2: // 4
				con[1] &= ~((1 << filter_2) | (1 << filter_0));
				con[1] |= (1 << filter_1);
			break;
			case 3: // 8
				con[1] &= ~(1 << filter_2);
				con[1] |= ((1 << filter_1) | (1 << filter_0));
			break;
			case 4: // 16
				con[1] &= ~((1 << filter_1) | (1 << filter_0));
				con[1] |= (1 << filter_2);
			break;
		}

	I2C_write(hum, 2);
	I2C_write(meas, 2);
	I2C_write(con, 2);
}

//================================================================================================
uint8_t BME280_ReadStatus(void)
{
	uint8_t stat;
	I2C_write_read(&STATUS, 1, &stat, 1);
	// Clear unuset bits and return
	return (stat & 0x09);
}
//================================================================================================

void BME280_read_calib_Data(void)
{
	// Адреса регистров с коэффициентами
	uint8_t addr = 0x88;
		
	// Массив для считывания данных температуры и давления
	uint8_t data[24] = {0};
	// Считываем компенсационные параметры
	I2C_write_read(&addr, 1, data, 24);
	// Для температуры =====================================================================================
	dig_T1 = (((uint16_t)data[1] << 8) | (uint16_t)data[0]);
	dig_T2 = (((uint16_t)data[3] << 8) | (uint16_t)data[2]);
	dig_T3 = (((uint16_t)data[5] << 8) | (uint16_t)data[4]);
	// Для давления ========================================================================================
	dig_P1 = (((uint16_t)data[7] << 8) | (uint16_t)data[6]);
	dig_P2 = (((uint16_t)data[9] << 8) | (uint16_t)data[8]);
	dig_P3 = (((uint16_t)data[11] << 8) | (uint16_t)data[10]);
	dig_P4 = (((uint16_t)data[13] << 8) | (uint16_t)data[12]);
	dig_P5 = (((uint16_t)data[15] << 8) | (uint16_t)data[14]);
	dig_P6 = (((uint16_t)data[17] << 8) | (uint16_t)data[16]);
	dig_P7 = (((uint16_t)data[19] << 8) | (uint16_t)data[18]);
	dig_P8 = (((uint16_t)data[21] << 8) | (uint16_t)data[20]);
	dig_P9 = (((uint16_t)data[23] << 8) | (uint16_t)data[22]);
	// Для влажности =======================================================================================
	uint8_t H1 = 0xA1;
	I2C_write_read(&H1, 1, &dig_H1, 1);
	//dig_H1 = data[24];
	uint8_t H2 = 0xE1;
	uint8_t data1[7] = {0};
	I2C_write_read(&H2, 1, data1, 1);
	dig_H2 = (((uint16_t)data1[1] << 8) | (uint16_t)data1[0]);
	dig_H3 = data[2];
	dig_H4 = (((uint16_t)data1[3] << 4) | (uint16_t)(data1[4] & 0b00001111));
	dig_H5 = (((uint16_t)data1[5] << 4) | ((uint16_t)data1[4] >> 4));
	dig_H6 = data1[6];
	
}
//================================================================================================
double BME280_get_temperature(void)
{
	int32_t adc_T, var1, var2, T;
	
	const uint8_t adress = 0xFA; // Begin read from address 0xFA to 0xFC 
	uint8_t data[3] = {0, 0, 0};
	I2C_write_read(&adress, 1, data, 3);
	adc_T = (uint32_t)(((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4));
	
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2 - t_fine_adjust;
	T = (t_fine * 5 + 128) >> 8;
	return (double)T/100;
}
//================================================================================================
double BME280_get_pressure(void)
{
	int64_t var1, var2, p;
	int32_t adc_P;
	
	const uint8_t adress = 0xF7; // Begin read from address 0xF7 to 0xF9
	uint8_t data[3] = {0, 0, 0};
	I2C_write_read(&adress, 1, data, 3);
	adc_P = (((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4));
	
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)*131072/*<<17*/);
	var2 = var2 + (((int64_t)dig_P4)*34359738368/*<<35*/);
	var1 = ((var1 * var1 * (int64_t)dig_P3)/256/*>>8*/) + ((var1 * (int64_t)dig_P2)*4096/*<<12*/);
	var1 = (((((int64_t)1)*140737488355328/*<<47*/)+var1))*((int64_t)dig_P1)/8589934592/*>>33*/;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p*2147483648/*<<31*/) - var2) * 3125) / var1;
	var1 = (((int64_t)dig_P9) * (p / 8192/*>>13*/) * (p / 8192/*>>13*/)) / 33554432 /*>> 25*/;
	var2 = (((int64_t)dig_P8) * p) / 524288 /*>> 19*/;
	p = ((p + var1 + var2) / 256 /*>> 8*/) + (((int64_t)dig_P7) * 16/*<<4*/);
	return ((double)p *0.01 / 256.0)/1.333 + 18;
}
//================================================================================================
double BME280_get_humidity(void)
{
	int32_t v_x1_u32r;
	int32_t adc_H;
	
	const uint8_t adress = 0xFD; // Begin read from address 0xFD to 0xFE
	uint8_t data[2] = {0, 0};
	I2C_write_read(&adress, 1, data, 2);
	adc_H = ((uint32_t)data[0] << 8) | data[1];
	
	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * 
	(((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (double)(v_x1_u32r >> 12) / 1024;
}
//================================================================================================
void BME280_forsed_mode(void)
{
	uint8_t meas[2] = {CTRL_MEAS, 0};
	I2C_write_read(&CTRL_MEAS, 1, &meas[1], 1);
	meas[1] |= (1 << mode_0);
	meas[1] &= ~(1 << mode_1);
	I2C_write(meas, 2);

}

