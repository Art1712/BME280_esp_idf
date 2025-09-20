
#include "BME280.c"
#include "HEADER.h"
#include "I2C_sourse.c"

//================================================================================================
void app_main(void)
{
	I2C_init(0x76);
	

// Инициируем датчик BME280
	BME280_init(0, 1, 1, 1, 0, 2);

	while (1) {
		BME280_forsed_mode();
		double TT = BME280_get_temperature();
		double PP = BME280_get_pressure();
		double HH = BME280_get_humidity();
		printf("%.1f\t\tdegC \n", TT);
		printf("%.0f\t\tmmHg \n", PP);
		printf("%.1f\t\t%%rH \n", HH);
		printf("==================================================================\n");
		vTaskDelay(10000 / portTICK_PERIOD_MS);
		
    }
}
