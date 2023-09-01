#include "stdio.h"
#include "driver/i2c.h"
#include "stdlib.h"
#include "freertos/FreeRTOS.h"

#include "ms5xxx.h"
#include "pin.h"

/****************************************************************************************
 *
 * This is a sample demo code to get temperature and pressure readings from 2 sensors:
 * - MS5611 and
 * - MS5803_01BA
 *
 * In my case both sensors are wired to the same I2C bus and use default addresses:
 * - MS5803 = 0x76,
 * - MS5611 = 0x77.
 *
 * The MS5xxx library supports both sensors. Currently MS5803_01BA version supported.
 * If needed, anyone can modify MS5xxx.H and MS5xxx.C files to add calibration routines
 * as per corresponding datasheets of other sensor versions.
 *
 *****************************************************************************************/


void app_main(){

	MS5xxx_HANDLE *ptrMs5803, *ptrMs5611;
	int32_t temp_5611, press_5611, temp_5803, press_5803;

	printf("MS5611 / MS5803_01BA sensor demo project\n"); fflush(0);
	// NOTE: calling fflush(0) after printf() is just my common practice in the FreeRTOS mulitask environment.
	// Otherwise due to task switching and buffering the debug info might not be purged on time


	while(1){

		// Allocating resources
		printf("Starting MS5611 sensor driver... ");
		if(!(ptrMs5611=ms5xxx_DriverCreate(MS5xxx_TYPE_5611, I2C_NUM_0, 0x77, PIN_SDA, PIN_SCL, 400000))){
			printf("FAILED\n"); fflush(0);
			// ...
			// ...
			return;
			}
		else{
			printf("OK\n");
			}


		printf("Starting MS5801 sensor driver... ");
		if(!(ptrMs5803=ms5xxx_DriverCreate(MS5xxx_TYPE_5803, I2C_NUM_0, 0x76, PIN_SDA, PIN_SCL, 400000))){
			printf("FAILED\n"); fflush(0);
			// ...
			// ...
			return;
			}
		else{
			printf("OK\n");
			}

		fflush(0);

		// going through 10 cycles
		for(uint8_t i = 0; i < 10; i++){

			// get data from both sensors
			ms5xxx_getSensorData(ptrMs5611, MS5xxx_RESOLUTION_512, &temp_5611, &press_5611);
			ms5xxx_getSensorData(ptrMs5803, MS5xxx_RESOLUTION_512, &temp_5803, &press_5803);

			// display human-readable values in float format
			printf("Temperature, C = %.02f (%.02f).	Pressure, mbar = %.02f (%.02f)\n", (float) temp_5611/100, (float) temp_5803/100, (float)press_5611/100, (float)press_5803/100); fflush(0);

			vTaskDelay(1000); // delay 1000 ticks. On 1000Hz system it gives 1 sec delay
			}

		// releasing the resources allocated
		ms5xxx_DriverDelete(ptrMs5611);
		ms5xxx_DriverDelete(ptrMs5803);
		}
}

