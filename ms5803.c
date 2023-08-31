#include "stdio.h"
#include "stdlib.h"
#include "esp_timer.h"
#include "driver/i2c.h"

#include "ms5803.h"


///////////////////////////////////////////////// LOCAL variables ///////////////////////////////////////////////

static uint8_t ms5803_addr;
static uint16_t prom[8];				// 8 16-bit words for PROM registers
static i2c_port_t ms5803_i2c;

static esp_timer_handle_t hTimer=0;		// Resources for fine timer delay
static SemaphoreHandle_t hSemaphore=0;	// Resources for fine timer delay

static uint8_t	ms5803_adc_delay[MS5803_RESOLUTION_MAX]	= {MS5803_ADC_256_DELAY_MS, MS5803_ADC_512_DELAY_MS, MS5803_ADC_1024_DELAY_MS, MS5803_ADC_2048_DELAY_MS, MS5803_ADC_4096_DELAY_MS};
static uint8_t	ms5803_adc_cmd[5] =	{0x40, 0x42, 0x44, 0x46, 0x48};

///////////////////////////////////////////////// LOCAL functions ///////////////////////////////////////////////

bool ms5803_send_cmd(uint8_t);
bool ms5803_read(uint8_t, uint8_t *, uint8_t);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool ms5803_read(uint8_t addr, uint8_t *ptrData, uint8_t len){
// Read LEN bytes from ADDR registry and write to PTRDATA
	i2c_cmd_handle_t link = i2c_cmd_link_create();
	i2c_master_start(link);
	i2c_master_write_byte(link, (ms5803_addr << 1), WAIT_ACK);
	i2c_master_write_byte(link, addr, WAIT_ACK); // çàãðóçêà öåëåâîãî àäðåñà
	// ïîâòîðíûé ñòàðò
	i2c_master_start(link);
	i2c_master_write_byte(link, (ms5803_addr << 1) | I2C_MASTER_READ, I2C_MASTER_ACK);
	i2c_master_read(link, ptrData, len, I2C_MASTER_LAST_NACK);
	i2c_master_stop(link);
	esp_err_t result = i2c_master_cmd_begin(ms5803_i2c, link, MS5803_I2C_TIMEOUT_TICKS);
	i2c_cmd_link_delete(link);
	return (result == ESP_OK)? 1 : 0;
}

bool ms5803_send_cmd(uint8_t cmd){
// Single-byte command transmission
	i2c_cmd_handle_t link = i2c_cmd_link_create();
	i2c_master_start(link);
	i2c_master_write_byte(link, (ms5803_addr << 1) | I2C_MASTER_WRITE, WAIT_ACK);
	i2c_master_write_byte(link, cmd, WAIT_ACK);
	i2c_master_stop(link);
	esp_err_t result = i2c_master_cmd_begin(ms5803_i2c, link, MS5803_I2C_TIMEOUT_TICKS);
	i2c_cmd_link_delete(link);
	return result == ESP_OK? 1 : 0;
}

uint8_t getCRC4(uint8_t *ptrData, uint32_t len){
// Calculate CRC-4 for an array of LEN bytes. General purpose code, use elsewhere.

	uint16_t n_rem=0;
	uint8_t n_bit;

	for(uint32_t i=0; i < len; i++){
		n_rem ^= ptrData[i];

		for(n_bit = 8; n_bit > 0; n_bit--){
			if(n_rem & (0x8000)) n_rem = (n_rem << 1) ^ 0x3000;
			else n_rem = (n_rem << 1);
			}
		}
	n_rem= (0x000F & (n_rem >> 12));
	return (n_rem ^ 0x0);
}


static void ms5803_timer_cb(){
// Simple precise delay mechanism
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(hSemaphore, &xHigherPriorityTaskWoken);
}

bool ms5803_Init(i2c_port_t i2c, uint8_t addr, int16_t pin_sda, int16_t pin_scl, uint32_t speed){
// I2C driver setup & config, communicate with a sensor, read prom data and verify CRC-4
	bool	i2c_installed=0, result = 1;
	i2c_config_t	config;

	// NOTE: I2C bus requires CLS and SDA lines be pulled up to VCC. Resistance is to be calculated based on the number of nodes on the bus.
	// If your sensor has already been pulled up on the board, you can disable internal 45K ESP32 pullups. However, there's no big problem
	// to have two parallel pullups. In this case the actual resistance will be:  1/X = 1/R + 1/45000
	// where R is the resistor found on the sensor board wiring.

	config.mode = I2C_MODE_MASTER;
	config.scl_pullup_en = GPIO_PULLUP_ENABLE;	// may lease enabled (see comments above)
	config.sda_pullup_en = GPIO_PULLUP_ENABLE;	// may lease enabled (see comments above)
	config.sda_io_num = pin_sda;
	config.scl_io_num = pin_scl;
	config.master.clk_speed = speed;
	config.clk_flags = 0;

	for(;;){ // this is a fake cycle instead of using GOTO

		// I2C driver startup
		if((i2c_param_config(i2c, &config) != ESP_OK) ||  (i2c_driver_install(i2c, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK)) break;
		ms5803_i2c = i2c;
		ms5803_addr = addr;

		// get a semaphore created
		if(!(hSemaphore = xSemaphoreCreateBinary())) break;

		// get a timer created
		esp_timer_create_args_t timer_args = {
			.callback = &ms5803_timer_cb,
			.arg = 0,
			};
		if(esp_timer_create(&timer_args, &hTimer)!=ESP_OK) break;

		// sensor reset
		if(!ms5803_Reset()) break;

		// ATTN: the delay below is not required if fine pullup resistors are used.
		// Under some wierd conditions my ESP-IDF 5.0.1 app later on won't communicate with my GY63-MS58003-01BA sensor on the proto breadboard with stock (10K) resistors and long 40cm unshielded wires.
		// This is temporary solution. You might go without this delay!
		vTaskDelay(100);

		// PROM readout
		for(uint8_t i = 0; i < 8; i++ ){
			prom[i] = 0;
			if(!ms5803_read(MS5803_CMD_READ_PROM + i*2, (uint8_t*)&prom[i], 2)){result = 0; break;}
			}
		if(!result) break; // getting out of the fake cycle

		// CRC-4 verification
		uint8_t crc_orig = ((uint8_t*)&prom)[15] & (0b1111);
		((uint8_t*)&prom)[15] = 0; // ñáðîñ CRC â ìàññèâå
		uint8_t crc_calc = getCRC4(((uint8_t*)&prom), 16);
		if(crc_orig != crc_calc) return 0;

		// NOTE: consecutive uint8_t byte array should be repacked for further 16-bit word usage
		for(uint8_t i = 0; i < 8; i++){
			prom[i] = FLIP_ENDIAN16(prom[i]);
			}
		return 1;
		}

	// shit happened :( freeing up allocated resources and getting the hell out of here
	if(hTimer) esp_timer_delete(hTimer);
	if(hSemaphore) vSemaphoreDelete(hSemaphore);
	if(i2c_installed) i2c_driver_delete(ms5803_i2c);
	return 0;
}

bool ms5803_getSensors(uint8_t resolution, float *ptrTempC, float *ptrMbar){
// Getting raw data from the sensor and making proper corrections as to datasheet. RTFM.
// Variable names correspond to datasheet ones but for C1, C2... C7 = prom[1], prom[2]... prom[7]
	
	uint8_t	data[4];
	uint32_t D1 = 0, D2 = 0;
	int32_t	dT = 0, TEMP = 0, mbarInt;
	int64_t	Offset = 0, Sensitivity = 0, T2 = 0, OFF2 = 0, Sens2 = 0;

	// start off D1 ADC (pressure)
	if(!ms5803_send_cmd(ms5803_adc_cmd[resolution])) return 0;
	// start timer to make a delay required for current resolution
	esp_timer_start_once(hTimer, 1000*ms5803_adc_delay[resolution]);
	xSemaphoreTake(hSemaphore, portMAX_DELAY);
	// read 24-bit result from ADC
	if(!ms5803_read(MS5803_CMD_READ_ADC_RESULT, (uint8_t*)&data, 3)) return 0;
	D1 = ((data[0] << 16) | (data[1] << 8) | data[2]);

	// start off D2 ADC (temperature)
	if(!ms5803_send_cmd(ms5803_adc_cmd[resolution] | (1 << 4))) return 0;
	// start timer to make a delay required for current resolution mode
	esp_timer_start_once(hTimer, 1000*ms5803_adc_delay[resolution]);
	xSemaphoreTake(hSemaphore, portMAX_DELAY);
	// read 24-bit result from ADC
	if(!ms5803_read(MS5803_CMD_READ_ADC_RESULT, (uint8_t*)&data, 3)) return 0;

	D2 = ((data[0] << 16) | (data[1] << 8) | data[2]);
	dT = D2 - (uint32_t)(((uint32_t)prom[5]) << 8);
   	TEMP = (2000 + (((int64_t)dT * prom[6]) >> 23));

	// getting correction parameters against current temperature
	if(TEMP < 2000){ // under 20C
		T2 = (int32_t) (((int64_t)dT * dT) >> 31);
		OFF2 = 3 * ((TEMP-2000) * (TEMP-2000));
		Sens2 = 7 * (((TEMP-2000) * (TEMP-2000)) >> 3);

		if(TEMP < -1500){ // below -15C
			Sens2 += (((TEMP+1500) * (TEMP+1500)) << 1);
			}
    	}
    else{ // above 20C
		T2 = 0;
		OFF2 = 0;
		Sens2 = 0;
		if(TEMP > 4500){ // above 45C
			Sens2 -= (((TEMP-4500) * (TEMP-4500)) >> 3);
			}
    	}

	// final coefficients calculation
	Offset = ((uint64_t)prom[2] << 16) + (((int64_t)prom[4] * dT) >> 7) - OFF2;
	Sensitivity = ((int64_t)prom[1] << 15) + ((((int64_t)prom[3] * dT)) >> 8) - Sens2;

	// temperature offset
	TEMP -= T2;

	// pressure calculation with correction against temperature
	mbarInt = (((((int64_t)D1 * Sensitivity) >> 21) - Offset) >> 15);
	// load results via pointer specified
	*ptrMbar = (float)mbarInt / 100;
	*ptrTempC  = (float)TEMP / 100;
	return 1;
}


bool ms5803_Reset(void){
	return ms5803_send_cmd(MS5803_CMD_RESET);
}
