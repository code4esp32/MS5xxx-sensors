#ifndef MAIN_MS5611_H_
#define MAIN_MS5611_H_

#include "esp_timer.h"

#define WAIT_ACK					1


#define MS5xxx_TYPE_5803			0
#define MS5xxx_TYPE_5611			1


#define MS5xxx_D1					0
#define MS5xxx_D2					1

// Комманды сенсора
#define MS5xxx_CMD_READ_ADC_RESULT	0
#define MS5xxx_CMD_READ_PROM		0xA0
#define MS5xxx_CMD_RESET			0x1E

// разрешение сенсора
#define MS5xxx_RESOLUTION_256		0
#define MS5xxx_RESOLUTION_512		1
#define MS5xxx_RESOLUTION_1024		2
#define MS5xxx_RESOLUTION_2048		3
#define MS5xxx_RESOLUTION_4096		4
#define MS5xxx_RESOLUTION_MAX		MS5xxx_RESOLUTION_4096 + 1

// задержки для заданного разрешения, мс
#define MS5xxx_ADC_256_DELAY_MS		1
#define MS5xxx_ADC_512_DELAY_MS		3
#define MS5xxx_ADC_1024_DELAY_MS	4
#define MS5xxx_ADC_2048_DELAY_MS	6
#define MS5xxx_ADC_4096_DELAY_MS	10

#define MS5xxx_I2C_TIMEOUT_TICKS	100

#define FLIP_ENDIAN16(x) (((uint16_t)(x) >> 8) | ((uint16_t)(x) << 8))

typedef struct{
	uint8_t				sensor_type;	// тип датчика // type of sensor
	i2c_port_t			i2c;			// номер I2C порта // I2C port num
	uint8_t				addr;			// адрес датчика // sensor address
	uint16_t			prom[8];		// 8 слов настроечных регистров PROM датчика // calibration data (8 words)
	esp_timer_handle_t	hTimer;		// Ресурсы для таймера точной задержки // hi-resolution timer for delay
	SemaphoreHandle_t	hSemaphore;	// Ресурсы для таймера точной задержки // semaphore to syncronize out of callback 
}MS5xxx_HANDLE;


///////////////////////////////////////////////// ОБЪЯВЛЕНИЕ ЭКСПОРТНЫХ ПЕРЕМЕННЫХ ///////////////////////////////////////////////

///////////////////////////////////////////////// ОБЪЯВЛЕНИЕ ЭКСПОРТНЫХ ФУНКЦИЙ ///////////////////////////////////////////////

extern MS5xxx_HANDLE *ms5xxx_DriverCreate(uint8_t, i2c_port_t, uint8_t, int16_t, int16_t, uint32_t);
extern uint8_t ms5xxx_DriverDelete(MS5xxx_HANDLE *);
extern bool ms5xxx_getSensorData(MS5xxx_HANDLE *, uint8_t, int32_t *, int32_t*);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif
