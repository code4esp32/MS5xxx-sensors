#ifndef MAIN_MS5803_H_
#define MAIN_MS5803_H_

#define WAIT_ACK					1

#define MS5803_D1					0
#define MS5803_D2					1

// Sensor commands
#define MS5803_CMD_READ_ADC_RESULT	0
#define MS5803_CMD_READ_PROM		0xA0
#define MS5803_CMD_RESET			0x1E

// Resolution
#define MS5803_RESOLUTION_256		0
#define MS5803_RESOLUTION_512		1
#define MS5803_RESOLUTION_1024		2
#define MS5803_RESOLUTION_2048		3
#define MS5803_RESOLUTION_4096		4
#define MS5803_RESOLUTION_MAX		MS5803_RESOLUTION_4096 + 1

// Delays for the given resolution
#define MS5803_ADC_256_DELAY_MS		1
#define MS5803_ADC_512_DELAY_MS		3
#define MS5803_ADC_1024_DELAY_MS	4
#define MS5803_ADC_2048_DELAY_MS	6
#define MS5803_ADC_4096_DELAY_MS	10

#define MS5803_I2C_TIMEOUT_TICKS	100

#define FLIP_ENDIAN16(x) (((uint16_t)(x) >> 8) | ((uint16_t)(x) << 8))

///////////////////////////////////////////////// EXPORT functions declaration ///////////////////////////////////////////////

extern bool ms5803_Init(i2c_port_t, uint8_t, int16_t, int16_t, uint32_t);
extern bool ms5803_Reset(void);
extern bool ms5803_ReadProm(void);
extern bool ms5803_getSensors(uint8_t, float *, float*);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif
