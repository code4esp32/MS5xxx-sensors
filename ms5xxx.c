#include "stdio.h"
#include "stdlib.h"
#include "esp_timer.h"
#include "driver/i2c.h"

#include "ms5xxx.h"

///////////////////////////////////////////////// ОБЪЯВЛЕНИЕ ЛОКАЛЬНЫХ ПЕРЕМЕННЫХ ///////////////////////////////////////////////
///////////////////////////////////////////////// LOCAL variables ///////////////////////////////////////////////


static uint8_t	ms5611_adc_delay[MS5xxx_RESOLUTION_MAX]	= {MS5xxx_ADC_256_DELAY_MS, MS5xxx_ADC_512_DELAY_MS, MS5xxx_ADC_1024_DELAY_MS, MS5xxx_ADC_2048_DELAY_MS, MS5xxx_ADC_4096_DELAY_MS};
static uint8_t	ms5611_adc_cmd[5] =	{0x40, 0x42, 0x44, 0x46, 0x48};
static uint8_t	i2c_driver_users_count=0;

///////////////////////////////////////////////// ОБЪЯВЛЕНИЕ ЛОКАЛЬНЫХ ФУНКЦИЙ ///////////////////////////////////////////////
///////////////////////////////////////////////// LOCAL functions ///////////////////////////////////////////////


bool ms5xxx_send_cmd(MS5xxx_HANDLE *, uint8_t);
bool ms5xxx_read(MS5xxx_HANDLE *, uint8_t, uint8_t *, uint8_t);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool ms5xxx_read(MS5xxx_HANDLE *ptrMs5xxx, uint8_t reg, uint8_t *ptrData, uint8_t len){
// Чтение len байт из регистра по адресу reg и загрузка по указателю ptrData
// Read LEN bytes from REG and write data via pointer ptrData
	i2c_cmd_handle_t link = i2c_cmd_link_create();
	i2c_master_start(link);
	i2c_master_write_byte(link, (ptrMs5xxx->addr << 1), WAIT_ACK);
	i2c_master_write_byte(link, reg, WAIT_ACK);
	// повторный старт
	// re-start
	i2c_master_start(link);
	i2c_master_write_byte(link, (ptrMs5xxx->addr << 1) | I2C_MASTER_READ, I2C_MASTER_ACK);
	i2c_master_read(link, ptrData, len, I2C_MASTER_LAST_NACK);
	i2c_master_stop(link);
	esp_err_t result = i2c_master_cmd_begin(ptrMs5xxx->i2c, link, MS5xxx_I2C_TIMEOUT_TICKS);
	i2c_cmd_link_delete(link);
	return (result == ESP_OK)? 1 : 0;
}

bool ms5xxx_send_cmd(MS5xxx_HANDLE *ptrMs5xxx, uint8_t cmd){
// Отправка однобайтной команды
// Send a single-byte command
	i2c_cmd_handle_t link = i2c_cmd_link_create();
	i2c_master_start(link);
	i2c_master_write_byte(link, (ptrMs5xxx->addr << 1) | I2C_MASTER_WRITE, WAIT_ACK);
	i2c_master_write_byte(link, cmd, WAIT_ACK);
	i2c_master_stop(link);
	esp_err_t result = i2c_master_cmd_begin(ptrMs5xxx->i2c, link, MS5xxx_I2C_TIMEOUT_TICKS);
	i2c_cmd_link_delete(link);
	return result == ESP_OK? 1 : 0;
}

uint8_t getCRC4(uint8_t *ptrData, uint32_t len){
// функция по расчету CRC4 для ПОСЛЕДОВАТЕЛЬНОСТИ БАЙТ заданной длины
// General-purpose function to calculate CRC-4 for a given byte array of LEN size
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

static void ms5xxx_timer_cb(MS5xxx_HANDLE *ptrMs5xxx){
// Функция для точной задержки
// Precision delay function (us resolution)
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(ptrMs5xxx->hSemaphore, &xHigherPriorityTaskWoken);
	if(xHigherPriorityTaskWoken) taskYIELD();
}

MS5xxx_HANDLE *ms5xxx_DriverCreate(uint8_t sensor_type, i2c_port_t i2c, uint8_t addr, int16_t pin_sda, int16_t pin_scl, uint32_t speed){
// Функция создает новый драйвер датчика:
// - устанавливает драйвер I2C, если еще не поднят
// - настраивает ножки и скорость
// - выделяет ресурсы для внутренних сервисных операций
// - отправляет датчику команду сброса
// - считывает данные калибровки
// - проверяет CRC-4
//
// Возврат: дескриптор драйвера или НОЛЬ в случае неудачи

// The function creates a new instance of a sensor driver:
// - installs I2C driver if none installed yet
// - configures the pins and speed
// - allocates resources for internal use
// - sends a reset command to the sensor
// - reads calibration data
// - verifies CRC-4
//
// Return: handle of the newely created driver or NULL

	bool	result = 1;
	i2c_config_t	config;
	MS5xxx_HANDLE	*ptrMs5xxx=0;

	// ВАЖНО: На шине I2C всегда ОБЯЗАНА быть подтяжка к питанию!
	// Если в обвязке датчика уже есть подтяжка, то можно встроенные 45К подтяжки ESP32 можно отключить
	// Если использовать одновременно внешнюю и встроенную, то ничего криминального не произойдет. Суммарное сопротивление
	// можно вывести: 1/X = 1/R + 1/45000, где X - искомая величина, а R - подтяжка в обвязке.

	// NOTE: I2C bus requires CLS and SDA lines be pulled up to VCC. Resistance is to be calculated based on the number of nodes on the bus.
	// If your sensor has already been pulled up on the board, you can disable internal 45K ESP32 pullups. However, there's no big problem
	// to have two parallel pullups. In this case the actual resistance will be:  1/X = 1/R + 1/45000
	// where R is the resistor found on the sensor board wiring.


	config.mode = I2C_MODE_MASTER;
	config.scl_pullup_en = GPIO_PULLUP_ENABLE;	// may leave enabled (see comments above)
	config.sda_pullup_en = GPIO_PULLUP_ENABLE;	// may leave enabled (see comments above)
	config.sda_io_num = pin_sda;
	config.scl_io_num = pin_scl;
	config.master.clk_speed = speed;
	config.clk_flags = 0;

	for(;;){ // искусственный цикл вместо GOTO // this is a fake cycle instead of using GOTO

		// выделение памяти под структуру // memory allocation for the main structure
		ptrMs5xxx = malloc(sizeof(MS5xxx_HANDLE));
		if(!ptrMs5xxx) break;
		// инициализация нулями // clearing allocated memory
		for(uint8_t i=0; i<sizeof(MS5xxx_HANDLE); i++){((uint8_t*)ptrMs5xxx)[i]=0;}

		// подъем драйвера I2C, если еще не поднят // install I2C driver if not installed yet
		if(!i2c_driver_users_count){
			if((i2c_param_config(i2c, &config) != ESP_OK) ||  (i2c_driver_install(i2c, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK)) break;
			}
		// инкремент количества пользователей I2C // increment the number of driver users
		i2c_driver_users_count++;

		ptrMs5xxx->i2c = i2c;
		ptrMs5xxx->addr = addr;

		// создание семафора для таймера // create semaphore for sync purposes
		if(!(ptrMs5xxx->hSemaphore = xSemaphoreCreateBinary())) break;

		// создание таймера // create a timer
		esp_timer_create_args_t timer_args = {
			.callback = &ms5xxx_timer_cb,
			.arg = ptrMs5xxx,
			};
		if(esp_timer_create(&timer_args, &ptrMs5xxx->hTimer)!=ESP_OK) break;
		// сохранение типа используемого сенсора // save the type of sensor used
		ptrMs5xxx->sensor_type = sensor_type;

		// сброс // sensor reset
		if(!ms5xxx_send_cmd(ptrMs5xxx, MS5xxx_CMD_RESET)) break;

		// TODO TODO TODO после замены 10К резисторов удалить задержку.

		// ATTN: the delay below is not required if fine pullup resistors are used.
		// Under some wierd conditions my ESP-IDF 5.0.1 app later on won't communicate with my GY63-MS58003-01BA sensor on the proto breadboard with stock (10K) resistors and long 40cm unshielded wires.
		// This is temporary solution. You might go without this delay!
		vTaskDelay(100);

		// загрузка PROM (калибровка) // load PROM (calibration data)
		for(uint8_t i = 0; i < 8; i++ ){
			ptrMs5xxx->prom[i] = 0;
			if(!ms5xxx_read(ptrMs5xxx, MS5xxx_CMD_READ_PROM + i*2, (uint8_t*)&ptrMs5xxx->prom[i], 2)){result = 0; break;}
			}
		if(!result) break; // отработка выхода из вложенного цикла выше // quit the nested cycle above

		// сверка CRC-4 в формате ПОСЛЕДОВАТЕЛЬНЫХ БАЙТ // CRC-4 verification as byte array
		uint8_t crc_orig = ((uint8_t*)&ptrMs5xxx->prom)[15] & (0b1111);
		((uint8_t*)&ptrMs5xxx->prom)[15] = 0; // сброс CRC в массиве
		uint8_t crc_calc = getCRC4(((uint8_t*)&ptrMs5xxx->prom), 16);
		if(crc_orig != crc_calc) return 0;

		// ВАЖНО: переворот байт в словах // NOTE: consecutive uint8_t byte array should be repacked for further 16-bit word usage
		for(uint8_t i = 0; i < 8; i++){
			ptrMs5xxx->prom[i] = FLIP_ENDIAN16(ptrMs5xxx->prom[i]);
			}
		return ptrMs5xxx;
		}

	// тут ошибка. освобождение ресурсов // shit happened :( free up allocated resources and exit with fail code
	if(ptrMs5xxx->hTimer) esp_timer_delete(ptrMs5xxx->hTimer);
	if(ptrMs5xxx->hSemaphore) vSemaphoreDelete(ptrMs5xxx->hSemaphore);
	if(ptrMs5xxx)free(ptrMs5xxx);
	if(i2c_driver_users_count){
		i2c_driver_delete(ptrMs5xxx->i2c);	// удаление драйвера, если больше не осталось владельцев
		i2c_driver_users_count = 0;
		}
	return 0;
}

uint8_t ms5xxx_DriverDelete(MS5xxx_HANDLE *ptrMs5xxx){
// функция освобождает выделенные ресурсы и удаляет драйвер датчика
// The function deletes the driver and frees us allocated resources

	if(ptrMs5xxx && i2c_driver_users_count){
		if(ptrMs5xxx->hTimer) esp_timer_delete(ptrMs5xxx->hTimer);
		if(ptrMs5xxx->hSemaphore) vSemaphoreDelete(ptrMs5xxx->hSemaphore);
		if(ptrMs5xxx)free(ptrMs5xxx);
		i2c_driver_users_count--;
		if(!i2c_driver_users_count) i2c_driver_delete(ptrMs5xxx->i2c);	// удаление драйвера, если больше не осталось владельцев
		}
	return i2c_driver_users_count;
}


bool ms5xxx_getSensorData(MS5xxx_HANDLE *ptrMs5xxx, uint8_t resolution, int32_t *ptrTempC, int32_t *ptrMbar){
// Функция чтения температуры и давления:
// - запускает преобразования АЦП
// - считывает 24-битные значения
// - проводит корректировку в зависимости от температуры (строго по даташиту)
// - записывает целочисленные значения по указателям ptrTempC и ptrMbar (значение 1000 = 10.00С итд)
//
// Возврат: 1 = ОК, 0 = ошибка

// The function gets temperature and pressure readings from the sensors
// - starts ADC
// - gets the 24-bit values
// - adjusts the figures against current temperature (as per datasheet)
// - writes integer output values via pointers ptrTempC and ptrMbar (value 1000 = 10.00С etc)
//
// RETURN: 1 = OK, 0 = ERROR

	uint8_t	data[4];
	uint32_t D1 = 0, D2 = 0;
	int32_t	dT = 0, TEMP = 0;
	int64_t	Offset = 0, Sensitivity = 0, T2 = 0, OFF2 = 0, Sens2 = 0;

	// отправка команды на запуск АЦП (D1 = давление) // starts D1 ADC (pressure)
	if(!ms5xxx_send_cmd(ptrMs5xxx, ms5611_adc_cmd[resolution])) return 0;
	// запуск таймера для задержки // sets a timeout in us
	esp_timer_start_once(ptrMs5xxx->hTimer, 1000*ms5611_adc_delay[resolution]);
	xSemaphoreTake(ptrMs5xxx->hSemaphore, portMAX_DELAY);
	// загрузка 24-бит результата АЦП // reads the 24-bit ADC result
	if(!ms5xxx_read(ptrMs5xxx, MS5xxx_CMD_READ_ADC_RESULT, (uint8_t*)&data, 3)) return 0;
	D1 = ((data[0] << 16) | (data[1] << 8) | data[2]);

	// отправка команды на запуск АЦП (D2 = температура) // starts D2 ADC (temperature)
	if(!ms5xxx_send_cmd(ptrMs5xxx, ms5611_adc_cmd[resolution] | (1 << 4))) return 0;
	// запуск таймера для задержки // sets a timeout in us
	esp_timer_start_once(ptrMs5xxx->hTimer, 1000*ms5611_adc_delay[resolution]);
	xSemaphoreTake(ptrMs5xxx->hSemaphore, portMAX_DELAY);
	// загрузка 24-бит результата АЦП // reads the 24-bit ADC result
	if(!ms5xxx_read(ptrMs5xxx, MS5xxx_CMD_READ_ADC_RESULT, (uint8_t*)&data, 3)) return 0;

	D2 = ((data[0] << 16) | (data[1] << 8) | data[2]);
	dT = D2 - (uint32_t)(((uint32_t)ptrMs5xxx->prom[5]) << 8);
    TEMP = (2000 + (((int64_t)dT * ptrMs5xxx->prom[6]) >> 23));

	// расчет параметров для поправочных коэф-тов в зависимости от температуры
	// ВАЖНО: алгоритм и константы взяты строго из даташита!
	// ВАЖНО: разрядность переменных соответствует требованиям даташита!

	// correction coefficients calculation
	// IMPORTANT: algorythm and constants are set as per datasheet!
	// IMPORTANT: the sizes of variables are as per datasheet!

	switch(ptrMs5xxx->sensor_type){
		case MS5xxx_TYPE_5803:
			if(TEMP < 2000){ // менее 20С // below 20C
				T2 = (int32_t) (((int64_t)dT * dT) >> 31);
				OFF2 = 3 * ((TEMP-2000) * (TEMP-2000));
				Sens2 = 7 * (((TEMP-2000) * (TEMP-2000)) >> 3);
				if(TEMP < -1500){ // ниже -15С // below -15C
					Sens2 += (((TEMP+1500) * (TEMP+1500)) << 1);
					}
				}
			else{ // выше 20С // above 20C
				T2 = 0;
				OFF2 = 0;
				Sens2 = 0;
				if(TEMP > 4500){ // выше 45С // above 45C
					Sens2 -= (((TEMP-4500) * (TEMP-4500)) >> 3);
					}
				}
			Offset = ((uint64_t)ptrMs5xxx->prom[2] << 16) + (((int64_t)ptrMs5xxx->prom[4] * dT) >> 7) - OFF2;
			Sensitivity = ((int64_t)ptrMs5xxx->prom[1] << 15) + ((((int64_t)ptrMs5xxx->prom[3] * dT)) >> 8) - Sens2;
			break;

		case MS5xxx_TYPE_5611:
			if(TEMP < 2000){ // менее 20С // below 20C
				T2 = (int32_t) (((int64_t)dT * dT) >> 31);
				OFF2 = (5 * ((TEMP-2000) * (TEMP-2000))) >> 1;
				Sens2 = 5 * (((TEMP-2000) * (TEMP-2000)) >> 2);
				if(TEMP < -1500){ // ниже -15С // below -15C
					OFF2 += (7*(TEMP+1500)*(TEMP+1500));
					Sens2 += ((11 * (TEMP+1500) * (TEMP+1500)) >> 1);
					}
				}
			else{ // выше 20С // above 20C
				T2 = 0;
				OFF2 = 0;
				Sens2 = 0;
				}
			break;
			}

		// Итоговые значения для коррекции // Final values for correction
		Offset = ((uint64_t)ptrMs5xxx->prom[2] << 16) + (((int64_t)ptrMs5xxx->prom[4] * dT) >> 7) - OFF2;
		Sensitivity = ((int64_t)ptrMs5xxx->prom[1] << 15) + ((((int64_t)ptrMs5xxx->prom[3] * dT)) >> 8) - Sens2;

		// выгрузка скорректированной температуры*100 // save corrected temperature*100
		if(ptrTempC){
			*ptrTempC = TEMP - T2;
			}

		// выгрузка скорректированного давления*100 / save corrected pressure*100
		if(ptrMbar){
			*ptrMbar = (((((int64_t)D1 * Sensitivity) >> 21) - Offset) >> 15);
			}
	return 1;
}
