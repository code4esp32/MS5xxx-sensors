# MS5803_01BA

Sample working code for MS5803_01BA sensor to run on ESP32 under ESP-IDF

DISCLAIMER: This code is for those who:
- are really pissed off by the tons of ugly arduino garbage-code
- can read and understand datasheets
- code native ESP-IDF FreeRTOS applications

NOTE: There's a range of MS5803_xx sensors with different pressure ranges. This code uses correction coefficients applicable for _01BA series only!
You can run this code with other sensors provided that you modify correction parameters in accordance with the datasheet for your sensor.
the algorythm remains the same.

PS. Moreover, I has a short look over MS5611 datasheet and it's likely the same algorythm applies as well.

Feel free to use in any way whatsoever. 

Just add the header file to your project (#include "ms5803.h") and call the following functions:

- bool ms5803_Init(i2c_port_t, uint8_t, int16_t, int16_t, uint32_t) - set up the hardware and configure the software
- bool ms5803_Reset(void) - reset the sensor on power-up
- bool ms5803_getSensors(uint8_t, float *, float*) - get pressure and temperature (with final correction done)


* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
                      CAUTION: Beware arduino garbage-coders
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
