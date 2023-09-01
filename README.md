This code demonstrates the usage of two sensors (MS5611 and MS5803_01BA) along with ESP32 (ESP-IDF) in the FreeRTOS environment.

The sample code (main.c) shows how to create a driver for either or both of the sensors, read temperature and pressure values and delete the driver(s). The two sensors are wired to the same I2C bus on my breadboard and use default addresses (MS5611 = 0x77, MS5803 = 0x76).

* * *

INFO: This code is for those who:
- are really pissed off by the tons of ugly arduino garbage-code and garbage-coders
- can read and understand datasheets
- code native ESP-IDF FreeRTOS applications

NOTE: There's a range of MS5803_xx sensors with different pressure ranges. This code uses correction coefficients applicable for the mentioned sensors only! You can run this code with other sensors provided that you modify correction parameters in accordance with the datasheet for your sensor. The algorythm remains the same. Refer to the code in "ms5xxx_getSensorData()" function.

* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
                      CAUTION: Beware arduino garbage-coders
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
