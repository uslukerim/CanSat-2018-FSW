# CanSat-2018-FSW
We use Arduino Nano 328p 5V 16MHz.
The I2C address of the MPU9255 and the EEPROM I2C address of the RTC we use are the same.
That way we can't use our sensors.
Therefore, the AD0 pin specified in the datasheet of mpu is 3.3 volts.
Fixed the problem by changing MPU_adress  to 0x69 in the MPU9255.h header file.
---------------------------------------------------------------------------------------
GOOD LUCK ...
