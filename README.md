# Adafruit_ILI9341_8bit_STM
8 bit parallel library for ILI9341 and STM32F1

##How to use
###8 bit parallel interface

//Port data |D7 |D6 |D5 |D4 |D3 |D2 |D1 |D0 |
//Pin stm32 |PA7|PA6|PA5|PA4|PA3|PA2|PC1|PA0|

### Control Pins

//Control pins |RD |WR |RS |CS |RST|
//Pin stm32    |PB4|PB5|PB6|PB7|PB8|

Control pins can be modified in Adafruit_ILI9341_8bit_STM.h header file

If you want to change data bus pins, you will have to chnge the function called write8() accordingly
