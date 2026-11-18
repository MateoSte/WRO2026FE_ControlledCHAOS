#ifndef FE_INCLUDES
#define FE_INCLUDES
#include <Arduino.h>

const int PIN_PWMA = 4, PIN_AIN1 = 5, PIN_AIN2 = 6, PIN_STBY = 7;
const int PIN_ENC_A = 15, PIN_ENC_B = 16;
const int PIN_SRV_STEER = 17, PIN_SRV_TOWER = 11;
const int PIN_BTN = 12;
const int PIN_VBAT = 1;
const int PIN_I2C_SDA = -1, PIN_I2C_SCL = -1;   // fill in the numbers from phase C, step 3
const uint8_t MUX_ADDR = 0x70;

#endif