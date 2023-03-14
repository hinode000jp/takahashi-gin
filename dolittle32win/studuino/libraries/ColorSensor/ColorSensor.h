#ifndef _COLORSENSOR_H_
#define _COLORSENSOR_H_

#define COLOR_SENSOR_ADDR  0x39//the I2C address for the color sensor 
#define REG_CTL 0x80
#define REG_TIMING 0x81
#define REG_INT 0x82
#define REG_INT_SOURCE 0x83
#define REG_ID 0x84
#define REG_GAIN 0x87
#define REG_LOW_THRESH_LOW_BYTE 0x88
#define REG_LOW_THRESH_HIGH_BYTE 0x89
#define REG_HIGH_THRESH_LOW_BYTE 0x8A
#define REG_HIGH_THRESH_HIGH_BYTE 0x8B
#define REG_BLOCK_READ 0xCF
#define REG_GREEN_LOW 0xD0
#define REG_GREEN_HIGH 0xD1
#define REG_RED_LOW 0xD2
#define REG_RED_HIGH 0xD3
#define REG_BLUE_LOW 0xD4
#define REG_BLUE_HIGH 0xD5
#define REG_CLEAR_LOW 0xD6
#define REG_CLEAR_HIGH 0xD7
#define CTL_DAT_INIITIATE 0x03
#define CLR_INT 0xE0
//Timing Register
#define SYNC_EDGE 0x40
#define INTEG_MODE_FREE 0x00
#define INTEG_MODE_MANUAL 0x10
#define INTEG_MODE_SYN_SINGLE 0x20
#define INTEG_MODE_SYN_MULTI 0x30
 
#define INTEG_PARAM_PULSE_COUNT1 0x00
#define INTEG_PARAM_PULSE_COUNT2 0x01
#define INTEG_PARAM_PULSE_COUNT4 0x02
#define INTEG_PARAM_PULSE_COUNT8 0x03
//Interrupt Control Register 
#define INTR_STOP 40
#define INTR_DISABLE 0x00
#define INTR_LEVEL 0x10
#define INTR_PERSIST_EVERY 0x00
#define INTR_PERSIST_SINGLE 0x01
//Interrupt Souce Register
#define INT_SOURCE_GREEN 0x00
#define INT_SOURCE_RED 0x01
#define INT_SOURCE_BLUE 0x10
#define INT_SOURCE_CLEAR 0x03
//Gain Register
#define GAIN_1 0x00
#define GAIN_4 0x10
#define GAIN_16 0x20
#define GANI_64 0x30
#define PRESCALER_1 0x00
#define PRESCALER_2 0x01
#define PRESCALER_4 0x02
#define PRESCALER_8 0x03
#define PRESCALER_16 0x04
#define PRESCALER_32 0x05
#define PRESCALER_64 0x06

// Color code corresponding to Studuino ICON Programming.
#define COLOR_UNDEF 0
#define COLOR_RED 1
#define COLOR_GREEN 2
#define COLOR_BLUE 3
#define COLOR_WHITE 4
#define COLOR_YELLOW 5
#define COLOR_BROWN 6
#define COLOR_BLACK 7
// Color Value Type
#define VALUE_RED 0
#define VALUE_GREEN 1
#define VALUE_BLUE 2
#define VALUE_CLEAR 3

// Definition of color sensor's red value for each block.
#define MIN_R_RED 1000
#define MAX_R_RED 2400
#define MIN_R_BLUE 200
#define MAX_R_BLUE 550
#define MIN_R_GREEN 200
#define MAX_R_GREEN 800
#define MIN_R_YELLOW 1500
#define MAX_R_YELLOW 4400
#define MIN_R_BROWN 700
#define MAX_R_BROWN 1900
#define MIN_R_WHITE 1700
#define MAX_R_WHITE 4900
#define MIN_R_BLACK 200
#define MAX_R_BLACK 300

// Definition of color sensor's green value for each block.
#define MIN_G_RED 700
#define MAX_G_RED 1300
#define MIN_G_BLUE 700
#define MAX_G_BLUE 1300
#define MIN_G_GREEN 1200
#define MAX_G_GREEN 2600
#define MIN_G_YELLOW 2200
#define MAX_G_YELLOW 6000
#define MIN_G_BROWN 1000
#define MAX_G_BROWN 1900
#define MIN_G_WHITE 2800
#define MAX_G_WHITE 7700
#define MIN_G_BLACK 500
#define MAX_G_BLACK 800

// Definition of color sensor's blue value for each block.
#define MIN_B_RED 200
#define MAX_B_RED 800
#define MIN_B_BLUE 700
#define MAX_B_BLUE 1600
#define MIN_B_GREEN 500
#define MAX_B_GREEN 1100
#define MIN_B_YELLOW 700
#define MAX_B_YELLOW 1300
#define MIN_B_BROWN 500
#define MAX_B_BROWN 800
#define MIN_B_WHITE 1700
#define MAX_B_WHITE 4700
#define MIN_B_BLACK 200
#define MAX_B_BLACK 550

// Definition of color sensor's clear value for each block.
#define MIN_C_RED 2200
#define MAX_C_RED 4400
#define MIN_C_BLUE 2000
#define MAX_C_BLUE 3650
#define MIN_C_GREEN 2500
#define MAX_C_GREEN 4900
#define MIN_C_YELLOW 4800
#define MAX_C_YELLOW 12100
#define MIN_C_BROWN 2200
#define MAX_C_BROWN 4700
#define MIN_C_WHITE 6600
#define MAX_C_WHITE 19000
#define MIN_C_BLACK 1000
#define MAX_C_BLACK 1800

#define MIN_X_RED 0.44


#include <Arduino.h>

class COLOR {
  private:
    int readingdata[8];
    unsigned int green, red, blue, clr;
    double X, Y, Z, x, y, z;
    void updateValue();
  public:
    COLOR();
    bool testConnection();
    void initialize(void);
    void update(void);
    int getColorCode(void);
    unsigned int getColorRed();
    unsigned int getColorGreen();
    unsigned int getColorBlue();
    unsigned int getColorClear();
    double getX(void);
    double getY(void);
};
#endif // _COLORSENSOR_H_

