// ColorSensor Library for Studuino
//
#include "ColorSensor.h"
#include <Wire.h>
#include <Arduino.h>

COLOR::COLOR() {
}

bool COLOR::testConnection() {
  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(REG_ID);
  Wire.endTransmission();  

  Wire.requestFrom(COLOR_SENSOR_ADDR, 1);
  if(Wire.available()) {
    byte msg = Wire.read();
    // ID register contains PARTNO(upper 4bits) and REVNO(lower 4bits).
    // TCS3410 and TCS341x's Part No. are 0000 and 0001, respectively.
    return ((msg & 0xf0) >> 4 == 0x01);   
  }
  return false;
}

void COLOR::initialize() {
  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(REG_TIMING);
  Wire.write(INTEG_MODE_FREE);
  Wire.endTransmission();  

  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(REG_INT_SOURCE);
  Wire.write(INT_SOURCE_GREEN);
  Wire.endTransmission();  

  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(REG_INT);
  Wire.write(INTR_LEVEL|INTR_PERSIST_EVERY);
  Wire.endTransmission();  

  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(REG_GAIN);
  Wire.write(GAIN_1|PRESCALER_4);
  Wire.endTransmission();
}

void COLOR::update() {
  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(REG_CTL);
  Wire.write(CTL_DAT_INIITIATE);
  Wire.endTransmission();  

  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(REG_BLOCK_READ);
  Wire.endTransmission();

  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.requestFrom(COLOR_SENSOR_ADDR,8);
  if(8<= Wire.available())  { 
    for(byte i = 0; i < 8; i++)
    {
      readingdata[i]=Wire.read();
      //Serial.println(readingdata[i],BIN);
    }
  }
  green = readingdata[1] * 256 + readingdata[0];
  red = readingdata[3] * 256 + readingdata[2];
  blue = readingdata[5] * 256 + readingdata[4];
  clr = readingdata[7] * 256 + readingdata[6];

  X = (-0.14282) * red + (1.54924) * green + (-0.95641) * blue;
  Y = (-0.32466) * red + (1.57837) * green + (-0.73191) * blue;
  Z = (-0.68202) * red + (0.77073) * green + (0.56332) * blue;
  x = X / (X + Y + Z);
  y = Y / (X + Y + Z);

  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(CLR_INT);
  Wire.endTransmission();
}

// Getting the color code corresponding to Studuino ICON Programming.
int COLOR::getColorCode() {
  byte val = COLOR_UNDEF;

  if (red > MIN_R_WHITE && red < MAX_R_WHITE && green > MIN_G_WHITE && green < MAX_G_WHITE &&
      blue > MIN_B_WHITE && blue < MAX_B_WHITE && clr > MIN_C_WHITE && clr < MAX_C_WHITE) { // White
    val = COLOR_WHITE;
  } else if (red > MIN_R_BLACK && red < MAX_R_BLACK && green > MIN_G_BLACK && green < MAX_G_BLACK &&
             blue > MIN_B_BLACK && blue < MAX_B_BLACK && clr > MIN_C_BLACK && clr < MAX_C_BLACK) { // Black
    val = COLOR_BLACK;
  } else if (red > MIN_R_YELLOW && red < MAX_R_YELLOW && green > MIN_G_YELLOW && green < MAX_G_YELLOW &&
             blue > MIN_B_YELLOW && blue < MAX_B_YELLOW && clr > MIN_C_YELLOW && clr < MAX_C_YELLOW) { // Yellow
    val = COLOR_YELLOW;
  } else if (red > MIN_R_RED && red < MAX_R_RED && green > MIN_G_RED && green < MAX_G_RED &&
             blue > MIN_B_RED && blue < MAX_B_RED && clr > MIN_C_RED && clr < MAX_C_RED && X > MIN_X_RED) { // Red
    val = COLOR_RED;
    if (red < 1350 && clr > 3000) { // Brown
      val = COLOR_BROWN;
    }
  } else if (red > MIN_R_BROWN && red < MAX_R_BROWN && green > MIN_G_BROWN && green < MAX_G_BROWN &&
             blue > MIN_B_BROWN && blue < MAX_B_BROWN && clr > MIN_C_BROWN && clr < MAX_C_BROWN) { // Brown
    val = COLOR_BROWN;
  } else if (red > MIN_R_GREEN && red < MAX_R_GREEN && green > MIN_G_GREEN && green < MAX_G_GREEN &&
             blue > MIN_B_GREEN && blue < MAX_B_GREEN && clr > MIN_C_GREEN && clr < MAX_C_GREEN) { // Green
    val = COLOR_GREEN;
  } else if (red > MIN_R_BLUE && red < MAX_R_BLUE && green > MIN_G_BLUE && green < MAX_G_BLUE &&
             blue > MIN_B_BLUE && blue < MAX_B_BLUE && clr > MIN_C_BLUE && clr < MAX_C_BLUE) { // Blue
    val = COLOR_BLUE;
  }

  return val;
}

// Getting sensor value of each filter;
unsigned int COLOR::getColorRed() {
  return red;
}
unsigned int COLOR::getColorGreen() {
  return green;
}
unsigned int COLOR::getColorBlue() {
  return blue;
}
unsigned int COLOR::getColorClear() {
  return clr;
}

// Getting x value of chromaticity coordinate(x, y).
double COLOR::getX() {
  return x;
}

// Getting y value of chromaticity coordinate(x, y).
double COLOR::getY() {
  return y;
}

