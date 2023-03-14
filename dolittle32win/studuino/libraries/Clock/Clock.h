#ifndef __CLOCK_H__
#define __CLOCK_H__

#include <Arduino.h>

//#define SS_pin SS     // 10
#define SCK_pin SCK   // 13
#define MISO_pin MISO // 12
#define MOSI_pin MOSI // 11
#define BZR_pin  8

#define STUDUINO_MINI 1
#ifdef STUDUINO_MINI

// Delay time after writeCommand.
#define DELAY1 10000       // > 5000 * 1.5
// Delay times for write.
#define DELAY_WRITE1 300  // Delay_2US(100);
#define DELAY_WRITE2 90   // Delay_2US(30);
#define DELAY_WRITE3 6000 // Delay_2US(2000);
#define DELAY_WRITE4 300  // Delay_2US(100);
// Delay times for read.
#define DELAY_READ1 300   // Delay_2US(100);
#define DELAY_READ2 6000  // Delay_2US(2000);
#define DELAY_READ3 150   // Delay_2US(50);
#define DELAY_READ4 180   // Delay_2us(60);
#define DELAY_READ5 240   // Delay_2US(80);
#define DELAY_READ6 600   // Delay_2US(200);

#else

// Delay time after writeCommand.
#define DELAY1 6000       // > 5000 * 1.5
// Delay times for write.
#define DELAY_WRITE1 200  // Delay_2US(100);
#define DELAY_WRITE2 60   // Delay_2US(30);
#define DELAY_WRITE3 4000 // Delay_2US(2000);
#define DELAY_WRITE4 200  // Delay_2US(100);
// Delay times for read.
#define DELAY_READ1 200   // Delay_2US(100);
#define DELAY_READ2 4000  // Delay_2US(2000);
#define DELAY_READ3 100   // Delay_2US(50);
#define DELAY_READ4 120   // Delay_2us(60);
#define DELAY_READ5 160   // Delay_2US(80);
#define DELAY_READ6 400   // Delay_2US(200);

#endif

#define KEYTONE_ENABLE 0x10
#define ALARM_FNCT_ENABLE 0x20

#define T2CYCLE (256 * 1024UL / (F_CPU / 1000))   // 21.3ms@12MHz

typedef struct {
  byte hour;
  byte minute;
  byte year;
  byte month;
  byte day;
  int temperature;
  byte mode;
  byte alhour;
  byte alminute;
}clockparams_t;
extern volatile clockparams_t clockparams;

void updateTest();
void writeCommand(byte* command, byte length);
byte writeByteData(byte _send);
byte readByteData();
void updateAll();
void updateAlarm();
void updateDate();
void updateTemperature();
void updateMode();


#define REGET_TH 1000
#define UPD_INTVAL_TIME  1000
#define UPD_INTVAL_DATE  1000
#define UPD_INTVAL_ALARM 1000
#define UPD_INTVAL_TEMP  1000
#define UPD_INTVAL_MODE  200
#define SNOOZE_TIME 5  // in minute

class Clock {
private:
public:
  Clock();
  void initClockPinSet(void);
  void setTime(byte hour, byte minute);
  void setDate(byte year, byte month, byte day);
  void setTimeTempMode(byte mode);
  void setAlarm(byte hour, byte minute);
  void setBackLight(boolean on, byte red, byte green, byte blue);
  byte getHour();
  byte getMinute();
  unsigned int getYear();
  byte getMonth();
  byte getDay();
  byte getMode();
  byte getTimeMode();
  byte getTempMode();
  byte getAlarmMode();
  byte getPushed();
  byte getAlarmHour();
  byte getAlarmMinute();
  int getTemperature();
  boolean isAlarmTime();
  void buzzer(boolean on, unsigned int frequency);
  void buzzer(unsigned int frequency, unsigned long duration);
};


#endif // __CLOCK_H__
