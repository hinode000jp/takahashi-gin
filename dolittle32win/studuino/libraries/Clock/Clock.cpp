/********************************************************
 * Clock Library for Studuino mini
 * 2016/3/29  ArTec Co.,Ltd.
 * Software SPI
 *
 *
 ********************************************************/
#include "Clock.h"

volatile clockparams_t clockparams;
byte rFirst;
unsigned long lastUpdateTime;
unsigned long lastUpdateDate;
unsigned long lastUpdateAlarm;
unsigned long lastUpdateMode;
unsigned long lastUpdateTemp;
boolean fSending;
byte alarmState;      // 0:off 1:on 2:snooze
boolean fOnAlarm;     // false:off true:on
boolean pushMonitor;
boolean fBackLightOff;
unsigned long lastPushTime;
unsigned long t2counter;


//割り込み関数
// Pushボタン、Alarmボタンの監視をする
ISR(TIMER1_COMPB_vect)
{
  if(pushMonitor) {
    byte pushed = (clockparams.mode >> 6) & 0x01;
    byte almode = (clockparams.mode >> 2) & 0x03;
  
    if(pushed == 1) {
      if(alarmState == 1) {
        alarmState = 0;
        if(fOnAlarm) {
          noTone(8);
	  fBackLightOff = true;
          fOnAlarm = false;
  	  pushMonitor = false;
          lastPushTime = clockparams.hour * 60 + clockparams.minute;
        }
      }
    }
  
    if(almode == 0) {
      alarmState = 0;
      if(fOnAlarm) {
        noTone(8);
	fBackLightOff = true;
        fOnAlarm = false;
  	pushMonitor = false;
        lastPushTime = clockparams.hour * 60 + clockparams.minute;
      }
    }
  } else {
    if(fBackLightOff) {
      byte data[3];
      data[0] = 0xa5;
      data[1] = 0;
      data[2] = 0;
      writeCommand(data, 3);
      fBackLightOff = false;
    }
  }
  if(lastPushTime != (clockparams.hour * 60 + clockparams.minute)) alarmState = 1;
}

ISR(TIMER2_OVF_vect)
{
	t2counter++;
}

void writeCommand(byte* command, byte length) {
  fSending = true;
  rFirst = 1;
  for(byte i = 0; i < length; i++) {
    writeByteData(command[i]);
  }
  delayMicroseconds(DELAY1);
  fSending = false;
}

byte writeByteData(byte _send) {  // This function is what bitbangs the data
  byte Read_Write_Count = 0;
  
  for(Read_Write_Count = 0; Read_Write_Count < 8; Read_Write_Count++) { // There are 8 bits in a byte
    PORTB |= B00100000;                // SCK high
    delayMicroseconds(DELAY_WRITE1);   // Delay_2US(100);
    if(_send & 0x80) {
      PORTB |= B00001000;              // MOSI high
    } else {
      PORTB &= B11110111;              // MOSI low
    }
    delayMicroseconds(DELAY_WRITE2);   // Delay_2US(30);
    _send = (_send << 1);
    PORTB &= B11011111;                // SCK low
    if(rFirst) {
      rFirst = 0;
      delayMicroseconds(DELAY_WRITE3); // Delay_2US(2000);
    } else {
      delayMicroseconds(DELAY_WRITE4); // Delay_2US(100);
    }
  }

  PORTB |= B00101000;        // MOSI, SCK high
  return _send;        // Return the received data
}

byte readByteData() {  // This function is what bitbangs the data
  byte Read_Write_Count = 0;
  byte _receive = 0;
  rFirst = 1;
  
  for(Read_Write_Count = 0; Read_Write_Count < 8; Read_Write_Count++) { // There are 8 bits in a byte
    PORTB |= B00100000;             // SCK high
    delayMicroseconds(DELAY_READ1); // Delay_2US(100);
    PORTB &= B11011111;             // SCK low

    if(rFirst) {
      rFirst = 0;
      delayMicroseconds(DELAY_READ2); // Delay_2US(2000);
    } else {
      delayMicroseconds(DELAY_READ3); // Delay_2US(50);
    }
    _receive = _receive << 1;
    if(digitalRead(MISO_pin))
	    _receive |= 0x01;
    delayMicroseconds(DELAY_READ4);   // Delay_2us(60);
    PORTB |= B00100000;               // SCK high
    delayMicroseconds(DELAY_READ5);   // Delay_2US(80);
  }

  delayMicroseconds(DELAY_READ6);     // Delay_2US(200);
  PORTB |= B00100000; // SCK high
  return _receive;        // Return the received data
}

// update functions
void updateTime() { // 0: hour  1: minute
  if(!fSending) {
    if(millis() - lastUpdateTime + T2CYCLE * t2counter > UPD_INTVAL_TIME) {
      rFirst = 1;
      writeByteData(0xb1);
      byte buf = readByteData();
      clockparams.hour = (buf / 16) * 10 + (buf % 16);
      buf = readByteData();
      clockparams.minute = (buf / 16) * 10 + (buf % 16);
      lastUpdateTime = millis() + T2CYCLE * t2counter;
    }
  }
}

void updateDate() {
  if(!fSending) {
    if(millis() - lastUpdateDate + T2CYCLE * t2counter > UPD_INTVAL_DATE) {
      rFirst = 1;
      writeByteData(0xb2);
      byte buf = readByteData();
      clockparams.year = 2000 + (buf / 16) * 10 + (buf % 16);
      buf = readByteData();
      clockparams.month = (buf / 16) * 10 + (buf % 16);
      buf = readByteData();
      clockparams.day = (buf / 16) * 10 + (buf % 16);
      lastUpdateDate = millis() + T2CYCLE * t2counter;
    }
  }
}

void updateMode() {
  if(!fSending) {
    if(millis() - lastUpdateMode + T2CYCLE * t2counter > UPD_INTVAL_MODE) {
      rFirst = 1;
      writeByteData(0xb3);
      clockparams.mode = readByteData();
      lastUpdateMode = millis() + T2CYCLE * t2counter;
    }
  }
}

void updateAlarm() {
  if(!fSending) {
    if(millis() - lastUpdateAlarm + T2CYCLE * t2counter > UPD_INTVAL_ALARM) {
      rFirst = 1;
      writeByteData(0xb4);
      byte buf = readByteData();
      clockparams.alhour = (buf / 16) * 10 + buf % 16;
      buf = readByteData();
      clockparams.alminute = (buf / 16) * 10 + buf % 16;
      lastUpdateAlarm = millis() + T2CYCLE * t2counter;
    }
  }
}

void updateTemperature() {
  if(!fSending) {
    if(millis() - lastUpdateTemp + T2CYCLE * t2counter > UPD_INTVAL_TEMP) {
      rFirst = 1;
      writeByteData(0xb5);
      byte data[2];
      data[0] = readByteData();
      data[1] = readByteData();
      byte var = data[0] / 16;
      clockparams.temperature = (data[0] % 16) * 100 + (data[1] / 16) * 10 + (data[1] % 16);
      if(var != 0)
        clockparams.temperature = 1000 - clockparams.temperature;
      lastUpdateTemp = millis() + T2CYCLE * t2counter;
    }
  }
}

void updateAll() {
  fSending = true;
  updateTime();
  delayMicroseconds(5000);
  updateDate();
  delayMicroseconds(5000);
  updateAlarm();
  delayMicroseconds(5000);
  updateMode();
  delayMicroseconds(5000);
  fSending = false;
}

/*****    Clock class function definitions    *****/
Clock::Clock() {
}

// 液晶時計に使用するピンを初期化し、SCK_pinをHIGHに設定する
void Clock::initClockPinSet() {
  pinMode(SCK_pin, OUTPUT);
  pinMode(MISO_pin, INPUT);
  pinMode(MOSI_pin, OUTPUT);
  pinMode(BZR_pin, OUTPUT);

  digitalWrite(SCK_pin, HIGH);
  digitalWrite(MOSI_pin, HIGH);

  // タイマー1コンペアB割り込みを使用する。
  // Servoライブラリが使用するコンペアAの設定と合わせる。
  TCCR1A = 0b00000000;
  TCCR1B = 0b00000010;
  OCR1BH = 0x00;
  OCR1BL = 0xff;
  TIMSK1 |= 0b00000100;

  // タイマー2オーバーフロー割り込みを使用する
  TCCR2A = 0;
  TCCR2B = 7;  // 分周比 1/1024
  

  delay(2000);
  
  TIMSK2 = 1;
  alarmState = 1;
}

/* :::::::::::::: 設定用関数 ::::::::::::::::::::::::::: */
// 時間の設定
void Clock::setTime(byte hour, byte minute) {
  byte data[3];
  data[0] = 0xa1;
  data[1] = (hour / 10) * 16 + (hour % 10);
  data[2] = (minute / 10) * 16 + (minute % 10);
  writeCommand(data, 3);
}

// 日付の設定
void Clock::setDate(byte year, byte month, byte day) {
  byte data[4];
  data[0] = 0xa2;
  data[1] = (year / 10) * 16 + (year % 10);
  data[2] = (month / 10) * 16 + (month % 10);
  data[3] = (day / 10) * 16 + (day % 10);
  writeCommand(data, 4);
}

// 12h/24h, C/Fの設定
void Clock::setTimeTempMode(byte mode) {
  byte data[2];
  data[0] = 0xa3;
  data[1] = mode;
  writeCommand(data, 2);
}

// アラーム時間の設定
void Clock::setAlarm(byte hour, byte minute) {
  byte data[3];
  data[0] = 0xa4;
  data[1] = (hour / 10) * 16 + (hour % 10);
  data[2] = (minute / 10) * 16 + (minute % 10);
  writeCommand(data, 3);
}

// 液晶バックライトの設定
void Clock::setBackLight(boolean on, byte red, byte green, byte blue) {
  byte data[3];
  data[0] = 0xa5;
  data[1] = on * 16 + red;
  data[2] = blue * 16 + green;
  writeCommand(data, 3);
  delay(50);  // Only for this function.
}

/* :::::::::::::: データ取得用関数 ::::::::::::::::::::::::::: */
// 時間取得
byte Clock::getHour() {
  updateTime();
  return clockparams.hour;
}

byte Clock::getMinute() {
  updateTime();
  return clockparams.minute;
}

// 日付取得
unsigned int Clock::getYear() {
  updateDate();
  return clockparams.year;
}

byte Clock::getMonth() {
  updateDate();
  return clockparams.month;
}

byte Clock::getDay() {
  updateDate();
  return clockparams.day;
}

// モード取得
byte Clock::getMode() {
  updateMode();
  return clockparams.mode;
}

byte Clock::getTimeMode() {
 return getMode() % 2;
}

byte Clock::getTempMode() {
 return (getMode() >> 1) % 2;
}

byte Clock::getAlarmMode() {
 return (getMode() >> 2) & 0x03;
}

byte Clock::getPushed() {
 return (getMode() >> 6) & 0x01;
}

// アラーム時間取得
byte Clock::getAlarmHour() {
  updateAlarm();
  return clockparams.alhour;
}

byte Clock::getAlarmMinute() {
  updateAlarm();
  return clockparams.alminute;
}

// 温度取得
int Clock::getTemperature() {
  updateTemperature();
  return clockparams.temperature;
}

boolean Clock::isAlarmTime() {
  if(getAlarmMode() == 0) return false;
  int delta = (getHour() - getAlarmHour()) * 60 + (getMinute() - getAlarmMinute());
  if(getAlarmMode() == 1 && (delta != 0)) return false;
  if(getAlarmMode() == 2 && ((delta % SNOOZE_TIME) != 0)) return false;  // Snooze
  if(alarmState == 0) return false;

  //TIMSK1 |= 0b00000100;  //アラーム時間になった時のみ、Timer1でプッシュボタンを監視する
  pushMonitor = true;
  fOnAlarm = true;
  return true;
}

void Clock::buzzer(boolean on, unsigned int frequency) {
  if(on) {
    if(alarmState == 1) {
      updateMode();
      tone(BZR_pin, frequency);
    }
  } else {
    noTone(BZR_pin);
  }
}

void Clock::buzzer(unsigned int frequency, unsigned long duration) {
  if(alarmState == 1) {
    updateMode();
    tone(BZR_pin, frequency);
    delay(duration);
    noTone(BZR_pin);
  }
}

