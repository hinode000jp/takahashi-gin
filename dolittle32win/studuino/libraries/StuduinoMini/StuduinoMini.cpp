/*
 * STUDUINO library
 * (C) 2014 Artec Corporation
 *
 * This program is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 */
#include "wiring_private.h"
#include <avr/sleep.h>
#include "StuduinoMini.h"
// -------------------------------------------------------
// Macro
// -------------------------------------------------------
// Power calibration when moving backward. (For Studuino mini)
#define DCBackCalib(power)  (-0.00212 * (power) * (power) + 1.5206 * (power) - 1.146)
#define DCBack(power)  (max((power), (DCBackCalib(power))))
// -------------------------------------------------------
// constant definition
// -------------------------------------------------------

// -------------------------------------------------------
// Global variable
// -------------------------------------------------------


// -------------------------------------------------------
// Constructor
// -------------------------------------------------------
Studuino::Studuino()
{
  DCMDA1   = 2;  // DC motor driver A1
  DCMDA2   = 4;  // DC motor driver A2
  DDMDPWMA = 3;  // DC motor driver PRM A
  DCMDB1   = 7;  // DC motor driver B1
  DCMDB2   = 8;  // DC motor driver B2
  DDMDPWMB = 5;  // DC motor driver PRM B

  DCMCRTA = 100;
  DCMCRTB = 100;
  SVSIZE = 8;
  //SVOFF = new byte[SVSIZE];
  //servomotors = new Servo[SVSIZE];
  //SVMPORT = new byte[SVSIZE];
  SVMPORT[0] = 9;
  SVMPORT[1] = 10;
  SVMPORT[2] = 11;
  SVMPORT[3] = 12;
  SVMPORT[4] = 2;
  SVMPORT[5] = 4;
  SVMPORT[6] = 7;
  SVMPORT[7] = 8;
}

// -------------------------------------------------------
// Initialize DC motor port
// Argument   : byte connector       A port ID which is connected DC motor
// -------------------------------------------------------
void Studuino::InitDCMotorPort(byte connector)
{
  if (connector == PORT_M1) {
    // Initialize conflict pins
    servomotors[4].detach();    
    servomotors[5].detach();    
    // Setup DC motor pins
    pinMode(DCMDA1, OUTPUT);
    pinMode(DCMDA2, OUTPUT);
    digitalWrite(DCMDA1, LOW);
    digitalWrite(DCMDA2, LOW);
  } else {
    // Initialize conflict pins
    servomotors[6].detach();    
    servomotors[7].detach();    
    
    // Setup DC motor pins
    pinMode(DCMDB1, OUTPUT);
    pinMode(DCMDB2, OUTPUT);
    digitalWrite(DCMDB1, LOW);
    digitalWrite(DCMDB2, LOW);
  }    
}

// -------------------------------------------------------
// Initialize servomotor port
// Argument   : byte connector       A port ID which is connected servomotor
// -------------------------------------------------------
void Studuino::InitServomotorPort(byte connector)
{
  int index = svPortToIndex(connector);
  servomotors[index].attach(connector, 500, 2500);
}

// -------------------------------------------------------
// Initialize servomotor port for LED
// Argument   : byte connector       A port ID which is connected LED
// -------------------------------------------------------
void Studuino::InitServomotorPortForLED(byte connector)
{
  int index = svPortToIndex(connector);
  // If servomotor is attatched to target port, detach servomotor.
  if (servomotors[index].attached())  servomotors[index].detach();
  
}

void Studuino::InitSensorPort(byte connector, byte pid)
{
  switch (pid) {
    case PIDOPEN:           // Not connect
      // Do nothing
    break;
    case PIDLED:            // LED
    case PIDBUZZER:         // Buzzer
      // Digital output parts
      pinMode(connector, OUTPUT);
    break;
    case PIDLIGHTSENSOR:       // Light sensor
    case PIDSOUNDSENSOR:       // Sound sensor
    case PIDIRPHOTOREFLECTOR:  // IR Photorefrector
      // Analog input parts 
    break;
    case PIDACCELEROMETER:  // Accelerometer
      // Setup I2C input
      Wire.begin();
      accel.begin(false, 2);
    break;
    case PIDTOUCHSENSOR:    // Touch sensor
    case PIDPUSHSWITCH:     // Push switch
      // Digital input parts
      pinMode(connector, INPUT_PULLUP);
    break;
  }
}

// -------------------------------------------------------
// Set DCMotor calibration rate
// Argument   : byte rateA             Clibration % for DC Motor M1 
//            : byte rateB             Clibration % for DC Motor M2
// -------------------------------------------------------
void Studuino::SetDCMotorCalibration(byte* rate)
{
  DCMCRTA = rate[0];
  DCMCRTB = rate[1];
}

// -------------------------------------------------------
// Set Servomotor offset angles
// Argument   : byte offset            Offset angles of servomotor
// -------------------------------------------------------
void Studuino::SetServomotorCalibration(char* offset)
{
  SVOFF[0] = offset[4]; 
  SVOFF[1] = offset[5]; 
  SVOFF[2] = offset[6]; 
  SVOFF[3] = offset[7]; 
  SVOFF[4] = offset[0]; 
  SVOFF[5] = offset[1]; 
  SVOFF[6] = offset[2]; 
  SVOFF[7] = offset[3]; 
}

// -------------------------------------------------------
// Move motorcar
// Argument   : byte direct             Direction of movement
//            : byte pace               Speed of motorcar
//            : byte duration           Time
//            : byte brake              Way of stopping motorcar
// -------------------------------------------------------
void Studuino::Move(byte direct, byte pace, unsigned long duration, byte brake)
{
  // Set speed.
  DCMotorPower(PORT_M1, pace);
  DCMotorPower(PORT_M2, pace);

  // Set direction and go.
  switch (direct) {
    case FORWARD:  // Forward
      DCMotorControl(PORT_M1, NORMAL);
      DCMotorControl(PORT_M2, NORMAL);
    break;
    case BACKWARD:  // Backward
      DCMotorControl(PORT_M1, REVERSE);
      DCMotorControl(PORT_M2, REVERSE);
    break;
    case FORWARD_RIGHT:  // Turn right (forward)
      DCMotorControl(PORT_M1, BRAKE);
      DCMotorControl(PORT_M2, NORMAL);
    break;
    case FORWARD_LEFT:  // Turn left (forward)
      DCMotorControl(PORT_M1, NORMAL);
      DCMotorControl(PORT_M2, BRAKE);
    break;
    case BACKWARD_RIGHT:  // Turn right (backward)
      DCMotorControl(PORT_M1, BRAKE);
      DCMotorControl(PORT_M2, REVERSE);
    break;
    case BACKWARD_LEFT:  // Turn left (backward)
      DCMotorControl(PORT_M1, REVERSE);
      DCMotorControl(PORT_M2, BRAKE);
    break;
    case CLOCKWISE:   // Clockwise
      DCMotorControl(PORT_M1, REVERSE);
      DCMotorControl(PORT_M2, NORMAL);
    break;
    case COUNTERCLOCKWISE:  // Counter clockwise
      DCMotorControl(PORT_M1, NORMAL);
      DCMotorControl(PORT_M2, REVERSE);
    break;
  }
  delay(duration);
  DCMotorControl(PORT_M1, brake);
  DCMotorControl(PORT_M2, brake);
}

// -------------------------------------------------------
// Outline    : Control DC motor
// Argument   : byte connector       A port ID which is connected DC motor
//            : byte rotation        Rotation of DC motor
//            : byte pace            Speed of DC motor rotation
//            : byte duration        DC motor rotation times
//            : byte brake           Way of stopping DC motor
// -------------------------------------------------------
void Studuino::DCMotor(byte connector, byte rotation, byte pace, unsigned long duration, byte brake)
{
  DCMotorPower(connector, pace);
  DCMotorControl(connector, rotation);
  delay(duration);
  DCMotorControl(connector, brake);  
}

// -------------------------------------------------------
// Outline    : Set DC motor power
// Argument   : byte connector       A port ID which is connected DC motor
//            : byte pace            Speed
// -------------------------------------------------------
void Studuino::DCMotorPower(byte connector, byte pace)
{
  if (connector == PORT_M1) analogWrite(DDMDPWMA, pace * DCMCRTA / 100);
  if (connector == PORT_M2) analogWrite(DDMDPWMB, pace * DCMCRTB / 100);
}

// -------------------------------------------------------
// Outline    : Set DC motor rotation
// Argument   : byte connector       A port ID which is connected DC motor
//            : byte rotation        Rotation
// -------------------------------------------------------
void Studuino::DCMotorControl(byte connector, byte rotation)
{
  // Set DC motor's rotation.
  if (connector == PORT_M1) {
    if (rotation == NORMAL) {
      digitalWrite(DCMDA1, HIGH);
      digitalWrite(DCMDA2, LOW);
    } else if (rotation == REVERSE) {
      digitalWrite(DCMDA1, LOW);
      digitalWrite(DCMDA2, HIGH);
    } else if (rotation == BRAKE) {
      digitalWrite(DCMDA1, HIGH);
      digitalWrite(DCMDA2, HIGH);
    } else if (rotation == COAST) {
      digitalWrite(DCMDA1, LOW);
      digitalWrite(DCMDA2, LOW);
    }
  } else if (connector == PORT_M2) {
    if (rotation == NORMAL) {
      digitalWrite(DCMDB1, HIGH);
      digitalWrite(DCMDB2, LOW);
    } else if (rotation == REVERSE) {
      digitalWrite(DCMDB1, LOW);
      digitalWrite(DCMDB2, HIGH);
    } else if (rotation == BRAKE) {
      digitalWrite(DCMDB1, HIGH);
      digitalWrite(DCMDB2, HIGH);
    } else if (rotation == COAST) {
      digitalWrite(DCMDB1, LOW);
      digitalWrite(DCMDB2, LOW);
    }
  }
}

// -------------------------------------------------------
// Outline    : Set servomotor angle
// Argument   : byte connector       A port ID which is connected servomotor
//            : byte degree          Angle of the servomotor
// -------------------------------------------------------
void Studuino::Servomotor(byte connector, byte degree)
{
  int index = svPortToIndex(connector);
  byte calibedDegree = min(180, max(0, degree + SVOFF[index])); // Calibrating the given angle.
  servomotors[index].write(calibedDegree);  // Set angle for servomotor
  
}

// -------------------------------------------------------
// Outline    : Set some servomotors angles
// Argument   : byte connector       Port IDs which is connected servomotor
//            : byte degree          Angles of the servomotor
//            : byte number          Number of servomotors setting angle
// -------------------------------------------------------
void Studuino::AsyncServomotors(byte* connector, byte* degree, byte number)
{
  byte calibedDegree;
  // Set angles for each servomotor
  for (int i = 0; i < number; i++) {
    int index = svPortToIndex(connector[i]);
    calibedDegree = min(180, max(0, degree[i] + SVOFF[index])); // Calibrating the given angle.
    servomotors[index].write(calibedDegree);
    
  }
}

// -------------------------------------------------------
// Outline    : Set some servomotors angles(synchronized rotation) with delay time
// Argument   : byte connector       Port IDs which is connected servomotor
//            : byte degree          Angles of the servomotor
//            : byte number          Number of servomotors setting angle
//            : byte time            Delay time per 1 degree [ms]
// -------------------------------------------------------
void Studuino::SyncServomotors(byte* connector, byte* degree, byte number, byte time)
{
  byte before[8];        // Current angles of servomotor
  double delta[8];       // delta <- target angle - current angle
  byte maxDelta = 0;     // max of delta
  byte calibedDegree;    // Temporary value for calibrating angle.

  // If there are not servomotors setting angle, do nothing.
  if (number == 0) return;
  
  // Get maximum difference between current angle and target angle.
  for (int i = 0;i < number;i++) {
    int index = svPortToIndex(connector[i]);
    before[i] = servomotors[index].read();  // Current angle.
    
    calibedDegree = min(180, max(0, degree[i] + SVOFF[index])); // Calibrating the given angle.
    delta[i] = calibedDegree - before[i];              // Get difference.
    // Get maximum difference.
    maxDelta = (abs(delta[i]) > maxDelta) ? abs(delta[i]) : maxDelta;
  }
  
  // Set angles for each servomotor 
  if (time == 0) {  // If delay time is 0...
    // Set angles for each servomotor
    for (int i = 0;i < number;i++) {
      int index = svPortToIndex(connector[i]);
      calibedDegree = min(180, max(0, degree[i] + SVOFF[index])); // Calibrating the given angle.
      servomotors[index].write(calibedDegree);    
    }
    // Wait until all servomotors reach their target angles.
    delay(maxDelta * 3);    
  } else {          // If delay time is over 1...
    for (int i = 0;i < number;i++) {
      delta[i] = (double)(delta[i]) / (double)(maxDelta);
    }
    // Set angles for each servomotor
    for (int t = 1; t <= (int)maxDelta; t++) {
      for (int i = 0; i < number; i++) {
        int index = svPortToIndex(connector[i]);
        servomotors[index].write(before[i]+delta[i]*t);
      }
      delay(time);
    }
  }
}

// -------------------------------------------------------
// Outline    : Output Beep sound
// Argument   : byte connector          Port IDs which is connected buzzer
//            : word pitch              Code index
//            : unsigned long duration  Duration
// -------------------------------------------------------
void Studuino::Buzzer(byte connector, word pitch, unsigned long duration)
{
  tone(connector, pitch, duration);            // Output beep
  delay(duration);                        // Wait until finish beep
}

// -------------------------------------------------------
// Outline    : Control buzzer
// Argument   : byte connector       Port IDs which is connected buzzer
//            : boolean ononff       Buzzer ON/OFF
//            : word pitch           Code index
// -------------------------------------------------------
void Studuino::BuzzerControl(byte connector, boolean onoff, word pitch)
{
  if (onoff == ON) {
    tone(connector, pitch);   // Output beep
  } else {
    noTone(connector);        // Stop beep
  }
}

// -------------------------------------------------------
// Outline    : Output Melody
// Argument   : byte connector          Port IDs which is connected buzzer
//            : word* scales            Pitches
//            : float* notes            Notes
//            : byte number             Number of notes
//            : byte tempo              Tempo
// -------------------------------------------------------
void Studuino::Melody(byte connector, word* scales, float* notes, byte number, byte tempo)
{
  float beat = 60000.0 / TEMPO[tempo];  // 1拍を算出
  
  for (int i = 0;i < number;i++) {
    int duration = beat * notes[i];
  
    if (notes[i] >= 8) {
      delay(duration);
    }
    else
    {
      pinMode(connector, OUTPUT);
      tone(connector, scales[i], duration);
      delay(duration);
      noTone(connector);
    }
  }
}

// -------------------------------------------------------
// Outline    : LED ON/OFF
// Argument   : byte connector          Port IDs which is connected LED
//            : boolean onoff           ON/OFF switch
// -------------------------------------------------------
void Studuino::LED(byte connector, boolean onoff)
{
  if (onoff == ON) {
    digitalWrite(connector, HIGH);
  } else {
    digitalWrite(connector, LOW);
  }
}

// -------------------------------------------------------
// Outline    : Gradataion
// Argument   : byte connector          Port IDs which is connected LED
//            : byte ratio              Brightness
// -------------------------------------------------------
void Studuino::Gradation(byte connector, byte ratio)
{
  analogWrite(connector, ratio);
}

// -------------------------------------------------------
// Outline    : Delay
// Argument   : unsigned long time      Delay time
// -------------------------------------------------------
void Studuino::Timer(unsigned long time)
{
  delay(time);
}

// -------------------------------------------------------
// Outline    : Get push switch status
// Argument   : byte connector          Port IDs which is connected push switch
// -------------------------------------------------------
byte Studuino::GetPushSwitchValue(byte connector)
{
  return digitalRead(connector);
}

// -------------------------------------------------------
// Outline    : Get touch sensor status
// Argument   : byte connector          Port IDs which is connected touch sensor
// -------------------------------------------------------
byte Studuino::GetTouchSensorValue(byte connector)
{
  return digitalRead(connector);
}

// -------------------------------------------------------
// Outline    : Get light sensor status
// Argument   : byte connector          Port IDs which is connected light sensor
// -------------------------------------------------------
int Studuino::GetLightSensorValue(byte connector)
{
  return analogRead(connector);
}

// -------------------------------------------------------
// Outline    : Get sound sensor status
// Argument   : byte connector          Port IDs which is connected sound sensor
// -------------------------------------------------------
int Studuino::GetSoundSensorValue(byte connector)
{
  return analogRead(connector);
}

// -------------------------------------------------------
// Outline    : Get IR photoreflector status
// Argument   : byte connector          Port IDs which is connected IR photoreflector
// -------------------------------------------------------
int Studuino::GetIRPhotoreflectorValue(byte connector)
{
  return analogRead(connector);
}

// -------------------------------------------------------
// Outline    : Get Accelerometer status
// Argument   : byte axis               Port IDs which is connected accelerometer
// -------------------------------------------------------
int Studuino::GetAccelerometerValue(byte axis)
{
  int sv = 0;
  accel.update();
  if (axis == X_AXIS) {
    sv = accel.getX();
  } else if (axis == Y_AXIS) {
    sv = accel.getY();
  } else if (axis == Z_AXIS) {
    sv = accel.getZ();
  }
  return sv;
}

int Studuino::svPortToIndex(byte connector)
{
  for(byte i = 0; i < SVSIZE; i++) {
    if(connector == SVMPORT[i])
      return i;
  }
  return -1;
}

/****************************************************************
 * Studuino mini
 * 
 *
 ****************************************************************/

// -------------------------------------------------------
// Constructor
// -------------------------------------------------------
StuduinoMini::StuduinoMini()
{
  DCMDA1   = 5;  // DC motor driver A1
  DCMDA2   = 9;  // DC motor driver A2
  DCMDB1   = 6;  // DC motor driver B1
  DCMDB2   = 10;  // DC motor driver B2

  DCMCRTA = 100;
  DCMCRTB = 100;

  SVSIZE = 5;
  //SVOFF = new byte[SVSIZE];
  //servomotors = new Servo[SVSIZE];
  //SVMPORT = new byte[SVSIZE];
  SVMPORT[0] = 5;
  SVMPORT[1] = 6;
  SVMPORT[2] = 9;
  SVMPORT[3] = 10;
  SVMPORT[4] = 11;

  pinMode(PORT_D3, OUTPUT);
}

// -------------------------------------------------------
// Set Servomotor offset angles
// Argument   : byte offset            Offset angles of servomotor
// -------------------------------------------------------
void StuduinoMini::SetServomotorCalibration(char* offset)
{
  SVOFF[0] = offset[0]; 
  SVOFF[1] = offset[1]; 
  SVOFF[2] = offset[2]; 
  SVOFF[3] = offset[3]; 
  SVOFF[4] = offset[4]; 
}

// -------------------------------------------------------
// Initialize digital port
// Argument   : byte connector       A port ID which is connected parts
//            : byte pid             Parts ID
// -------------------------------------------------------
void StuduinoMini::InitDigitalPort(byte connector, byte pid) {
  InitSensorPort(connector, pid);
  if(pid == PIDLED) {
    digitalWrite(PORT_D3, HIGH);
  }
}

// -------------------------------------------------------
// Initialize DC motor port
// Argument   : byte connector       A port ID which is connected DC motor
// -------------------------------------------------------
void StuduinoMini::InitDCMotorPort(byte connector)
{
  if (connector == PORT_M1) {
    pinMode(DCMDA2, OUTPUT);
  } else {
    pinMode(DCMDB2, OUTPUT);
  }    
}

// -------------------------------------------------------
// Outline    : Set DC motor power
// Argument   : byte connector       A port ID which is connected DC motor
//            : byte pace            Speed
// -------------------------------------------------------
void StuduinoMini::DCMotorPower(byte connector, byte pace)
{
  if (connector == PORT_M1) {
    DCPWRA = pace;
    if((TCCR0A >> 5 & 0x01) == 0) { // 停止状態のときはanalogWriteせずに終了
      if((PORTB >> 1 & 0x01) == 1 && (PORTD >> 5 & 0x01) == 1) return;  // reverse
      if((PORTB >> 1 & 0x01) == 0 && (PORTD >> 5 & 0x01) == 0) return;  // reverse
    }
    // PWM mode change
    if(pace == 0) {
      sbi(TCCR0A, COM0B1);
      if((PORTB >> 1 & 0x01) == 1)   // reverse
        sbi(TCCR0A, COM0B0);
      else
        cbi(TCCR0A, COM0B0);
      OCR0B = 0;
      return;
    }else {
      cbi(TCCR0A, COM0B0);
    }
    if((PORTB >> 1 & 0x01) == 1) {
      //pace = 255 - pace;  // reverse
      if(pace > 25)
        analogWrite(DCMDA1, 255 - DCBack(pace) * DCMCRTA / 100);
      else
        analogWrite(DCMDA1, 255 - pace * DCMCRTA / 100);
    } else {
      analogWrite(DCMDA1, pace * DCMCRTA / 100);
    }
  }
  if (connector == PORT_M2) {
    DCPWRB = pace;
    if((TCCR0A >> 7 & 0x01) == 0) { // 停止状態のときはanalogWriteせずに終了
      if((PORTB >> 2 & 0x01) == 1 && (PORTD >> 6 & 0x01) == 1) return;  // reverse
      if((PORTB >> 2 & 0x01) == 0 && (PORTD >> 6 & 0x01) == 0) return;  // reverse
    }
    // PWM mode change
    if(pace == 0) {
      sbi(TCCR0A, COM0A1);
      if((PORTB >> 2 & 0x01) == 1)   // reverse
	sbi(TCCR0A, COM0A0);
      else
	cbi(TCCR0A, COM0A0);
      OCR0A = 0;
      return;
    }else {
      cbi(TCCR0A, COM0A0);
    }
    if((PORTB >> 2 & 0x01) == 1) {
      //pace = 255 - pace;  // reverse
      if(pace > 25)
        analogWrite(DCMDB1, 255 - DCBack(pace) * DCMCRTB / 100);
      else
        analogWrite(DCMDB1, 255 - pace * DCMCRTB / 100);
    } else {
      analogWrite(DCMDB1, pace * DCMCRTB / 100);
    }
  }
}

// -------------------------------------------------------
// Outline    : Set DC motor rotation
// Argument   : byte connector       A port ID which is connected DC motor
//            : byte rotation        Rotation
// -------------------------------------------------------
void StuduinoMini::DCMotorControl(byte connector, byte rotation)
{
  // Set DC motor's rotation.
  if (connector == PORT_M1) {
    if (rotation == NORMAL) {
      if(DCPWRA != 0) {              // power > 0
        cbi(TCCR0A, COM0B0);
        analogWrite(DCMDA1, DCPWRA * DCMCRTA / 100);
      } else {                           // pwoer = 0
        sbi(TCCR0A, COM0B1);
        cbi(TCCR0A, COM0B0);
        OCR0B = 0;
      }
      digitalWrite(DCMDA2, LOW);
    } else if (rotation == REVERSE) {
      if(DCPWRA != 0) {              // power > 0
        cbi(TCCR0A, COM0B0);
	if(DCPWRA > 25)
          analogWrite(DCMDA1, 255 - DCBack(DCPWRA) * DCMCRTA / 100);
	else
          analogWrite(DCMDA1, 255 - DCPWRA * DCMCRTA / 100);
      } else {                           // pwoer = 0
        sbi(TCCR0A, COM0B1);
        sbi(TCCR0A, COM0B0);
        OCR0B = 0;
      }
      digitalWrite(DCMDA2, HIGH);
    } else if (rotation == BRAKE) {
      analogWrite(DCMDA1, 255);
      digitalWrite(DCMDA2, HIGH);
    } else if (rotation == COAST) {
      analogWrite(DCMDA1, 0);
      digitalWrite(DCMDA2, LOW);
    }
  } else if (connector == PORT_M2) {
    if (rotation == NORMAL) {
      if(DCPWRB != 0) {              // power > 0
        cbi(TCCR0A, COM0A0);
        analogWrite(DCMDB1, DCPWRB * DCMCRTB / 100);
      } else {                           // pwoer = 0
        sbi(TCCR0A, COM0A1);
        cbi(TCCR0A, COM0A0);
        OCR0A = 0;
      }
      digitalWrite(DCMDB2, LOW);
    } else if (rotation == REVERSE) {
      if(DCPWRB != 0) {              // power > 0
        cbi(TCCR0A, COM0A0);
	if(DCPWRB > 25)
          analogWrite(DCMDB1, 255 - DCBack(DCPWRB) * DCMCRTB / 100);
	else
          analogWrite(DCMDB1, 255 - DCPWRB * DCMCRTB / 100);
      } else {                           // pwoer = 0
        sbi(TCCR0A, COM0A1);
        sbi(TCCR0A, COM0A0);
        OCR0A = 0;
      }
      digitalWrite(DCMDB2, HIGH);
    } else if (rotation == BRAKE) {
      analogWrite(DCMDB1, 255);
      digitalWrite(DCMDB2, HIGH);
    } else if (rotation == COAST) {
      analogWrite(DCMDB1, 0);
      digitalWrite(DCMDB2, LOW);
    }
  }
}

// -------------------------------------------------------
// Clock functions
// -------------------------------------------------------
void StuduinoMini::InitClock() {
  clock.initClockPinSet();
  clock.setTimeTempMode(16);  // Turn alrm function off.
}

void StuduinoMini::setTime(byte hour, byte min) {
  clock.setTime(hour, min);
}

void StuduinoMini::setDate(unsigned int year, byte month, byte day) {
  if(year < 2000 || year > 2040) 
    year = 2000;
  else
    year -= 2000;
  clock.setDate((byte)year, month, day);
}

void StuduinoMini::setAlarm(byte hour, byte min) {
  clock.setAlarm(hour, min);
}

void StuduinoMini::setBackLight(byte red, byte green, byte blue) {
  clockRed = red;
  clockGreen = green;
  clockBlue = blue;
  clock.setBackLight(clockOnOff, red, green, blue);
}

void StuduinoMini::backLight(boolean onoff) {
  clockOnOff = onoff;
  clock.setBackLight(onoff, clockRed, clockGreen, clockBlue);
}

void StuduinoMini::clockBuzzer(boolean onoff, word pitch) {
  clock.buzzer(onoff, pitch);
}

void StuduinoMini::clockBuzzer(word pitch, unsigned long duration) {
  clock.buzzer(pitch, duration);
}

void StuduinoMini::sleep() {
  TCCR2A = 0;
  TCCR2B = 7;
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);  //  "0.75mA" = "83"days
  //TIMSK2 = 1;
  sleep_mode();
  //TIMSK2 = 0;
  delay(50);
}

int StuduinoMini::GetHour() {
  return clock.getHour();
}

int StuduinoMini::GetMinute() {
  return clock.getMinute();
}

int StuduinoMini::GetYear() {
  return clock.getYear();
}

int StuduinoMini::GetMonth() {
  return clock.getMonth();
}

int StuduinoMini::GetDay() {
  return clock.getDay();
}

float StuduinoMini::GetTemperature() {
  int temp = clock.getTemperature();
  int mod = temp % 10;
  temp = temp / 10;
  float res = temp + mod / 10.f;
  return res;
}

int StuduinoMini::GetAlarmHour() {
  return clock.getAlarmHour();
}

int StuduinoMini::GetAlarmMinute() {
  return clock.getAlarmMinute();
}

bool StuduinoMini::isAlarmTime() {
  return clock.isAlarmTime();
}

int StuduinoMini::GetOnboardLightSensor() {
  return GetLightSensorValue(PORT_A6);
}

float StuduinoMini::GetBatteryVoltage() {
  return (3.3 * analogRead(PORT_A7) * 3.2 / 1024.f);
}
 
