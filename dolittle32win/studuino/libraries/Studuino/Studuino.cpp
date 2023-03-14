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

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <MMA8653.h>
#include <MPU6050.h>
#include <IRremoteForStuduino.h>
#include <ColorSensor.h>
#include "Bluetooth.h"
#include "Studuino.h"
// -------------------------------------------------------
// Macro
// -------------------------------------------------------
#define ANALOG2DIGITAL(port)  (port+14)
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
  SVMPORT[0] = 9;
  SVMPORT[1] = 10;
  SVMPORT[2] = 11;
  SVMPORT[3] = 12;
  SVMPORT[4] = 2;
  SVMPORT[5] = 4;
  SVMPORT[6] = 7;
  SVMPORT[7] = 8;
  TEMPO[0] = 60;
  TEMPO[1] = 90;
  TEMPO[2] = 120;
  TEMPO[3] = 150;

  DCMCRTA = 100;
  DCMCRTB = 100;
  SVOFF[0] = 0;
  SVOFF[1] = 0;
  SVOFF[2] = 0;
  SVOFF[3] = 0;
  SVOFF[4] = 0;
  SVOFF[5] = 0;
  SVOFF[6] = 0;
  SVOFF[7] = 0;
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
  byte index = connector;
  servomotors[index].attach(SVMPORT[index], 500, 2500);
}

// -------------------------------------------------------
// Initialize servomotor port for LED
// Argument   : byte connector       A port ID which is connected LED
// -------------------------------------------------------
void Studuino::InitServomotorPortForLED(byte connector)
{
  byte index = connector;
  // If servomotor is attatched to target port, detach servomotor.
  if (servomotors[index].attached())  servomotors[index].detach();
}

// -------------------------------------------------------
// Initialize sensor / LED / buzzer port
// Argument   : byte connector       A port ID which is connected parts
//            : byte pid             Parts ID
// -------------------------------------------------------
void Studuino::InitSensorPort(byte connector, byte pid)
{
  switch (pid) {
    case PIDOPEN:           // Not connect
      // Do nothing
    break;
    case PIDLED:            // LED
    case PIDBUZZER:         // Buzzer
      // Digital output parts
      pinMode(ANALOG2DIGITAL(connector), OUTPUT);
    break;
    case PIDLIGHTSENSOR:       // Light sensor
    case PIDSOUNDSENSOR:       // Sound sensor
    case PIDIRPHOTOREFLECTOR:  // IR Photorefrector
      // Analog input parts 
    break;
    case PIDTOUCHSENSOR:    // Touch sensor
    case PIDPUSHSWITCH:     // Push switch
      // Digital input parts
      pinMode(ANALOG2DIGITAL(connector), INPUT_PULLUP);
    break;
    case PIDIRRECEIVER:     // IR receiver
      // Digital input parts
      irrecv = new IRrecv(ANALOG2DIGITAL(connector));
      irrecv->enableIRIn(); // Start the receiver
    break;
  }
}

// -------------------------------------------------------
// Initialize sensor port (Using 2 connector)
// Argument   : byte connector1, connector2  Port IDs connecting
//            : byte pid                     Parts ID
// -------------------------------------------------------
void Studuino::InitSensorPort(byte connector1, byte connector2, byte pid)
{
  switch (pid) {
    case PIDULTRASONICSENSOR: // Ultrasonic sensor
      pinMode(ANALOG2DIGITAL(connector1), OUTPUT);
      pinMode(ANALOG2DIGITAL(connector2), INPUT);
    break;
  }
}

// -------------------------------------------------------
// Initialize I2C devices
// Argument   : byte pid             Parts ID
// -------------------------------------------------------
void Studuino::InitI2CPort(byte pid)
{
  switch (pid) {
    case PIDACCELEROMETER:  // Accelerometer
      // Setup I2C input
      Wire.begin();
      accel = new MMA8653();
      isAccel = accel -> testConnection();
      if(isAccel) accel -> begin(false, 2);
    break;
    case PIDGYROSCOPE:  // Gyroscope
      // Setup I2C input
      Wire.begin();
      gyro = new MPU6050();
      isGyro = gyro -> testConnection();
      if(isGyro) gyro -> initialize();
    break;
    case PIDCOLORSENSOR:
      // Setup I2C input
      Wire.begin();
      color = new COLOR();
      isColor = color -> testConnection();
      if(isColor) color -> initialize();
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
  byte index = connector;
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
    calibedDegree = min(180, max(0, degree[i] + SVOFF[connector[i]])); // Calibrating the given angle.
    servomotors[connector[i]].write(calibedDegree);
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
    before[i] = servomotors[connector[i]].read();  // Current angle.
    calibedDegree = min(180, max(0, degree[i] + SVOFF[connector[i]])); // Calibrating the given angle.
    delta[i] = calibedDegree - before[i];              // Get difference.
    // Get maximum difference.
    maxDelta = (abs(delta[i]) > maxDelta) ? abs(delta[i]) : maxDelta;
  }
  
  // Set angles for each servomotor 
  if (time == 0) {  // If delay time is 0...
    // Set angles for each servomotor
    for (int i = 0;i < number;i++) {
      calibedDegree = min(180, max(0, degree[i] + SVOFF[connector[i]])); // Calibrating the given angle.
      servomotors[connector[i]].write(calibedDegree);    
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
        servomotors[connector[i]].write(before[i]+delta[i]*t);
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
  byte port = ANALOG2DIGITAL(connector);  // Get buzzer port number
  tone(port, pitch, duration);            // Output beep
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
  byte port = ANALOG2DIGITAL(connector);  // Get buzzer port number
  if (onoff == ON) {
    tone(port, pitch);   // Output beep
  } else {
    noTone(port);        // Stop beep
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
  byte port = ANALOG2DIGITAL(connector);  // Get buzzer port number
  float beat = 60000.0 / TEMPO[tempo];  // 1拍を算出
  
  for (int i = 0;i < number;i++) {
    int duration = beat * notes[i];
  
    if (notes[i] >= 8) {
      delay(duration);
    }
    else
    {
      pinMode(port, OUTPUT);
      tone(port, scales[i], duration);
      delay(duration);
      noTone(port);
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
  byte port = ANALOG2DIGITAL(connector);  // Get buzzer port number
  if (onoff == ON) {
    digitalWrite(port, HIGH);
  } else {
    digitalWrite(port, LOW);
  }
}

// -------------------------------------------------------
// Outline    : Gradataion
// Argument   : byte connector          Port IDs which is connected LED
//            : byte ratio              Brightness
// -------------------------------------------------------
void Studuino::Gradation(byte connector, byte ratio)
{
  byte index = connector;
  byte ledPort = SVMPORT[index];
  analogWrite(ledPort, ratio);
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
  byte port = ANALOG2DIGITAL(connector);  // Get buzzer port number
  return digitalRead(port);
}

// -------------------------------------------------------
// Outline    : Get touch sensor status
// Argument   : byte connector          Port IDs which is connected touch sensor
// -------------------------------------------------------
byte Studuino::GetTouchSensorValue(byte connector)
{
  byte port = ANALOG2DIGITAL(connector);  // Get buzzer port number
  return digitalRead(port);
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
// Outline    : Get Temperature sensor status
// Argument   : byte connector          Port IDs which is connected temperature sensor 
// -------------------------------------------------------
int Studuino::GetTemperatureSensorValue(byte connector)
{
  return analogRead(connector);
}

// -------------------------------------------------------
// Outline    : Get Ultrasonic sensor status
// Argument   : byte trigger          Port ID connecting to ultrasonic sensor's trigger pin. 
//            : byte echo             Port ID connecting to ultrasonic sensor's echo pin. 
// -------------------------------------------------------
unsigned long Studuino::GetUltrasonicSensorValue(byte trigger, byte echo)
{
  unsigned long wait;
  //pinMode(ANALOG2DIGITAL(trigger), OUTPUT);
  digitalWrite(ANALOG2DIGITAL(trigger), LOW);
  delayMicroseconds(10);
  digitalWrite(ANALOG2DIGITAL(trigger), HIGH);
  delayMicroseconds(110);
  digitalWrite(ANALOG2DIGITAL(trigger), LOW);
  delayMicroseconds(10);
  //pinMode(ANALOG2DIGITAL(echo), INPUT);

  wait = pulseIn(ANALOG2DIGITAL(echo), HIGH, 30000);       //(timeout > (8m / 344m/s) = 23.5ms )
  delayMicroseconds(30000L - wait); // Wait 30ms in total.

  return wait;
}

// -------------------------------------------------------
// Outline    : Get Accelerometer status
// Argument   : byte axis             [XYZ]_AXIS
// -------------------------------------------------------
int Studuino::GetAccelerometerValue(byte axis)
{
  int sv = 0;
  if(isAccel) {
    accel -> update();
    if (axis == X_AXIS) {
      sv = accel -> getX();
    } else if (axis == Y_AXIS) {
      sv = accel -> getY();
    } else if (axis == Z_AXIS) {
      sv = accel -> getZ();
    }
  }
  return sv;
}

// -------------------------------------------------------
// Outline    : Get Gyroscope status
// Argument   : byte axis             [XYZ]_AXIS
// -------------------------------------------------------
int Studuino::GetGyroscopeValue(byte axis)
{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
	
  int sv = 0;
  if(isGyro) {
    gyro -> getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if (axis == X_AXIS) {
      sv = ax;
    } else if (axis == Y_AXIS) {
      sv = ay;
    } else if (axis == Z_AXIS) {
      sv = az;
    } else if (axis == GX_AXIS) {
      sv = gx;
    } else if (axis == GY_AXIS) {
      sv = gy;
    } else if (axis == GZ_AXIS) {
      sv = gz;
    }
  }
  return sv;
}

// -------------------------------------------------------
// Outline    : Get received infrared signal.
//              If the value is equal registered value, return the value.
//              If not, resume process.
// -------------------------------------------------------
unsigned long Studuino::GetIRReceiverValue()
{
  unsigned long res = 0;
  if (irrecv->decode(&ir_result)) {    //  赤外線リモコンからデータ受信。resultsに格納
    irrecv->resume();           // Reset if unregistered values come.
    res = ir_result.value;
  }
  return res;
}

// -------------------------------------------------------
// Outline    : Enable to receive infraged signal.
// -------------------------------------------------------
void Studuino::EnableIRReceiver()
{
  irrecv->enableIRIn(); // Receive the next value
}

// -------------------------------------------------------
// Outline    : Disable to receive infraged signal.
// -------------------------------------------------------
void Studuino::DisableIRReceiver()
{
  irrecv->disableIRIn(); // Receive the next value
}

// ColorSensor
unsigned int Studuino::GetColorSensorValue(byte type)
{
  color -> update();
  unsigned int val;
  switch(type) {
    case VALUE_RED:
    val = color -> getColorRed();
    break;
    case VALUE_GREEN:
    val = color -> getColorGreen();
    break;
    case VALUE_BLUE:
    val = color -> getColorBlue();
    break;
    case VALUE_CLEAR:
    val = color -> getColorClear();
    break;
    default:
    val = 0;
    break;
  }
  return val;
}

void Studuino::GetColorSensorXY(double* x, double* y)
{
  color -> update();
  *x = color -> getX();
  *y = color -> getY();
}

byte Studuino::GetColorCode()
{
  color -> update();
  return color -> getColorCode();
}

// -------------------------------------------------------
// [Bluetooth] Starting Bluetooth serial communication.
// Argument   : None
// -------------------------------------------------------
void Studuino::InitBluetooth()
{
  Serial.begin(9600);
}

// -------------------------------------------------------
// [Bluetooth] Receiving data via Bluetooth serial.
// Argument   : None
// -------------------------------------------------------
boolean Studuino::UpdateBluetooth()
{
  // Flushing receive buffer when more than 20 bytes data has reached.
  // flush() doesn't work in that way since Arduino 1.0. Need to restart serial to clear receive buffer.
  if(Serial.available() > 20) {
    Serial.end();
    Serial.begin(9600);
  }
  
  byte recvStatus = RecvHeader;
  byte recvHeaderCount = 0;
  
  int numData = 13;    // Data amount
  int data[numData];
  int dataCount = 0;

  long startTime = millis();
  while(millis() - startTime < 30) {
    while (Serial.available() > 0){
      int inByte = Serial.read();
  
      switch(recvStatus) {
        case RecvHeader:  // Receive Header
          if (inByte == HEADER) {
            recvHeaderCount++;
            if (recvHeaderCount >= HEADER_RECV_TIME) {
              recvHeaderCount = 0;
              recvStatus = RecvData;  // Moving to next state.
            }
          } else { // Receiving data is not Header.
            recvHeaderCount = 0;
          }
        break;
        
        case RecvData:  // Receive Data
          data[dataCount] = inByte;
          dataCount++;
          if(dataCount == numData)
          {
            recvStatus = RecvFooter;
            dataCount = 0;
          }
        break;
  
        case RecvFooter:  // Receive Footer
          bt_result.flagButton = data[0] | data[1] << 8;
          bt_result.flagOther = data[2] | data[3] << 8;
          bt_result.accX = data[4];
          bt_result.accY = data[5];
          bt_result.accZ = data[6];

          return true;
        break;
  
        default:
        break;
      }
    }    // while serial
  }      // while timeout
  return false;
}


// -------------------------------------------------------
// [Bluetooth] Checking if the button(id) is pushed.
// Argument   : None
// -------------------------------------------------------
boolean Studuino::GetBTCommandIDState(byte id)
{
  byte cat = id & 0xf0;
  byte shift = id & 0x0f;
  
  if(cat == 0) {
    return (bt_result.flagButton >> shift & 0x01) == 1;
  }else {
    return (bt_result.flagOther >> shift & 0x01) == 1;
  }
}

// -------------------------------------------------------
// [Bluetooth] Retrieving accelerometer data.
// Argument   : byte axis
// -------------------------------------------------------
int Studuino::GetBTAccelValue(byte axis)
{
  if(axis == X_AXIS) {
    return bt_result.accX;
  }else if(axis == Y_AXIS) {
    return bt_result.accY;
  }else {
    return bt_result.accZ;
  }
}

