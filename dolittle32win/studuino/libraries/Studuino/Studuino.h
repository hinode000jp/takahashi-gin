#ifndef Studuino_h
#define Studuino_h

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <MMA8653.h>
#include <MPU6050.h>
#include <IRremoteForStuduino.h>
#include <ColorSensor.h>
#include "Bluetooth.h"

// -------------------------------------------------------
// constant definition
// -------------------------------------------------------
enum STDN {

};
// Direction of movement
const byte FORWARD          = 0;  // Forward
const byte BACKWARD         = 1;  // Backward
const byte FORWARD_RIGHT    = 2;  // Right turn (forward)
const byte FORWARD_LEFT     = 3;  // Left turn (forward)
const byte BACKWARD_RIGHT   = 4;  // Right turn (backward)
const byte BACKWARD_LEFT    = 5;  // Left turn (backward)
const byte CLOCKWISE        = 6;  // Right rotation
const byte COUNTERCLOCKWISE = 7;  // Left rotation
// Stop procedures
const byte BRAKE            = 8;  // Brake
const byte COAST            = 9;  // Coast
// DC motor's rotation
const byte NORMAL           = 10; // Normal rotation
const byte REVERSE          = 11; // Reverse rotation
// Connector
const byte PORT_M1  = 0;
const byte PORT_M2  = 1;
const byte PORT_D2  = 4;
const byte PORT_D4  = 5;
const byte PORT_D7  = 6;
const byte PORT_D8  = 7;
const byte PORT_D9  = 0;
const byte PORT_D10 = 1;
const byte PORT_D11 = 2;
const byte PORT_D12 = 3;
const byte PORT_A0  = 0;
const byte PORT_A1  = 1;
const byte PORT_A2  = 2;
const byte PORT_A3  = 3;
const byte PORT_A4  = 4;
const byte PORT_A5  = 5;
const byte PORT_A6  = 6;
const byte PORT_A7  = 7;
// ON/OFF
const boolean ON = true;
const boolean OFF = !ON;
// Tempo
const byte TEMPO60  = 0;
const byte TEMPO90  = 1;
const byte TEMPO120 = 2;
const byte TEMPO150 = 3;
// Scales(Hz)
const word BZR_C3  = 130;  // do
const word BZR_CS3 = 139;  // do#
const word BZR_D3  = 147;  // re
const word BZR_DS3 = 156;  // re#
const word BZR_E3  = 165;  // mi
const word BZR_F3  = 175;  // fa
const word BZR_FS3 = 185;  // fa#
const word BZR_G3  = 196;  // so
const word BZR_GS3 = 208;  // so#
const word BZR_A3  = 220;  // la
const word BZR_AS3 = 233;  // la#
const word BZR_B3  = 247;  // ti
const word BZR_C4  = 262;  // do
const word BZR_CS4 = 277;  // do#
const word BZR_D4  = 294;  // re
const word BZR_DS4 = 311;  // re#
const word BZR_E4  = 330;  // mi
const word BZR_F4  = 349;  // fa
const word BZR_FS4 = 370;  // fa#
const word BZR_G4  = 392;  // so
const word BZR_GS4 = 415;  // so#
const word BZR_A4  = 440;  // la
const word BZR_AS4 = 466;  // la#
const word BZR_B4  = 494;  // ti
const word BZR_C5  = 523;  // do
const word BZR_CS5 = 554;  // do#
const word BZR_D5  = 587;  // re
const word BZR_DS5 = 622;  // re#
const word BZR_E5  = 659;  // mi
const word BZR_F5  = 698;  // fa
const word BZR_FS5 = 740;  // fa#
const word BZR_G5  = 784;  // so
const word BZR_GS5 = 831;  // so#
const word BZR_A5  = 880;  // la
const word BZR_AS5 = 932;  // la#
const word BZR_B5  = 988;  // ti
const word BZR_C6  = 1047; // do
const word BZR_CS6 = 1109; // do#
const word BZR_D6  = 1175; // re
const word BZR_DS6 = 1245; // re#
const word BZR_E6  = 1319; // mi
const word BZR_F6  = 1397; // fa
const word BZR_FS6 = 1480; // fa#
const word BZR_G6  = 1568; // so
const word BZR_GS6 = 1661; // so#
const word BZR_A6  = 1760; // la
const word BZR_AS6 = 1865; // la#
const word BZR_B6  = 1976; // ti
const word BZR_C7  = 2093; // do
const word BZR_CS7 = 2217; // do#
const word BZR_D7  = 2349; // re
const word BZR_DS7 = 2489; // re#
const word BZR_E7  = 2637; // mi
const word BZR_F7  = 2794; // fa
const word BZR_FS7 = 2960; // fa#
const word BZR_G7  = 3136; // so
const word BZR_GS7 = 3322; // so#
const word BZR_A7  = 3520; // la
const word BZR_AS7 = 3729; // la#
const word BZR_B7  = 3951; // ti
const word BZR_C8  = 4186; // do
const word BZR_S   = 0;    // silent
// Accelerometer
const byte X_AXIS = 0;
const byte Y_AXIS = 1;
const byte Z_AXIS = 2;
// Gyroscope
const byte GX_AXIS = 3;
const byte GY_AXIS = 4;
const byte GZ_AXIS = 5;
// ColorSensor
//const byte COLOR_UNDEF  = 0;
//const byte COLOR_RED    = 1;
//const byte COLOR_GREEN  = 2;
//const byte COLOR_BLUE   = 3;
//const byte COLOR_WHITE  = 4;
//const byte COLOR_YELLOW = 5;
//const byte COLOR_BROWN  = 6;
//const byte COLOR_BLACK  = 7;
//const byte VALUE_RED   = 0;
//const byte VALUE_GREEN = 1;
//const byte VALUE_BLUE  = 2;
//const byte VALUE_CLEAR = 3;

// Parts ID
const byte PIDOPEN              = 0x00;  // Not connect
const byte PIDDCM               = 0x01;  // DC motor
const byte PIDSVM               = 0x02;  // Servomotor
const byte PIDLED               = 0x03;  // LED
const byte PIDBUZZER            = 0x04;  // Buzzer
const byte PIDLIGHTSENSOR       = 0x10;  // Light sensor
const byte PIDTOUCHSENSOR       = 0x11;  // Touch sensor
const byte PIDSOUNDSENSOR       = 0x12;  // Sound sensor
const byte PIDIRPHOTOREFLECTOR  = 0x13;  // IR Photoreflector
const byte PIDACCELEROMETER     = 0x14;  // Accelerometer
const byte PIDPUSHSWITCH        = 0x15;  // Push switch
const byte PIDIRRECEIVER        = 0x16;  // IR Receiver
const byte PIDGYROSCOPE         = 0x17;  // Gyroscope
const byte PIDTEMPERATURESENSOR = 0x18;  // Temperature sensor
const byte PIDULTRASONICSENSOR  = 0x19;  // Ultrasonic sensor
const byte PIDCOLORSENSOR       = 0x1a;  // Color sensor


// -------------------------------------------------------
// Prototype declaration
// -------------------------------------------------------
class Studuino {
  public:
  Studuino();
  void InitDCMotorPort(byte direct);
  void InitServomotorPort(byte direct);
  void InitServomotorPortForLED(byte connector);
  void InitSensorPort(byte connector, byte pid);
  void InitSensorPort(byte connector1, byte connector2, byte pid);
  void InitI2CPort(byte pid);
  void InitBluetooth();
  void SetDCMotorCalibration(byte* rate);
  void SetServomotorCalibration(char* offset);
  void Move(byte direct, byte pace, unsigned long duration, byte brake);
  void DCMotor(byte connector, byte rotation, byte pace, unsigned long duration, byte brake);
  void DCMotorPower(byte connector, byte pace);
  void DCMotorControl(byte connector, byte rotation);
  void Servomotor(byte connector, byte degree);
  void SyncServomotors(byte* connector, byte* degree, byte number, byte pace);
  void AsyncServomotors(byte* connector, byte* degree, byte number);
  void Buzzer(byte connector, word pitch, unsigned long duration);
  void BuzzerControl(byte connector, boolean onoff, word pitch);
  void Melody(byte connector, word* scales, float* notes, byte number, byte tempo);
  void LED(byte connector, boolean onoff);
  void Gradation(byte connector, byte ratio);
  void Timer(unsigned long time);
  byte GetPushSwitchValue(byte connector);
  byte GetTouchSensorValue(byte connector);
  int GetLightSensorValue(byte connector);
  int GetSoundSensorValue(byte connector);
  int GetTemperatureSensorValue(byte connector);
  int GetIRPhotoreflectorValue(byte connector);
  unsigned long GetUltrasonicSensorValue(byte trigger, byte echo);
  int GetAccelerometerValue(byte connector);
  int GetGyroscopeValue(byte axis);
  unsigned long GetIRReceiverValue();
  void EnableIRReceiver();
  void DisableIRReceiver();
  unsigned int GetColorSensorValue(byte type);
  void GetColorSensorXY(double* x, double* y);
  byte GetColorCode();
  boolean UpdateBluetooth();
  boolean GetBTCommandIDState(byte id);
  int GetBTAccelValue(byte axis);

  private:
  byte DCMDA1;    // DC motor driver A1
  byte DCMDA2;    // DC motor driver A2
  byte DDMDPWMA;  // DC motor driver PRM A
  byte DCMDB1;    // DC motor driver B1
  byte DCMDB2;    // DC motor driver B2
  byte DDMDPWMB;  // DC motor driver PRM B
  byte SVMPORT[8];
  byte TEMPO[4];
  byte DCMCRTA;   // DC motor M1 calibration rate
  byte DCMCRTB;   // DC motor M2 calibration rate
  char SVOFF[8];  // Servomotor offsets
  
  Servo  servomotors[8]; // Servomotors
  MMA8653* accel;        // Accelerometer
  bool isAccel;
  MPU6050* gyro;         // Gyroscope
  bool isGyro;
  COLOR* color;    // ColorSensor
  bool isColor;
  IRrecv* irrecv;
  decode_results ir_result;
  bt_param_t bt_result;
};
#endif
