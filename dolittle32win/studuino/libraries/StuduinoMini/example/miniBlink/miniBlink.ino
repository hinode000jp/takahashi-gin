#include <Servo.h>
#include <Wire.h>
#include <MMA8653.h>
#include <Clock.h>
#include <Studuino.h>

StuduinoMini board;

void setup() {
  board.InitDigitalPort(PORT_D5, PIDLED);
  board.InitDigitalPort(PORT_D6, PIDLED);
  board.InitDigitalPort(PORT_D9, PIDLED);
}

void loop() {
  board.LED(PORT_D5,ON);
  board.Timer(1000);
  board.LED(PORT_D6,ON);
  board.Timer(1000);
  board.LED(PORT_D9,ON);
  board.Timer(1000);
  board.LED(PORT_D5,OFF);
  board.Timer(1000);
  board.LED(PORT_D6,OFF);
  board.Timer(1000);
  board.LED(PORT_D9,OFF);
  board.Timer(1000);
}

