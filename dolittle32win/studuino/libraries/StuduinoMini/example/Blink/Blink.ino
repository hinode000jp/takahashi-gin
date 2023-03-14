#include <Servo.h>
#include <Wire.h>
#include <MMA8653.h>
#include <Clock.h>
#include <Studuino.h>

Studuino board;

void setup() {
  board.InitSensorPort(PORT_A0, PIDLED);
}

void loop() {
  board.LED(PORT_A0, ON);
  board.Timer(1000);
  board.LED(PORT_A0, OFF);
  board.Timer(1000);
}

