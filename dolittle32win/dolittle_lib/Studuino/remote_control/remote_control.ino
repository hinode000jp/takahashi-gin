#include"arduino.h"
#include<Arduino.h>
#include<Servo.h>
#include<Wire.h>
#include<I2Cdev.h>
#include<MMA8653.h>
#include<MPU6050.h>
#include<ColorSensor.h>
#include<IRremoteForStuduino.h>
#include"Bluetooth.h"
#include"Studuino.h"

//定数定義（[!] 本当はconst変数のほうがいいので後で対応すること）
#define WAIT_TIME 5

//== プロトタイプ宣言 ==
void Undefined(byte pram);
void ReadAcceleration(byte pram);
void SelectPinModeOut(byte pram);
void SelectPinModeIn(byte pram);
void ReadInputPin(byte pram);
void OutputPin(byte pram);
void OutputAnalogPin(byte pram);
void ReadAnalogPin(byte pram);

//== グローバル 宣言 ==
Studuino board;


// ///////////////////////////////////
// 関数  :setup
// 説明  :イニシャライズ処理
// 戻り値:void
// 引数  :なし
// その他:
// ///////////////////////////////////
void setup(){
	Serial.begin(9600);
//	board.InitI2CPort(PIDACCELEROMETER);
//	board.InitSensorPort(PORT_A0, PIDLED);
        Serial.write(' ');
}

// ///////////////////////////////////
// 関数  :loop
// 説明  :メインループ
// 戻り値:void
// 引数  :なし
// その他:
// ///////////////////////////////////
void loop(){	
	if (Serial.available() > 0) {
		byte in = Serial.read();
		byte cmd = (in & 0xe0)>>5;
		byte port = in & 0x1f;

		// cmdに応じた命令を定義
		void (*FUNCTION_TBL[8])(byte)  = {
                        // 0x0X    0x20              0x40              0x60             0x80       0xA0          0xC0  �iPWN�j           0XE0
			Undefined, ReadAcceleration, SelectPinModeOut, SelectPinModeIn, OutputPin, ReadInputPin, OutputAnalogPin, ReadAnalogPin
		};
		
		FUNCTION_TBL[cmd] (port);
         }
	
}

// ///////////////////////////////////
// 関数  :Undefined
// 説明  :未処理
// 戻り値:void
// 引数  :pram：未使用
// その他:
// ///////////////////////////////////
void
Undefined(byte pram){}

// ///////////////////////////////////
// 関数  :ReadAcceleration
// 説明  :加速度計測
// 戻り値:void
// 引数  :pram：軸指定(0:X ,1:Y,2:Z,other:X)
// その他:
// ///////////////////////////////////
void
ReadAcceleration(byte pram){
  byte axis = X_AXIS;
  switch(pram){
    case 0:
      axis = X_AXIS;
      break;
    case 1:
      axis = Y_AXIS;
      break;
    case 2:
      axis = Z_AXIS;
      break;
    default :
      break;
  }
  Serial.write(board.GetAccelerometerValue(axis) );	
}

// ///////////////////////////////////
// 関数  :SelectPinModeOut
// 説明  :端子を出力に設定
// 戻り値:void
// 引数  :pram：端子番号
// その他:端子番号Axは+12(14?)しなくてはいけない点に注意
// ///////////////////////////////////
void
SelectPinModeOut(byte pram){
	pinMode(pram ,OUTPUT);
//	delay(WAIT_TIME);
}

// ///////////////////////////////////
// 関数  :SelectPinModeOut
// 説明  :端子を入力に設定
// 戻り値:void
// 引数  :pram：端子番号
// その他:端子番号Axは+12(14?)しなくてはいけない点に注意
// ///////////////////////////////////
void
SelectPinModeIn(byte pram){
	pinMode(pram ,INPUT);
//	delay(WAIT_TIME);
}

// ///////////////////////////////////
// 関数  :ReadInputPin
// 説明  :入力端子の値を取得
// 戻り値:void
// 引数  :pram：端子番号
// その他:端子番号Axは+12(14?)しなくてはいけない点に注意
// ///////////////////////////////////
void
ReadInputPin(byte pram){
	 Serial.write(digitalRead(pram));	
}

// ///////////////////////////////////
// 関数  :OutputPin
// 説明  :デジタル出力端子の出力値（Low/High）変更
// 戻り値:void
// 引数  :pram：端子番号
// その他:端子番号Axは+12(14?)しなくてはいけない点に注意
//        また、端子出力値は関数内部で、シリアル通信により
//        取得するというひどい設計なので注意する。
// ///////////////////////////////////
void
OutputPin(byte pram){
//	delay(WAIT_TIME);
        while(Serial.available() == 0);
	if (Serial.read() == 1) { 
		digitalWrite(pram, HIGH);
	} else { 
		digitalWrite(pram, LOW); 
	}
}

// ///////////////////////////////////
// 関数  :OutputAnalogPin
// 説明  :アナログ出力端子のPWM値を変更
// 戻り値:void
// 引数  :pram：端子番号
// その他:端子番号Axは+12(14?)しなくてはいけない点に注意
//        また、端子出力値は関数内部で、シリアル通信により
//        取得するというひどい設計なので注意する。
// ///////////////////////////////////
void
OutputAnalogPin(byte pram){
//	delay(WAIT_TIME);
	analogWrite(pram, Serial.read());
}

// ///////////////////////////////////
// 関数  :ReadAnalogPin
// 説明  :アナログ入力端子の値を取得
// 戻り値:void
// 引数  :pram：端子番号
// その他:端子番号Axは+12(14?)しなくてはいけない点に注意
// ///////////////////////////////////
void
ReadAnalogPin(byte pram){
	Serial.write(analogRead(pram)/4);
}
