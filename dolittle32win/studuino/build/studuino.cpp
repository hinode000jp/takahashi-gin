#include"StuduinoMini.h"
struct __MotorInfo{
	int LeftPower;
	int RightPower;
	int LeftOffset;
	int RightOffset;
}MotorInfo={200,200,0,0};
	void SetOffset(int lo, int ro){MotorInfo.LeftOffset= lo;MotorInfo.RightOffset = ro;};
	void SetLeftPower(int pw){if((pw - MotorInfo.LeftOffset)< 0){MotorInfo.LeftPower=0;}else{MotorInfo.LeftPower -= MotorInfo.LeftOffset;}}
	void SetRightPower(int pw){if((pw - MotorInfo.RightOffset)< 0){MotorInfo.RightPower=0;}else{MotorInfo.RightPower -= MotorInfo.RightOffset;}}
	int GetLeftPower(){return MotorInfo.LeftPower;}
	int GetRightPower(){return MotorInfo.RightPower;}
StuduinoMini board;
 
void setup(){
board.InitClock();
board.Timer(1000);
board.setDate(2017,  6 ,  12 );
board.setTime(6,  59 );
board.setAlarm(7,  0 );
board.setBackLight(5,  5 ,  5 );
board.backLight(ON);
}
 
void loop(){
if (
board.isAlarmTime()
)
{
board.clockBuzzer((word) (440) ,  (unsigned long)(100) );
board.Timer(900);
board.clockBuzzer((word) (440) ,  (unsigned long)(100) );
board.Timer(900);
board.clockBuzzer((word) (440) ,  (unsigned long)(100) );
board.Timer(900);
board.clockBuzzer((word) (880) ,  (unsigned long)(1000) );
}
board.sleep();
}
 
int main(void){
init();
setup();
for(;;){
loop();
}
}
