#include <NewPing.h>
#include <QTRSensors.h>
//#define TRIGGER_PIN  6
//#define ECHO_PIN     7
//#define MAX_DISTANCE 1500


#define leftmotor_dir 44
#define leftmotor_spd 45
#define rightmotor_dir 46
#define rightmotor_spd 47
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int spd = 0;
int FlagForCalibration = 0;
//int line = qtr.calibrationOn.minimum[4];
int sV[SensorCount];
int tmp;
//int lastFlag, currentFlag;
//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void moving (bool Left, bool Right, int spd_L, int spd_R){
  //Left = not Left;
  digitalWrite(leftmotor_dir, Left);
  analogWrite(leftmotor_spd, spd_L);
  digitalWrite(rightmotor_dir, Right);
  analogWrite(rightmotor_spd, spd_R);
}


void setup() {
  Serial.begin(9600);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {
    A0, A1, A2, A3, A4, A5, A6, A7
  }, SensorCount);
  qtr.setEmitterPin(2);
  pinMode(leftmotor_dir, OUTPUT);
  pinMode(leftmotor_spd, OUTPUT);
  pinMode(rightmotor_dir, OUTPUT);
  pinMode(rightmotor_spd, OUTPUT);
  
  //pinMode(LED_BUILTIN, OUTPUT);

//delay(200);
  //digitalWrite(LED_BUILTIN, HIGH);
  
   /*for (uint16_t i = 0; i < 15; i++){
        moving(1,0,spd/2,0);
        qtr.calibrate();
  }
   for (uint16_t i = 0; i < 30; i++){
        moving(0,1,0,spd/2);
        qtr.calibrate();
  }
    
   for (uint16_t i = 0; i < 15; i++){
        moving(1,0,spd/2,0);
        qtr.calibrate();
  }*/
if(FlagForCalibration != 1){
    for (uint16_t i = 0; i < 30; i++){
      moving(0,0,0,0);
      qtr.calibrate();
      delay(500);
      FlagForCalibration = 1;
    }
  }
}
int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  float err = setpoint - input;
  static float integral = 0, prevErr = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErr) / dt;
  prevErr = err;
  return constrain(err * kp + integral + D * kd, minOut, maxOut);
}

void loop() {
 ///////////////////////////////////
 //LeftMotor LOW - вперёд
 //RightMotor HIGH - вперед
  // digitalWrite(LED_BUILTIN, LOW);
  moving(1,1,0,0);
 qtr.read(sensorValues);
  for (int i = 0; i < SensorCount; i++)
  {
    sV[i] = sensorValues[i];
    tmp=(qtr.calibrationOn.maximum[i]+qtr.calibrationOn.minimum[i])/2;
  
/* if(sV[i]<tmp){
      sV[i]=0;
    }
    else{
      sV[i]=1;
    }*/
    Serial.print(sV[i]);
    Serial.print('\t');
  }
/*
if((sV[3]==1)||(sV[4]==1)){
 currentFlag = 34;
}
if((sV[1]==0)&&(sV[1]==0)&&(sV[2]==0)&&(sV[3]==0)&&(sV[4]==0)&&(sV[5]==0)&&(sV[6]==0)&&(sV[7]==0)){
 switch(lastFlag){
  case 7: moving(0,1,spd,spd);
  case 4: moving(1,1,spd,spd);
  case 3: moving(1,1,spd,spd);
  case 0: moving(1,0,spd,spd);
 }
}*/
/*if((sV[1]==1)||(sV[2]==1)){
 moving(1,1,spd/2,spd);
}
if((sV[5]==1)||(sV[6]==1)){
 moving(1,1,spd,spd/2);
}
if(sV[0]==1){
 moving(0,1,spd/5,spd+spd/5);
}
if(sV[7]==1){
 moving(1,0,spd+spd/5,spd/5);
}
if(sV[7]==1){
 moving(1,0,spd+spd/6,spd/2);
 currentFlag = 7;
}
if(sV[6]==1){
 moving(1,1,spd+spd/6,spd/3);
 currentFlag = 6;
}
if(sV[5]==1){
 moving(1,1,spd+spd/6,spd/2);
 currentFlag = 5;
}
if(sV[4]==1){
 moving(1,1,spd,spd);
 currentFlag = 4;
}
if(sV[3]==1){
 moving(1,1,spd,spd);
 currentFlag = 3;
}
if(sV[2]==1){
 moving(1,1,spd/2,spd+spd/6);
 currentFlag = 2;
}
if(sV[1]==1){
 moving(1,1,spd/3,spd+spd/6);
 currentFlag = 1;
}
if(sV[0]==1){
 moving(0,1,spd/4,spd+spd/6);
 currentFlag = 0;
}*/
// lastFlag = currentFlag;
  //Serial.print(lastFlag); 
  //Serial.print('-');
  //Serial.print(currentFlag);
  //Serial.print(' ');
  Serial.println(); 
  }
