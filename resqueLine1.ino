#include <QTRSensors.h>

//Моторы в бане
#define leftmotor_dir 44 
#define leftmotor_spd 45
#define rightmotor_dir 46
#define rightmotor_spd 47

QTRSensors qtr;
const uint8_t SensorCount = 8;
int sensorValues[SensorCount];
int spd = 60;


int err = 0;
float integral = 0;
float lastErr = 0;
float kp = 1;
float ki = 1;
float kd = 1;
float dt = 0.05;
float OUT = 0;//Переменная, в которую будут поступать данные из ПИДа. Просто для удобства, можно удалить.

//int lastFlag, currentFlag;

//Функция движения, остаётся просто для примера ниже.
void moving (bool Left, bool Right, int spd_L, int spd_R){
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

  //Моторы в бане
  pinMode(leftmotor_dir, OUTPUT);
  pinMode(leftmotor_spd, OUTPUT);
  pinMode(rightmotor_dir, OUTPUT);
  pinMode(rightmotor_spd, OUTPUT);
  
  pinMode(LED_BUILTIN, OUTPUT);//Лампочка на плате для удобного отслеживания процесса калибровки.
  
  //Калибровка - снимаем 20 показаний с каждого датчика, находим максимум и мунимум (чёрный и белый).
    for (uint8_t i = 0; i < 10; i++){
      //moving(0,0,0,0); В дальнейшем робот должен двигаться во время калибровки.
       digitalWrite(LED_BUILTIN, HIGH);
       qtr.calibrate();
       delay(500);
       digitalWrite(LED_BUILTIN, LOW);
       qtr.calibrate();
       delay(500);
    }
}
/*int PID(int input, int setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  float err = setpoint - input;
  float integral = 0, prevErr = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErr) / dt;
  prevErr = err;
  //return constrain(err * kp + integral + D * kd, minOut, maxOut);
  return err * kp + integral + D * kd;
}*/

void loop() {
  //digitalWrite(LED_BUILTIN, HIGH);
  int local_k = -4; //Число от -4 до +4 (без нуля), на которое домножаем ошибку с каждого датчика слева направо [-4,-3,-2,-1,+1,+2,+3,+4].
  err = 0;
  for(uint8_t i = 0; i<8;i++){
    //int tmp = (qtr.calibrationOn.maximum[i]+qtr.calibrationOn.minimum[i])/2;
    /*if(sensorValues[i]<tmp){
      sensorValues[i]=0;
    }
    else{
      sensorValues[i]=1;
    }*/
    //Serial.print(((qtr.calibrationOn.maximum[i]+qtr.calibrationOn.minimum[i])/2 - sensorValues[i]) * local_k);
    //Serial.print(" ");
    err += ((qtr.calibrationOn.maximum[i]+qtr.calibrationOn.minimum[i])/2 - sensorValues[i]) * local_k;
    local_k++;
    if (local_k == 0){
      local_k=1;// Ноль пропустили
    }
    //Serial.print(sensorValues[i]);
    //Serial.print(" ");
  }
  Serial.println();
   
   qtr.read(sensorValues);
   
   //ПИД
   err = 0 - err;
   integral = constrain(integral + err * dt * ki, -1000, 1000);
   float D = (err - lastErr) / dt;
   lastErr = err;
   OUT = constrain(err * kp + integral + D * kd, -1000, 1000);
   moving(1,1,spd+OUT,spd-OUT);


  Serial.print(err);
  Serial.print(" ");
  Serial.print(OUT);
  Serial.println();
  
  
/* if(sV[i]<tmp){
      sV[i]=0;
    }
    else{
      sV[i]=1;
    }*/
//    Serial.print(sV[i]);
  //  Serial.print('\t');
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
//  Serial.println(); 
//  }
