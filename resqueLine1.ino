#include <QTRSensors.h>
QTRSensors qtr;

//Моторы в бане
#define leftmotor_dir 44 
#define leftmotor_spd 45
#define rightmotor_dir 46
#define rightmotor_spd 47

const uint8_t SensorCount = 8;
int sensorValues[SensorCount];
int spd = 50;


int err = 0;
float integral = 0;
float lastErr = 0;
float kp = 1;
float ki = 0.1;
float kd = 0;
float dt = 1;
float OUT = 0;//Переменная, в которую будут поступать данные из ПИДа. Просто для удобства, можно удалить.

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

void loop() {
  qtr.read(sensorValues);
  int local_k = -4;
  err = 0;
  for(uint8_t i = 0; i<8;i++){
    int tmp = (qtr.calibrationOn.maximum[i]+qtr.calibrationOn.minimum[i])/2;
    if(sensorValues[i]<tmp){
      sensorValues[i]=0;
    }
    else{
      sensorValues[i]=1;
    }
    Serial.print(sensorValues[i]* local_k);
    Serial.print(" ");
  
    err += sensorValues[i] * local_k;
    local_k++;
    if (local_k == 0){
      local_k=1;// Ноль пропустили
    }
    //Serial.print(sensorValues[i]);
    //Serial.print(" ");
  }
  //Serial.print("err = ");
  //Serial.print(err);

   //ПИД
  err = (0 - err);
  if (err == 0) {
    integral = 0;
  }
  else {
    integral = integral + err * dt * ki;
  }
  float D = (err - lastErr) / dt;
  lastErr = err;
  OUT = err * kp + integral + D * kd;
  moving(1,1,spd+OUT,spd-OUT);

  Serial.print(err);
  Serial.print(" ");
  Serial.print(integral);
  Serial.print(" ");
  Serial.print(OUT);
  Serial.println();
}



