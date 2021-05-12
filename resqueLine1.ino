#include <VarSpeedServo.h>  //Установи библиотеку, если не работает. Но много времени ей можно не уделяем, пока не поедем
#include <QTRSensors.h>
#include <NewPing.h>

NewPing sonar(8, 7, 500); // Сонар: Trig = 8, Echo нужно указать.

QTRSensors qtr;

VarSpeedServo leftServo; //Объекты серв
VarSpeedServo rightServo; //Объекты серв
const int leftServoPin = 9; // the digital pin used for the first servo
const int rightServoPin = 10; // the digital pin used for the second servo

struct Color {
  int S0;
  int S1;
  int S2;
  int S3;
  int sensorOut;
  int red, green, blue;
};
Color leftColor = Color{28,29,32,33,4, 0, 0, 0};   
Color rightColor = Color{30,31,35,35, 5, 0, 0, 0};
struct Go {
  int forward;
  int revers;
  int spd;
};
Go leftMotor = Go{44, 45, 11};  //Указать Digital пины. ENA и EMB могут быть перепутаны.
Go rightMotor = Go{47, 46, 12}; //Указать Digital пины. ENA и EMB могут быть перепутаны.



struct {
  int err = 0;
  float integral = 0;
  float lastErr = 0;
  float kp = 200;
  float ki = 0;
  float kd = 0;
  float dt = 1;
  float OUT = 0;  //Переменная, в которую будут поступать данные из ПИДа.
  float plank = 0;  //Переменная для среднего числа датчика линии.
  float OUTprevious;
} pid;

int distance = 0;
float dt = 0.08;

const uint8_t SensorCount = 8;
int sensorValues[SensorCount];
int tmp = 0;
int sped = 100;
int freq = 0;

//функция для датчиков цвета
int color(Color funcColor, int k) {
  pinMode(funcColor.S0, OUTPUT);
  pinMode(funcColor.S1, OUTPUT);
  pinMode(funcColor.S2, OUTPUT);
  pinMode(funcColor.S3, OUTPUT);
  pinMode(funcColor.sensorOut, INPUT);
  digitalWrite(funcColor.S0, HIGH);
  digitalWrite(funcColor.S1, LOW);

  //R
  digitalWrite(funcColor.S2, 0);
  digitalWrite(funcColor.S3, 0);
  freq = pulseIn(funcColor.sensorOut, 0);
  funcColor.red = map(constrain(freq, 70, 120), 70, 120, 255, 0);
  Serial.print("\t");
  Serial.print("R");
  Serial.print(funcColor.red);


  //G
  digitalWrite(funcColor.S2, 1);
  digitalWrite(funcColor.S3, 1);
  freq = pulseIn(funcColor.sensorOut, 0);
  funcColor.green = map(constrain(freq, 100, 200), 100, 200, 255, 0);
  Serial.print("\t");
  Serial.print("G");
  Serial.print(funcColor.green);

  //B
  digitalWrite(funcColor.S2, 0);
  digitalWrite(funcColor.S3, 1);
  freq = pulseIn(funcColor.sensorOut, 0);
  funcColor.blue = map(constrain(freq, 60, 100), 60, 100, 255, 0);
  Serial.print("\t");
  Serial.print("B");
  Serial.print(funcColor.blue);
  Serial.println("\t");

  if((funcColor.red < 255)&&(funcColor.green == 255)&&(funcColor.blue <= 255)){   
    return k*5;
  }else{
    return 0;
  }
  
}


void moving (Go funcMotor, bool forvard, bool revers, int spd) {
  if (spd <= -1) {
    digitalWrite(funcMotor.forward, 0);
    digitalWrite(funcMotor.revers, 0);
    analogWrite(funcMotor.spd, 0);
  } else {
    digitalWrite(funcMotor.forward, forvard);
    digitalWrite(funcMotor.revers, revers);
    analogWrite(funcMotor.spd, spd);
  }
}


void setup() {

  /*leftServo.attach(leftServoPin);  // Привязка левой сервы к пину 6
    leftServo.write(0,255,false); // Начальное положение, выставляется с максимальной скоростью
    rightServo.attach(rightServoPin);  // Привязка правой сервы к пину 7
    rightServo.write(0,255,true);  // set the intial position of the servo, as fast as possible, wait until done
  */
  pinMode(LED_BUILTIN, OUTPUT);//Лампочка на плате для удобного отслеживания процесса калибровки.

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {
    A7, A6, A5, A4, A3, A2, A1, A0
  }, SensorCount);
  qtr.setEmitterPin(2);

  //Калибровка - снимаем 20 показаний с каждого датчика 10 раз, находим максимум и мунимум (чёрный и белый).
   for (uint8_t i = 0; i < 10; i++) {
    //moving(); В дальнейшем робот должен двигаться во время калибровки.
    digitalWrite(LED_BUILTIN, HIGH);
    qtr.calibrate();
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    qtr.calibrate();
    delay(500);
  }
  //Калибровка автоматичческая
  /*delay(1000);
   for (uint8_t i = 0; i < 5; i++) {
     moving(leftMotor,1,0,40);
     moving(rightMotor,0,1,40);
     qtr.calibrate();}
   for (uint8_t i = 0; i < 10; i++) {
     moving(leftMotor,0,1,80);
     moving(rightMotor,1,0,40);
     qtr.calibrate();}
   for (uint8_t i = 0; i < 5; i++) {
     moving(leftMotor,1,0,40);
     moving(rightMotor,0,1,40);
     qtr.calibrate();}
*/
  

  Serial.begin(9600);

}



void loop() {
  //Serial.println(sonar.ping_cm());
  distance = sonar.ping_cm();
  qtr.read(sensorValues);
  

  int local_k = -4; //Число от -4 до +4 (без нуля), на которое домножаем ошибку с каждого датчика слева направо [-4,-3,-2,-1,+1,+2,+3,+4].
  pid.err = 0;
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
 pid.plank = map(sensorValues[i], qtr.calibrationOn.minimum[i], qtr.calibrationOn.maximum[i], 0, 100);
      if (pid.plank<50){ // 79 - число, которое зависит исключительно от высоты, и от сенсы, которая нам нужна. Это 80% между 0-100%
      sensorValues[i] = 0;
    } else{
    sensorValues[i] = 1;}
  
 // Serial.print(sensorValues[i]* local_k);
  //Serial.print(" ");


  //Serial.print(pid.plank);
  //Serial.print(" ");

 pid.err += sensorValues[i] * local_k; //+ (color(leftColor, -1) + color(rightColor, 1));
  local_k++;
  if (local_k == 0) {
    local_k = 1; // Ноль пропустили
  }
  //Serial.print(sensorValues[i]);
  //Serial.print(" ");
  }
  //Serial.print("\t");
  
 /*Serial.print(color(leftColor, -1));
 Serial.print("\t");
 Serial.print(color(rightColor, 1));
 Serial.print("\t");
 Serial.println();*/
  
  //ПИД
  pid.err = 0 - pid.err;
  if ((pid.err == 0) && (sensorValues[3] == 1)&&(sensorValues[0] == 0)) {
    pid.integral = 0;
  }
  else {
    pid.integral = pid.integral + pid.err * pid.dt * pid.ki;
  }
  float D = (pid.err - pid.lastErr) / pid.dt;
  pid.lastErr = pid.err;
  pid.OUT = pid.err * pid.kp + pid.integral + D * pid.kd;
  //Конец ПИДов
  
  
  
  
  
  moving(leftMotor, 1, 0, sped - pid.OUT);//*constrain(abs((pid.OUT - pid.OUTprevious)*dt), 0.1,50));
  moving(rightMotor, 1, 0, sped + pid.OUT);//*constrain(abs((pid.OUT - pid.OUTprevious)*dt), 0.1,50));
 
  
  
  pid.OUTprevious = pid.OUT;
  
  Serial.print(pid.err);
  Serial.print("\t");
  Serial.print(pid.OUT);
  Serial.print("\t");
  Serial.print("  скорость -   ");
  Serial.print((sped - pid.OUT)*constrain(abs((pid.OUT - pid.OUTprevious)*dt), 1,50));
  Serial.print("   ");
  Serial.print((sped + pid.OUT)*constrain(abs((pid.OUT - pid.OUTprevious)*dt), 1,50));
  
  
  //leftServo.write(180,127,false);  //(градус (0-180), 127 - быстрая скорость, false значит, что серва будет будет крутиться одновременно со следующими до ближайшего true включительно)
  //rightServo.write(180,127,true);  // тут у сервы true - это значит, что эта серва и все сервы до неё (у которых false) работают одновременно
  //leftServo.write(0,30,false);     //(градус (0-180), 30 - медленная скорость, false значит, что серва будет будет крутиться одновременно со следующими до ближайшего true включительно)
  //rightServo.write(0,30,true);     // если непонятно, про true и false Ваня может нормально объяснить
  
  //Тут могут быть switch case по последнему положению lastFlag. Ссылка на код, где они остались  https://github.com/dmitrypavlikov/ResqueLine/blob/line-1.0/resqueLine1.ino



Serial.println();
}
