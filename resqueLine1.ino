#include <VarSpeedServo.h>  //Установи библиотеку, если не работает. Но много времени ей можно не уделяем, пока не поедем
#include <QTRSensors.h>
#include <NewPing.h>

//NewPing sonar(8, -, 500); // Сонар: Trig = 8, Echo нужно указать.

QTRSensors qtr;

/*VarSpeedServo leftServo; //Объекты серв
  VarSpeedServo rightServo; //Объекты серв
  const int leftServoPin = 6; // the digital pin used for the first servo
  const int rightServoPin = 7; // the digital pin used for the second servo
*/
struct Color {
  int S0;
  int S1;
  int S2;
  int S3;
  int sensorOut;
  int red, green, blue;
};
//Color leftColor = Color{-,-,-,-,4, 0, 0, 0};       //Указать Digital пины.
//Color rightColor = Color{-,-,-,-, 5, 0, 0, 0};     //Указать Digital пины.

struct Go {
  int forward;
  int revers;
  int spd;
};
Go leftMotor = Go{44, 45, 2};  //Указать Digital пины. ENA и EMB могут быть перепутаны.
Go rightMotor = Go{47, 46, 3}; //Указать Digital пины. ENA и EMB могут быть перепутаны.







struct {
  int err = 0;
  float integral = 0;
  float lastErr = 0;
  float kp = 3;
  float ki = 0.4;
  float kd = 0.3;
  float dt = 1;
  float OUT = 0;  //Переменная, в которую будут поступать данные из ПИДа.
  float plank = 0;  //Переменная для среднего числа датчика линии.
} pid;



const uint8_t SensorCount = 8;
int sensorValues[SensorCount];
int tmp = 0;
int sped = 20;
int freq = 0;


//функция для датчиков цвета
void color(Color funcColor) {
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
  Serial.println();
}


void moving (Go funcMotor, bool forvard, bool revers, int spd) {
  if (spd <= -1) {
    digitalWrite(funcMotor.forward, revers);
    digitalWrite(funcMotor.revers, forvard);
    analogWrite(funcMotor.spd, abs(spd));
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
    A0, A1, A2, A3, A4, A5, A6, A7
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
  /*
    for (uint8_t i = 0; i < 10; i++) {
      moving(leftMotor,1,0,60);
      moving(rightMotor,1,0,-60);
      qtr.calibrate();}
    for (uint8_t i = 0; i < 20; i++) {
      moving(leftMotor,1,0,-60);
      moving(rightMotor,1,0,60);
      qtr.calibrate();}
    for (uint8_t i = 0; i < 10; i++) {
      moving(leftMotor,1,0,60);
      moving(rightMotor,1,0,-60);
       qtr.calibrate();}

  */

  Serial.begin(9600);

}



void loop() {
  //Serial.println(sonar.ping_cm()); Проверка соника
  qtr.read(sensorValues);

  int local_k = -4; //Число от -4 до +4 (без нуля), на которое домножаем ошибку с каждого датчика слева направо [-4,-3,-2,-1,+1,+2,+3,+4].
  pid.err = 0;
  for (uint8_t i = 0; i < 8; i++) {
    //Serial.print(sensorValues[i]);
    //Serial.print(" ");

    pid.plank = ((qtr.calibrationOn.maximum[i] + qtr.calibrationOn.minimum[i]) / 1.92);
    if (sensorValues[i] < pid.plank) {

      //pid.plank = map(sensorValues[i], qtr.calibrationOn.minimum[i], qtr.calibrationOn.maximum[i], 0, 100);
      //if (pid.plank<79){

      sensorValues[i] = 0;
    }
    sensorValues[i] = 1;
  }
  // Serial.print(sensorValues[i]* local_k);
  //Serial.print(" ");
  //Serial.print(pid.plank);
  //Serial.print(" ");

  pid.err += sensorValues[i] * local_k;
  local_k++;
  if (local_k == 0) {
    local_k = 1; // Ноль пропустили
  }
  //Serial.print(sensorValues[i]);
  //Serial.print(" ");
}
//Serial.print("\t");

//ПИД
pid.err = 0 - pid.err;
if ((pid.err == 0) && (sensorValues[3] == 1)) {
  pid.integral = 0;
}
else {
  pid.integral = pid.integral + pid.err * pid.dt * pid.ki;
}

//ПИД№2
/*pid.err = 0 - pid.err;
  if ((pid.err == 0)&&(sensorValues[3] == 1)) { // &&(sensorValues[0] != 0))
  pid.dt = 0;
  pid.integral = 0;}
  else{
  pit.dt += 0.5;}
  pid.integral = pid.integral + pid.err * pid.dt * pid.ki;
*/

float D = (pid.err - pid.lastErr) / pid.dt;
pid.lastErr = pid.err;
pid.OUT = pid.err * pid.kp + pid.integral + D * pid.kd;
//Конец ПИДов





moving(leftMotor, 1, 0, sped - pid.OUT);
moving(rightMotor, 1, 0, sped + pid.OUT);
Serial.print(pid.err);
Serial.print("\t");
Serial.print(pid.OUT);
Serial.print("\t");
Serial.print("  скорость -   ");
Serial.print(sped - pid.OUT);
Serial.print("   ");
Serial.print(sped + pid.OUT);
Serial.println();
//color(rightColor);


//leftServo.write(180,127,false);  //(градус (0-180), 127 - быстрая скорость, false значит, что серва будет будет крутиться одновременно со следующими до ближайшего true включительно)
//rightServo.write(180,127,true);  // тут у сервы true - это значит, что эта серва и все сервы до неё (у которых false) работают одновременно
//leftServo.write(0,30,false);     //(градус (0-180), 30 - медленная скорость, false значит, что серва будет будет крутиться одновременно со следующими до ближайшего true включительно)
//rightServo.write(0,30,true);     // если непонятно, про true и false Ваня может нормально объяснить

//Если не получилось поднять сенсу у калибровки методом plank*1.5, из-за чего не получилось внедрить новое условие для обнуления I части ПИДа, или просто не сработало,
//Тут могут быть switch case по последнему положению lastFlag. Ссылка на код, где они остались  https://github.com/dmitrypavlikov/ResqueLine/blob/line-1.0/resqueLine1.ino



Serial.println();
}
