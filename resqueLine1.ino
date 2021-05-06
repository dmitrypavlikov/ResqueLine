#include <QTRSensors.h>
#include <NewPing.h>

NewPing sonar(6, 7, 500);
QTRSensors qtr;

struct Color {
  int S0;
  int S1;
  int S2;
  int S3;
  int sensorOut;
  int red, green, blue;
};
Color leftColor = Color{12, 3, 4, 5, 6, 0, 0, 0};
Color rightColor = Color{7, 8, 9, 10, 11, 0, 0, 0};

struct Go {
  int forward;
  int revers;
  int spd;
};
Go leftMotor = Go{9, 10, 11};
Go rightMotor = Go{8, 7, 6};

struct {
  int err = 0;
  float integral = 0;
  float lastErr = 0;
  float kp = 3;
  float ki = 0.5;
  float kd = 0.5;
  float dt = 1;
  float OUT = 0;//Переменная, в которую будут поступать данные из ПИДа. Просто для удобства, можно удалить.
  int plank = 0;
} pid;



const uint8_t SensorCount = 8;
int sensorValues[SensorCount];
int tmp = 0; //для того чтобы лишь 1 раз назначить выходы пинов



//функция для датчиков цвета color(leftcolor, 50
void color(Color funcColor, int freq = 0) {
  if (tmp = 0) {
    pinMode(funcColor.S0, OUTPUT);
    pinMode(funcColor.S1, OUTPUT);
    pinMode(funcColor.S2, OUTPUT);
    pinMode(funcColor.S3, OUTPUT);
    pinMode(funcColor.sensorOut, INPUT);
    digitalWrite(funcColor.S0, HIGH);
    digitalWrite(funcColor.S1, LOW);
    tmp++;
  }

  //R
  digitalWrite(funcColor.S2, 0);
  digitalWrite(funcColor.S3, 0);
  freq = pulseIn(funcColor.sensorOut, 0);
  funcColor.red = map(constrain(freq, 70, 120), 70, 120, 255, 0);
  delay(100);

  //G
  digitalWrite(funcColor.S2, 1);
  digitalWrite(funcColor.S3, 1);
  freq = pulseIn(funcColor.sensorOut, 0);
  funcColor.green = map(constrain(freq, 100, 200), 100, 200, 255, 0);
  delay(100);

  //B
  digitalWrite(funcColor.S2, 0);
  digitalWrite(funcColor.S3, 1);
  freq = pulseIn(funcColor.sensorOut, 0);
  funcColor.blue = map(constrain(freq, 60, 100), 60, 100, 255, 0);
  delay(100);
}

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);//Лампочка на плате для удобного отслеживания процесса калибровки.

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {
    A0, A1, A2, A3, A4, A5, A6, A7
  }, SensorCount);
  qtr.setEmitterPin(2);

  //Калибровка - снимаем 20 показаний с каждого датчика, находим максимум и мунимум (чёрный и белый).
  for (uint8_t i = 0; i < 10; i++) {
    //moving(0,0,0,0); В дальнейшем робот должен двигаться во время калибровки.
    digitalWrite(LED_BUILTIN, HIGH);
    qtr.calibrate();
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    qtr.calibrate();
    delay(500);
  }

  Serial.begin(9600);

}

//Функция движения, остаётся просто для примера ниже.
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

void loop() {
  //Serial.println(sonar.ping_cm());
  qtr.read(sensorValues);
  int local_k = -4; //Число от -4 до +4 (без нуля), на которое домножаем ошибку с каждого датчика слева направо [-4,-3,-2,-1,+1,+2,+3,+4].
  pid.err = 0;
  for (uint8_t i = 0; i < 8; i++) {
    pid.plank = (qtr.calibrationOn.maximum[i] + qtr.calibrationOn.minimum[i]) / 2;
    if (sensorValues[i] < pid.plank) {
      sensorValues[i] = 0;
    }
    else {
      sensorValues[i] = 1;
    }
    Serial.print(sensorValues[i]* local_k);
    Serial.print(" ");

    pid.err += sensorValues[i] * local_k;
    local_k++;
    if (local_k == 0) {
      local_k = 1; // Ноль пропустили
    }
    //Serial.print(sensorValues[i]);
    //Serial.print(" ");
  }
  Serial.print("\t");

  //ПИД
  pid.err = 0 - pid.err;
  if (pid.err == 0) {
    pid.integral = 0;
  }
  else {
    pid.integral = pid.integral + pid.err * pid.dt * pid.ki;
  }
  float D = (pid.err - pid.lastErr) / pid.dt;
  pid.lastErr = pid.err;
  //pid.OUT = map(constrain(pid.err * pid.kp + pid.integral + pid.D * pid.kd, -100000, 100000), -100000,100000, -100,100);
  pid.OUT = pid.err * pid.kp + pid.integral + D * pid.kd;
  //moving(1,1,spd-pid.OUT,1,1,spd+pid.OUT);

  Serial.print(pid.err);
  Serial.print("\t");
  Serial.print(pid.OUT);
  Serial.print("\t");
  Serial.print(" ");
  Serial.print("   ");
  Serial.print(" ");
  Serial.println();



}
