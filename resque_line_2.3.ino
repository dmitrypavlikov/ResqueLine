#include <NewPing.h>
#include <QTRSensors.h>
#define TRIGGER_PIN  6
#define ECHO_PIN     7
#define MAX_DISTANCE 400


#define leftmotor_dir 45
#define leftmotor_spd 44
#define rightmotor_dir 47
#define rightmotor_spd 46
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int spd = 60;
int line = qtr.calibrationOn.minimum[4];
int sV[SensorCount];
int k;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);




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
  pinMode(LED_BUILTIN, OUTPUT);

/*delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(leftmotor_dir, HIGH);
      analogWrite(leftmotor_spd, spd);
      digitalWrite(rightmotor_dir, HIGH);
      analogWrite(rightmotor_spd, spd);
      for (uint16_t i = 0; i < 15; i++)
  {
    qtr.calibrate();

  }digitalWrite(leftmotor_dir, LOW);
      analogWrite(leftmotor_spd, spd);
      digitalWrite(rightmotor_dir, LOW);
      analogWrite(rightmotor_spd, spd);
   for (uint16_t i = 0; i < 30; i++)
  {  
    qtr.calibrate();

  }digitalWrite(leftmotor_dir, HIGH);
      analogWrite(leftmotor_spd, spd);//
      digitalWrite(rightmotor_dir, HIGH);
      analogWrite(rightmotor_spd, spd);
    
    for (uint16_t i = 0; i < 15; i++)
  {
qtr.calibrate();
  }
  analogWrite(leftmotor_spd, spd-spd);
   analogWrite(rightmotor_spd, spd-spd);
   delay(200);
*/

}

void moving (bool Left, bool Right, int spd_L, int spd_R){
  Left = not Left;
  digitalWrite(leftmotor_dir, Left);
  analogWrite(leftmotor_spd, spd_L);
  digitalWrite(rightmotor_dir, Right);
  analogWrite(rightmotor_spd, spd_R);
}



int sonic (){
  int mass[4];
  int k = 0;
  int middle = 0;
  if (k == 3){
    for(int i = 0; i<4; i++){
      k = 0;
      middle += mass[i];
      mass[i] = k;
    }
    middle = middle/4;
  }
  mass[k] = sonar.ping_cm();
  k++;
  return(middle);
}


void loop() {
 ///////////////////////////////////
 //LeftMotor LOW - вперёд
 //RightMotor HIGH - вперед7
   digitalWrite(LED_BUILTIN, LOW);
  qtr.read(sensorValues);
  for (int i = 0; i < SensorCount; i++)
  {
    sV[i] = sensorValues[i];
    Serial.print(sV[i]);
    Serial.print('\t');
  }
  Serial.print(sonic());
  Serial.print(sonar.ping_cm());
  moving(1,1,60,60);
  Serial.println(); 
  }

  
  
  
 


