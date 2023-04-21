#include <QTRSensors.h>

#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN //Emitter Pin IS always high connected to 5v

// sensors 0 through 8 are connected to analog inputs A0 through A7, respectively
QTRSensorsAnalog qtra((unsigned char[]) {A0, A1, A2, A3, A4, A5, A6, A7},
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
int PWMA = 3;  //speed motor a
int M_A1 = 4; //motor a = +
int M_A2 = 5; //motor a = -
int M_B1 = 7; //motor b = -
int M_B2 = 8; //motor b = +
int PWMB = 9; 
int STB = 6;
int BASESpeedA = 70;
int BASESpeedB = 70;
int maxspeedA = 200;
int maxspeedB = 200;
int P;
int I;
int D ;
float kp = 0.020;  // Works somewhat --  kp - 0.27 , kd - 0.81  --- 0.17  d-- 0.3 
float ki = 0.000;
float kd = 2 ; // 4times  
float lasterror = 0 ;

void setup() {
pinMode(M_B1, OUTPUT);
pinMode(M_B2, OUTPUT);
pinMode(M_A1, OUTPUT);
pinMode(M_A2, OUTPUT);
pinMode(PWMA, OUTPUT);
pinMode(PWMB, OUTPUT);
digitalWrite(STB ,HIGH);

digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtra.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop () {
  uint16_t position = qtra.readLine(sensorValues);
  if (position < 1000) { right75(); }

  else if (position >= 6000 && position < 7000) 
{ left75();}
else if (position <6000 && position >1000)
{PID_control();}
}
//void calibration() {
  //digitalWrite(LED_BUILTIN, HIGH);
  //for (uint16_t i = 0; i < 400; i++)
  //{
   // qtra.calibrate();
  //}
  //digitalWrite(LED_BUILTIN, LOW);
//
void PID_control() {
  uint16_t position = qtra.readLine(sensorValues); //read the current position
  int error = 3500 - position; //3500 is the ideal position (the centre)

  P = error;
  I = I + error;
  D = error - lasterror;
  lasterror = error;
  int motorspeed = P*kp + I*ki + D*kd; //calculate the correction
                                       //needed to be applied to the speed
  
  int motorspeeda = BASESpeedA + motorspeed;
  int motorspeedb = BASESpeedB - motorspeed;
  
  if (motorspeeda > maxspeedA) {
    motorspeeda = maxspeedA;
  }
  if (motorspeedb > maxspeedB) {
    motorspeedb = maxspeedB;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  }
  forward (motorspeeda, motorspeedb); 
  delay (10);}
 
void forward (int SpeedA,int SpeedB) {
digitalWrite(M_A1,HIGH);
digitalWrite(M_A2, LOW);
 digitalWrite(M_B1, HIGH);
digitalWrite(M_B2, LOW);
analogWrite(PWMA, SpeedA); 
analogWrite(PWMB, SpeedB);}

void right75 () // 
{
digitalWrite(M_A1,HIGH);
digitalWrite(M_A2, LOW);
 digitalWrite(M_B1, LOW);
digitalWrite(M_B2, LOW);
analogWrite(PWMA, 255); 
analogWrite(PWMB, 0);}

void left75 () // 
{
digitalWrite(M_A1,LOW);
digitalWrite(M_A2, LOW);
 digitalWrite(M_B1, HIGH);
digitalWrite(M_B2, LOW);
analogWrite(PWMA, 0); 
analogWrite(PWMB, 255);}
  
