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
int BASESpeedA = 100;
int BASESpeedB = 100;
int maxspeedA = 200;
int maxspeedB = 200;
int P;
int I;
int D ;
float kp = 0.028;  // Works somewhat --  kp - 0.27 , kd - 0.81  --- 0.17  d-- 0.3 
float ki = 0.0000;
float kd = 0.28 ; // 4times  
float lasterror = 0 ;

void setup() {
  pinMode(A1, INPUT); // Set digital pin D1 as input pin
  pinMode(A2, INPUT); // Set digital pin D2 as input pin
  pinMode(A3, INPUT); // Set digital pin D3 as input pin
  pinMode(A4, INPUT); // Set digital pin D4 as input pin
  pinMode(A5, INPUT); // Set digital pin D5 as input pin
  pinMode(A6, INPUT); // Set digital pin D6 as input pin
  pinMode(A7, INPUT); // Set digital pin D7 as input pin
  pinMode(A0, INPUT); // Set digital pin D8 as input pin
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
  if((digitalRead(A0) == 1)&&(digitalRead(A1) ==1)&&(digitalRead(A2) == 1)&&(digitalRead(A3) == 1)&&(digitalRead(A4) ==0)&&(digitalRead(A5) == 0)&&(digitalRead(A6) == 0)&&(digitalRead(A7) ==0)) {ifright();}
  else if((digitalRead(A0) == 1)&&(digitalRead(A1) ==0)&&(digitalRead(A2) == 0)&&(digitalRead(A3) == 1)&&(digitalRead(A4) ==1)&&(digitalRead(A5) == 0)&&(digitalRead(A6) == 0)&&(digitalRead(A7) ==0)) {ifright();}
  else if((digitalRead(A0) == 1)&&(digitalRead(A1) ==1)&&(digitalRead(A2) == 0)&&(digitalRead(A3) == 1)&&(digitalRead(A4) ==1)&&(digitalRead(A5) == 0)&&(digitalRead(A6) == 0)&&(digitalRead(A7) ==0)) {ifright();}
  else if ( (digitalRead(A0) == 0)&&(digitalRead(A1)==0)&&(digitalRead(A2) == 0)&&(digitalRead(A3) == 1)&&(digitalRead(A4) ==1)&&(digitalRead(A5) == 0)&&(digitalRead(A6) == 0)&&(digitalRead(A7) ==1)) { ifleft();} 
  else if ( (digitalRead(A0) == 0)&&(digitalRead(A1)==0)&&(digitalRead(A2) == 0)&&(digitalRead(A3) == 1)&&(digitalRead(A4) ==1)&&(digitalRead(A5) == 0)&&(digitalRead(A6) == 1)&&(digitalRead(A7) ==1)) { ifleft();} 
  else if ( (digitalRead(A0) == 0)&&(digitalRead(A1)==0)&&(digitalRead(A2) == 0)&&(digitalRead(A3) == 1)&&(digitalRead(A4) ==1)&&(digitalRead(A5) == 1)&&(digitalRead(A6) == 1)&&(digitalRead(A7) ==1)) { ifleft();} 
  else {PID_control();} 
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
  delay (25);}
 
void forward (int SpeedA,int SpeedB) {
digitalWrite(M_A1,HIGH);
digitalWrite(M_A2, LOW);
 digitalWrite(M_B1, HIGH);
digitalWrite(M_B2, LOW);
analogWrite(PWMA, SpeedA); 
analogWrite(PWMB, SpeedB);}

void ifright(){
digitalWrite(M_A1,HIGH);
digitalWrite(M_A2, LOW);
 digitalWrite(M_B1, LOW);
digitalWrite(M_B2, HIGH);
analogWrite(PWMA, 255); 
analogWrite(PWMB, 255);
//delay (10);
} 

 
 void ifleft (){
digitalWrite(M_A1,LOW);
digitalWrite(M_A2, HIGH);
digitalWrite(M_B1, HIGH);
digitalWrite(M_B2, LOW);
analogWrite(PWMA, 255); 
analogWrite(PWMB, 255);
//delay (10) ;
}
