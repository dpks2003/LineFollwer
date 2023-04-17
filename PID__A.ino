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
int BASESpeedA = 200;
int BASESpeedB = 200;
int maxspeedA = 250;
int maxspeedB = 250;
int P;
int I;
int D ;
float kp = 0.08 ;
float ki = 0.0000007  ;
float kd = 0.1 ;
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
  PID_control();
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
  forward (motorspeeda, motorspeedb); }
 
void forward (int SpeedA,int SpeedB) {
digitalWrite(M_A1,HIGH);
digitalWrite(M_A2, LOW);
 digitalWrite(M_B1, HIGH);
digitalWrite(M_B2, LOW);
analogWrite(PWMA, SpeedA); 
analogWrite(PWMB, SpeedB);}
  
