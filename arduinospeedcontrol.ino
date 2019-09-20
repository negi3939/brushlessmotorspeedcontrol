/*
moterdrive pikupiku kando 3000 bai
*/
#include <MsTimer2.h>

#define IN1 3
#define IN2 5
#define IN3 6
 
#define H1 21
#define H2 20
#define H3 19

void HollA();
void HollB();
void HollC();
#define TIME (5)
unsigned int pospeed=0;
unsigned int speed=0;
volatile int HOLL_DIFF_WAIT_UPDATE=0;
//状態変数
float STRING_LENGTH=0;
float STRING_VELOCITY=0;
void TimerInterrupt();
void VelocityBiquad();

void setup() {
   Serial.begin(9600); 
   pinMode(IN1,OUTPUT);
   pinMode(IN2,OUTPUT);
   pinMode(IN3,OUTPUT);
   attachInterrupt(2,HollA,CHANGE);
   attachInterrupt(3,HollB,CHANGE);
   attachInterrupt(4,HollC,CHANGE);
   MsTimer2::set(1, VelocityBiquad); // 1ms period
   MsTimer2::start();
   pospeed = 1;
   Serial.begin(9600);
}

void ffloop(){
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);
    delay(TIME);
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,LOW);
    delay(TIME);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,LOW);
    delay(TIME);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,HIGH);
    delay(TIME);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,HIGH);
    delay(TIME);
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,HIGH);
    delay(TIME);
}

void halltopwmdigi(){
  if((digitalRead(H1)==HIGH)&&(digitalRead(H2)==LOW)&&(digitalRead(H3)==LOW)){
    analogWrite(IN1,speed);
    analogWrite(IN2,speed/2);
    digitalWrite(IN3,LOW);
  }
  if((digitalRead(H1)==HIGH)&&(digitalRead(H2)==HIGH)&&(digitalRead(H3)==LOW)){
    analogWrite(IN1,speed/2);
    analogWrite(IN2,speed);
    digitalWrite(IN3,LOW);
  }
  if((digitalRead(H1)==LOW)&&(digitalRead(H2)==HIGH)&&(digitalRead(H3)==LOW)){
    digitalWrite(IN1,LOW);
    analogWrite(IN2,speed);
    analogWrite(IN3,speed/2);
  }
  if((digitalRead(H1)==HIGH)&&(digitalRead(H2)==HIGH)&&(digitalRead(H3)==HIGH)){
    digitalWrite(IN1,LOW);
    analogWrite(IN2,speed);
    analogWrite(IN3,speed/2);
  }
  if((digitalRead(H1)==LOW)&&(digitalRead(H2)==HIGH)&&(digitalRead(H3)==HIGH)){
    digitalWrite(IN1,LOW);
    analogWrite(IN2,speed/2);
    analogWrite(IN3,speed);
  }
  if((digitalRead(H1)==LOW)&&(digitalRead(H2)==LOW)&&(digitalRead(H3)==HIGH)){
    analogWrite(IN1,speed/2);
    digitalWrite(IN2,LOW);
    analogWrite(IN3,speed);
  }
  if((digitalRead(H1)==HIGH)&&(digitalRead(H2)==LOW)&&(digitalRead(H3)==HIGH)){
    analogWrite(IN1,speed);
    digitalWrite(IN2,LOW);
    analogWrite(IN3,speed/2);
  }
  if((digitalRead(H1)==LOW)&&(digitalRead(H2)==LOW)&&(digitalRead(H3)==LOW)){
    analogWrite(IN1,speed);
    digitalWrite(IN2,LOW);
    analogWrite(IN3,speed/2);
  }
}

void UpdateHollPhase() {
  byte Aread = PIND & _BV(0); //digitalRead(21);//PD0
  byte Bread = (PIND & _BV(1))>>1; //digitalRead(20);//PD1
  byte Cread = (PIND & _BV(2))>>2; //digitalRead(19);//PD2
  byte Phase = ((Cread & (!Aread)) << 2) | ((Bread & (!Cread)) << 1) | ((Aread ^ Bread)^Cread);
  static byte oldPhase = Phase;
  int DIFF = (int)Phase - (int)oldPhase;
  
  if (DIFF == -5) {
    DIFF = 1;
  } else if (DIFF == 5) {
    DIFF = -1;
  }
  HOLL_DIFF_WAIT_UPDATE+=DIFF;//↑の計算が重すぎるので外に出す
  oldPhase = Phase;
}


void VelocityBiquad() {
  static float last_length;
  float input = (STRING_LENGTH - last_length) * 1000;
  const float  a0  = 1.415626938 ;
  const float a1  = -1.618033989  ;
  const float a2  = 0.584373062 ;
  const float b0  = 0.095491503 ;
  const float b1  = 0.190983006 ;
  const float b2  = 0.095491503 ;
  static float in1, in2, out1, out2;
  STRING_VELOCITY = b0 / a0 * input + b1 / a0 * in1 + b2 / a0 * in2 - a1 / a0 * out1 - a2 / a0 * out2;
  //STRING_VELOCITY=input;
  in2 = in1;
  in1 = input;
  out2 = out1;
  out1 = STRING_VELOCITY;
  last_length = STRING_LENGTH;
}

void HollA() {
  halltopwmdigi();
  UpdateHollPhase();
}
void HollB() {
  halltopwmdigi();
  UpdateHollPhase();
}
void HollC() {
  halltopwmdigi();
  UpdateHollPhase();
}

float kp = 0.01;
float ki = 0.01;
float dt = 0.01;
float integral;
void loop(){
   speed = 50;
}
