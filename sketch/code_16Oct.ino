#include "analogWrite.h"

int IN1 = 18;
int IN2 = 23;
int PWM = 2;
int ENCA = 32;
int ENCB = 34;

volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  analogWriteResolution(PWM, 8);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int target = Serial.parseInt();
  if(target != 0)
  {
    target = target * 46.0 * 11.0 / 360.0;
    PID_control(target);
  }
//  setMotor(1,125,PWM,IN1,IN2);
//  
//  // Read the position
//  int pos = 0; 
//  noInterrupts(); // disable interrupts temporarily while reading
//  pos = posi;
//  interrupts(); // turn interrupts back on
//  Serial.println(pos);
  
}

void PID_control(int target)
{
  while(1){
    
   // PID constants
  float kp = 10;
  float kd = 0.125;
  float ki = 0.4;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the positionL
  int pos = 0; 
  noInterrupts(); // disable interrupts temporarily while reading
  pos = posi;
  interrupts(); // turn interrupts back on
  
  // error
  float e = target - pos;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  if(dedt == 0)
    eintegral = eintegral + e*deltaT;
//  else
//    eintegral = 0;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print(" ");
  Serial.print(e);
  Serial.print(" ");
  Serial.print(dedt);
  Serial.print(" ");
  Serial.print(eintegral);
  Serial.print(" ");
  Serial.print(u);
  Serial.println();
  }
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
