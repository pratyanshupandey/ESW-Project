#include "analogWrite.h"
#include <time.h>

int IN1 = 18;
int IN2 = 23;
int PWM = 2;
int ENCA = 32;
int ENCB = 34;

volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;


uint lastTime = 0;
#define PID_TIMER 10000

void setup() {
     // put your setup code here, to run once:
    Serial.begin(9600);

    pinMode(ENCA,INPUT);
    pinMode(ENCB,INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);

    analogWriteResolution(PWM, 8);
    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    delay(1000);

}

void loop(){
  
  if (Serial.available() > 0) {
    // read the incoming byte:
    int num = Serial.parseInt();


    Serial.print("I received: ");
    Serial.println(num);
//    moveMotor(num);
    if(num != 0)
    {
        PID_control(num);
        setMotor(0,0,PWM,IN1,IN2);
    }
  }


}

void moveMotor(int tm)
{
      
  Serial.println("Start");
  uint start = millis();
  if(tm > 0)
    setMotor(1, 255, PWM, IN1, IN2);
  else
    setMotor(-1, 255, PWM, IN1, IN2);  

  tm = fabs(tm);

  while(true)
  {
    if (millis() - start > tm)
    {
      Serial.println("Stop");
      setMotor(0, 0, PWM, IN1, IN2);
      while(millis() - start < tm + 500);
      Serial.print("Final Pos: ");
      Serial.println(posi);
      Serial.println(posi * 0.64056939501);
      return;
    }
  }
}

void PID_control(float target)
{
    
    uint startTime = millis();
    uint lastTime = millis();
    bool use_integral = false;
    int md = 0;
    while(millis()- startTime < PID_TIMER){

      if(millis() - lastTime > 100){
        setMotor(0,0,PWM,IN1,IN2);

        // PID constants
        float kp = 10;
        float kd = 0.025;
        float ki = 5;

        // time difference
        long currT = micros();
        float deltaT = ((float) (currT - prevT))/( 1.0e6 );
        prevT = currT;

        // Read the positionL
        float pos = 0; 
        // noInterrupts(); // disable interrupts temporarily while reading
        pos = (float)posi * 0.63157894736;
        // interrupts(); // turn interrupts back on

        // error
        float e = target - pos;

        // derivative
        float dedt = (e-eprev)/(deltaT);

        // integral

        
        

        if(dedt == 0)
          use_integral = true;
        if (use_integral == true)
        {
            if (pos > target)
            {
              if (md == -1)
                eintegral = 0;
              md = 1;
            }
            else if (pos < target)
            {
              if (md == 1)
                eintegral = 0;
              md = -1;
            }
           
        
          eintegral = eintegral + e*deltaT;
        }
//        if(eintegral > 255)
//            eintegral = 255;
//        if (eintegral < -255)
//            eintegral = -255;

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
        lastTime = millis();

    }
    }
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
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
    analogWrite(pwm,pwmVal);
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
