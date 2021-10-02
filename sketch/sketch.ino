#include "analogWrite.h"
#define RESOLUTION 10 

int in1 = 18;
int in2 = 23;
int en = 2;

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  analogWriteResolution(en, 8);
}

void loop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);


  for(int i = 0; i <= 20; i++){
    analogWrite(en, i * 10);
    delay(100);
  }
  delay(2500);
  
  for(int i = 10; i >=20; i--){
    analogWrite(en, i* 10);
    delay(100);
  }
  delay(4000);
}
