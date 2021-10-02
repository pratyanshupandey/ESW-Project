#include <analogWrite.h>

int in1 = 0;
int in2 = 0;
int en = 0;


void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(en, OUTPUT);
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
