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
  digitalWrite(en, HIGH);
  delay(1000);
  digitalWrite(en, LOW);
  delay(1000);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(en, HIGH);
  delay(1000);
  digitalWrite(en, LOW);
  delay(1000);
}
