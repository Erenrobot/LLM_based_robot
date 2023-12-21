int mR_F=11;
int mR_B=10;
int mL_F=8;
int mL_B=9;
void setup() {
  pinMode(mR_F,OUTPUT);
  pinMode(mR_B,OUTPUT);
  pinMode(mL_F,OUTPUT);
  pinMode(mL_B,OUTPUT);
  

}

void loop() {
  analogWrite(mR_F,50);

}
