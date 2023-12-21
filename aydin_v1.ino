int mR_F=11;
int mR_B=10;
int mL_F=8;
int mL_B=9;

#define CLK1 51
#define DT1 50

#define CLK2 53
#define DT2 52

int diameter=15; //in cm
double dL;
double dR;
double dx;
double dy;
double d;
double dw;
double w0;

int counter1 = 0;
int currentStateCLK1;
int lastStateCLK1;
String currentDir1 ="";

int counter2 = 0;
int currentStateCLK2;
int lastStateCLK2;
String currentDir2 ="";
//Linear
double linear_x;
double linear_y;
double linear_x0;
double linear_y0;
//Twist
double w;

void enkoder_oku(int CLK,int DT,int &counter,int &currentStateCLK,int &lastStateCLK,String &currentDir){
currentStateCLK = digitalRead(CLK);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){

    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DT) != currentStateCLK) {
      counter --;
      currentDir ="CCW";
    } else {
      // Encoder is rotating CW so increment
      counter ++;
      currentDir ="CW";
    }

    Serial.print("Direction: ");
    Serial.print(currentDir);
    Serial.print(" | Counter: ");
    Serial.println(counter);
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;  
}

void setup() {
  pinMode(mR_F,OUTPUT);
  pinMode(mR_B,OUTPUT);
  pinMode(mL_F,OUTPUT);
  pinMode(mL_B,OUTPUT);
  
  pinMode(CLK1,INPUT);
  pinMode(DT1,INPUT);

  pinMode(CLK2,INPUT);
  pinMode(DT2,INPUT);

  lastStateCLK1 = digitalRead(CLK1);
  lastStateCLK2 = digitalRead(CLK2);
  // Setup Serial Monitor
  Serial.begin(9600);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  enkoder_oku(CLK1,DT1,counter1,currentStateCLK1,lastStateCLK1,currentDir1);
  enkoder_oku(CLK2,DT2,counter2,currentStateCLK2,lastStateCLK2,currentDir2);
  delay(1);
  dR=counter1*(15*3.14)/40;
  dL=counter2*(15*3.14)/40;

  d=(dL+dR)/2; //gidilen yol
  dw=(dR-dL)/26; //change in degree

  dx=d*cos(dw+w0/2);
  dy=d*sin(dw+w0/2);

  linear_x = (linear_x0+dx);
  linear_y = (linear_y0+dy);
  w = w0+ dw;
  Serial.print("linear_x ");
  Serial.print(linear_x);
  Serial.print(" | linear_y ");
  Serial.println(linear_y);
  //analogWrite(mR_F,50);
  //analogWrite(mL_F,50);
}
