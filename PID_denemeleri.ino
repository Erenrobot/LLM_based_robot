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


//Linear
double linear_x;
double linear_y;
double linear_x0;
double linear_y0;
//Twist
double w;

// PID parameters for each motor
double Kp = 1.0, Ki = 0.1, Kd = 0.01;
double integralR = 0, integralL = 0;
double previous_errorR = 0, previous_errorL = 0;
double setSpeedR = 50; // desired speed for right motor
double setSpeedL = 50; // desired speed for left motor cm/saniye
double seconds=0;
double actualSpeedL = 0;
double actualSpeedR = 0;

int counterl = 0;
  int counterl_eski=0;
  int currentStateCLKl;
  int lastStateCLKl;
  String currentDirl ="";

  int counterr = 0;
  int counterr_eski=0;
  int currentStateCLKr;
  int lastStateCLKr;
  String currentDirr ="";
unsigned long startTimeL;
unsigned long startTimeR;
unsigned long duration;
int PID_counter=0;
double calculatePID(double setSpeed, double actualSpeed, double &integral, double &previous_error) {
    double error = setSpeed - actualSpeed;
    integral += error;
    double derivative = error - previous_error;
    double output = Kp*error + Ki*integral + Kd*derivative;
    previous_error = error;
    return output;
}

void enkoder_oku(unsigned long &startTimeL,unsigned long &startTimeR,double &actualSpeedL,double &actualSpeedR ){
  #define CLKl 51
  #define DTl 50

  #define CLKr 53
  #define DTr 52
  double passed_time;
  unsigned long endTime;
  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //LEFT MOTOR CALCULATIONS
  currentStateCLKl = digitalRead(CLKl);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLKl != lastStateCLKl  && currentStateCLKl == 1){

    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DTl) != currentStateCLKl) {
      counterl --;
      currentDirl ="CCW";
      endTime = millis();
      passed_time= endTime-startTimeL;
      startTimeL = millis();
      actualSpeedL = ((15*3.14)*1/20)/(passed_time/1000);
    } else {
      // Encoder is rotating CW so increment
      counterl ++;
      currentDirl ="CW";
      endTime = millis();
      passed_time= endTime-startTimeL;
      startTimeL = millis();
      actualSpeedL = -((15*3.14)*1/20)/(passed_time/1000);
    }

    Serial.print("Direction of Left Motor: ");
    Serial.print(currentDirl);
    Serial.print(" | Counter of Left Motor: ");
    Serial.println(counterl);
    Serial.print(" |Actual Speed of Left Wheel ");
    Serial.println(actualSpeedL);
  }

  // Remember last CLK state
  lastStateCLKl = currentStateCLKl;

  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //RIGHT MOTOR CALCULATIONS

  currentStateCLKr = digitalRead(CLKr);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLKr != lastStateCLKr  && currentStateCLKr == 1){

    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DTr) != currentStateCLKr) {
      counterr --;
      currentDirr ="CCW";
      endTime = millis();
      passed_time= endTime-startTimeR;
      actualSpeedR = ((15*3.14)*1/20)/(passed_time/1000);
      startTimeR = millis();
    } else {
      // Encoder is rotating CW so increment
      counterr ++;
      currentDirr ="CW";
      endTime = millis();
      passed_time= endTime-startTimeR;
      actualSpeedR = -((15*3.14)*1/20)/(passed_time/1000);
      startTimeR = millis();
    }

    Serial.print("Direction of Left Motor: ");
    Serial.print(currentDirr);
    Serial.print(" | Counter of Left Motor: ");
    Serial.println(counterr);
  }

  // Remember last CLK state
  lastStateCLKr = currentStateCLKr;

  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //Position calculations global

  dR=counterl*(15*3.14)/20;
  dL=counterr*(15*3.14)/20;

  d=(dL+dR)/2; //gidilen yol
  dw=(dR-dL)/26; //change in degree

  dx=d*cos(dw+w0/2);
  dy=d*sin(dw+w0/2);

  linear_x = (linear_x0+dx);
  linear_y = (linear_y0+dy);
  w = w0+ dw;

  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //current speed calculations
  
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

  lastStateCLKl = digitalRead(51);
  lastStateCLKr = digitalRead(53);
  // Setup Serial Monitor
  Serial.begin(9600);

  
}

void loop() {
  
  
  delay(3);
  enkoder_oku(startTimeL,startTimeR,actualSpeedL,actualSpeedR);
  
 
  //Serial.println(counter1);



  
    // Calculate PID output
    //double pidOutputR = calculatePID(setSpeedR, actualSpeedR, integralR, previous_errorR);
    //double pidOutputL = calculatePID(setSpeedL, actualSpeedL, integralL, previous_errorL);

    // Adjust motor speed
    // Ensure the output is within valid PWM range and apply to motors
    //analogWrite(mR_F, constrain(pidOutputR, 0, 255));
    //analogWrite(mL_F, constrain(pidOutputL, 0, 255));
    
  
  
  
  
//  Serial.print("linear_x ");
//  Serial.print(linear_x);
//  Serial.print(" | linear_y ");
//  Serial.println(linear_y);
  //analogWrite(mR_F,50);
  analogWrite(mL_F,50);
  
  
  
  PID_counter+=1;
  
}
