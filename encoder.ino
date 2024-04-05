#include <PID_v1.h>
#include <Encoder.h>
#include <Servo.h>

#define Joint1Pin 50
#define Joint2Pin 51
#define Joint3Pin 52
#define GripperPin 53
int frontright = 13;
int frontleft = 12;
int pwmfrontl = 3;
int pwmfrontr = 11;
int horizonback = 4;
int horizonfront = 7;
int pwmhorif = 6;
int pwmhorib = 5;


Servo Joint1;
Servo Joint2;
Servo Joint3;
Servo Gripper;

// Set initial joint angles
int Joint1Angle = 90;
int Joint2Angle = 90;
int Joint3Angle = 90;
int GripperOpen = 0;
int GripperClose = 60;
// Joint Angle Offsets
int Joint1Offset = 42; // Your value may be different
int Joint2Offset = -48; // Your value may be different
int Joint3Offset = 10; // Your value may be different

bool spin = false;

double setPointL, outputL;
double setPointR, outputR;
double LeftReading, RightReading;
double setPointF, outputF;
double setPointB, outputB;
double FrontReading, BackReading;

// parameters of robot arm
double L2 = 0.095;
double L1 = 0;
double L3 = 0.15;


Encoder knobLeft(18, 22);
Encoder knobRight(19, 23);
Encoder knobFront(20, 24);
Encoder knobBack(21, 25);

double Kp = 0.1, Ki = 0.1, Kd = 0;
PID leftCTL(&LeftReading, &outputL, &setPointL, Kp, Ki, Kd, DIRECT);
PID rightCTL(&RightReading, &outputR, &setPointR, Kp, Ki, Kd, DIRECT);
PID frontCTL(&FrontReading, &outputF, &setPointF, Kp, Ki, Kd, DIRECT);
PID backCTL(&BackReading, &outputB, &setPointB, Kp, Ki, Kd, DIRECT);

void setup() {
  setPointL = 0;
  setPointR = 0;
  setPointF = 0;
  setPointB = 0;

  Joint1.attach(Joint1Pin);
  Joint2.attach(Joint2Pin);
  Joint3.attach(Joint3Pin);
  Gripper.attach(GripperPin);

  Joint1.write(Joint1Angle-Joint1Offset);
  Joint2.write(Joint2Angle-Joint2Offset);
  Joint3.write(Joint3Angle-Joint3Offset);

  Gripper.write(GripperOpen);
  leftCTL.SetMode(MANUAL);
  rightCTL.SetMode(MANUAL);
  frontCTL.SetMode(MANUAL);
  backCTL.SetMode(MANUAL);

  Serial.begin(9600);
  pinMode(frontright, OUTPUT);
  pinMode(pwmfrontl, OUTPUT);
  pinMode(frontleft, OUTPUT);
  pinMode(pwmfrontr, OUTPUT);
  pinMode(horizonback, OUTPUT);
  pinMode(pwmhorif, OUTPUT);
  pinMode(horizonfront, OUTPUT);
  pinMode(pwmhorib, OUTPUT);

  leftCTL.SetSampleTime(5);
  rightCTL.SetSampleTime(5);
  frontCTL.SetSampleTime(5);
  backCTL.SetSampleTime(5);

}

long positionLeft  = -999;
long positionRight = -999;
long positionFront  = -999;
long positionBack = -999;

void loop() {
  LeftReading = knobLeft.read();
  RightReading = knobRight.read();
  FrontReading = knobFront.read();
  BackReading = knobBack.read();
  long newLeft, newRight;
  long newFront, newBack;
  newFront = knobFront.read();
  newBack= knobBack.read();
  if (spin == false){
    digitalWrite(horizonback, HIGH);
    digitalWrite(horizonfront, LOW);
    analogWrite(pwmhorib,50);
    analogWrite(pwmhorif,50);
    if (knobBack.read() >= 1000){
      analogWrite(pwmhorif,0);
      analogWrite(pwmhorib,0);
      spin = true;
    }
  }
  else if (spin == true){
  //   leftCTL.SetMode(AUTOMATIC);
  //   rightCTL.SetMode(AUTOMATIC);
    frontCTL.SetMode(AUTOMATIC);
    backCTL.SetMode(AUTOMATIC);
    frontCTL.Compute();
    backCTL.Compute();
    Serial.print("Frontspped = ");
    Serial.print(outputF);
    Serial.print("Backspeed = ");
    Serial.print(outputB);
    Serial.print("Front = ");
    Serial.print(newFront);
    Serial.print(", Back = ");
    Serial.print(newBack);
    Serial.println();
    
    // 设置PID输出限幅
    leftCTL.SetOutputLimits(-255, 255);
    rightCTL.SetOutputLimits(-255, 255);
    frontCTL.SetOutputLimits(-255, 255);
    backCTL.SetOutputLimits(-255, 255);
    // 根据输出方向设置电机的旋转方向和PWM值
    if ((outputF >= 0) & (outputB >= 0)){
      digitalWrite(horizonfront, HIGH);
      analogWrite(pwmhorif,abs(outputF));
      digitalWrite(horizonback, HIGH);
      analogWrite(pwmhorib,abs(outputB));
    }
    else if ((outputF >= 0) & (outputB < 0)){
      digitalWrite(horizonfront, HIGH);
      analogWrite(pwmhorif,abs(outputF));
      digitalWrite(horizonback, LOW);
      analogWrite(pwmhorib,abs(outputB));
    }

    else if ((outputF < 0) & (outputB >= 0)){
      digitalWrite(horizonfront, LOW);
      analogWrite(pwmhorif,abs(outputF));
      digitalWrite(horizonback, HIGH);
      analogWrite(pwmhorib,abs(outputB));
    }
    else if((outputF < 0) & (outputB < 0)){
      digitalWrite(horizonfront, LOW);
      analogWrite(pwmhorif,abs(outputF));
      digitalWrite(horizonback, LOW);
      analogWrite(pwmhorib,abs(outputB));
    }


    if ((abs(knobFront.read()) <=2) && (abs(knobBack.read()) <=2)){
      analogWrite(pwmhorif,0);
      analogWrite(pwmhorib,0);
      frontCTL.SetMode(MANUAL);
      backCTL.SetMode(MANUAL);

    }
  }

  
}
