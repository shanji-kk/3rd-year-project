#include <PID_v1.h>
#include <Encoder.h>
#include <Servo.h>
#include "Encoder_funcs.h"

#define Joint1Pin 50
#define Joint2Pin 51
#define Joint3Pin 52
#define GripperPin 53
// motor pins for shield board 1
int horizonback = 4;
int horizonfront = 7;
int pwmhorif = 6;
int pwmhorib = 5;
//shield borad 2 buildin pins
int frontright = 12;
int frontleft = 13;
int pwmfrontl = 3;
int pwmfrontr = 11;

// IR sensro for road tracking and pick detection
int IRSensorleft = 30; 
int IRSensorright = 31;
//ultrasonic snesor pins
int trigPin = 9;
int echoPin = 10;

float duration, distance;
// Servo Objects
Servo Joint1;
Servo Joint2;
Servo Joint3;
Servo Gripper;

// Set initial joint angles
int Joint1Angle = 90;
int Joint2Angle = 90;
int Joint3Angle = 90;
int GripperOpen = 20;
int GripperClose = 60;
// Joint Angle Offsets
int Joint1Offset = 3; // Your value may be different
int Joint2Offset = 50; // Your value may be different
int Joint3Offset = -13; // Your value may be different
int counter = 0;

// parameters of robot arm
double L2 = 0.095;
double L1 = 0;
double L3 = 0.15;

// parameters for PID control
double setPointL, outputL, setPointR, outputR;
double LeftReading, RightReading, FrontReading, BackReading;
double setPointF, outputF,setPointB, outputB;
double inputL,inputR,inputF,inputB;
double outputL2,outputR2,setPointL2, setPointR2;
double outputF2,outputB2,setPointF2, setPointB2;
double Frontdis,Backdis;

// int for serial reading
int op;


//flags for main code
bool Walking = true; // flag for walking forward
bool grab = true; // Flag to detect whether holding the item
bool place = false;
bool reposition = false;
bool repositionh = false;
bool repositionv = false;
bool pickver = false;
bool pickhor = false;
bool pick = false;
bool put = false;
bool repositionput = false;
int count = 0;

//define encoders
Wheels knobLR(18,22,19, 23);
Wheels knobFB(20, 24,21, 25);
Encoder knobLeft(18,22);
Encoder knobRight(19,23);
Encoder knobFront(20,24);
Encoder knobBack(21,25);

//PID control for reposition
double Kp = 0.1, Ki = 0.1, Kd = 0;
PID leftCTL(&LeftReading, &outputL, &setPointL, Kp, Ki, Kd, DIRECT);
PID rightCTL(&RightReading, &outputR, &setPointR, Kp, Ki, Kd, DIRECT);
PID frontCTL(&FrontReading, &outputF, &setPointF, Kp, Ki, Kd, DIRECT);
PID backCTL(&BackReading, &outputB, &setPointB, Kp, Ki, Kd, DIRECT);

//PID control for line following
double Jp = 20, Ji = 0.2, Jd = 0.1;
PID left2CTL(&inputL, &outputL2, &setPointL2, Jp,Ji,Jd, DIRECT);
PID right2CTL(&inputR, &outputR2, &setPointR2, Jp,Ji,Jd, DIRECT);
PID front2CTL(&inputF, &outputF2, &setPointF2, Jp,Ji,Jd, DIRECT);
PID back2CTL(&inputB, &outputB2, &setPointB2, Jp,Ji,Jd, DIRECT);

long positionLeft  = -999;
long positionRight = -999;
long positionFront  = -999;
long positionBack = -999;

void setup() {
  // Line following speed
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  setPointF2 = 0.8;
  setPointB2 = 0.8;
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(1); 
  
  // Attach the servos to joint pins
  Joint1.attach(Joint1Pin);
  Joint2.attach(Joint2Pin);
  Joint3.attach(Joint3Pin);
  Gripper.attach(GripperPin);
  
  // Initial the pings 2
  pinMode(frontright, OUTPUT);
  pinMode(pwmfrontl, OUTPUT);
  pinMode(frontleft, OUTPUT);
  pinMode(pwmfrontr, OUTPUT);
  pinMode(horizonback, OUTPUT);
  pinMode(horizonfront, OUTPUT);
  pinMode(pwmhorif, OUTPUT);
  pinMode(pwmhorib, OUTPUT);


  // IR Sensor pin INPUT
  pinMode(IRSensorleft, INPUT); 
  pinMode(IRSensorright, INPUT);

  leftCTL.SetSampleTime(5);
  rightCTL.SetSampleTime(5);
  frontCTL.SetSampleTime(5);
  backCTL.SetSampleTime(5);
  left2CTL.SetSampleTime(5);
  right2CTL.SetSampleTime(5);


  leftCTL.SetMode(MANUAL);
  rightCTL.SetMode(MANUAL);
  frontCTL.SetMode(MANUAL);
  backCTL.SetMode(MANUAL);
  left2CTL.SetMode(MANUAL);
  right2CTL.SetMode(MANUAL);
  front2CTL.SetMode(MANUAL);
  back2CTL.SetMode(MANUAL);

  leftCTL.SetOutputLimits(-255, 255);
  rightCTL.SetOutputLimits(-255, 255);
  frontCTL.SetOutputLimits(-255, 255);
  backCTL.SetOutputLimits(-255, 255);
}


void loop()
{
  LeftReading = knobLeft.read();
  RightReading = knobRight.read();
  FrontReading = knobFront.read();
  BackReading = knobBack.read();
  knobLR.computeSpeeds();
  
  knobFB.computeSpeeds();

  inputL = knobLR.getLeftSpeed(true);
  inputR = knobLR.getRightSpeed(true);
  inputF = knobFB.getLeftSpeed(true);
  inputB = knobFB.getRightSpeed(true);

  while (Walking == true){
    // if (counter == n);{
    //   Joint1.write(Joint1Angle+180+Joint1offset);
    //   joint2.Write(Joint2Angle-30+Joint2offert);
    //   Gripper.write(GripperOpen);
    // }
    Joint1.write(Joint1Angle+Joint1Offset);
    Joint2.write(Joint2Angle+Joint2Offset);
    Joint3.write(Joint3Angle+Joint3Offset);
    Gripper.write(GripperOpen);
    setPointL2 = 0.8;
    setPointR2 = 0.8;
    left2CTL.SetMode(AUTOMATIC);
    right2CTL.SetMode(AUTOMATIC);
    left2CTL.Compute();
    right2CTL.Compute();
    if ((digitalRead(IRSensorleft) == 0) && (digitalRead(IRSensorright) == 0)){
      digitalWrite(frontleft, HIGH);
      digitalWrite(frontright, LOW);
      analogWrite(pwmfrontl,outputL2);
      analogWrite(pwmfrontr,outputR2);}
    if((digitalRead(IRSensorleft) == 0) && (digitalRead(IRSensorright) == 1)){
      digitalWrite(frontleft, HIGH);
      digitalWrite(frontright, LOW);
      analogWrite(pwmfrontl, outputL2);
      analogWrite(pwmfrontr, 10);}
    if((digitalRead(IRSensorleft) == 1) && (digitalRead(IRSensorright) == 0)){
      digitalWrite(frontleft, HIGH);
      digitalWrite(frontright, LOW); 
      analogWrite(pwmfrontl, 10);
      analogWrite(pwmfrontr, outputR2);}
    if((digitalRead(IRSensorleft) == 1) && (digitalRead(IRSensorright) == 1)){
      if (count == 0){
        while ((digitalRead(IRSensorleft) == 1) && (digitalRead(IRSensorright) == 1)){
        digitalWrite(frontleft, HIGH);
        digitalWrite(frontright, LOW);
        analogWrite(pwmfrontl,outputL2);
        analogWrite(pwmfrontr,outputR2);
        }
      }
      analogWrite(pwmfrontl, 0);
      analogWrite(pwmfrontr, 0);
      Walking = false;
      count = 0;
      left2CTL.SetMode(MANUAL);
      right2CTL.SetMode(MANUAL);
      knobBack.write(0);
      knobFront.write(0);
      knobLeft.write(0);
      knobRight.write(0);
    }
  }

  while ((grab == true)&&(Walking==false)){
    Serial.print("100");
    delay(100);
    //op=1 move forward op=-1 move backward op=-10 send stop
    Joint1.write(Joint1Angle+90+Joint1Offset);
    while (pickver == false){
      if (!Serial.available()){
        op = Serial.readString().toInt();
      }
      setPointL2 = 0.4;
      setPointR2 = 0.4;
      left2CTL.SetMode(AUTOMATIC);
      right2CTL.SetMode(AUTOMATIC);
      left2CTL.Compute();
      right2CTL.Compute();
      if (op == 1){
        digitalWrite(frontleft, HIGH);
        digitalWrite(frontright, LOW);
        analogWrite(pwmfrontl,outputL2);
        analogWrite(pwmfrontr,abs(outputR2));
      }
      else if (op == -1){
        digitalWrite(frontleft, LOW);
        digitalWrite(frontright, HIGH);
        analogWrite(pwmfrontl,abs(outputL2));
        analogWrite(pwmfrontr,outputR2);
      }
      else if (op==-10 ) {
        Serial.println("stop");
        analogWrite(pwmfrontl,0);
        analogWrite(pwmfrontr,0);
        left2CTL.SetMode(MANUAL);
        right2CTL.SetMode(MANUAL);
        pickver=true;
        counter = 0;
      }
      else if (op==10){
        Serial.print('ok');
        counter = counter+1;
        Walking = true;
      }
    }

    while (pick == false){
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
      distance = (duration*.0343)/2;
      Serial.print("Distance: ");
      Serial.println(distance);

      setPointF2 = 0.4;
      setPointB2 = 0.4;
      front2CTL.SetMode(AUTOMATIC);
      back2CTL.SetMode(AUTOMATIC);
      front2CTL.Compute();
      back2CTL.Compute();
      delay(10);
      if(distance>15){
        digitalWrite(horizonback, HIGH);
        digitalWrite(horizonfront, LOW);
        analogWrite(pwmhorif,outputF2);
        analogWrite(pwmhorib,outputB2);
      }
      else {
        analogWrite(pwmhorif,0);
        analogWrite(pwmhorib,0);
        front2CTL.SetMode(MANUAL);
        back2CTL.SetMode(MANUAL);
        delay(500);
        for (int i = 0; i<=20; i++){
          Joint3.write(Joint3Angle+Joint3Offset+i);
          delay(50);
        }
        Gripper.write(GripperClose);
        for (int i = 20; i>=0; i--){
          Joint3.write(Joint3Angle+Joint3Offset+i);
          delay(50);
        }
        pick = true;
        Frontdis = FrontReading;
        Backdis = BackReading;
      }
    }
    while (reposition == false){
      setPointL = 0;
      setPointR = 0;
      setPointF = 0;
      setPointB = 0;
      while (repositionh == false){
        frontCTL.SetMode(AUTOMATIC);
        backCTL.SetMode(AUTOMATIC);
        frontCTL.Compute();
        backCTL.Compute();
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
        if ((abs(knobFront.read()) <2) && (abs(knobBack.read()) < 2)){
          analogWrite(pwmhorif,0);
          analogWrite(pwmhorib,0);
          frontCTL.SetMode(MANUAL);
          backCTL.SetMode(MANUAL);
          
          repositionh = true;
          } 
        }
    while (repositionv == false){
        leftCTL.SetMode(AUTOMATIC);
        rightCTL.SetMode(AUTOMATIC);
        leftCTL.Compute();
        rightCTL.Compute();
      if ((outputF >= 0) & (outputB >= 0)){
        digitalWrite(frontleft, HIGH);
        digitalWrite(frontright, HIGH);
        analogWrite(pwmfrontl,abs(outputL));
        analogWrite(pwmfrontr,abs(outputR));
      }
        else if ((outputF >= 0) & (outputB < 0)){
          digitalWrite(frontleft, HIGH);
          digitalWrite(frontright, LOW);
          analogWrite(pwmfrontl,abs(outputL));
          analogWrite(pwmfrontr,abs(outputR));
      }

        else if ((outputF < 0) & (outputB >= 0)){
          digitalWrite(frontleft, LOW);
          digitalWrite(frontright, HIGH);
          analogWrite(pwmfrontl,abs(outputL));
          analogWrite(pwmfrontr,abs(outputR));
      }
        else if((outputF < 0) & (outputB < 0)){
          digitalWrite(frontleft, LOW);
          digitalWrite(frontright, LOW);
          analogWrite(pwmfrontl,abs(outputL));
          analogWrite(pwmfrontr,abs(outputR));
      }
        if ((abs(knobFront.read()) <2) && (abs(knobBack.read()) < 2)){
          analogWrite(pwmfrontl,0);
          analogWrite(pwmfrontr,0);
          leftCTL.SetMode(MANUAL);
          rightCTL.SetMode(MANUAL);
          repositionv = true;
          } 
        }
        reposition = true;
    }
    if((reposition == true) && (pick == true)){
      pickver = false;
      pick = false;
      reposition = false;
      repositionv = false;
      repositionh = false;
      Walking = true;
      place = true;
      pick = false;
      
    }
  }
      
  
  while ((place == true)&&(Walking==false)){
    Serial.print('place');
    Serial.println();
    if (op == 20){
      Joint1.write(Joint1Angle+Joint1Offset);
      while (put == false){
        setPointF = Frontdis;
        setPointB = Backdis;
        frontCTL.SetMode(AUTOMATIC);
        backCTL.SetMode(AUTOMATIC);
        frontCTL.Compute();
        backCTL.Compute();
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
        if ((abs(knobFront.read()-Frontdis) <2) && (abs(knobBack.read()-Backdis) < 2)){
          analogWrite(pwmhorif,0);
          analogWrite(pwmhorib,0);
          frontCTL.SetMode(MANUAL);
          backCTL.SetMode(MANUAL);
          delay(500);
          for (int i = 0; i<=20; i++){
          Joint3.write(Joint3Angle+Joint3Offset+i);
          delay(50);
          }
          Gripper.write(GripperOpen);
          for (int i = 20; i>=0; i--){
            Joint3.write(Joint3Angle+Joint3Offset+i);
            delay(50);
          }
          put = true;
          } 
        }
      while (repositionput == false){
        setPointF = 0;
        setPointB = 0;
        frontCTL.SetMode(AUTOMATIC);
        backCTL.SetMode(AUTOMATIC);
        frontCTL.Compute();
        backCTL.Compute();
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
        if ((abs(knobFront.read()) <2) && (abs(knobBack.read()) < 2)){
          analogWrite(pwmhorif,0);
          analogWrite(pwmhorib,0);
          frontCTL.SetMode(MANUAL);
          backCTL.SetMode(MANUAL);
          repositionput = true;
          } 
        
      }
      while ((put == true) && (repositionput == true))
      put = false;
      repositionput = false;
      Walking = true;
    }
    else if ( op == 10){
      Walking = true;
    }
  }
}
 

