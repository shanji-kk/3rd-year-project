#include "Encoder_funcs.h"

Wheels::Wheels(int leftA, int leftB, int rightA, int rightB):LeftEncoder(leftA,leftB),RightEncoder(rightA,rightB){
    this->deltaT = 1; // sampling time for the speed in milliseconds
    this-> countsperRev = 633;  // encoder counts per revolution
}

Wheels::~Wheels(){
    ~this->newLeft;
    ~this->newRight;
    ~this->startLeft;
    ~this->startRight;

    ~this->deltaT;
    ~this->startTime;
    ~this->countsperRev;
}

// intialises the values for start time and first left and right positions
void Wheels::startTimer(){
    // Serial.begin(9600);  only for debugging
    this->startTime = millis();
    this->startLeft = this->LeftEncoder.read();
    this->startRight = this->RightEncoder.read();
}

// calculates the speeds for both encoders
void Wheels::computeSpeeds(){
    unsigned long now = millis();
    this->newLeft = this->LeftEncoder.read();
    this->newRight = this->RightEncoder.read();

    if( now - this->startTime >= this->deltaT) {

        this->leftSpeed = (this->newLeft - this->startLeft) / (float)(this->deltaT * 1e-3 * this->countsperRev );
        this->rightSpeed = (this->newRight - this->startRight) / (float)(this->deltaT * 1e-3 * this->countsperRev );

        // Serial.print("Left Speed: ");    only for debugging
        // Serial.print(this->leftSpeed);
        // Serial.print("  Right Speed: ");
        // Serial.println(this->rightSpeed);

        this->startTime = now;
        this->startLeft = this->newLeft;
        this->startRight = this->newRight;
    }
}

// returns left wheel speed
float Wheels::getLeftSpeed(bool unit){
    if(unit){
        return this->leftSpeed; // this is in revolutions per second
    }
    else{
        return this->leftSpeed * 0.06 * M_PI;   // in metres per second
    }
}

// returns right wheel speed
float Wheels::getRightSpeed(bool unit){
    if(unit){
        return this->rightSpeed; // this is in revolutions per second
    }
    else{
        return this->rightSpeed * 0.06 * M_PI;   // in metres per second
    }
}