#ifndef Encoder_funcs_h
#define Encoder_funcs_h

#include <Arduino.h>
#include <math.h>
#include <Encoder.h>

class Wheels
{
public:
    Wheels(int leftA, int leftB, int rightA, int rightB);    // initialisation
    ~Wheels();   //destructor

    void computeSpeeds(); // calculate speeds
    void startTimer();
    float getLeftSpeed(bool unit);
    float getRightSpeed(bool unit);

private:

    volatile long newLeft, newRight;
    volatile long startLeft, startRight;
    volatile float leftSpeed;
    volatile float rightSpeed;

    Encoder LeftEncoder;
    Encoder RightEncoder;
    
    unsigned long deltaT; // sampling time period for speed in ms
    unsigned long startTime;
    unsigned int countsperRev;    // same for L and R
    float speedRatio;
};



#endif