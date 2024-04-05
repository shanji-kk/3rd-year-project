#include <Encoder.h>
#include <PID_v1.h>
#include <util/atomic.h>

Encoder LeftEncoder(21,20);

const unsigned long timePeriod = 1;   // sampling time for speed in milliseconds      1 better
unsigned long startTime;
long startLeft;
long startRight;
const unsigned int countsperRev = 632; // same for both left and right
const float speedRatio = 23.7342;

static int directionpinA = 12 ;
static int pwmpinA = 3 ;
static int brakepinA = 9 ;

double Setpoint, Input, Output;

double Kp = 17, Ki = 600, Kd = 0;  
volatile float Leftspeed;

PID LeftMotor(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// receiving variables
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;

double dataNumber = 0;             // new for this version


void setup() {
    pinMode(directionpinA, OUTPUT);
    pinMode(pwmpinA, OUTPUT);
    pinMode(brakepinA, OUTPUT);

    Input = 0;
    Setpoint = 1;   // revs per second, direction pin Low is forward

    Serial.begin(9600);
    delay(2000);
    startLeft = LeftEncoder.read();
    startTime = millis();
    LeftMotor.SetMode(AUTOMATIC);
    LeftMotor.SetSampleTime(timePeriod * 5);

    if(Setpoint < 0){
        digitalWrite(directionpinA,HIGH);
        LeftMotor.SetControllerDirection(REVERSE);
    }
    else{
        digitalWrite(directionpinA,LOW);
        LeftMotor.SetControllerDirection(DIRECT);
    }
}

long oldLeft = -999;

void loop() {
    // put your main code here, to run repeatedly:
    unsigned long now = millis();
    long newLeft = LeftEncoder.read();

    if ( now - startTime >= timePeriod ) {
        // time to calculate average encoder speed
        Leftspeed = (newLeft - startLeft) / (float)(timePeriod * 1e-3 * countsperRev * speedRatio)  ;
        Serial.print( "Left speed is ");
        Serial.println( Leftspeed, 4 );
        startTime = now;
        startLeft = newLeft;
    }

    Input = Leftspeed;
    LeftMotor.Compute();
    analogWrite(pwmpinA, Output);

    recvWithEndMarker();
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    if (Serial.available() > 0) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }

        dataNumber = atof(receivedChars);
        Setpoint = dataNumber;
    }
}
