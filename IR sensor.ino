int IRSensorleft = 30; 
int IRSensorright = 31;
void setup() { 
	Serial.begin(9600); 
	Serial.setTimeout(1); 
  pinMode(IRSensorleft, INPUT); 
  pinMode(IRSensorright, INPUT);
} 
void loop() { 
    Serial.print("right read  ");
    Serial.print(digitalRead(IRSensorright) );
    Serial.print("  left read "  );
    Serial.print(digitalRead(IRSensorleft)  );
    Serial.println();
      }




// int x; 
// void setup() { 
// 	Serial.begin(115200); 
// 	Serial.setTimeout(1); 
//   pinMode(LED_BUILTIN, OUTPUT);
// } 
// void loop() { 
//   digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (HIGH is the voltage level)

//   delay(1000);
//   digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
//   delay(1000);
// 	Serial.print(x + 1); 
// } 
