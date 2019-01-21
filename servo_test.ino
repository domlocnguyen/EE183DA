#include <Arduino.h>
#include <Servo.h>

const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;
Servo servo_left;
Servo servo_right;
int servo_left_ctr = 90;
int servo_right_ctr = 90;

String inputData = "";         
bool receivedData = false;

void setup() {
  // setup Serial, LEDs and Motors
    Serial.begin(9600);
    inputData.reserve(100);
    drive(servo_left_ctr,servo_right_ctr);
   
    servo_left.attach(SERVO_LEFT);
    servo_right.attach(SERVO_RIGHT);    
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputData:
    inputData += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    if (inChar == '\n') {
      receivedData = true;
    }
  } 
}

void drive(int left, int right) {
  servo_left.write(left);
  servo_right.write(right);
}

void loop() {
  serialEvent();
  if (receivedData) {
    delay(100);
    int data = inputData.toInt();
    Serial.println( inputData );
    
    drive(data,data);   
         
    // clear the data:
    inputData = "";
    receivedData = false; 
    } 
}
