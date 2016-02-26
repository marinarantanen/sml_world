/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int desiredServo = 90;
int currentServo = 90;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.setTimeout(10);

  myservo.attach(6);  // attaches the servo on pin 9 to the servo object
  
  myservo.write(currentServo);
    
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  pinMode(2, INPUT);

  //attachInterrupt(digitalPinToInterrupt(2), button_pressed, RISING);
  
}

// the loop routine runs over and over again forever:
void loop() {

  while (Serial.available() != 0){

    String received_string = Serial.readStringUntil('K');

    int token0Index = received_string.indexOf(';');
    int token1Index = received_string.indexOf(';', token0Index+1);
    int token2Index = received_string.indexOf(';', token1Index+1);
    int token3Index = received_string.indexOf(';', token2Index+1);
    int token4Index = received_string.indexOf(';', token3Index+1);
    int token5Index = received_string.indexOf(';', token4Index+1);

    String value0 = received_string.substring(0, token0Index);
    String value1 = received_string.substring(token0Index+1, token1Index);
    String value2 = received_string.substring(token1Index+1, token2Index);
    String value3 = received_string.substring(token2Index+1, token3Index);
    String value4 = received_string.substring(token3Index+1, token4Index);
    String value5 = received_string.substring(token4Index+1, token5Index);
    String value6 = received_string.substring(token5Index+1);

    int led0 = value0.toInt();
    int led1 = value1.toInt();
    int led2 = value2.toInt();
    int led3 = value3.toInt();
    int led4 = value4.toInt();
    int led5 = value5.toInt();    
    desiredServo = value6.toInt();

    if (led0 == 0){
      digitalWrite(7, LOW);
    }else{
      digitalWrite(7, HIGH);
    }
    if (led1 == 0){
      digitalWrite(8, LOW);
    }else{
      digitalWrite(8, HIGH);
    }
    if (led2 == 0){
      digitalWrite(9, LOW);
    }else{
      digitalWrite(9, HIGH);
    }
    if (led3 == 0){
      digitalWrite(10, LOW);
    }else{
      digitalWrite(10, HIGH);
    }
    if (led4 == 0){
      digitalWrite(11, LOW);
    }else{
      digitalWrite(11, HIGH);
    }
    if (led5 == 0){
      digitalWrite(12, LOW);
    }else{
      digitalWrite(12, HIGH);
    }
    // 0;0;0;0;0;0;0K    
  }

  move_servo();

  delay(25);        // delay in between reads for stability
}

void move_servo(){
    
  int max_delta = 1;

  if (currentServo != desiredServo){

    int desired_delta = desiredServo - currentServo;

    if ( desired_delta > max_delta ){
      desired_delta = max_delta;
    }
    if ( desired_delta < -max_delta ){
      desired_delta = -max_delta;
    }

    currentServo = currentServo + desired_delta;

    myservo.write(currentServo);

    delay(25);
   
  }

}


