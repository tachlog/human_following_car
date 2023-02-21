#include <NewPing.h>
#include<AFMotor.h>

#define ULTRASONIC_SENSOR_TRIG 11 // trig pin of HC-SR04
#define ULTRASONIC_SENSOR_ECHO 12 // trig pin of HC-SR04

#define MAX_FORWARD_MOTOR_SPEED 120
#define MAX_MOTOR_TURN_SPEED_ADJUSTMENT 50

#define MIN_DISTANCE 1
#define MAX_DISTANCE 25

#define IR_SENSOR_RIGHT 2   //ir sensor Right
#define IR_SENSOR_LEFT 3   //ir sensor Left

//Right motor
int enableRightMotor=5;  //Enables PWM signal for Right motor
int rightMotorPin1=7;   // forward motion of Right motor
int rightMotorPin2=8;  // reverse motion of Right motor

//Left motor
int enableLeftMotor=6;   //Enables PWM signal for Left motor
int leftMotorPin1=9;    // forward motion of Left motor
int leftMotorPin2=10;  // reverse motion of Left motor

long duration, distance; // variable for the duration of sound wave travel and distance measurement
NewPing mySensor(ULTRASONIC_SENSOR_TRIG, ULTRASONIC_SENSOR_ECHO);

void setup()
{
  // put your setup code here, to run once:
  pinMode(enableRightMotor, OUTPUT); // declare as output for L293D Pins
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(ULTRASONIC_SENSOR_TRIG, OUTPUT);  // set trig pin as output to Transmit Waves
  pinMode(ULTRASONIC_SENSOR_ECHO, INPUT);   // set echo pin as input to capture reflected 

  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  rotateMotor(0,0);   
}


void loop()
{

    digitalWrite(ULTRASONIC_SENSOR_TRIG, LOW); // Clears the trigPin condition
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_SENSOR_TRIG, HIGH);// send waves for 10 us (the trigPin HIGH (ACTIVE))
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_SENSOR_TRIG, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(ULTRASONIC_SENSOR_ECHO, HIGH); // receive reflected waves
    // Calculating the distance
    distance = duration / 58.2;        // convert to distance
    delay(10);
   
    if (distance < 24)
    {   
        analogWrite(enableRightMotor, 250); 
        analogWrite(enableLeftMotor, 245);
        digitalWrite(rightMotorPin1, HIGH); // move forward
        digitalWrite(rightMotorPin2, LOW);
        digitalWrite(leftMotorPin1, HIGH);
        digitalWrite(leftMotorPin2, LOW);
    }

    if (distance > 25)
    {   
        analogWrite(enableRightMotor, 250); 
        analogWrite(enableLeftMotor, 245);
        digitalWrite(rightMotorPin1, LOW); // Stop
        digitalWrite(rightMotorPin2, LOW);
        digitalWrite(leftMotorPin1, LOW);
        digitalWrite(leftMotorPin2, LOW);
        
    }

  int distance = mySensor.ping_cm();
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  //NOTE: If IR sensor detects the hand then its value will be LOW else the value will be HIGH
  
  //If right sensor detects hand, then turn right. We increase left motor speed and decrease the right motor speed to turn towards right
  if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
  {
      rotateMotor(MAX_FORWARD_MOTOR_SPEED - MAX_MOTOR_TURN_SPEED_ADJUSTMENT, MAX_FORWARD_MOTOR_SPEED + MAX_MOTOR_TURN_SPEED_ADJUSTMENT ); 
  }
  //If left sensor detects hand, then turn left. We increase right motor speed and decrease the left motor speed to turn towards left
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
  {
      rotateMotor(MAX_FORWARD_MOTOR_SPEED + MAX_MOTOR_TURN_SPEED_ADJUSTMENT, MAX_FORWARD_MOTOR_SPEED - MAX_MOTOR_TURN_SPEED_ADJUSTMENT); 
  }
  //If distance is between min and max then go straight
  else if (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE)
  {
    rotateMotor(MAX_FORWARD_MOTOR_SPEED, MAX_FORWARD_MOTOR_SPEED);
  }
  //stop the motors
  else 
  {
    rotateMotor(0, 0);
  }
}


void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)               //turn right
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }

  if (leftMotorSpeed < 0)               //turn left
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else 
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}
