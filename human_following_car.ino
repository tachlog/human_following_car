// Define pin connections for the different sensors and motor driver
const int leftIRPin = 3;
const int rightIRPin = 2;
const int trigPin = 11;
const int echoPin = 12;
const int enAPin = 5;
const int in1Pin = 7;
const int in2Pin = 8;
const int enBPin = 6;
const int in3Pin = 9;
const int in4Pin = 10;

// Initialize Variables
int leftIRValue = 0;
int rightIRValue = 0;
int distance = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Set pin modes for all the sensors and motor driver
  pinMode(leftIRPin, INPUT);
  pinMode(rightIRPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(enAPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enBPin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
}

void loop() {
  // Read left and right IR sensor values
  leftIRValue = digitalRead(leftIRPin);
  rightIRValue = digitalRead(rightIRPin);

  // Determine direction to turn based on IR sensor values
  if (leftIRValue == HIGH && rightIRValue == LOW) {
    // Turn left
    analogWrite(enAPin, 150); 
    analogWrite(enBPin, 0);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    digitalWrite(in3Pin, HIGH);
    digitalWrite(in4Pin, HIGH);
  } else if (leftIRValue == LOW && rightIRValue == HIGH) {
    // Turn right
    analogWrite(enAPin, 0); 
    analogWrite(enBPin, 150);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    digitalWrite(in3Pin, LOW);
    digitalWrite(in4Pin, HIGH);
  } else {
    // Go straight
    analogWrite(enAPin, 120);
    analogWrite(enBPin, 120);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    digitalWrite(in3Pin, LOW);
    digitalWrite(in4Pin, HIGH);
  }

  // Read distance from ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  distance = pulseIn(echoPin, HIGH) / 58;

  // Stop the car if the distance is less than 10 cm
  if (distance > 10) {
    analogWrite(enAPin, 0); 
    analogWrite(enBPin, 0);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    digitalWrite(in3Pin, LOW);
    digitalWrite(in4Pin, LOW);
  }

  // Print sensor values to serial monitor
  Serial.print("Left IR value: ");
  Serial.print(leftIRValue);
  Serial.print(", Right IR value: ");
  Serial.print(rightIRValue);
  Serial.print(", Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Wait for some time before looping again
  delay(100);
}
