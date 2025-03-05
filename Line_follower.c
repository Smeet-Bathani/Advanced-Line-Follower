#include <Arduino.h>

// Define sensor pins (adjust these to your actual sensor connections)
const int lineSensor1 = 22;  // Leftmost sensor
const int lineSensor2 = 24;  // 
const int lineSensor3 = 26;  // 
const int lineSensor4 = 28;  // 
const int lineSensor5 = 30;  // Rightmost sensor

// Motor driver pins (adjust these to your motor driver connections)
const int motorLeft1 = 8;   // IN1
const int motorLeft2 = 9;   // IN2
const int motorRight1 = 10;  // IN3
const int motorRight2 = 11;  // IN4
const int enableLeft = 5;    // ENA
const int enableRight = 6;   // ENB

// PID Control parameters
float Kp = 20;   // Proportional gain
float Ki = 0.01; // Integral gain
float Kd = 5;    // Derivative gain

// PID variables
float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;

// Motor speed variables
int motorSpeedLeft = 0;
int motorSpeedRight = 0;
int baseSpeed = 150; // Base motor speed

// Threshold for line detection
const int lineThreshold = 500; // Adjust based on sensor readings

// Function to read line sensor values
int readLineSensor(int sensorPin) {
  return analogRead(sensorPin);
}

// Function to calculate motor speeds based on PID control
void calculateMotorSpeeds() {
  // Read sensor values
  int sensorValues[5] = {
    readLineSensor(lineSensor1),
    readLineSensor(lineSensor2),
    readLineSensor(lineSensor3),
    readLineSensor(lineSensor4),
    readLineSensor(lineSensor5)
  };

  // Calculate line position (weighted average)
  int weightedSum = 0;
  int totalWeight = 0;
  for (int i = 0; i < 5; i++) {
    if (sensorValues[i] > lineThreshold) {
      weightedSum += (i + 1) * sensorValues[i]; // Weight by sensor position
      totalWeight += sensorValues[i];
    }
  }

  if (totalWeight > 0) {
    error = (weightedSum / (float)totalWeight) - 3; // Normalize error to -2 to 2
  } else {
    // Handle corner cases and intersections
    if (previousError < 0) {
      error = -3; // Continue turning left
    } else {
      error = 3; // Continue turning right
    }
  }

  // Calculate PID terms
  integral += error;
  derivative = error - previousError;

  // Calculate motor speed adjustments
  int adjustment = Kp * error + Ki * integral + Kd * derivative;

  // Apply adjustments to motor speeds
  motorSpeedLeft = baseSpeed - adjustment;
  motorSpeedRight = baseSpeed + adjustment;

  // Constrain motor speeds
  motorSpeedLeft = constrain(motorSpeedLeft, 0, 255);
  motorSpeedRight = constrain(motorSpeedRight, 0, 255);

  // Store current error for next iteration
  previousError = error;
}

// Function to control motors
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  analogWrite(enableLeft, leftSpeed);
  analogWrite(enableRight, rightSpeed);

  if (leftSpeed > 0) {
    digitalWrite(motorLeft1, HIGH);
    digitalWrite(motorLeft2, LOW);
  } else {
    digitalWrite(motorLeft1, LOW);
    digitalWrite(motorLeft2, HIGH);
  }

  if (rightSpeed > 0) {
    digitalWrite(motorRight1, HIGH);
    digitalWrite(motorRight2, LOW);
  } else {
    digitalWrite(motorRight1, LOW);
    digitalWrite(motorRight2, HIGH);
  }
}

void setup() {
  // Initialize sensor pins as inputs
  pinMode(lineSensor1, INPUT);
  pinMode(lineSensor2, INPUT);
  pinMode(lineSensor3, INPUT);
  pinMode(lineSensor4, INPUT);
  pinMode(lineSensor5, INPUT);

  // Initialize motor driver pins as outputs
  pinMode(motorLeft1, OUTPUT);
  pinMode(motorLeft2, OUTPUT);
  pinMode(motorRight1, OUTPUT);
  pinMode(motorRight2, OUTPUT);
  pinMode(enableLeft, OUTPUT);
  pinMode(enableRight, OUTPUT);

  // Begin serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Calculate motor speeds based on line position
  calculateMotorSpeeds();

  // Set motor speeds
  setMotorSpeeds(motorSpeedLeft, motorSpeedRight);

  // Print sensor values and motor speeds for debugging (optional)
  Serial.print("Sensor Values: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(readLineSensor(lineSensor1 + i * 2)); // Print sensor values
    Serial.print(" ");
  }
  Serial.print(", Error: ");
  Serial.print(error);
  Serial.print(", Left Speed: ");
  Serial.print(motorSpeedLeft);
  Serial.print(", Right Speed: ");
  Serial.println(motorSpeedRight);
}
