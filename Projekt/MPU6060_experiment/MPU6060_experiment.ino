#include "MPU6050.h"
MPU6050 MPU;
long Tstart;
int deltaT = 20000;  //sampling rate in mircroseconds
////ultrasonic sensor variables///
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
const int echoPin = 2;
const int trigPin = 3;
void setup() {
  // put your setup code here, to run once:
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  Serial.begin(9600);
  MPU.beginMPU();
  //MPU.calculate_IMU_error();
}

void loop() {
  Tstart = micros();
  MPU.updateMeasurements();
  String datastr = micros() + String(",") + String(MPU.AccX)+String(",")+String(MPU.AccY)+String(",")+String(MPU.yaw);
  Serial.println(datastr);
  
//  digitalWrite(trigPin, LOW);
//  delayMicroseconds(2);
//  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPin, LOW);
//  duration = pulseIn(echoPin, HIGH);
//  distance = duration * 0.034 / 2;
//  Serial.print("Distance: ");
//  Serial.print(distance);
//  Serial.println(" cm");
   while (deltaT>(micros()-Tstart));
}
