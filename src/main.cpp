#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>

#define motorA_EN 2
#define motorA_IN1 6
#define motorA_IN2 5
#define motorB_EN 10
#define motorB_IN1 4
#define motorB_IN2 3

float kp = 2.0;
float ki = 0.0;
float kd = 1.0;

float targetAngle = 0.0;
float previousError = 0.0;
float integral = 0.0;

//Function Prototypes
void controlMotors(float output);

void setup() {
  Serial.begin(115200);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  pinMode(motorA_EN, OUTPUT);
  pinMode(motorA_IN1, OUTPUT);
  pinMode(motorA_IN2, OUTPUT);
  pinMode(motorB_EN, OUTPUT);
  pinMode(motorB_IN1, OUTPUT);
  pinMode(motorB_IN2, OUTPUT);
}

void loop() {
  float ax, ay, az, gx, gy, gz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    float angle = atan2(ay, az) * 180 / PI;

    float error = targetAngle - angle;
    integral += error;
    float derivative = error - previousError;
    float output = kp * error + ki * integral + kd * derivative;
    previousError = error;

    controlMotors(output);

    delay(10);
  }
}

void controlMotors(float output) {
  int motorSpeed = constrain(abs(output), 0, 255);
  if (output > 0) {
    digitalWrite(motorA_IN1, HIGH);
    digitalWrite(motorA_IN2, LOW);
    analogWrite(motorA_EN, motorSpeed);
    
    digitalWrite(motorB_IN1, HIGH);
    digitalWrite(motorB_IN2, LOW);
    analogWrite(motorB_EN, motorSpeed);
  } else {
    digitalWrite(motorA_IN1, LOW);
    digitalWrite(motorA_IN2, HIGH);
    analogWrite(motorA_EN, motorSpeed);
    
    digitalWrite(motorB_IN1, LOW);
    digitalWrite(motorB_IN2, HIGH);
    analogWrite(motorB_EN, motorSpeed);
  }
}
