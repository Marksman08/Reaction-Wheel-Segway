#include <Wire.h>               // Include Wire library for I2C communication
#include <MPU6050_light.h>      // Include MPU6050 library

MPU6050 mpu(Wire);              // Create an MPU6050 object using the Wire library

// Pin definitions for the bottom motors (left and right)
const int IN1_MOTOR1 = 8;       // Motor 1 control pin 1
const int IN2_MOTOR1 = 9;       // Motor 1 control pin 2
const int PWM_MOTOR1 = 10;      // PWM pin for Motor 1 speed control

const int IN1_MOTOR2 = 12;      // Motor 2 control pin 1
const int IN2_MOTOR2 = 13;      // Motor 2 control pin 2
const int PWM_MOTOR2 = 11;      // PWM pin for Motor 2 speed control

// PID controller variables (outer loop - tilt control)
float Kp_outer = 40.0;          // Proportional gain for outer loop
float Ki_outer = 0.0;           // Integral gain for outer loop
float Kd_outer = 0.0;           // Derivative gain for outer loop
float setpoint_angle = 0.0;     // Desired angle (setpoint)
float lastAngleError = 0.0;     // Previous angle error
float integral_angle = 0.0;     // Accumulated integral of angle error

// PID controller variables (inner loop - velocity control)
float Kp_inner = 80.0;          // Proportional gain for inner loop
float Ki_inner = 0.0;           // Integral gain for inner loop
float Kd_inner = 0.0;           // Derivative gain for inner loop
float lastVelocityError = 0.0;  // Previous velocity error
float integral_velocity = 0.0;  // Accumulated integral of velocity error

// Motor control constants
const int MIN_SPEED = 150;      // Minimum PWM value to overcome motor's static friction
const int MAX_SPEED = 255;      // Maximum PWM value (max speed)

// Timing variables
unsigned long lastTime = 0;     // Last time control loop was executed
const unsigned long SAMPLE_TIME = 5; // Control loop sample time in milliseconds

void setup() {
  Serial.begin(115200);         // Initialize serial communication at 115200 baud
  Wire.begin();                 // Initialize I2C communication

  // Initialize pins for the two bottom motors
  pinMode(IN1_MOTOR1, OUTPUT);
  pinMode(IN2_MOTOR1, OUTPUT);
  pinMode(PWM_MOTOR1, OUTPUT);

  pinMode(IN1_MOTOR2, OUTPUT);
  pinMode(IN2_MOTOR2, OUTPUT);
  pinMode(PWM_MOTOR2, OUTPUT);

  // Initialize MPU6050 sensor
  byte status = mpu.begin();
  while (status != 0) {         // Check if MPU6050 is connected properly
    Serial.println("MPU6050 initialization failed. Please check your connections.");
    delay(1000);                // Wait 1 second before retrying
  }

  Serial.println("MPU6050 initialized successfully!");

  // Calibrate MPU6050 sensor
  Serial.println("Calibrating MPU6050...");
  mpu.calcOffsets();            // Calculate offsets to improve accuracy
  Serial.println("Calibration complete!");
}

void loop() {
  unsigned long currentTime = millis(); // Get current time in milliseconds

  // Execute control loop at specified sample time intervals
  if (currentTime - lastTime >= SAMPLE_TIME) {
    float deltaTime = (float)(currentTime - lastTime) / 1000.0; // Calculate elapsed time in seconds

    // Update sensor readings and compute current angle
    float angle = updateMPU(deltaTime);

    // Outer PID loop to compute desired velocity based on angle error
    float desiredVelocity = calculateOuterPID(angle, deltaTime);

    // Inner PID loop to compute motor output for bottom motors based on desired velocity
    float bottomMotorOutput = calculateInnerPID(desiredVelocity, deltaTime);

    // Set motor speeds for bottom motors
    setMotorSpeed(bottomMotorOutput);

    Serial.println(angle);

    // Update lastTime for next iteration
    lastTime = currentTime;
  }
}

// Helper function to update MPU6050 readings and compute angle
float updateMPU(float deltaTime) {
  mpu.update();  // Update sensor data

  // Complementary filter to combine accelerometer and gyroscope data
  // Currently set to use only accelerometer data (adjust weights as needed)
  return 0.0 * (mpu.getAngleX() + mpu.getGyroX() * deltaTime) + 1.0 * mpu.getAngleX();
}

// Calculate outer PID control (tilt control)
float calculateOuterPID(float angle, float deltaTime) {
  float angleError = setpoint_angle - angle; // Compute angle error
  integral_angle += angleError * deltaTime;  // Update integral term
  float derivative_angle = (angleError - lastAngleError) / deltaTime; // Compute derivative term

  // Compute PID output
  float output = Kp_outer * angleError + Ki_outer * integral_angle + Kd_outer * derivative_angle;

  lastAngleError = angleError; // Update last angle error for next derivative calculation
  return output;               // Return desired velocity for inner loop
}

// Calculate inner PID control (velocity control)
float calculateInnerPID(float desiredVelocity, float deltaTime) {
  // Assume actual velocity is zero (could be modified to include velocity measurements)
  float velocityError = desiredVelocity;       // Compute velocity error
  integral_velocity += velocityError * deltaTime; // Update integral term
  float derivative_velocity = (velocityError - lastVelocityError) / deltaTime; // Compute derivative term

  // Compute PID output
  float output = Kp_inner * velocityError + Ki_inner * integral_velocity + Kd_inner * derivative_velocity;

  lastVelocityError = velocityError; // Update last velocity error for next derivative calculation
  return output;                     // Return motor output for bottom motors
}

// Set motor speeds based on PID output for both bottom motors
void setMotorSpeed(float bottomOutput) {
  // Map the PID output to PWM values within the motor speed range
  int bottomMotorSpeed = map(abs(bottomOutput), 0, 180, MIN_SPEED, MAX_SPEED);
  bottomMotorSpeed = constrain(bottomMotorSpeed, MIN_SPEED, MAX_SPEED);

  // Control bottom motors (left and right)
  if (bottomOutput > 0) {
    // Set direction for forward motion
    digitalWrite(IN1_MOTOR1, LOW);
    digitalWrite(IN2_MOTOR1, HIGH);
    digitalWrite(IN1_MOTOR2, LOW);
    digitalWrite(IN2_MOTOR2, HIGH);
  } else {
    // Set direction for reverse motion
    digitalWrite(IN1_MOTOR1, HIGH);
    digitalWrite(IN2_MOTOR1, LOW);
    digitalWrite(IN1_MOTOR2, HIGH);
    digitalWrite(IN2_MOTOR2, LOW); 
  }
  // Set PWM speed for bottom motors
  analogWrite(PWM_MOTOR1, bottomMotorSpeed);
  analogWrite(PWM_MOTOR2, bottomMotorSpeed);
}
