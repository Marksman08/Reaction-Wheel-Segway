#include <Wire.h>           // Includes the Wire library for I2C communication
#include <MPU6050_light.h>   // Includes the MPU6050 library for interacting with the MPU6050 sensor

// Initialize MPU6050 object with Wire (I2C)
MPU6050 mpu(Wire);

// Pin definitions for motor control
const int PWM_PIN = 5;    // PWM pin used to control motor speed (connected to motor driver)
const int IN1_PIN = 7;    // Pin to control motor direction (connected to motor driver)
const int IN2_PIN = 6;    // Pin to control motor direction (connected to motor driver)

// PID controller variables for maintaining balance
float Kp = 20000.0;       // Proportional gain (adjusts the motor power based on how far the system is from the setpoint)
float Ki = 10.0;          // Integral gain (helps eliminate steady-state error by considering the accumulated error over time)
float Kd = 50000.0;       // Derivative gain (reacts to the rate of change of the error, helping dampen oscillations)
float setpoint_angle = 0.0; // Desired angle of balance (setpoint) - 0 means upright
float lastAngleError = 0.0; // Used to store the previous error for the derivative term
float integral_angle = 0.0; // Used to accumulate the error over time for the integral term

// Timing variables for controlling the sample rate (time between updates)
unsigned long lastTime = 0;  // Stores the last time the loop was updated
const unsigned long SAMPLE_TIME = 5; // Sample time for the loop in milliseconds (5ms = 200Hz)

// Timing variable for calculating delta time between PID updates
long prevT = 0;

void setup() {
  Serial.begin(9600);   // Initialize serial communication at a baud rate of 9600 bits per second for debugging
  Wire.begin();         // Initialize I2C communication with the MPU6050 sensor
  
  // Initialize the motor control pins
  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  // Initialize the MPU6050 sensor
  byte status = mpu.begin();
  while (status != 0) {  // If the MPU6050 fails to initialize, print an error message and retry
    Serial.println("MPU6050 initialization failed. Please check your connections.");
    delay(1000);  // Wait 1 second before retrying
  }
  
  Serial.println("MPU6050 initialized successfully!");  // Confirm successful initialization

  // Calibrate the MPU6050 to get accurate offsets for accelerometer and gyroscope
  Serial.println("Calibrating MPU...");
  mpu.calcOffsets();  // Automatically calculate the offsets for accurate readings
  Serial.println("Calibration complete!");

  // Print CSV header for data output (in this case, we only print the angle)
  Serial.println("Time,Angle,MotorOutput");  // Commented out because we only print the angle in this version
}

void loop() {
  // Get the current time
  unsigned long currentTime = millis();  // millis() gives the number of milliseconds since the Arduino started running

  // If enough time (SAMPLE_TIME) has passed since the last loop, proceed with the control
  if (currentTime - lastTime >= SAMPLE_TIME) {
    // Calculate the time that has passed since the last loop (delta time)
    float deltaTime = (float)(currentTime - lastTime) / 1000.0;  // Convert from milliseconds to seconds
    
    // Update the MPU6050 sensor and calculate the current tilt angle
    float angle = updateMPU(deltaTime);

    // Calculate the required motor output using the PID controller
    float motorOutput = calculatePID(angle, deltaTime);

    // Set the motor speed and direction based on the PID output
    setMotorSpeed(motorOutput);

    // Output the current angle to the Serial Monitor (CSV format)
    Serial.println(angle);  // Only print the angle

    // Update the last time the loop ran
    lastTime = currentTime;
  }
}

// Function to update MPU6050 and calculate the current tilt angle
float updateMPU(float deltaTime) {
  mpu.update();              // Get updated sensor readings from the MPU6050
  float rawAngle = mpu.getAngleX();  // Get the current angle around the X-axis (tilt angle)
  return rawAngle;            // Return the current tilt angle
}

// PID control function: calculates the required motor output based on the current angle
float calculatePID(float angle, float deltaTime) {
  // Calculate the error (difference between the setpoint angle and the current angle)
  float angleError = setpoint_angle - angle;

  // Accumulate the error over time (integral term)
  integral_angle += angleError * deltaTime;

  // Calculate the rate of change of the error (derivative term)
  float derivative_angle = (angleError - lastAngleError) / deltaTime;

  // Calculate the PID output (motor speed and direction)
  float output = Kp * angleError + Ki * integral_angle + Kd * derivative_angle;

  // Update the previous error for the next loop iteration
  lastAngleError = angleError;

  return output;  // Return the motor control signal (output from the PID controller)
}

// Function to control the motor speed and direction based on the PID output
void setMotorSpeed(float motorOutput) {
  // If the motor output is positive, rotate the motor forward
  if (motorOutput > 0) {
    digitalWrite(IN1_PIN, HIGH);  // Set motor direction to forward
    digitalWrite(IN2_PIN, LOW);
  } 
  // If the motor output is negative, rotate the motor in reverse
  else {
    digitalWrite(IN1_PIN, LOW);   // Set motor direction to reverse
    digitalWrite(IN2_PIN, HIGH);
    motorOutput = -motorOutput;   // Make the motor speed positive
  }

  // Set the motor speed using PWM (Pulse Width Modulation)
  analogWrite(PWM_PIN, constrain(motorOutput, 0, 255));  // Constrain motor speed to the range 0-255 (valid for PWM)
}
