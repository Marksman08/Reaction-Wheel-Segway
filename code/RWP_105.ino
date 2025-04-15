#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);  // Create an MPU6050 object

// Pin definitions for the motors
const int PWM_PIN = 5;
const int IN1_PIN = 7;
const int IN2_PIN = 6;
const int IN1_MOTOR1 = 8;
const int IN2_MOTOR1 = 9;
const int PWM_MOTOR1 = 10;
const int IN1_MOTOR2 = 13;
const int IN2_MOTOR2 = 12;
const int PWM_MOTOR2 = 11;

// PID controller variables
float Kp_outer = 50.0, Ki_outer = 0.0, Kd_outer = 0.0;
float Kp_inner = 80.0, Ki_inner = 0.0, Kd_inner = 0.0;
float setpoint_angle = 0.0, lastAngleError = 0.0, integral_angle = 0.0;
float lastVelocityError = 0.0, integral_velocity = 0.0;

unsigned long lastTime = 0;
const unsigned long SAMPLE_TIME = 5; // Control loop sample time
bool systemShutDown = false;  // Flag to track if system is shut down

void setup() {
  Serial.begin(115200);         // Initialize serial communication
  Wire.begin();                 // Initialize I2C communication

  // Initialize motor pins
  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN1_MOTOR1, OUTPUT);
  pinMode(IN2_MOTOR1, OUTPUT);
  pinMode(PWM_MOTOR1, OUTPUT);
  pinMode(IN1_MOTOR2, OUTPUT);
  pinMode(IN2_MOTOR2, OUTPUT);
  pinMode(PWM_MOTOR2, OUTPUT);

  // Initialize MPU6050 sensor
  byte status = mpu.begin();
  while (status != 0) {  // Check if MPU6050 is connected properly
    Serial.println("MPU6050 initialization failed.");
    delay(1000);
  }
  Serial.println("MPU6050 initialized successfully!");
  mpu.calcOffsets();  // Calibrate MPU6050
  Serial.println("Calibration complete!");
}

void loop() {
  // Check if system is shut down, and stop motors immediately if so
  if (systemShutDown) {
    stopMotors();  // Stop all motors
    Serial.println("System is shutting down...");
    while (true) {
      // Keep the system halted indefinitely
      delay(100);
    }
  }

  // Check for serial input
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // Read incoming command

    if (command == "SHUTDOWN") {
      systemShutDown = true;  // Set shutdown flag to true
    }
  }

  // Continue running the stabilization logic if not shut down
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= SAMPLE_TIME) {
    float deltaTime = (float)(currentTime - lastTime) / 1000.0;
    maintainStability(deltaTime);
    lastTime = currentTime;
  }
}

// Function to maintain stability
void maintainStability(float deltaTime) {
  mpu.update();
  float angle = mpu.getAngleX();  // Get the tilt angle
  Serial.print("Maintaining stability, Angle: ");
  Serial.println(angle);
  float desiredVelocity = calculateOuterPID(angle, deltaTime);
  float bottomMotorOutput = calculateInnerPID(desiredVelocity, deltaTime);
  setMotorSpeed(bottomMotorOutput, desiredVelocity);  // Control motors
}

// Function to stop all motors
void stopMotors() {
  analogWrite(PWM_MOTOR1, 0);
  analogWrite(PWM_MOTOR2, 0);
  analogWrite(PWM_PIN, 0);
  digitalWrite(IN1_MOTOR1, LOW);
  digitalWrite(IN2_MOTOR1, LOW);
  digitalWrite(IN1_MOTOR2, LOW);
  digitalWrite(IN2_MOTOR2, LOW);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}

// Outer PID control (tilt control)
float calculateOuterPID(float angle, float deltaTime) {
  float angleError = setpoint_angle - angle;
  integral_angle += angleError * deltaTime;
  float derivative_angle = (angleError - lastAngleError) / deltaTime;
  float output = Kp_outer * angleError + Ki_outer * integral_angle + Kd_outer * derivative_angle;
  lastAngleError = angleError;
  return output;
}

// Inner PID control (velocity control)
float calculateInnerPID(float desiredVelocity, float deltaTime) {
  float velocityError = desiredVelocity;
  integral_velocity += velocityError * deltaTime;
  float derivative_velocity = (velocityError - lastVelocityError) / deltaTime;
  float output = Kp_inner * velocityError + Ki_inner * integral_velocity + Kd_inner * derivative_velocity;
  lastVelocityError = velocityError;
  return output;
}

// Set motor speeds based on PID output
void setMotorSpeed(float bottomOutput, float reactionOutput) {
  int bottomMotorSpeed = map(abs(bottomOutput), 0, 180, 150, 255);
  bottomMotorSpeed = constrain(bottomMotorSpeed, 150, 255);
  int reactionWheelSpeed = map(abs(reactionOutput), 0, 180, 150, 255);
  reactionWheelSpeed = constrain(reactionWheelSpeed, 150, 255);

  // Control bottom motors (left and right)
  if (bottomOutput > 0) {
    digitalWrite(IN1_MOTOR1, LOW);
    digitalWrite(IN2_MOTOR1, HIGH);
    digitalWrite(IN1_MOTOR2, LOW);
    digitalWrite(IN2_MOTOR2, HIGH);
  } else {
    digitalWrite(IN1_MOTOR1, HIGH);
    digitalWrite(IN2_MOTOR1, LOW);
    digitalWrite(IN1_MOTOR2, HIGH);
    digitalWrite(IN2_MOTOR2, LOW);
  }
  analogWrite(PWM_MOTOR1, bottomMotorSpeed);
  analogWrite(PWM_MOTOR2, bottomMotorSpeed);

  // Control reaction wheel motor
  if (reactionOutput > 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  }
  analogWrite(PWM_PIN, reactionWheelSpeed);
}
