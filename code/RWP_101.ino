#include <Wire.h>           // Includes the Wire library for I2C communication
#include <MPU6050_light.h>   // Includes the MPU6050 library for interacting with the MPU6050 sensor

// Initialize MPU6050 object with Wire (I2C)
MPU6050 mpu(Wire);

// Timing variables for controlling the sample rate (time between updates)
unsigned long lastTime = 0;  // Stores the last time the loop was updated
const unsigned long SAMPLE_TIME = 5; // Sample time for the loop in milliseconds (5ms = 200Hz)

// Timing variable for calculating delta time between PID updates
long prevT = 0;

void setup() {
  Serial.begin(9600);   // Initialize serial communication at a baud rate of 9600 bits per second for debugging
  Wire.begin();         // Initialize I2C communication with the MPU6050 sensor
  

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
