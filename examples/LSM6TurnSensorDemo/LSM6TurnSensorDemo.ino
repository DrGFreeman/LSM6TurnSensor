/*
LSM6TurnSensorDemo.ino
Source: https://github.com/DrGFreeman/LSM6TurnSensor

MIT License

Copyright (c) 2017 Julien de la Bruere-Terreault <drgfreeman@tuta.io>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
This example shows how to use the LSM6TurnSensor library to continuously read
the relative turn angle (around z axis) using a LSM6DS33 IMU and how to reset it.
*/

#include <Wire.h>
#include <LSM6.h>
#include <LSM6TurnSensor.h>

// Create LSM6 IMU object instance (Pololu LSM6 library)
LSM6 imu;

// Create Turn Sensor object instance
// Pass IMU object by reference
LSM6TurnSensor turnSensor(&imu);

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize IMU
  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  
  // Enable IMU with default settings
  imu.enableDefault();

  // Set IMU gyro ODR & FS_G values
  // 0x84 = 0b10000100
  // ODR = 1000 (1.66 kHz (high performance)), FS_G = 01 (500 dps), FS_125 = 0, 0
  imu.writeReg(LSM6::CTRL2_G, 0x84);

  // Set Turn Sensor full scale turn rate sensitivity (ajusted)
  turnSensor.setFullScaleDPS(575);

  // Calibrate Turn Sensor (do not move)
  turnSensor.calibrate(200);
}

void loop() {
  // Get loop pass initial time
  unsigned long tInitial = millis();

  // Run for 10 seconds
  while (millis() - tInitial < 10000)
  {
    // Print turn angle to Serial (in 1/1000 of a degree)
    Serial.println(turnSensor.getAngle());
    delay(5);
  }
  // Then reset the angle to zero
  turnSensor.resetAngle();  
}
