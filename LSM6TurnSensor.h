/*
LSM6TurnSensor.h
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
LSM6TurnSensor; LSM6DS33 based turn angle sensor

This is a library for the Arduino IDE that provides methods to easily measure
turn angle using a LSM6DS33 IMU.

The relative turn angle is calculated by integrating the turn rate around the Z
axis measured by the IMU over the time between calls to the getAngle method.

The calibrate method is to be ran when the sensor is stationary. The measured
turn rate will be stored and substracted from future turn rate measurements.

The resetAngle method allows to reset the turn angle to zero.

Due to the gyro drift over time, the absolute turn angle will be inaccurate over
long periods but will be precise for a short period after call to the resetAngle
method.

This library interfaces with a LSM6 object from the Pololu Arduino library for
LSM6DS33 accelerometer and gyro https://github.com/pololu/lsm6-arduino. Refer to
this library documentation and to the ST LSM6DS33 datasheet
(https://www.pololu.com/file/0J1087/LSM6DS33.pdf) for details on how to set the
gyro turn rate sensitivity and other settings.
*/


#ifndef LSM6TurnSensor_h
#define LSM6TurnSensor_h

#include <Arduino.h>
#include <Wire.h>
#include <LSM6.h>

class LSM6TurnSensor
{
  public:
    /*
    Constructor. The constructor takes a LSM6 object passed by reference.
    Example:
      LSM6 imu;  // IMU object instance
      LSM6TurnSensor turnSensor(&imu);  // TurnSensor object instance
    */
    LSM6TurnSensor(LSM6* imu);

    /*
    Method to be ran when the sensor is stationary. It measures and stores the
    turn rate (in dps) and substracts is from subsequent turn rate readings.
    nbPts defines the number of measures to be taken and averaged (10 ms per
    measure).
    */
    void calibrate(int nbPts);

    // Returns the current angle (in 1/1000 of a degree)
    long getAngle();

    // Resets the angle to zero
    void resetAngle();

    // Sets the gyro full scale turn rate sensitivity (in dps)
    // Adjust value as needed to get accurate turn angles.
    void setFullScaleDPS(unsigned int fullScaleDPS);

  private:
    // LSM6 IMU object
    LSM6* _imu;

    // turn angle (in 1/1000 of a degree)
    long _angle;

    // Gyro full scale turn rate sensitivity (in dps)
    unsigned int _fullScaleDPS;

    // Time when IMU turn rate was last read (in ms)
    long _timeLastRead;

    // Turn rate set by the calibrate method (in dps)
    int _turnRateError;
};

#endif
