/*
LSM6TurnSensor.cpp
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

#include <LSM6TurnSensor.h>

// Constructor
LSM6TurnSensor::LSM6TurnSensor(LSM6* imu)
{
  // LSM6 IMU object
  _imu = imu;

  // turn angle (in 1/1000 of a degree)
  _angle = 0;

  //

  // Time when IMU turn rate was last read (in ms)
  _timeLastRead = 0;

  // Turn rate set by the calibrate method (in dps)
  _turnRateError = 0;
}

// Measures and stores the turn rate (in dps) when stationary
void LSM6TurnSensor::calibrate(int nbPts)
{
  // Sum turn rates from multiple readings
  long turnRateSum = 0;
  for (int pt = 0; pt < nbPts; pt++)
  {
    _imu->read();
    turnRateSum += _imu->g.z;
    delay(10);
  }
  // Calculate average turn rate
  _turnRateError = turnRateSum / nbPts;
}

/*
Reads the turn rate from the IMU gyro, integrates it over the time from the
previous method call, adds the corresponding angle to the turn angle and returns
the updated turn angle.
*/
long LSM6TurnSensor::getAngle()
{
  // Calculate time step for integration
  long timeNow = millis();
  long dTime = timeNow - _timeLastRead;
  _timeLastRead = timeNow;

  // Read IMU
  _imu->read();

  // Integrate and add new delta angle to the turn angle
  _angle += (_imu->g.z - _turnRateError) * dTime * _fullScaleDPS / 32767;

  // Return angle
  return _angle;
}

// Resets the turn angle to zero
void LSM6TurnSensor::resetAngle()
{
  _angle = 0;
}

// Sets the gyro turn rate full scale sensitivity (in dps)
void LSM6TurnSensor::setFullScaleDPS(unsigned int fullScaleDPS)
{
  _fullScaleDPS = fullScaleDPS;
}
