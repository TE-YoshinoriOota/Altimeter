/* 
 * MIT License
 * 
 * Copyright (c) 2018 Juan Gallostra Acin
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */ 
/*
    altitude.cpp: Altitude estimation via barometer/accelerometer fusion
*/

#include "filters.h"
#include "algebra.h"
#include "altitude.h"

AltitudeEstimator::AltitudeEstimator(float sigmaAccel, float sigmaGyro, float sigmaBaro,
                                     float ca, float accelThreshold)
:kalman(ca, sigmaGyro, sigmaAccel), complementary(sigmaAccel, sigmaBaro, accelThreshold)
{
  this->sigmaAccel = sigmaAccel;
  this->sigmaGyro = sigmaGyro;
  this->sigmaBaro = sigmaBaro;
  this->ca = ca;
  this->accelThreshold = accelThreshold;
}

void AltitudeEstimator::estimate(float accel[3], float gyro[3], float baroHeight, uint32_t timestamp)
{

  float deltat = (float)(timestamp-previousTime)/1000000.0f;
  float verticalAccel = kalman.estimate(pastGyro, pastAccel, deltat);
  complementary.estimate(&estimatedVelocity, &estimatedAltitude,
                         baroHeight,
                         pastAltitude, pastVerticalVelocity, pastVerticalAccel,
                         deltat);
  // update values for next iteration
  copyVector(pastGyro, gyro);
  copyVector(pastAccel, accel);
  pastAltitude = estimatedAltitude;
  pastVerticalVelocity = estimatedVelocity;
  pastVerticalAccel = verticalAccel;
  previousTime = timestamp;
}

float AltitudeEstimator::getAltitude()
{
  // return the last estimated altitude
  return estimatedAltitude;
}

float AltitudeEstimator::getVerticalVelocity()
{
  // return the last estimated vertical velocity
  return estimatedVelocity;
}

float AltitudeEstimator::getVerticalAcceleration()
{
  // return the last estimated vertical acceleration
  return pastVerticalAccel;
}

void AltitudeEstimator::updateSigmaAccel(float sigmaAccel) {
  this->sigmaAccel = sigmaAccel;
}

void AltitudeEstimator::updateSigmaGyro(float sigmaGyro) {
  this->sigmaGyro = sigmaGyro;
}

void AltitudeEstimator::updateSigmaBaro(float sigmaBaro) {
  this->sigmaBaro = sigmaBaro;
}

void AltitudeEstimator::updateCA(float ca) {
  this->ca = ca;
}

void AltitudeEstimator::updateZUPT(float accelThreshold) {
  this->accelThreshold = accelThreshold;
}

