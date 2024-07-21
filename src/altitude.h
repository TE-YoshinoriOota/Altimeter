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
    altitude.h: Altitude estimation via barometer/accelerometer fusion
*/

# pragma once

#include "filters.h"
#include "algebra.h"

#include <Arduino.h> // XXX For micros; eventually need to compute micros() elsewhere

#define DEFAULT_SIGMA_ACCEL  0.01
#define DEFAULT_SIGMA_GYRO   0.01
#define DEFAULT_SIGMA_BARO   0.02
#define DEFAULT_CONST_ACCEL  0.2
#define DEFAULT_ACCEL_THRESH 0.05

class AltitudeEstimator {

  private:
    // required parameters for the filters used for the estimations
    // sensor's standard deviations
    float sigmaAccel;
    float sigmaGyro;
    float sigmaBaro;
    // Acceleration markov chain model state transition constant
    float ca;
    // Zero-velocity update acceleration threshold
    float accelThreshold;
    // gravity
    float g = 9.81;
    // For computing the sampling period
    uint32_t previousTime = micros();
    // required filters for altitude and vertical velocity estimation
    KalmanFilter kalman;
    ComplementaryFilter complementary;
    // Estimated past vertical acceleration
    float pastVerticalAccel = 0;
    float pastVerticalVelocity = 0;
    float pastAltitude = 0;
    float pastGyro[3] = {0, 0, 0};
    float pastAccel[3] = {0, 0, 0};
    // estimated altitude and vertical velocity
    float estimatedAltitude = 0;
    float estimatedVelocity = 0;

  public:

    AltitudeEstimator(float sigmaAccel = DEFAULT_SIGMA_ACCEL, 
                      float sigmaGyro = DEFAULT_SIGMA_GYRO, 
                      float sigmaBaro = DEFAULT_SIGMA_BARO,
                      float ca = DEFAULT_CONST_ACCEL, 
                      float accelThreshold = DEFAULT_ACCEL_THRESH);

    void estimate(float accel[3], float gyro[3], float baroHeight, uint32_t timestamp);

    float getAltitude();

    float getVerticalVelocity();

    float getVerticalAcceleration();

    void updateSigmaAccel(float sigmaAccel);
    void updateSigmaGyro(float sigmaGyro);
    void updateSigmaBaro(float sigmaBaro);
    void updateCA(float ca);
    void updateZUPT(float accelThreshold);
}; // class AltitudeEstimator
