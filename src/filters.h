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
   filters.h: Filter class declarations
 */

#pragma once

//#include <cmath>
#include <math.h>
#include <stdint.h>

#include "algebra.h"

class KalmanFilter {
  private:
    float currentState[3] = {0, 0, 1};
    float currErrorCovariance[3][3] = {{100, 0, 0},{0, 100, 0},{0, 0, 100}};
    float H[3][3] = {{9.81, 0, 0}, {0, 9.81, 0}, {0, 0, 9.81}};
    float previousAccelSensor[3] = {0, 0, 0};
    float ca;
    float sigmaGyro;
    float sigmaAccel;

    void getPredictionCovariance(float covariance[3][3], float previousState[3], float deltat);

    void getMeasurementCovariance(float covariance[3][3]);

    void predictState(float predictedState[3], float gyro[3], float deltat);

    void predictErrorCovariance(float covariance[3][3], float gyro[3], float deltat);

    void updateGain(float gain[3][3], float errorCovariance[3][3]);

    void updateState(float updatedState[3], float predictedState[3], float gain[3][3], float accel[3]);

    void updateErrorCovariance(float covariance[3][3], float errorCovariance[3][3], float gain[3][3]);

  public:

    KalmanFilter(float ca, float sigmaGyro, float sigmaAccel);

    float estimate(float gyro[3], float accel[3], float deltat);

}; // Class KalmanFilter

class ComplementaryFilter {

  private:

    // filter gain
    float gain[2];
    // Zero-velocity update
    float accelThreshold;
    static const uint8_t ZUPT_SIZE = 12;
    uint8_t ZUPTIdx;
    float   ZUPT[ZUPT_SIZE];

    float ApplyZUPT(float accel, float vel);

  public:

    ComplementaryFilter(float sigmaAccel, float sigmaBaro, float accelThreshold);

    void estimate(float * velocity, float * altitude, float baroAltitude,
                  float pastAltitude, float pastVelocity, float accel, float deltat);
}; // Class ComplementaryFilter
