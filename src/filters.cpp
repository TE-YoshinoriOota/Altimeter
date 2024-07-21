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
   filters.cpp: Filter class implementations
 */

//#include <cmath>
#include <stdlib.h> // XXX eventually use fabs() instead of abs() ?
#include <stdio.h>

#include "filters.h"

void KalmanFilter::getPredictionCovariance(float covariance[3][3], float previousState[3], float deltat)
{
    // required matrices for the operations
    float sigma[3][3];
    float identity[3][3];
    identityMatrix3x3(identity);
    float skewMatrix[3][3];
    skew(skewMatrix, previousState);
    float tmp[3][3];
    // Compute the prediction covariance matrix
    scaleMatrix3x3(sigma, pow(sigmaGyro, 2), identity);
    matrixProduct3x3(tmp, skewMatrix, sigma);
    matrixProduct3x3(covariance, tmp, skewMatrix);
    scaleMatrix3x3(covariance, -pow(deltat, 2), covariance);
}

void KalmanFilter::getMeasurementCovariance(float covariance[3][3])
{
    // required matrices for the operations
    float sigma[3][3];
    float identity[3][3];
    identityMatrix3x3(identity);
    float norm;
    // Compute measurement covariance
    scaleMatrix3x3(sigma, pow(sigmaAccel, 2), identity);
    vectorLength(& norm, previousAccelSensor);
    scaleAndAccumulateMatrix3x3(sigma, (1.0/3.0)*pow(ca, 2)*norm, identity);
    copyMatrix3x3(covariance, sigma);
}

void KalmanFilter::predictState(float predictedState[3], float gyro[3], float deltat)
{
    // helper matrices
    float identity[3][3];
    identityMatrix3x3(identity);
    float skewFromGyro[3][3];
    skew(skewFromGyro, gyro);
    // Predict state
    scaleAndAccumulateMatrix3x3(identity, -deltat, skewFromGyro);
    matrixDotVector3x3(predictedState, identity, currentState);
    normalizeVector(predictedState);
}

void KalmanFilter::predictErrorCovariance(float covariance[3][3], float gyro[3], float deltat)
{
    // required matrices
    float Q[3][3];
    float identity[3][3];
    identityMatrix3x3(identity);
    float skewFromGyro[3][3];
    skew(skewFromGyro, gyro);
    float tmp[3][3];
    float tmpTransposed[3][3];
    float tmp2[3][3];
    // predict error covariance
    getPredictionCovariance(Q, currentState, deltat);
    scaleAndAccumulateMatrix3x3(identity, -deltat, skewFromGyro);
    copyMatrix3x3(tmp, identity);
    transposeMatrix3x3(tmpTransposed, tmp);
    matrixProduct3x3(tmp2, tmp, currErrorCovariance);
    matrixProduct3x3(covariance, tmp2, tmpTransposed);
    scaleAndAccumulateMatrix3x3(covariance, 1.0, Q);
}

void KalmanFilter::updateGain(float gain[3][3], float errorCovariance[3][3])
{
    // required matrices
    float R[3][3];
    float HTransposed[3][3];
    transposeMatrix3x3(HTransposed, H);
    float tmp[3][3];
    float tmp2[3][3];
    float tmp2Inverse[3][3];
    // update kalman gain
    // P.dot(H.T).dot(inv(H.dot(P).dot(H.T) + R))
    getMeasurementCovariance(R);
    matrixProduct3x3(tmp, errorCovariance, HTransposed);
    matrixProduct3x3(tmp2, H, tmp);
    scaleAndAccumulateMatrix3x3(tmp2, 1.0, R);
    invert3x3(tmp2Inverse, tmp2);
    matrixProduct3x3(gain, tmp, tmp2Inverse);
}

void KalmanFilter::updateState(float updatedState[3], float predictedState[3], float gain[3][3], float accel[3])
{
    // required matrices
    float tmp[3];
    float tmp2[3];
    float measurement[3];
    scaleVector(tmp, ca, previousAccelSensor);
    subtractVectors(measurement, accel, tmp);
    // update state with measurement
    // predicted_state + K.dot(measurement - H.dot(predicted_state))
    matrixDotVector3x3(tmp, H, predictedState);
    subtractVectors(tmp, measurement, tmp);
    matrixDotVector3x3(tmp2, gain, tmp);
    sumVectors(updatedState, predictedState, tmp2);
    normalizeVector(updatedState);
}

void KalmanFilter::updateErrorCovariance(float covariance[3][3], float errorCovariance[3][3], float gain[3][3])
{
    // required matrices
    float identity[3][3];
    identityMatrix3x3(identity);
    float tmp[3][3];
    float tmp2[3][3];
    // update error covariance with measurement
    matrixProduct3x3(tmp, gain, H);
    matrixProduct3x3(tmp2, tmp, errorCovariance);
    scaleAndAccumulateMatrix3x3(identity, -1.0, tmp2);
    copyMatrix3x3(covariance, tmp2);
}


KalmanFilter::KalmanFilter(float ca, float sigmaGyro, float sigmaAccel)
{
    this->ca = ca;
    this->sigmaGyro = sigmaGyro;
    this->sigmaAccel = sigmaAccel;
}

float KalmanFilter::estimate(float gyro[3], float accel[3], float deltat)
{
    float predictedState[3];
    float updatedState[3];
    float errorCovariance[3][3];
    float updatedErrorCovariance[3][3];
    float gain[3][3];
    float accelSensor[3];
    //float tmp[3];
    float accelEarth;

    /*
    printf("%f, %f, %f, ", accel[0], accel[1], accel[2]);
    scaleVector(accel, 9.81, accel); // Scale accel readings since they are measured in gs
    printf("%f, %f, %f\n", accel[0], accel[1], accel[2]);
    */

    // perform estimation
    // predictions
    predictState(predictedState, gyro, deltat);
    predictErrorCovariance(errorCovariance, gyro, deltat);

    // updates
    updateGain(gain, errorCovariance);
    updateState(updatedState, predictedState, gain, accel);
    updateErrorCovariance(updatedErrorCovariance, errorCovariance, gain);

    // Store required values for next iteration
    copyVector(currentState, updatedState);
    copyMatrix3x3(currErrorCovariance, updatedErrorCovariance);

    // return vertical acceleration estimate
    /*
    scaleVector(tmp, 9.81, updatedState);
    subtractVectors(accelSensor, accel, tmp);
    */
    subtractVectors(accelSensor, accel, updatedState);
    copyVector(previousAccelSensor, accelSensor);
    dotProductVectors(& accelEarth, accelSensor, updatedState);
    return accelEarth;
}


float ComplementaryFilter::ApplyZUPT(float accel, float vel)
{
    // first update ZUPT array with latest estimation
    ZUPT[ZUPTIdx] = accel;
    // and move index to next slot
    uint8_t nextIndex = (ZUPTIdx + 1) % ZUPT_SIZE;
    ZUPTIdx = nextIndex;
    // Apply Zero-velocity update
    for (uint8_t k = 0; k < ZUPT_SIZE; ++k) {
        if (abs(ZUPT[k]) > accelThreshold) return vel;
    }
    return 0.0;
}


ComplementaryFilter::ComplementaryFilter(float sigmaAccel, float sigmaBaro, float accelThreshold)
{
    // Compute the filter gain
    gain[0] = sqrt(2 * sigmaAccel / sigmaBaro);
    gain[1] = sigmaAccel / sigmaBaro;
    // If acceleration is below the threshold the ZUPT counter
    // will be increased
    this->accelThreshold = accelThreshold;
    // initialize zero-velocity update
    ZUPTIdx = 0;
    for (uint8_t k = 0; k < ZUPT_SIZE; ++k) {
        ZUPT[k] = 0;
    }
}

void ComplementaryFilter::estimate(float * velocity, float * altitude, float baroAltitude,
        float pastAltitude, float pastVelocity, float accel, float deltat)
{
    // Apply complementary filter
    *altitude = pastAltitude + deltat*(pastVelocity + (gain[0] + gain[1]*deltat/2)*(baroAltitude-pastAltitude))+
        accel*pow(deltat, 2)/2;
    *velocity = pastVelocity + deltat*(gain[1]*(baroAltitude-pastAltitude) + accel);
    // Compute zero-velocity update
    *velocity = ApplyZUPT(accel, *velocity);
}
