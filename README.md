# Altimeter
Altimeter can measure more precise altitude using an IMU and a Barometer fusion filter than a system using only a barometer. This system consists of SPRESENSE, BMI270, and BMP581. BMI270 is an IMU board and BMP581 is a barometer.

The motivation for fusing an IMU sensor and a barometer is to get stable altitude data. 
An altitude measurement system using only a barometer is unstable because the air pressure is constantly changing. 
For example, the air pressure is significantly affected by strong winds or the opening and closing of doors.

The system fusing an IMU sensor and a barometer can reduce large fluctuations over short periods. The graph below shows a comparison of the altitude estimation data between the barometer and the IMU/barometer fusion system. It seems that there is a time delay in the IMU and the Barometer fusion data due to the Kalman filter. However, the effect of noise reduction is quite good.

![image](https://github.com/user-attachments/assets/7ce72d97-9529-451d-9037-b912e8258149)

<br/>


Once you can obtain a stable altitude, you can achieve a more precise measurement of height between two different positions.
If altitude is measured with only one device, the value will be significantly affected by weather conditions over long periods.
If you can set a base sensor on the ground and a hand-carried sensor with you, you can get a precise height from the ground 
by taking the difference between the value of the base sensor and the value of the hand-carried sensor.
In my experiments, the accuracy achieved to ±0.15 m.

![image](https://github.com/user-attachments/assets/22fe7304-3baf-4bcc-8de9-8f0c9c1ca5a9)

![image](https://github.com/user-attachments/assets/0a91027a-e1a2-4c4e-869a-3716b5680ff7)


# Hardware Configuration
Here is the hardware configuration for the Altimeter system. The base altimeter and the hand-carried altimeter are the same system. But in my case, I used the Spresense LTE-M Extension board to get the small form factor to do the test easily. It means that you can also use LTE-M for communication.

- [Spresense Main Board](https://developer.sony.com/spresense/products/spresense-main-board/)
- [Spresense Extension Board](https://developer.sony.com/spresense/products/spresense-ext-board/) or [LTE-M Extension Board](https://developer.sony.com/spresense/products/spresense-lte-ext-board/)
- [SparkFun BMP581](https://www.sparkfun.com/products/20170)
- Self-made BMI270 Addon Board (if you want to use the diff_altitude example)
- [BLE1507](https://crane-elec.co.jp/products/vol-24/) (if you want to do the differential measurement)

### Base Altimeter Configuration
![image](https://github.com/user-attachments/assets/6689577a-6a2c-4e4c-8ba6-e52f75bb702e)

### Hand-carried Altimeter Configuration
Hand-carried Altimeter uses Spresense LTE-M Extension Board to get the small form factor.
![image](https://github.com/user-attachments/assets/a7f2e391-faaf-40c2-9303-ccb9e175de50)

# Software Configuration
For details about the Spresense development environment, please visit [this site](https://developer.sony.com/spresense). 
You can find a bunch of technical documents

The driver for BMI270 used a part of [this Arduino library](https://github.com/arduino-libraries/Arduino_BMI270_BMM150) 
and added a minor modification to select the I2C device address in the begin function.

The driver for BMP581 used [this Sparkfun Arduino library](https://github.com/sparkfun/SparkFun_BMP581_Arduino_Library)

### Sensor Fusion Filter for Altitude Estimation
Altimeter estimates altitude by a two-step Kalman/Complimentary filter fusing values of the barometer, the accelerometer, and the gyroscope.
The implementation of the two-step Kalman/Complementary filter comes [here](https://github.com/juangallostra/AltitudeEstimation/tree/master), 
and the algorithm is explained in [this paper](https://simondlevy.academic.wlu.edu/files/2022/11/TwoStepFilter.pdf).

There are five parameters for the filter. It should be calibrated for each use case. For a pedestrian case, the parameter ranges are as follows:

<br/>

*Table. Parameters for Kalman/Complimentary filter*
| Parameter      | Description | Typical Range | Default value |
| -------------- | ------------- | ------------- | ------------- |
| AccelSigma     | Sigma value of Acceleration Sensor  | 0.03 - 0.001  | 0.01 | 
| GyroSigma      | Sigma value of Gyroscope   | 0.03 - 0.001  |  0.01 |
| BaroSigma      | Sigma value of Barometer   | 0.1 - 0.01  | 0.03 |
| ConstantAccel  | The constant acceleration value for a model in the Kalman Filter  | 0.2 - 0.05  | 0.1  | 
| AccelThreshold | Threshold value of the acceleration to reset the offset caused by sensor drift  | 0.05 - 0.005  | 0.02  | 

<br/>

These parameters should be set for the use case that you want to apply. Please do the experiments to fix these value before the deployment.
For the details of the contant acceleration and the threshold value of the acceleration, 
please refer to [this paper](https://simondlevy.academic.wlu.edu/files/2022/11/TwoStepFilter.pdf).

### Differential measurement
If you want to measure the height of two different points, you can refer to the diff_altitude example. 
The example contains the base_altimeter, the device_altimeter, and Python programs using [the bleak library](https://bleak.readthedocs.io/en/latest/).
The BLE device on Spresense to connect the BLE central on a Host PC running Python program above is [BLE1507](https://crane-elec.co.jp/products/vol-24/).
You can get the Arduino library from [here](https://github.com/TE-YoshinoriOota/BLE1507_Arduino)

<br/>

![image](https://github.com/user-attachments/assets/1c7bcd55-1adb-4024-8c72-2d637f3400f3)


### Overview of Altimeter Library

| API name                 | Description | Default |
| ------------------------ | ----------- | ------- |
| begin                    |             |         |
| startCalibration         |             |         |
| startSensing             |             |         |
| endSensing               |             |         |
| setSensingInterval       |             |         |
| setUpdateInterval        |             |         |
| setAccelSigma            |             |         |
| setGyroSigma             |             |         |
| setBaroSigma             |             |         |
| setConstantAccel         |             |         |
| setAccelThreshold        |             |         |
| setIIRFilter             |             |         |
| setAverageFilter         |             |         |
| setBarometerMode         |             |         |
| setFusionMode            |             |         |
| setSeaLevelPressure      |             |         |
| setCalibConversionThresh |             |         |
| setCalibDiffThresh       |             |         |
| isCalibrated             |             |         |
| isUpdate                 |             |         |
| getTemperature           |             |         |
| getPressure              |             |         |
| getAltitude              |             |         |


# License
This program contains multiple open-source licenses.

- Altimeter library under the [LGPL2.1](https://github.com/TE-YoshinoriOota/Altimeter/blob/main/LICENSE-LGPL2.1)
- The driver of BMI270 under the [LGPL2.1](https://github.com/TE-YoshinoriOota/Altimeter/blob/main/LICENSE-LGPL2.1)
- The driver of BMP581 under the [MIT license](https://github.com/TE-YoshinoriOota/Altimeter/blob/main/LICENSE-MIT)
- The two-step Kalman/Complementary filter under the [BSD-3-Clause license](https://github.com/TE-YoshinoriOota/Altimeter/blob/main/LICENSE-BSD)
