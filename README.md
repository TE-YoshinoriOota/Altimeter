# Altimeter
This system measures altitude or height using SPRESENSE, BMI270, and BMP581.
BMI270 is an IMU board and BMP581 is a barometer.
The motivation for fusing an IMU sensor and a barometer is to get stable altitude data. 
An altitude measurement system using only a barometer is unstable because the air pressure is constantly changing. 
For example, the air pressure is significantly affected by strong winds or the opening and closing of doors.


The system fusing an IMU sensor and a barometer can reduce large fluctuations over short periods.

Once you can obtain a stable altitude, you can achieve a more precise measurement of height between two different positions.
If altitude is measured with only one device, the value will be significantly affected by weather conditions over long periods.

If you can set a base sensor on the ground and a hand-carried sensor with you, you can get a precise height from the ground 
by taking the difference between the value of the base sensor and the value of the hand-carried sensor.

In my experiments, the accuracy achieved to ±0.15 m.


# Hardware Configuration
Here is the hardware configuration for the Altimeter system

- [Spresense Main Board](https://developer.sony.com/spresense/products/spresense-main-board/)
- [Spresense Extension Board](https://developer.sony.com/spresense/products/spresense-ext-board/) or [LTE-M Extension Board](https://developer.sony.com/spresense/products/spresense-lte-ext-board/)
- [SparkFun BMP581](https://www.sparkfun.com/products/20170)
- Self-made BMI270 Addon Board (if you want to use the diff_altitude example)

### Base Altimeter Configuration

### Hand-carried Altimeter Configuration

# Software Configuration
For details about the Spresense development environment, please visit [this site](https://developer.sony.com/spresense). 
You can find a bunch of technical documents

The driver for BMI270 used a part of [this Arduino library](https://github.com/arduino-libraries/Arduino_BMI270_BMM150) 
and added a minor modification to select the I2C device address in the begin function.

The driver for BMP581 used [this Sparkfun Arduino library](https://github.com/sparkfun/SparkFun_BMP581_Arduino_Library)

Altimeter estimates altitude by a two-step Kalman/Complimentary filter fusing values of the barometer, the accelerometer, and the gyroscope.
The implementation of the two-step Kalman/Complementary filter comes [here](https://github.com/juangallostra/AltitudeEstimation/tree/master), 
and the algorithm is explained in [this paper](https://simondlevy.academic.wlu.edu/files/2022/11/TwoStepFilter.pdf).

If you want to measure the height of two different points, you can refer to the diff_altitude example. 
The example contains the base_altimeter, the device_altimeter, and Python programs using [the bleak library](https://bleak.readthedocs.io/en/latest/).
The BLE device on Spresense to connect the BLE central on a Host PC running Python program above is BLE1507(https://crane-elec.co.jp/products/vol-24/).
You can get the Arduino library from [here](https://github.com/TE-YoshinoriOota/BLE1507_Arduino)



# License
This program contains multiple open-source licenses.

- Altimeter library under the [LGPL2.1](https://github.com/TE-YoshinoriOota/Altimeter/blob/main/LICENSE-LGPL2.1)
- The driver of BMI270 under the [LGPL2.1](https://github.com/TE-YoshinoriOota/Altimeter/blob/main/LICENSE-LGPL2.1)
- The driver of BMP581 under the [MIT license](https://github.com/TE-YoshinoriOota/Altimeter/blob/main/LICENSE-MIT)
- The two-step Kalman/Complementary filter under the [BSD-3-Clause license](https://github.com/TE-YoshinoriOota/Altimeter/blob/main/LICENSE-BSD)
