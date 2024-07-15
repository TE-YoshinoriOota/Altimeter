#ifndef ALTIMETER_HEADER_GUARD
#define ALTIMETER_HEADER_GUARD

#include <Wire.h>
#include "SparkFun_BMP581_Arduino_Library.h"
#include "BMI270_Arduino.h"
#include "altitude.h"
#include <pthread.h>

// BMP581 ODR is 240Hz 
enum IIR_BANDWIDTH {
   IIR_BANDWIDTH_00_3HZ
  ,IIR_BANDWIDTH_00_6HZ
  ,IIR_BANDWIDTH_01_2HZ
  ,IIR_BANDWIDTH_02_5HZ
  ,IIR_BANDWIDTH_05_1HZ
  ,IIR_BANDWIDTH_11_0HZ
  ,IIR_BANDWIDTH_27_5HZ
  ,IIR_BANDWIDTH_BYPASS
};

const int SEALEVELPRESSURE_HPA = 1013.25;
const int G_CONSTANT = 9.81;
const int DEFAULT_SENSING_INTERVAL_mSEC = 20; // msec
const int DEFAULT_UPDATE_INTERVAL_SEC = 1; // sec
const float DEFAULT_CALIB_THRESH_CONV = 0.15; // meter
const float DEFAULT_CALIB_THRESH_DIFF = 1.80; // meter
const int DEFAULT_CALIB_STABILITY_COUNT = 10; 

const float DEFAULT_SIGMA_ACCEL_VALUE = 0.01;
const float DEFAULT_SIGMA_GYRO_VALUE = 0.01;
const float DEFAULT_SIGMA_BARO_VALUE = 0.03;
const float DEFAULT_CONST_ACCEL_VALUE = 0.1;
const float DEFAULT_ACCEL_THRESH_VALUE = 0.02;

extern "C" {
  void* do_calibration_thread(void*arg);
  void* do_monitor_thread(void*arg);
} // extern "C"

class AltimeterClass {
  public:
    AltimeterClass();
    ~AltimeterClass();
    bool begin(uint32_t interval_msec = DEFAULT_SENSING_INTERVAL_mSEC
              ,uint32_t update_sec = DEFAULT_UPDATE_INTERVAL_SEC
              ,enum IIR_BANDWIDTH bw = IIR_BANDWIDTH_00_3HZ
              ,bool average_filter = true 
              );
    void startCalibration(uint32_t timeout_ms = 0);
    void startSensing(uint32_t duration_sec = 0);
    void endSensing() { m_monitor_end = true; }

    void setSensingInteraval(int msec) { m_interval_msec = msec; }
    void setUpdateInterval(int msec) { m_update_sec = msec; }

    void setAccelSigma(float sigma) { m_altitude.updateSigmaAccel(sigma); }
    void setGyroSigma(float sigma) { m_altitude.updateSigmaGyro(sigma); }
    void setBaroSigma(float sigma) { m_altitude.updateSigmaBaro(sigma); }
    void setConstantAccel(float ca) { m_altitude.updateCA(ca); }
    void setAccelThreshold(float th) { m_altitude.updateZUPT(th); }

    bool setIIRFilter(enum IIR_BANDWIDTH bw);
    bool setAverageFilter(bool enable = true);
    void setBarometerMode() { m_fusion_mode = false; }
    void setFusionMode() { m_fusion_mode = true; }
    void setSeaLevelPressure(float pressure) { m_sea_level_pressure = pressure; }
    void setDebugMode(bool enable) { m_debug = enable; }
    void setCalibConversionThresh(float calib_th) { m_calib_th_conv = calib_th; }
    void setCalibDiffThresh(float calib_th) { m_calib_th_diff = calib_th; }

    uint32_t getSensingInterval() { return m_interval_msec; }
    uint32_t getUpdateInterval() { return m_update_sec; }
    uint32_t getCalibrationTimeout() { return m_calibration_timeout_msec; }
    uint32_t getMonitorTimeout() { return m_monitor_timeout_sec; }

    enum IIR_BANDWIDTH getIIRFilterMode() { return m_iir_bw; }
    float getAccelSigma() { return m_accel_sigma; }
    float getGyroSigma() { return m_gyro_sigma; }
    float getBaroSigma() { return m_baro_sigma; }
    float getConstantAccel() { return m_ca; }
    float getAccelThreshold() { return m_accel_threshold; }

    bool isBegin() { return (m_bmp581_begin && m_bmi270_begin); }
    bool isCalibrated() { return m_calibrated; }
    bool isUpdate() { return m_update; }
    bool isFusionMode() { return m_fusion_mode; }
    bool isAverageFilterMode() { return m_average_filter; }
    bool isDebugMode() { return m_debug; }

    float getTemperature() { m_update = false; return m_temp; }
    float getPressure() { m_update = false; return m_pressure; }
    float getAltitude() { m_update = false; if (!m_fusion_mode) return m_baro_altitude; return m_fusion_altitude; }
    float getFusionAltitude() { m_update = false; return m_fusion_altitude; }
    float getBaroAltitude() { m_update = false; return m_baro_altitude; }

  protected:
    friend void* do_calibration_thread(void*arg);
    friend void* do_monitor_thread(void*arg);
    /** these functions are used by thread functions **/
    bool isMonitorEnd() { return m_monitor_end; }
    void setCalibrated() { m_calibrated = true; }
    void push_pressure(float pressure);
    void push_temp(float temp);
    void push_fusion_altitude(float altitude);
    void push_baro_altitude(float altitude);
    void updateTemperature();
    void updatePressure();
    void updateAltitude();
    bool altitude_estimation(float *pressure, float *temp, float *baro_altitude, float *fusion_altitude);
    float get_calib_thresh_conv() { return m_calib_th_conv; }
    float get_calib_thresh_diff() { return m_calib_th_diff; }

  private:
    float calc_baro_altitude(float pressure);
    bool imu_read(float gyro[3], float accel[3]);
    void panic_led_trap();
 
  private:
    uint32_t m_interval_msec;
    uint32_t m_update_sec;
    uint32_t m_calibration_timeout_msec;
    uint32_t m_monitor_timeout_sec;

    float m_accel_sigma;
    float m_gyro_sigma;
    float m_baro_sigma;
    float m_ca;
    float m_accel_threshold;

    float m_sea_level_pressure;
    float m_calib_th_conv;
    float m_calib_th_diff;

    bool m_monitor_end;
    bool m_debug;
    bool m_calibrated;
    bool m_update;
    bool m_bmp581_begin;
    bool m_bmi270_begin;
    bool m_fusion_mode;
    bool m_average_filter;
    enum IIR_BANDWIDTH m_iir_bw;

    float m_pressure;
    float m_temp;
    float m_fusion_altitude;
    float m_baro_altitude;
    float *m_pressure_array;
    float *m_temp_array;
    float *m_fusion_altitude_array;
    float *m_baro_altitude_array;
    uint16_t m_array_size;

    AltitudeEstimator m_altitude;
    BMI270Class m_bmi270;
    BMP581 m_bmp581;
};

extern AltimeterClass Altimeter;

#endif // ALTIMETER_HEADER_GUARD
