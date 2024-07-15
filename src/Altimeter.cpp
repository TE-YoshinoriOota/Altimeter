#include "Altimeter.h"

AltimeterClass Altimeter;

pthread_t calib_thread;
pthread_t monitor_thread;


extern "C" {

  /* calibration thread */
  void* do_calibration_thread(void *arg) {

    if (Altimeter.isCalibrated()) {
      fprintf(stderr, "WARNING: the system is already calibrated.\n");
      return nullptr;
    }
    if (!Altimeter.isBegin()) {
      fprintf(stderr, "ERROR: BMP581 and/or BMI270 is not initialized\n");
      return nullptr;
    }
    if (!Altimeter.isFusionMode()) {
      fprintf(stderr, "WARNING: not fusion mode. the altitude caluculated only barometer.\n");
      return nullptr;
    }

    static uint32_t start_calibration_time_ms = millis();
    float last_baro_altitude = 0.0;
    float last_fusion_altitude = 0.0;

    while (!Altimeter.isMonitorEnd()) {
      /* acquire altitude data */
      float pressure, temp, baro_altitude, fusion_altitude;
      Altimeter.altitude_estimation(
          &pressure, &temp, &baro_altitude, &fusion_altitude);

      /* average filter process */
      Altimeter.push_pressure(pressure);
      Altimeter.push_temp(temp);
      Altimeter.push_fusion_altitude(fusion_altitude);
      Altimeter.push_baro_altitude(baro_altitude);

      /* wait for coming the data from sensors */
      if (baro_altitude == 0.0 && fusion_altitude == 0.0) continue;

      /* judge whether the calibration is convergence */
      static int stability_counter = 0;
      float diff_baro_altitude = abs(baro_altitude - last_baro_altitude); 
      float diff_fusion_altitude = abs(fusion_altitude - last_fusion_altitude); 
      float diff_baro_fusion_altitude = abs(baro_altitude - fusion_altitude); 
      if ((diff_baro_altitude < Altimeter.get_calib_thresh_conv()) 
        && (diff_fusion_altitude < Altimeter.get_calib_thresh_conv())
        && (diff_baro_fusion_altitude < Altimeter.get_calib_thresh_diff())
        )
      {
        if (stability_counter++ == DEFAULT_CALIB_STABILITY_COUNT) {
          Altimeter.setCalibrated();
          fprintf(stderr, "INFO: Calibration Finished\n");
          break;
        }
      } else {
        stability_counter = 0;
      }

      if (Altimeter.isDebugMode()){
        printf("%f, %f, %f\n", diff_baro_altitude, diff_fusion_altitude, diff_baro_fusion_altitude); 
      }

      /* check timeout */
      uint32_t duration = millis() - start_calibration_time_ms;
      if ((Altimeter.getCalibrationTimeout() > 0) && (duration >= Altimeter.getCalibrationTimeout())) {
        if (Altimeter.isCalibrated() == false) {
          fprintf(stderr, "WARNING: Calibration is not finished\n");
        }
        fprintf(stderr, "INFO: Calibration timeout\n");
        break;
      }

      /* memory last value */
      last_baro_altitude = baro_altitude;
      last_fusion_altitude = fusion_altitude;

      /* sleep to yeild computing resources to other process */
      usleep(Altimeter.getSensingInterval()*1000);
    }
    return nullptr;
  }

  /* monitoring thread */
  void* do_monitor_thread(void *arg) {
    if (!Altimeter.isBegin()) {
      fprintf(stderr, "ERROR: BMP581 and/or BMI270 is not initialized\n");
      return nullptr;
    }
    if (!Altimeter.isCalibrated()) {
      fprintf(stderr, "ERROR: the system is not calibrated.\n");
      return nullptr;
    }
    if (!Altimeter.isAverageFilterMode()) {
      fprintf(stderr, "WARNING: not average mode. the altitude is measured only by single shot data.\n");
      return nullptr;
    }
    if (!Altimeter.isFusionMode()) {
      fprintf(stderr, "WARNING: not fusion mode. the altitude caluculated only barometer.\n");
      return nullptr;
    }

    static uint32_t start_monitor_time_sec = millis() / 1000;
    static uint32_t last_update_sec = 0;

    while (true) {

      uint32_t current_sec = millis() / 1000;

      float pressure, temp, baro_altitude, fusion_altitude;
      Altimeter.altitude_estimation(
          &pressure, &temp, &baro_altitude, &fusion_altitude);

      /* push data */
      Altimeter.push_pressure(pressure);
      Altimeter.push_temp(temp);
      Altimeter.push_fusion_altitude(fusion_altitude);
      Altimeter.push_baro_altitude(baro_altitude);

      uint32_t update_duration_sec = current_sec - last_update_sec;
      if (update_duration_sec > Altimeter.getUpdateInterval()) {
        Altimeter.updateTemperature();
        Altimeter.updatePressure();
        Altimeter.updateAltitude();
        /* memory last update */
        last_update_sec = current_sec;
      }

      /* check timeout */
      uint32_t monitor_duration_sec = current_sec - start_monitor_time_sec;
      if ((Altimeter.getMonitorTimeout() > 0) && (monitor_duration_sec >= Altimeter.getMonitorTimeout())) {
        fprintf(stderr, "INFO: sensing finished %ld sec\n", monitor_duration_sec);
        break;
      }

      /* sleep to yeild computing resources to other process */
      usleep(Altimeter.getSensingInterval()*1000);
    }
    return nullptr;
  }

} // extern "C"


AltimeterClass::AltimeterClass() {
  m_interval_msec = DEFAULT_SENSING_INTERVAL_mSEC;
  m_update_sec = DEFAULT_UPDATE_INTERVAL_SEC;
  m_calibration_timeout_msec = 0;
  m_monitor_timeout_sec = 0;

  m_accel_sigma = DEFAULT_SIGMA_ACCEL_VALUE;
  m_gyro_sigma =  DEFAULT_SIGMA_GYRO_VALUE;
  m_baro_sigma =  DEFAULT_SIGMA_BARO_VALUE;
  m_ca = DEFAULT_CONST_ACCEL_VALUE;
  m_accel_threshold = DEFAULT_ACCEL_THRESH_VALUE;

  m_monitor_end = false;
  m_debug = false;
  m_calibrated = false;
  m_update = false;
  m_bmp581_begin = false;
  m_bmi270_begin = false;
  m_fusion_mode = true;
  m_average_filter = true;

  m_sea_level_pressure = SEALEVELPRESSURE_HPA;
  m_calib_th_conv = DEFAULT_CALIB_THRESH_CONV;
  m_calib_th_diff = DEFAULT_CALIB_THRESH_DIFF;

  m_fusion_altitude = 0.0;
  m_baro_altitude = 0.0;

  m_pressure_array = nullptr;
  m_temp_array = nullptr;
  m_fusion_altitude_array = nullptr;
  m_baro_altitude_array = nullptr;
  m_array_size = int(m_update_sec*1000/m_interval_msec) + 1;
}

AltimeterClass::~AltimeterClass() {
  if (m_pressure_array != nullptr) {
    free(m_pressure_array);
    m_pressure_array = nullptr;
  }
  if (m_temp_array != nullptr) {
    free(m_temp_array);
    m_temp_array = nullptr;
  }
  if (m_fusion_altitude_array != nullptr) {
    free(m_fusion_altitude_array);
    m_fusion_altitude_array = nullptr;
  }
  if (m_baro_altitude_array != nullptr) {
    free(m_baro_altitude_array);
    m_baro_altitude_array = nullptr;
  }
}

bool AltimeterClass::begin(uint32_t interval_msec, uint32_t update_sec
                          ,enum IIR_BANDWIDTH bw, bool average_filter) {
  m_interval_msec = interval_msec;
  m_update_sec = update_sec;

  // initialize BMI270 IMU Sensor
  if (!m_bmi270.begin()) {
    fprintf(stderr, "BMI270 failed to begin\n");
    while(1);
  }
  usleep(100000);
  if (m_debug) {
    printf("BMI270 is initialized\n");
    printf("- Accelerometer sample rate = %lf Hz\n", m_bmi270.accelerationSampleRate());
    printf("- Gyroscope     sample rate = %lf Hz\n", m_bmi270.gyroscopeSampleRate());
  }
  m_bmi270_begin = true;

  // initialize BMP581 Pressure Sensor
  uint8_t retry = 0;
  while (m_bmp581.beginI2C(BMP581_I2C_ADDRESS_DEFAULT) != BMP5_OK) {
    fprintf(stderr, "ERROR: BMP581 not connected, check wiring and I2C address!\n");
    delay(1000);
    if (retry++ == 10) {
      fprintf(stderr, "FAITAL ERROR: BMP581 not found or brokedn\n");
      panic_led_trap();
    }
  }
  m_bmp581_begin = true;
  if (m_debug) printf("BMP581 is initialized\n");

  /* The default IIR Filter cutt off is 0.3Hz */
  m_iir_bw = bw;
  setIIRFilter(m_iir_bw);
  if (m_debug) printf("IIR Filter is set: %d\n", bw);

  /* The average window is determined by m_update and m_interval */
  if (average_filter) {
    setAverageFilter(true);
    if (m_debug) printf("The average filter is set : array_size (%d)\n", m_array_size);
  }

  /* set parameters of altitude estimator */
  m_altitude.updateSigmaAccel(m_accel_sigma);
  m_altitude.updateSigmaGyro(m_accel_sigma);
  m_altitude.updateSigmaBaro(m_baro_sigma);
  m_altitude.updateCA(m_ca);
  m_altitude.updateZUPT(m_accel_threshold);
  if (m_debug) {
    printf("The altitude estimator is set : \n");
    printf("    Sigma Accel: %f\n", m_accel_sigma);
    printf("    Sigma Gyro : %f\n", m_gyro_sigma);
    printf("    Sigma Baro : %f\n", m_baro_sigma);
    printf(" Constant Accel: %f\n", m_ca);
    printf("           ZUPT: %f\n", m_accel_threshold);
  }

  return true;
}


void AltimeterClass::startCalibration(uint32_t timeout_ms) {
  if (timeout_ms > 0) {
    m_calibration_timeout_msec = timeout_ms;
  }
  pthread_create(&calib_thread, NULL, do_calibration_thread, NULL);
  if (m_debug) {
    if (timeout_ms == 0) {
      printf("start do_calibration_thread until convergence\n");
    } else {
      printf("start do_calibration_thread with timeout %ld msec\n", timeout_ms);
    }
  }
}

void AltimeterClass::startSensing(uint32_t duration_sec) {
  if (duration_sec > 0) {
    m_monitor_timeout_sec = duration_sec;
  }
  pthread_create(&monitor_thread, NULL, do_monitor_thread, NULL);
  if (m_debug) {
    if (duration_sec == 0) {
      printf("start do_monitor_thread until calling endSensing() \n");
    } else {
      printf("start do_monitor_thread with timeout %ld sec\n", duration_sec);
    }
  }
}


bool AltimeterClass::setIIRFilter(enum IIR_BANDWIDTH bw) {
  if (!m_bmp581_begin) {
    fprintf(stderr, "ERROR: BMP581 is not began!\n");
    return false;
  }
  bmp5_iir_config config;
  config.shdw_set_iir_t = BMP5_ENABLE;
  config.shdw_set_iir_p = BMP5_ENABLE;
  config.iir_flush_forced_en = BMP5_DISABLE;
  switch (bw) {
    case IIR_BANDWIDTH_00_3HZ:
      config.set_iir_t = BMP5_IIR_FILTER_COEFF_127;
      config.set_iir_p = BMP5_IIR_FILTER_COEFF_127;
      break;
    case IIR_BANDWIDTH_00_6HZ:
      config.set_iir_t = BMP5_IIR_FILTER_COEFF_63;
      config.set_iir_p = BMP5_IIR_FILTER_COEFF_63;
      break;
    case IIR_BANDWIDTH_01_2HZ:
      config.set_iir_t = BMP5_IIR_FILTER_COEFF_31;
      config.set_iir_p = BMP5_IIR_FILTER_COEFF_31;
      break;
    case IIR_BANDWIDTH_02_5HZ:
      config.set_iir_t = BMP5_IIR_FILTER_COEFF_15;
      config.set_iir_p = BMP5_IIR_FILTER_COEFF_15;
      break;
    case IIR_BANDWIDTH_05_1HZ:
      config.set_iir_t = BMP5_IIR_FILTER_COEFF_7;
      config.set_iir_p = BMP5_IIR_FILTER_COEFF_7;
      break;
    case IIR_BANDWIDTH_11_0HZ:
      config.set_iir_t = BMP5_IIR_FILTER_COEFF_3;
      config.set_iir_p = BMP5_IIR_FILTER_COEFF_3;
      break;
    case IIR_BANDWIDTH_27_5HZ:
      config.set_iir_t = BMP5_IIR_FILTER_COEFF_1;
      config.set_iir_p = BMP5_IIR_FILTER_COEFF_1;
      break;
    case IIR_BANDWIDTH_BYPASS:
      config.set_iir_t = BMP5_IIR_FILTER_BYPASS;
      config.set_iir_p = BMP5_IIR_FILTER_BYPASS;
      break;
    default:
      fprintf(stderr, "ERROR: invalid IIR_BANDWIDTH\n");
      return false;
  }

  int err = m_bmp581.setFilterConfig(&config);
  if (err != BMP5_OK) {
    // Setting coefficient failed, most likely an invalid coefficient (code -12)
    fprintf(stderr, "ERROR: IIR filter coefficient error! Error code: %d\n", err);
    return false;
  }
  return true;
}

bool AltimeterClass::setAverageFilter(bool enable) {
  if (m_pressure_array != nullptr) {
    free(m_pressure_array);
    m_pressure_array = nullptr;
  }
  if (m_temp_array != nullptr) {
    free(m_temp_array);
    m_temp_array = nullptr;
  }
  if (m_fusion_altitude_array != nullptr) {
    free(m_fusion_altitude_array);
    m_fusion_altitude_array = nullptr;
  }
  if (m_baro_altitude_array != nullptr) {
    free(m_baro_altitude_array);
    m_baro_altitude_array = nullptr;
  }

  if (enable) {
    m_array_size = (m_update_sec*1000/m_interval_msec) + 1;

    m_pressure_array = (float*)malloc(sizeof(float)*m_array_size);
    if (m_pressure_array == nullptr) {
      fprintf(stderr, "ERROR: no memory for pressure_array\n");
      return false;
    }
    memset(m_pressure_array, 0, sizeof(float)*m_array_size);

    m_temp_array = (float*)malloc(sizeof(float)*m_array_size);
    if (m_temp_array == nullptr) {
      fprintf(stderr, "ERROR: no memory for temp_array\n");
      return false;
    }
    memset(m_temp_array, 0, sizeof(float)*m_array_size);

    m_fusion_altitude_array = (float*)malloc(sizeof(float)*m_array_size);
    if (m_fusion_altitude_array == nullptr) {
      fprintf(stderr, "ERROR: no memory for fusion_altitude_array\n");
      return false;
    }
    memset(m_fusion_altitude_array, 0, sizeof(float)*m_array_size);

    m_baro_altitude_array = (float*)malloc(sizeof(float)*m_array_size);
    if (m_baro_altitude_array == nullptr) {
      fprintf(stderr, "ERROR: no memory for baro_altitude_array\n");
      return false;
    }
    memset(m_baro_altitude_array, 0, sizeof(float)*m_array_size);

    m_average_filter = true;
  } else {
    m_average_filter = false;
  }
  return true;
}


void AltimeterClass::updateTemperature() {
  if (m_average_filter) {
    float temp = 0.0;
    for (int n = 0; n < m_array_size; ++n) {
      temp += m_temp_array[n];
    }
    m_temp = temp / m_array_size;
  }
  m_update = true;
}

void AltimeterClass::updatePressure() {
  if (m_average_filter) {
    float pressure = 0.0;
    for (int n = 0; n < m_array_size; ++n) {
      pressure += m_pressure_array[n];
    }
    m_pressure = pressure / m_array_size;
  }
  m_update = true;
}

void AltimeterClass::updateAltitude() {
  if (m_average_filter) {
    float altitude = 0.0;
    for (int n = 0; n < m_array_size; ++n) {
      altitude += m_fusion_altitude_array[n];
    }
    m_fusion_altitude = altitude / m_array_size;

    altitude = 0.0;
    for (int n = 0; n < m_array_size; ++n) {
      altitude += m_baro_altitude_array[n];
    }
    m_baro_altitude = altitude / m_array_size;
  }
  m_update = true;
}

void AltimeterClass::push_pressure(float pressure) {
  static uint16_t counter = 0;
  if (m_average_filter) {
    m_pressure_array[counter++] = pressure;
    counter %= m_array_size;
  } else {
    m_pressure = pressure;
  }
}

void AltimeterClass::push_temp(float temp) {
  static uint16_t counter = 0;
  if (m_average_filter) {
    m_temp_array[counter++] = temp;
    counter %= m_array_size;
  } else {
    m_temp = temp;
  }
}

void AltimeterClass::push_fusion_altitude(float altitude) {
  static uint16_t counter = 0;
  if (m_average_filter) {
    m_fusion_altitude_array[counter++] = altitude;
    counter %= m_array_size;
  } else {
    m_fusion_altitude = altitude;
  }
}

void AltimeterClass::push_baro_altitude(float altitude) {
  static uint16_t counter = 0;
  if (m_average_filter) {
    m_baro_altitude_array[counter++] = altitude; 
    counter %= m_array_size;
  } else {
    m_baro_altitude = altitude;
  }
}

bool AltimeterClass::altitude_estimation(
       float *pressure, float *temp ,float *baro_altitude, float *fusion_altitude) 
{

  int err;

  // the interval time is better to be more than 20 msec 
  static uint32_t pastTime = millis();

  uint32_t currentTime = millis();
  if ((currentTime - pastTime) > m_interval_msec) { 
    // Get measurements from the sensor
    bmp5_sensor_data data = {0,0};
    err = m_bmp581.getSensorData(&data);
    if (err != BMP5_OK) {
      fprintf(stderr, "WARNING: Failed to perform reading imu (BMI270): err = %d\n", err);
      return false;
    }

    *pressure = data.pressure / 100.;
    *temp = data.temperature;
    *baro_altitude = calc_baro_altitude(*pressure);

    // get imu data
    uint32_t timestamp = micros();
    float accelData[3];
    float gyroData[3];
    imu_read(gyroData, accelData);

    // perform altitude estimation
    m_altitude.estimate(&accelData[0], &gyroData[0], *baro_altitude, timestamp);
    *fusion_altitude = m_altitude.getAltitude();
    // fprintf(stderr, "%f, %f, %f, %f\n", *pressure, *temp, *baro_altitude, *fusion_altitude);

    pastTime = currentTime;
  } 
  return true;
}


float AltimeterClass::calc_baro_altitude(float pressure) {
  float atmospheric = pressure;
  return 44330.0 * (1.0 - pow(atmospheric / SEALEVELPRESSURE_HPA, 0.1903));
}


bool AltimeterClass::imu_read(float gyro[3], float accel[3]) {
  // get IMU sensor data
  if (m_bmi270.accelerationAvailable() && m_bmi270.gyroscopeAvailable()) { 
    float ax, ay, az;
    float gx, gy, gz;
    m_bmi270.readAcceleration(ax, ay, az);
    m_bmi270.readGyroscope(gx, gy, gz);
    // Copy gyro values back out in rad/sec
    gyro[0] = gx * M_PI / 180.0f;
    gyro[1] = gy * M_PI / 180.0f;
    gyro[2] = gz * M_PI / 180.0f;
    // acceleration values in per G
    accel[0] = ax/G_CONSTANT;
    accel[1] = ay/G_CONSTANT;
    accel[2] = az/G_CONSTANT;
    return true;
  } else {
    fprintf(stderr, "ERROR: BMI270 cannot read\n");
  }
  return false;
}

void AltimeterClass::panic_led_trap() {
  while (true) {
    digitalWrite(LED0, HIGH);
    delay(100);
    digitalWrite(LED0, LOW);
    delay(100);
  }
}




