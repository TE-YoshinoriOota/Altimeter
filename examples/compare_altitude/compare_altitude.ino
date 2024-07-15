#include <Altimeter.h>

void setup() {
  Serial.begin(115200);
  // Altimeter.setDebugMode(true);
  Altimeter.begin();
  Altimeter.startCalibration();

  Serial.println("Wait for calibrating the system...");
  while (!Altimeter.isCalibrated()) {
    sleep(1);
  }
  Serial.println("Calibrated");

  Altimeter.startSensing();  // montitor for 60 seconds
}

void loop() {
  static uint32_t g_counter = 0;
  if (Altimeter.isUpdate()) {
    float fusion_altitude = Altimeter.getFusionAltitude();
    float baro_altitude = Altimeter.getBaroAltitude();
    Serial.print(fusion_altitude);
    Serial.print(", ");
    Serial.print(baro_altitude);
    Serial.println();
  }

  if (g_counter++ == 100) {
    Altimeter.endSensing();
  }
  usleep(100000);
}

