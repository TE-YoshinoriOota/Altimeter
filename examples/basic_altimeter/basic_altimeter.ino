#include <Altimeter.h>

void setup() {
  Serial.begin(115200);
  Altimeter.setDebugMode(true);
  Altimeter.begin();
  Altimeter.startCalibration();

  Serial.println("Wait for calibrating the system...");
  while (!Altimeter.isCalibrated()) {
    sleep(1);
  }
  Serial.println("Calibrated");

  Altimeter.startSensing(60);  // montitor for 60 seconds
}

void loop() {
  if (Altimeter.isUpdate()) {
    float temp = Altimeter.getTemperature();
    float pressure = Altimeter.getPressure();
    float altitude = Altimeter.getAltitude();
    Serial.print(temp);
    Serial.print(", ");
    Serial.print(pressure);
    Serial.print(", ");
    Serial.print(altitude);
    Serial.println();
  }
  usleep(100000);
}
