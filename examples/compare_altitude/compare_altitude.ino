/*
 * compare_altitude.ino
 * Copyright (c) 2024 Yoshinori Oota
 *
 * This file is part of Altimeter
 *
 * Altimeter is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

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

