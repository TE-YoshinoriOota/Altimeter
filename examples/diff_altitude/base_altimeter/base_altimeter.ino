/*
 * base_altimeter.ino
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
#include <BLE1507.h>

#define PRINT_DEBUG

BLE1507 *ble1507;

/************************************************/
/*         UUID definition for BLE1507          */
/************************************************/
static BT_ADDR addr = {{0x19, 0x84, 0x06, 0x14, 0xAB, 0xCD}};
static char ble_name[BT_NAME_LEN] = "SPR-BASE-ALTIMETER";
#define UUID_SERVICE  0x3802
#define UUID_CHAR     0x4a02

/************************************************/
/*      base altitude measured by Altimeter     */
/************************************************/
float base_altitude = 0.0;

/************************************************/
/*        Callback function for BLE1507         */
/*   This callback is called when the central   */
/*       requires the base altitude data        */
/************************************************/
void BleCB(struct ble_gatt_char_s *ble_gatt_char) {
  if (ble_gatt_char->value.length == 0) return;
  uint8_t str_base_altitude[16] = {0};
  uint32_t base_altitude_in_cm = (uint32_t)(base_altitude*100);
  sprintf(str_base_altitude, "%08d", base_altitude_in_cm);
  ble1507->writeNotify(str_base_altitude, 8);
#ifdef PRINT_DEBUG
  Serial.println("BleCB: " + String((char*)ble_gatt_char->value.data));
#endif // PRINT_DEBUG    
}

void setup() {
  bool ret = false;

  Serial.begin(115200);

  /* setup BLE1507 */
  ble1507 = BLE1507::getInstance();
  ret = ble1507->begin(ble_name, addr, UUID_SERVICE, UUID_CHAR);
  if (!ret) {
    Serial.println("BLE1507 begin failed");
  }
  ble1507->setWriteCallback(BleCB);

  /* setup Altimeter */
  Altimeter.begin();

  /* start the calibration for BMI270 and BMP581 */
  Serial.println("Waiting for calibrating the system...");
  Altimeter.startCalibration();
  while (!Altimeter.isCalibrated()) {
    sleep(1);
  }
  Serial.println("Calibrated");

  /* monitor altitude for every 60 seconds (default) */
  Altimeter.startSensing();    
  sleep(1);
}

void loop() {
  static uint32_t g_counter = 0;
  if (Altimeter.isUpdate()) {
    base_altitude = Altimeter.getAltitude();
    Serial.print(base_altitude);
    Serial.println();
  }

  /* if the data is accumurated to 1000, ends sensing */
  if (g_counter++ == 1000) {
    Altimeter.endSensing();
  }

  /* need this usleep to yield the computer resources to the monitor thread */
  usleep(100000);
}


