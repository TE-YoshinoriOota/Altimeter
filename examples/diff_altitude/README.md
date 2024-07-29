# README

This program package can obtain a precise height by calculating the differential altitude between the device (hand-carried) altimeter and the base altimeter. Since the base altimeter and the device (hand-carried) altimeter use BLE1507,  You need to install the [BLE1507 Arduino library](https://github.com/TE-YoshinoriOota/BLE1507_Arduino) first. 

<br/>

<!-- ![image](https://github.com/user-attachments/assets/1c7bcd55-1adb-4024-8c72-2d637f3400f3) -->

<br/>

## Contents

| Components                          | Desciption | 
| ----------------------------------- | ---------- | 
| ./base_altimeter                    | Spresense Arduino Sketch for the base altimeter   |
| ./device altimeter                  | Spresense Arduino Sketch for the device altimeter |
| ./extras/bleak/scan_ble_device.py   | Scanning BLE devices to get the MAC address and the device name |
| ./extras/bleak/paring_windows.py    | Pair the device with a Windows PC. Please note that this program can be used with Windows PCs |
| ./extras/bleak/check_uuid.py        | Check the UUID specified by the MAC address or the device name and test the connection    | 
| ./extras/bleak/altimeter_central.py | The Altimeter central program. Connect the base altimeter and the device altimeter and transfer the base altitude from the base altimeter to the device altimeter |
