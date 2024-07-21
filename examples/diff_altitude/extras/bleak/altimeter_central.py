# altimeter_central.py
# MIT License
# 
# Copyright (c) 2024 Yoshinori Oota
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import asyncio
import time
import struct
import argparse
import concurrent.futures
from bleak import BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.exc import BleakError

char_uuid = "4a02"
last_height = 0
base_height = 128 

timeout = 10

#
# this callback function receives the altitude value that comes from the base altimeter
#
def notification_handler(characteristic: BleakGATTCharacteristic, data:bytearray):
  global base_height
  str_data = data.decode('utf-8')
  base_height = int(str_data)
  print(str_data)

#
# connect to the base altimeter and handling notifications
#
async def connect_and_start_notify(address):
  while True:
    try:
      async with BleakClient(address) as client:
        try:              
          if client.is_connected:
            print(f"Already connected to {address} as the base")
            # activate the notification_handler 
            await client.start_notify(char_uuid, notification_handler)
            while True:
              try: 
                await client.write_gatt_char(char_uuid, b'\x5a')
              except Exception as e:
                if "Insufficient Authentication" in str(e):
                  print(f"Warning: Insufficient Authentication. Please ensure that {device.name} is properly paired") 
                else:
                  print(f"An error occurred: {e}")
                  print(f"(re)try to connect {address}")
                  break;

              await asyncio.sleep(1)
          else:
            try:
              print(f"Try to connect {address} as the base")
              await asyncio.wait_for(client.connect(), timeout)
              if client.is_connected:
                print(f"Connected to {address} as the base")
              else:
                print(f"Failed to connect {address}")                        

            except asyncio.TimeoutError:
              print(f"Timout retry to connect {address}")

        except Exception as e:
          print(f"Connection lost with {address} : {e}")
          print(f"Warning: Please ensure that {address} is properly paired") 

        finally:
          try:
            await client.stop_notify(char_uuid)
          except BleakError as e:
            print(f"Stop notify error: {e}")

    except BleakError as e:
      print(f"BleakError: {e}")
      print("(re)try to connect to {address}")

    except Exception as e:
      print(f"Unexpected error: {e}")
      print("Terminate this process")
      break

    await asyncio.sleep(1)

#
# connect to the device altimeter and writing the base altitude to the device altimeter
#
async def connect_and_start_write(address):
  while True:
    try:
      async with BleakClient(address) as client:
        try:
          if client.is_connected:
            print(f"Already connected to {address} as the device")
            while True:
              try:
                # change the numeric value to the string value to send 
                # the data via the serial port protocol defined by the BLE device
                data_str = str(base_height)
                await client.write_gatt_char(char_uuid, data_str.encode('utf-8'))
              except Exception as e:
                if "Insufficient Authentication" in str(e):
                  print(f"Warning: Insufficient Authentication. Please ensure that {device.name} is properly paired") 
                else:
                  print(f"An error occurred: {e}")
                  print(f"(re)try to connect {address}")
                  break

              await asyncio.sleep(1)                    

          else:
            try:
              print(f"Try to connect {address} as the device")
              await asyncio.wait_for(client.connect(), timeout)
  
              if client.is_connected:
                print(f"Connected to {address} as the device")
              else:
                print(f"Failed to connect {address} as the device")                            
  
            except asyncio.TimeoutError:
              print(f"Timout retry to connect {address}")
  
        except Exception as e:
          print(f"Connection lost with {address} : {e}")
          print(f"Warning: Please ensure that {address} is properly paired") 
                                   
    except BleakError as e:
      print(f"BleakError: {e}")
      print("(re)try to connect to {address}")

    except Exception as e:
      print(f"Unexpected error: {e}")
      print("Terminate this process")
      break

    await asyncio.sleep(1)
 

async def main(address_base, address_device, uuid):
  global char_uuid
  char_uuid = uuid

  await asyncio.gather(
      connect_and_start_notify(address_base),
      connect_and_start_write(address_device),
    )


if __name__ == "__main__":
  parser = argparse.ArgumentParser()

  parser.add_argument("--address_base", type=str, required=True, help="the address of the base to connect to")
  parser.add_argument("--address_device", type=str, required=True, help="the address of the device to connect to")
  parser.add_argument("--uuid", type=str, default="4a02", help="the characteristic uuid for the altimeter devices")

  args = parser.parse_args()
  print(f"the base is {args.address_base}")
  print(f"the device is {args.address_device}")
  print(f"the characteristic uuid is {args.uuid}")

  asyncio.run(main(args.address_base, args.address_device, args.uuid))
