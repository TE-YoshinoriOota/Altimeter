# check_uuid.py
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
import argparse
from bleak import BleakClient, BleakScanner


async def check_uuid(args: argparse.Namespace):

  if args.address:
    device = await BleakScanner.find_device_by_address(args.address)

    if device is None:
      print(f"Could not find device with address {args.address}")
      return
    else:
      print(f"Device {device.address} found")

  else:
    device = await BleakScanner.find_device_by_name(
          args.name, cb=dict(use_bdaddr=True)
      )
    if device is None:
      print(f"cound not find device with name {args.name}")
      return
    else:
      print(f"Device {device.name} found")

  async with BleakClient(device) as client:
    try:
      if client.is_connected:
        print(f"Already connected to {device.address}")
      else:
        print(f"Try to connect to {device.address} ({device.name})")
        await client.connect(timeout=10)
        if client.is_connected:
          print(f"connected to {device.address}")
        else:
          print(f"Failed to connect to {device.address} ({device.name})")
          return

      # confirmation to service uuid and chacateristic uuid
      services = client.services
      for service in services:
        print(f"Service: {service.uuid}")
        for char in service.characteristics:
          print(f"Characteristic: {char.uuid}, Properties: {char.properties}")
          if char and ("write" in char.properties or "write_without_response" in char.properties):
            print(f"Try to write {char.uuid}")
            data = b'\x23\x24\x25' 
            await client.write_gatt_char(char.uuid, data)
            print(f"Data written to {char.uuid}")
          else:
            print(f"Characteristic {char.uuid} is not writable or not found")

    except Exception as e:
      if "Insufficient Authentication" in str(e):
        print(f"Warning: Insufficient Authentication. Please ensure that {device.name} is properly paired") 
      else:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  device_group = parser.add_mutually_exclusive_group(required=True)

  device_group.add_argument(
      "--name",
      metavar="<name>",
      help="the name of the buletooth device to connect to",
    )

  device_group.add_argument(
      "--address",
      metavar="<address>",
      help="the address of the buletooth device to connect to",
    )


  args = parser.parse_args()
  asyncio.run(check_uuid(args))

