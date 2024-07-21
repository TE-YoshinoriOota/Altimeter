# paring_windows.py
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

import argparse
import asyncio
import logging

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic


async def loop(args: argparse.Namespace):
  print("starting scan...")

  if args.address:
    device = await BleakScanner.find_device_by_address(
        args.address, cb=dict(use_bdaddr=True)
      )

    if device is None:
      print(f"could not find device with address {args.address}")
      return

  else:
    device = await BleakScanner.find_device_by_name(
        args.name, cb=dict(use_bdaddr=True)
      )

    if device is None:
      print(f"cound not find device with name {args.name}")
      return

  print("paring to device...")

  async with BleakClient(device) as client:
    paired = await client.pair()
    if paired:
      print(f"Successfully paired with {device.name} ({device.address})")
    else:
      print(f"Failed to pare with {device.name} ({device.address})")

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
  asyncio.run(loop(args))

