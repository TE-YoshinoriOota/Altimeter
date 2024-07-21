# scan_ble_device.py
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
from bleak import BleakScanner

async def run():
  async with BleakScanner() as scanner:
    print("Scanning...")

    n = 5
    print(f"\n{n} advertisement packets:")
    async for bd, ad in scanner.advertisement_data():
      print(f" {n}. {bd!r} with {ad!r}")
      n -= 1
      if n == 0:
        break

    n = 10
    print(f"\nFind device with name longer than {n} characters...")
    async for bd, ad in scanner.advertisement_data():
      found = len(bd.name or "") > n or len(ad.local_name or "") > n
      print(f" Found{' it' if found else ''} {bd!r} with {ad!r}")
      if found:
        break

try:
  loop = asyncio.get_running_loop()
except RuntimeError:
  loop = asyncio.new_event_loop()
  asyncio.set_event_loop(loop)

loop.run_until_complete(run())
