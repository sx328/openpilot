#!/usr/bin/env python

import sys
import time
import spidev
import struct
from functools import reduce

DEBUG = True
VERBOSE = False

# Constants
SYNC = 0x5A
ACK = 0x79
NACK = 0x1F

def add_checksum(data):
  return data + bytes([reduce(lambda a, b: a ^ b, data)])

def xfer(data):
  if type(data) != list:
    if type(data) == bytes:
      data = list(data)
    else:
      data = [data]
  ret = spi.xfer(data[:])

  if DEBUG and VERBOSE:
    print(f"\t Out: {[hex(x) for x in data]} In: {[hex(x) for x in ret]}")

  return ret

def read(n):
  return xfer([0x00 for _ in range(n)])

def get_ack():
  if DEBUG:
    print("- Getting ACK")
  data = 0x00
  while not data in [ACK, NACK]:
    data = read(1)[0]
    time.sleep(0.001)
  acked = (data == ACK)
  xfer(ACK)
  if DEBUG:
    print("- ACK done")
  return acked

def start_comms():
  if DEBUG:
    print("Starting comms")
  xfer(SYNC)
  assert(get_ack())

def send_command(cmd, data=None, read_bytes=0):
  if DEBUG:
    print(f"Sending command: {hex(cmd)}")
  xfer(SYNC)
  ret = xfer([cmd, cmd ^ 0xFF])
  print('CMD ret:', [hex(x) for x in ret])
  assert(get_ack())

  if type(data) == list and len(data) > 0:
    for d in data:
      xfer(d)
      assert(get_ack())

  if read_bytes == 0:
    return
  
  ret = read(read_bytes + 1)[1:]
  assert(get_ack())
  return ret

def get_bootloader_version():
  return send_command(0x01, read_bytes=1)[0]

def get_id():
  ret = send_command(0x02, read_bytes=3)
  assert(ret[0] == 1)
  return ((ret[1] << 8) + ret[2])

def global_erase():
  print("Global erase...")
  send_command(0x44, data=[add_checksum(struct.pack('>H', 0xFFFF))])
  print("Erase done!")

def program_file(path, address):
  with open(path, 'rb') as f:
    code = f.read()

  i = 0
  while i < len(code):
    block = code[i:i+256]
    if len(block) < 256:
      block += b'\xFF' * (256 - len(block))

    send_command(0x31, data=[
      add_checksum(struct.pack('>I', address + i)),
      add_checksum(bytes([len(block) - 1]) + block),
    ])
    print(f"Written {len(block)} bytes to {hex(address + i)}")
    i += 256

BOOTSTUB_ADDRESS = 0x8000000
APP_ADDRESS = 0x8020000

if __name__ == "__main__":
  # Open SPI device
  spi = spidev.SpiDev()
  spi.open(0, 0)
  spi.max_speed_hz = 1000000

  # Connect
  start_comms()
  time.sleep(0.5)
  print(f"Bootloader version: {hex(get_bootloader_version())}")
  print(f"Chip ID: {hex(get_id())}")

  # Global erase (should do just the sectors we need in the future)
  global_erase()

  # Program bootstub
  program_file(sys.argv[1], BOOTSTUB_ADDRESS)

  # Program application
  program_file(sys.argv[2], APP_ADDRESS)  

  # Clean up
  spi.close()

