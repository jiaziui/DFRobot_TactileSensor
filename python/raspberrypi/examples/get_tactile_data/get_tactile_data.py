# -*- coding: utf-8 -*
'''!
@file get_tactile_data.py
@brief This example demonstrates how to get the tactile data of the DFRobot Tactile Sensor.
@copyright Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
@license The MIT License (MIT)
@author [JiaLi](zhixinliu@dfrobot.com)
@version V1.0
@date 2025-09-04
@url https://github.com/cdjq/DFRobot_TactileSensor
'''

from __future__ import print_function
import sys

sys.path.append("../../")
import time

from DFRobot_Tactile_Sensor import *

# device address
address = 1

# array size
# 6*6 -> 36
# 4*8 -> 32
array = 32

if array == 32:
  array_x = 8
  array_y = 4
else:
  array_x = 6
  array_y = 6

tactile = DFRobot_Tactile_Sensor(array, address, 115200)


def setup():
  while tactile.begin() == False:
    print("Sensor initialize failed!!")
    time.sleep(1)
  print("Sensor  initialize success!!")

  ver = version_info()
  # Get device information
  ver = tactile.get_device_info()
  print(" VID :", ver.VID, end='\n')
  print(" PID :", ver.PID, end='\n')
  print(" Version: V{:X}.{:X}.{:X}.{:X}".format((ver.version >> 12) & 0xF, (ver.version >> 8) & 0xF, (ver.version >> 4) & 0xF, ver.version & 0xF))
  time.sleep(1)

  # Set threshold value
  tactile.set_thld(50)
  time.sleep(0.1)

  # The sensor returns to its default value,Addr doesn't change
  # tactile.reset_val()
  # time.sleep(0.1)

  # #Set device address
  # tactile.set_dev_addr(1)
  # time.sleep(0.1)

  # #set uart baudrate ，default is 115200
  # tactile.set_baudrate(tactile.BAUD_9600)
  # tactile.set_baudrate(tactile.BAUD_115200)
  # time.sleep(0.1)

  # #set sample rate
  tactile.set_sample_rate(tactile.SAMPLE_RATE_20HZ)
  # tactile.set_sample_rate(tactile.SAMPLE_RATE_50HZ)
  time.sleep(0.1)


def loop():
  adc_datas = adc_data()
  # Get tactile data
  adc_datas = tactile.get_datas()
  # Print tactile data
  for i in range(array_y):
    for j in range(array_x):
      print(adc_datas.adcval[i][j], end='\t')
    print('\n')
  print('\n')

  time.sleep(0.1)


if __name__ == "__main__":
  setup()
  while True:
    loop()
