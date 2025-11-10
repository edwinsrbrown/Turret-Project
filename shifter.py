"""
Connor Chang
ENME441 - Lab 6
"""

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

serialPin, latchPin, clockPin = 23, 24, 25

class Shifter:
  def __init__(self, serialPin, latchPin, clockPin): #initialize the Shifter class with its attributes
    self.serialPin = serialPin
    self.latchPin = latchPin
    self.clockPin = clockPin
    
    GPIO.setup(serialPin, GPIO.OUT)
    GPIO.setup(latchPin, GPIO.OUT, initial=0)
    GPIO.setup(clockPin, GPIO.OUT, initial=0)

  def __ping(self, p):
    GPIO.output(p, 1)
    time.sleep(0)
    GPIO.output(p, 0)

  def shiftByte(self, b):
    for i in range(8):
      GPIO.output(self.serialPin, b & (1 << i))
      """
      creates 8 separate bytes, each with the binary number 1 shifted to the left by 1 index each run through the for loop
      for each byte, checks if both the b pattern and the shifted byte has a 1 at each index
      if the both have a 1, GPIO pin is TRUE (turns LED on), if not, pin is FALSE (keeps LED off)
      """
      self.__ping(self.clockPin)
    self.__ping(self.latchPin)

if __name__ == "__main__": #only run the following code when shifter.py is directly run (used to test code)
  shifter = Shifter(serialPin, latchPin, clockPin)
 
  try:
    while 1: 
      shifter.shiftByte(0b01010101)
  
  except KeyboardInterrupt:
    GPIO.cleanup()
