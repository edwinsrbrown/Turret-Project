
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

class Shifter:
  GPIO.setmode(GPIO.BCM)
  #Def for various pins
  def __init__(self, serialPin, latchPin, clockPin):
    self.serialPin = serialPin
    self.latchPin = latchPin
    self.clockPin = clockPin

    GPIO.setup(self.serialPin, GPIO.OUT)
    GPIO.setup(self.latchPin, GPIO.OUT, initial=0)
    GPIO.setup(self.clockPin, GPIO.OUT, initial=0)
  #ping method
  def ping(self, pin):
    GPIO.output(pin, 1)
    time.sleep(0)
    GPIO.output(pin, 0)
  #shift method
  def shiftByte(self, b):
    for i in range(8):
      GPIO.output(self.serialPin, b & (1 << i))
      self.ping(self.clockPin)
    self.ping(self.latchPin)

#only run if file is executed directly
if __name__=="__main__":
  #use class
  shift = Shifter(serialPin=23, latchPin=24, clockPin=25)

  try:
    while 1:
      for i in range(2**8):
        shift.shiftByte(i)
        time.sleep(0.5)
  except KeyboardInterrupt:
    GPIO.cleanup()
  






