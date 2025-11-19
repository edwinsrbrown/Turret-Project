import RPi.GPIO as GPIO
import time

class Shifter:
    def __init__(self, data, latch, clock):
        self.data_pin = data
        self.latch_pin = latch
        self.clock_pin = clock
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.data_pin, GPIO.OUT)
        GPIO.setup(self.latch_pin, GPIO.OUT)
        GPIO.setup(self.clock_pin, GPIO.OUT)
        
        # Set initial states
        GPIO.output(self.latch_pin, GPIO.LOW)
        GPIO.output(self.clock_pin, GPIO.LOW)

    def shiftByte(self, value):
        """
        Shifts a byte (integer) out to the register.
        """
        GPIO.output(self.latch_pin, GPIO.LOW)
        
        # Shift out 8 bits, MSB first
        for i in range(7, -1, -1):
            GPIO.output(self.clock_pin, GPIO.LOW)
            
            bit = (value >> i) & 1
            GPIO.output(self.data_pin, bit)
            
            GPIO.output(self.clock_pin, GPIO.HIGH)
            
        GPIO.output(self.latch_pin, GPIO.HIGH)

    def cleanup(self):
        GPIO.cleanup()
