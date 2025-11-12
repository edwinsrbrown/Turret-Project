# stepper_class_shiftregister_multiprocessing.py
#
# Stepper class
#
# Because only one motor action is allowed at a time, multithreading could be
# used instead of multiprocessing. However, the GIL makes the motor process run 
# too slowly on the Pi Zero, so multiprocessing is needed.

import time
import multiprocessing
# *** Q3 UPDATE 1: Import Value for shared angle tracking ***
from multiprocessing import Value 
from shifter import Shifter   # our custom Shifter class

class Stepper:
    """
    Supports operation of an arbitrary number of stepper motors using
    one or more shift registers.
  
    A class attribute (shifter_outputs) keeps track of all
    shift register output values for all motors.  In addition to
    simplifying sequential control of multiple motors, this schema also
    makes simultaneous operation of multiple motors possible.
   
    Motor instantiation sequence is inverted from the shift register outputs.
    For example, in the case of 2 motors, the 2nd motor must be connected
    with the first set of shift register outputs (Qa-Qd), and the 1st motor
    with the second set of outputs (Qe-Qh). This is because the MSB of
    the register is associated with Qa, and the LSB with Qh (look at the code
    to see why this makes sense).
 
    An instance attribute (shifter_bit_start) tracks the bit position
    in the shift register where the 4 control bits for each motor
    begin.
    """

    # Class attributes:
    num_steppers = 0      # track number of Steppers instantiated
    shifter_outputs = 0   # track shift register outputs for all motors
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001] # CCW sequence
    delay = 5000          # delay between motor steps [us]
    steps_per_degree = 4096/360    # 4096 steps/rev * 1/360 rev/deg

    def __init__(self, shifter, lock):
        self.s = shifter           # shift register
        # *** Q3 UPDATE 2: Use multiprocessing.Value for shared angle ***
        self.angle = Value('d', 0.0) # current output shaft angle (Multiprocessing Value)
        self.step_state = 0        # track position in sequence
        self.shifter_bit_start = 4*Stepper.num_steppers  # starting bit position
        self.lock = lock           # multiprocessing lock

        Stepper.num_steppers += 1   # increment the instance count

    # Signum function:
    def __sgn(self, x):
        if x == 0: return(0)
        else: return(int(abs(x)/x))

    # Move a single +/-1 step in the motor sequence:
    def __step(self, dir):
        self.step_state += dir    # increment/decrement the step
        self.step_state %= 8      # ensure result stays in [0,7]
        
        # *** Q2 UPDATE: Correct Bitwise Logic for Simultaneous Control ***
        new_pattern = Stepper.seq[self.step_state]
        mask = 0b1111 << self.shifter_bit_start 
        
        # 1. Clear the motor's 4 bits in the global outputs by ANDing with the inverse mask (~mask)
        # We also AND with 0xFF to ensure the mask doesn't extend infinitely in Python's large integers
        Stepper.shifter_outputs &= (~mask) & 0xFF 
        
        # 2. Set the new pattern by ORing with the new, shifted pattern
        Stepper.shifter_outputs |= new_pattern << self.shifter_bit_start
        
        # Send the entire byte to the shift register
        self.s.shiftByte(Stepper.shifter_outputs)
        
        # *** Q3 UPDATE 3: Access angle value with .value ***
        self.angle.value += dir/Stepper.steps_per_degree
        self.angle.value %= 360         # limit to [0,359.9+] range

    # Move relative angle from current position:
    def __rotate(self, delta):
        self.lock.acquire()                 # wait until the lock is available
        numSteps = int(Stepper.steps_per_degree * abs(delta))    # find the right # of steps
        dir = self.__sgn(delta)        # find the direction (+/-1)
        for s in range(numSteps):      # take the steps
            self.__step(dir)
            time.sleep(Stepper.delay/1e6)
        self.lock.release()

    # Move relative angle from current position (uses the standard __rotate functionality)
    def rotate(self, delta):
        time.sleep(0.1)
        p = multiprocessing.Process(target=self.__rotate, args=(delta,))
        p.start()

    # Move to an absolute angle taking the shortest possible path:
    # *** Q3 COMPLETION: Implement shortest path to absolute angle ***
    def goAngle(self, angle):
        # 1. Normalize target angle to [0, 360)
        angle %= 360 
        
        # 2. Calculate the shortest delta
        # Access the current angle value with .value
        current_angle = self.angle.value 
        
        # Calculate direct difference
        delta = angle - current_angle
        
        # Adjust for shortest path (wrap around 360 degrees)
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
            
        # 3. Launch rotation process with the calculated shortest delta
        p = multiprocessing.Process(target=self.__rotate, args=(delta,))
        p.start()

    # Set the motor zero point
    def zero(self):
        # *** Q3 UPDATE 4: Access and set angle value with .value ***
        self.angle.value = 0


# Example use:

if __name__ == '__main__':

    # NOTE: The serialPin argument name was fixed in the Shifter class example
    # but the Shifter class code provided (shifter.py) uses 'data', 'clock', 'latch'
    # Assuming 'data' is the serial input pin, 'clock' is clock, and 'latch' is latch.
    # The pins below match the example in shifter.py, NOT lab8.py's placeholder
    s = Shifter(data=16,clock=20,latch=21)   # set up Shifter 
    
    # Use multiprocessing.Lock() to prevent motors from trying to 
    # execute multiple operations at the same time:
    lock = multiprocessing.Lock()

    # Instantiate 2 Steppers:
    m1 = Stepper(s, lock)
    m2 = Stepper(s, lock)

    # Zero the motors (now uses the corrected zero method):
    m1.zero()
    m2.zero()

    # *** Test commands required for Lab 8 Question 4 ***
    print("Executing Question 4 Command Sequence...")
    m1.goAngle(90)
    m2.goAngle(-90) # Should run simultaneously with m1's first command
    m1.goAngle(-45) # Should wait for the first m1 command to finish
    m2.goAngle(45)  # Should wait for the first m2 command to finish
    m1.goAngle(-135)
    m1.goAngle(135)
    m1.goAngle(0)

    # While the motors are running in their separate processes, the main
    # code can continue doing its thing: 
    try:
        while True:
            # You can print current angles here if needed, e.g.:
            # print(f"M1 Angle: {m1.angle.value:6.2f} | M2 Angle: {m2.angle.value:6.2f}")
            time.sleep(1)
            pass
    except:
        print('\nend')
