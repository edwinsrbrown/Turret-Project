# stepper_class_shiftregister_multiprocessing.py
# Lab 8 Solution

import time
import multiprocessing
import ctypes
from shifter import Shifter

class Stepper:
    """
    Supports operation of an arbitrary number of stepper motors using
    one or more shift registers.
    """

    # Class attributes:
    num_steppers = 0      
    
    # FIX 1: Use multiprocessing.Value for shared memory across processes
    # 'i' = signed integer. Initialize to 0.
    shifter_outputs = multiprocessing.Value('i', 0)
    
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001] # CCW sequence
    delay = 1200          # delay between motor steps [us]
    steps_per_degree = 4096.0/360.0    

    def __init__(self, shifter, lock):
        self.s = shifter           
        
        # FIX 2: Use multiprocessing.Value for the angle so updates in the 
        # child process are reflected in the main process.
        # 'd' = double (float).
        self.angle = multiprocessing.Value('d', 0.0)
        
        self.step_state = 0        
        self.shifter_bit_start = 4 * Stepper.num_steppers 
        self.lock = lock           

        Stepper.num_steppers += 1

    # Signum function:
    def __sgn(self, x):
        if x == 0: return(0)
        else: return(int(abs(x)/x))

    # Move a single +/-1 step in the motor sequence:
    def __step(self, dir):
        self.step_state += dir    
        self.step_state %= 8      

        # FIX 3: Thread-safe Bitwise Operations
        with self.lock:
            # 1. Read current shared state
            current_val = Stepper.shifter_outputs.value
            
            # 2. Clear the 4 bits belonging to this motor
            # Create a mask of 1111 shifted to the correct position, then invert it
            mask = ~(0b1111 << self.shifter_bit_start)
            current_val &= mask
            
            # 3. Set the new 4 bits based on the sequence
            new_bits = Stepper.seq[self.step_state] << self.shifter_bit_start
            current_val |= new_bits
            
            # 4. Write back to shared memory and Hardware
            Stepper.shifter_outputs.value = current_val
            self.s.shiftByte(current_val)

        # Update the angle in shared memory
        # We use .value to access the underlying float
        self.angle.value = (self.angle.value + dir/Stepper.steps_per_degree) % 360

    # Worker function (runs in separate process)
    def __rotate(self, delta):
        # NOTE: The lock is handled inside __step now to allow interleaving 
        # of steps between different motors if needed, though holding it 
        # for the whole move is also valid if you want perfectly smooth motion
        # at the cost of blocking the other motor.
        # Given the requirements, finer locking in __step is usually preferred
        # so both motors can "step" effectively at the same time.
        
        numSteps = int(Stepper.steps_per_degree * abs(delta))
        dir = self.__sgn(delta)
        
        for s in range(numSteps):      
            self.__step(dir)
            time.sleep(Stepper.delay/1e6)

    # Move relative angle from current position:
    def rotate(self, delta):
        # Start the process
        p = multiprocessing.Process(target=self.__rotate, args=(delta,))
        p.start()
        # Return the process object so we can .join() it if we want to wait
        return p

    # Move to an absolute angle taking the shortest possible path:
    def goAngle(self, target_angle):
        # FIX 4: Shortest Path Logic
        
        # Get current angle from shared memory
        current = self.angle.value
        
        # Calculate raw difference
        delta = target_angle - current
        
        # Normalize to [-180, 180]
        # This math forces the delta to be the shortest route
        delta = (delta + 180) % 360 - 180

        # Delegate movement to rotate
        return self.rotate(delta)

    # Set the motor zero point
    def zero(self):
        self.angle.value = 0.0

# ==========================================
# MAIN EXECUTION BLOCK
# ==========================================

if __name__ == '__main__':
    
    # Initialize Hardware
    s = Shifter(data=16, latch=20, clock=21)
    lock = multiprocessing.Lock()

    # Instantiate Motors
    m1 = Stepper(s, lock)
    m2 = Stepper(s, lock)

    m1.zero()
    m2.zero()

    print("--- Starting Command Sequence ---")

    # Requirement: Execute commands in sequence, but simultaneous operation is allowed.
    # CRITICAL: Because 'rotate' is non-blocking (starts a process), we must
    # wait (join) before sending a NEW command to the SAME motor, otherwise
    # two processes will fight over Motor 1.
    
    # 1. m1 moves to 90
    p1 = m1.goAngle(90)
    p1.join() # Wait for m1 to finish before next command

    # 2. m1 moves to -45
    p1 = m1.goAngle(-45)
    p1.join()

    # 3. m2 moves to -90
    p2 = m2.goAngle(-90)
    p2.join()

    # 4. m2 moves to 45
    p2 = m2.goAngle(45)
    p2.join()

    # 5. Simultaneous Test: m1 to -135, m2 (implied) could move here if needed.
    # The lab asks for specific sequence:
    
    # m1.goAngle(-135)
    p1 = m1.goAngle(-135)
    p1.join()
    
    # m1.goAngle(135)
    p1 = m1.goAngle(135)
    p1.join()

    # m1.goAngle(0)
    p1 = m1.goAngle(0)
    p1.join()
    
    print("--- Sequence Complete ---")
    
    # DEMONSTRATION OF SIMULTANEOUS MOVEMENT (Requirement 2 & 4)
    # If you want to see them actually move together as requested in step 2:
    print("--- Demonstrating Simultaneous Movement ---")
    p1 = m1.rotate(360)
    p2 = m2.rotate(-360)
    
    # Now both are moving. We wait for both to finish.
    p1.join()
    p2.join()
    print("--- Demo Complete ---")
