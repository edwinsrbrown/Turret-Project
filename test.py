import time
import multiprocessing
from shifter_test import Shifter

class Stepper:
    """
    Supports operation of multiple stepper motors using a shared shift register.
    Uses Multiprocessing for smooth timing, but shared memory to track state.
    """

    # Class attributes:
    num_steppers = 0
    # Shared memory integer for the Shift Register state (all motors combined)
    shifter_outputs = multiprocessing.Value('i', 0)
    
    # Stepper Sequence (Half-stepping or Full-stepping depending on preference)
    # Current: Half-stepping sequence
    seq = [0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001]
    
    delay = 1200          # delay between motor steps [us]
    steps_per_degree = 4096.0 / 360.0

    def __init__(self, shifter, lock):
        self.s = shifter           
        # FIX 1: Angle must be shared memory ('d' = double/float) so processes 
        # can update the "real" angle.
        self.angle = multiprocessing.Value('d', 0.0)             
        self.step_state = 0        
        self.shifter_bit_start = 4 * Stepper.num_steppers
        self.lock = lock           

        Stepper.num_steppers += 1

    def __sgn(self, x):
        if x == 0: return 0
        else: return int(abs(x)/x)

    def __step(self, dir):
        """
        Moves the motor one physical step.
        Protected by lock because it writes to hardware.
        """
        self.step_state += dir
        self.step_state %= 8 
        
        # Critical Section: Writing to the Shift Register
        with self.lock:
            sep = Stepper.shifter_outputs.value
            # Clear the 4 bits for this specific motor
            sep &= ~(0b1111 << self.shifter_bit_start)
            # Set the new 4 bits based on the sequence
            sep |= (Stepper.seq[self.step_state] << self.shifter_bit_start)
            
            # Write back to shared memory and hardware
            Stepper.shifter_outputs.value = sep
            self.s.shiftByte(sep)

        # Update the Angle in Shared Memory
        # We use get_lock() to ensure we don't have a race condition on the angle math
        with self.angle.get_lock():
            self.angle.value = (self.angle.value + dir / Stepper.steps_per_degree) % 360

    def rotate(self, delta):
        """
        Rotates the motor by delta degrees.
        This is now a BLOCKING function. It does not return until the move is done.
        """
        numSteps = int(Stepper.steps_per_degree * abs(delta))
        direction = self.__sgn(delta)
        
        for _ in range(numSteps):
            self.__step(direction)
            # sleep expects seconds, delay is in microseconds
            time.sleep(Stepper.delay / 1e6)

    def goAngle(self, target_angle):
        """
        Calculates shortest path to target_angle and rotates there.
        """
        target_angle %= 360
        
        # Read the current angle from shared memory
        current = self.angle.value
        
        delta = target_angle - current

        # Shortest path logic
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360

        self.rotate(delta)

    def zero(self):
        with self.angle.get_lock():
            self.angle.value = 0.0

# ==========================================
# MAIN EXECUTION BLOCK
# ==========================================

def run_motor_sequence(motor, angle_list):
    """
    Helper function to run a specific motor through a list of angles.
    Used as a target for multiprocessing.
    """
    for ang in angle_list:
        motor.goAngle(ang)
        time.sleep(0.5) # Pause between moves

if __name__ == '__main__':
    # Initialize Hardware
    # Adjust pins (data, latch, clock) for your specific Pi setup
    s = Shifter(data=16, latch=20, clock=21)
    lock = multiprocessing.Lock()

    # Instantiate Motors
    m1 = Stepper(s, lock)
    m2 = Stepper(s, lock)

    m1.zero()
    m2.zero()

    print("--- TEST 1: Sequential Moves (Main Thread) ---")
    # Because rotate/goAngle are blocking, these run one after another
    # This is safer for simple scripts.
    print("Moving M1 to 90...")
    m1.goAngle(90)
    print("Moving M1 to 0...")
    m1.goAngle(0)
    
    print("\n--- TEST 2: Parallel Moves (Multiprocessing) ---")
    # To move both at the same time, we create processes here.
    
    # Define what M1 will do
    p1 = multiprocessing.Process(target=run_motor_sequence, args=(m1, [90, 180, 0]))
    
    # Define what M2 will do
    p2 = multiprocessing.Process(target=run_motor_sequence, args=(m2, [-90, -180, 0]))
    
    # Start them both
    p1.start()
    p2.start()
    
    # Wait for them to finish
    p1.join()
    p2.join()
    
    print("Done.")
    
    # Cleanup GPIO (via the shifter class if implemented, or manually)
    try:
        s.cleanup()
    except:
        pass
