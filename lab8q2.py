# stepper_class_shiftregister_multiprocessing.py
#
# Stepper class (shared shift register, parallel multiprocessing operation)

import time
import multiprocessing
from shifter import Shifter   # custom Shifter class

class Stepper:
    """
    Supports operation of multiple stepper motors using one shift register.
    Each motor updates only its own 4 control bits (Qa–Qd, Qe–Qh, etc.).
    """

    num_steppers = 0
    shifter_outputs = multiprocessing.Value('i', 0)  # shared integer across processes
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001]  # CCW sequence
    delay = 1200  # microseconds between steps
    steps_per_degree = 4096 / 360  # steps per degree

    def __init__(self, shifter, lock):
        self.s = shifter
        self.angle = 0
        self.step_state = 0
        self.shifter_bit_start = 4 * Stepper.num_steppers  # motor bit offset
        self.lock = lock  # shared among all motors (for safe shifting)
        Stepper.num_steppers += 1

    # sign function
    def __sgn(self, x):
        return 0 if x == 0 else int(abs(x) / x)

    # single step (+1 or -1)
    def __step(self, direction):
        self.step_state = (self.step_state + direction) % 8

        with self.lock:  # protect access to shared shifter outputs
            # Read, clear this motor's bits, then set its new ones
            val = Stepper.shifter_outputs.value
            val &= ~(0b1111 << self.shifter_bit_start)
            val |= (Stepper.seq[self.step_state] << self.shifter_bit_start)
            Stepper.shifter_outputs.value = val
            self.s.shiftByte(val)

        # update angle
        self.angle = (self.angle + direction / Stepper.steps_per_degree) % 360

    # internal rotate routine (run in a separate process)
    def __rotate(self, delta):
        numSteps = int(Stepper.steps_per_degree * abs(delta))
        direction = self.__sgn(delta)
        for _ in range(numSteps):
            self.__step(direction)
            time.sleep(Stepper.delay / 1e6)

    # public rotate (spawns a new process)
    def rotate(self, delta):
        p = multiprocessing.Process(target=self.__rotate, args=(delta,))
        p.start()

    # move to an absolute angle via shortest path
    def goAngle(self, target_angle):
        # normalize angles
        target_angle %= 360
        current = self.angle
        delta = target_angle - current

        # choose shortest path (e.g., -350° → +10°)
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360

        self.rotate(delta)

    # set current position as zero
    def zero(self):
        self.angle = 0


# Example use
if __name__ == '__main__':
    s = Shifter(data=16, latch=20, clock=21)
    lock = multiprocessing.Lock()  # shared lock (safe write to shift register)

    # two steppers sharing one shift register
    m1 = Stepper(s, lock)  # uses bits Qe–Qh
    m2 = Stepper(s, lock)  # uses bits Qa–Qd

    m1.zero()
    m2.zero()

    # both motors move simultaneously
    print("Rotating both motors...")
    m1.goAngle(90)
    m2.goAngle(180)

    # main loop continues while motors run
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nend")
