import time
import multiprocessing
from RPi import GPIO

GPIO.setmode(GPIO.BCM)


class Stepper:
    # Class attributes:
    seq = [0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001]  # CCW sequence
    stepsPerDegree = 4096 / 360  # 4096 steps/rev * 1/360 rev/deg

    def __init__(self, pins, lock, delay=1200):
        self.delay = delay  # delay between motor steps [us]
        self.pins = pins  # motor drive pins (4-element list)
        self.angle = multiprocessing.Value('d', 0.0)  # current output shaft angle, shared across processes
        self.seq_state = 0  # track position in sequence
        self.lock = lock  # multiprocessing lock

        for p in self.pins:
            GPIO.setup(p, GPIO.OUT)

    # Signum function:
    def __sgn(self, x):
        return 0 if x == 0 else int(abs(x) / x)

    # Move a single +/-1 step in the motor sequence:
    def __step(self, dir):
        seq = Stepper.seq[self.seq_state]
        self.seq_state = (self.seq_state + dir) % 8  # increment/decrement the step and wrap within [0,7]
        for idx in range(4):
            GPIO.output(self.pins[idx], seq & (1 << idx))

        # If code doesn't work, changed the 'with' code below:
        with self.angle.get_lock():  # ensure angle is updated safely
            self.angle.value = (self.angle.value + dir / Stepper.stepsPerDegree) % 360

    # Move relative angle from current position:
    def __rotate(self, delta, lock):
        lock.acquire()  # wait until the lock is available
        numSteps = int(Stepper.stepsPerDegree * abs(delta))  # find the right # of steps
        dir = self.__sgn(delta)  # find the direction (+/-1)
        for s in range(numSteps):  # take the steps
            self.__step(dir)
            time.sleep(self.delay / 1e6)
        lock.release()

    # Move relative angle from current position:
    def rotate(self, delta):
        time.sleep(0.1)
        p = multiprocessing.Process(target=self.__rotate, args=(delta, self.lock))
        p.start()

    def getAngle(self):
        return self.angle.value

    # Move to an absolute angle taking the shortest possible path:
    def goAngle(self, target_angle):
        with self.angle.get_lock():  # acquire lock for safe access to shared angle
            current_angle = self.angle.value

            # Calculate the shortest path delta
            delta = (target_angle - current_angle) % 360
            if delta > 180:
                delta -= 360  # go the shorter way

            self.angle.value = target_angle

            # Start rotation
            self.rotate(delta)

    # Set the motor zero point
    def zero(self):
        with self.angle.get_lock():  # acquire lock for safe access to shared angle
            self.angle.value = 0.0
