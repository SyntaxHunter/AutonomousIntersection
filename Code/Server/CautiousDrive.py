# Imports
import threading
import logging
from Motor import *
from Ultrasonic import *

ultrasonic = Ultrasonic()
ultrasonic_lock = threading.Lock()

MIN_DISTANCE = 20

class DrivingThread(threading.Thread):
    
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, *, daemon=None):
        super().__init__(group=group, target=target, name=name, daemon=daemon)
        self.PWM = Motor()

    def run(self):
        dist = 0
        speed = 0
        go = True

        while True:
            # Read distance
            ultrasonic_lock.acquire()
            dist = ultrasonic.get_distance()
            ultrasonic_lock.release()

            print()

            if (dist > MIN_DISTANCE):
                speed = 1000                    # Forward
            elif (dist < MIN_DISTANCE):
                speed = 0                       # Stop
                
            logging.debug("Distance =" + str(dist) + "cm\tSpeed =" + str(speed))
            PWM.setMotorModel(speed,speed,speed,speed)
            time.sleep(0.1)

logging.basicConfig(
    level=logging.DEBUG,
    format='(%(threadName)-10s) %(message)s',
)

if __name__ == "__main__":
    tDriving = DrivingThread(name="Driver")
    tDriving.start()