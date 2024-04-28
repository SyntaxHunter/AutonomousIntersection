# Imports
from threading import Thread, Event
from enum import Enum
import time
import board
import logging
import RPi.GPIO as GPIO

# for I2C use:
from adafruit_as726x import AS726x_I2C

from Motor import *
from Ultrasonic import *

# GPIO pins for infrared sensors
IR01 = 14
IR02 = 15
IR03 = 23

PWM = Motor()
ultrasonic = Ultrasonic()
servo = Servo()

# for I2C use:
i2c = board.I2C()  # uses board.SCL and board.SDA
sensor = AS726x_I2C(i2c)
sensor.conversion_mode = sensor.MODE_2
sensor.integration_Time = 20

# Setup GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(IR01, GPIO.IN)
GPIO.setup(IR02, GPIO.IN)
GPIO.setup(IR03, GPIO.IN)

MIN_DISTANCE = 20
MIDDLE_ANGLE = 80
ANGLE_DELTA = 50

obstacle_event = Event()
scan_event = Event()

color_detection_event = Event()
red_event = Event()
yellow_event = Event()

driving_event = Event()
color_event = Event()

class State(Enum):
    LINE_FOLLOWING = 0
    STOPPING = 1
    YIELDING = 2
    CROSSING = 3
    TERMINATE = 4
    WAITING = 5

class DrivingThread(Thread):
    
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, *, daemon=None):
        super().__init__(group=group, target=target, name=name, daemon=daemon)

    def run(self):
        curr_state = State.LINE_FOLLOWING
        color_detection_event.set()
        scan_event.clear()

        logging.debug("Drive control starting")

        while True:      
            self.LMR=0x00
            if GPIO.input(IR01)==True:
                self.LMR=(self.LMR | 4)
            if GPIO.input(IR02)==True:
                self.LMR=(self.LMR | 2)
            if GPIO.input(IR03)==True:
                self.LMR=(self.LMR | 1)

            if red_event.is_set():
                curr_state = State.STOPPING
                red_event.clear()
                color_detection_event.clear()
            elif yellow_event.is_set():
                curr_state = State.YIELDING
                yellow_event.clear()
                color_detection_event.clear()

            logging.debug("Current State = " + curr_state.name)

            match curr_state:
                case State.TERMINATE:
                    PWM.setMotorModel(0,0,0,0)
                case State.STOPPING:
                    PWM.setMotorModel(0,0,0,0)
                    curr_state = State.WAITING
                case State.WAITING:
                    logging.debug("Scanning for obstacles")
                    first = True
                    while(first or obstacle_event.is_set()):
                        first = False
                        obstacle_event.clear()

                        scan_event.set()
                        for i in range(MIDDLE_ANGLE,MIDDLE_ANGLE + ANGLE_DELTA, 1):    
                            servo.setServoPwm('4',i)
                            driving_event.wait(0.02)
                        for i in range(MIDDLE_ANGLE + ANGLE_DELTA, MIDDLE_ANGLE - ANGLE_DELTA, -1):
                            servo.setServoPwm('4',i)
                            driving_event.wait(0.02)
                        for i in range(MIDDLE_ANGLE - ANGLE_DELTA, MIDDLE_ANGLE, 1):
                            servo.setServoPwm('4',i)
                            driving_event.wait(0.02)
                        scan_event.clear()
                    
                    curr_state = State.CROSSING
                case State.YIELDING:
                    #TODO: CHeck intersection for cars
                    PWM.setMotorModel(0,0,0,0)
                    driving_event.wait(1)
                    curr_state = State.CROSSING
                case State.CROSSING:
                    PWM.setMotorModel(800,800,800,800)
                    if (self.LMR != 0):
                        curr_state = State.LINE_FOLLOWING
                        color_detection_event.set()
                case State.LINE_FOLLOWING:
                    # Line-following
                    if self.LMR==2:
                        PWM.setMotorModel(800,800,800,800)
                    elif self.LMR==4:
                        PWM.setMotorModel(-2000,-2000,3000,3000)
                    elif self.LMR==6:
                        PWM.setMotorModel(-4500,-4500,4000,4000)
                    elif self.LMR==1:
                        PWM.setMotorModel(3000,3000,-2000,-2000)
                    elif self.LMR==3:
                        PWM.setMotorModel(4000,4000,-4500,-4500)
                    elif self.LMR==7:
                        PWM.setMotorModel(0,0,0,0)
                    # elif self.LMR==0:
                    #     PWM.setMotorModel(0,0,0,0)

            driving_event.wait(0.01)

class UltrasonicThread(Thread):
    
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, *, daemon=None):
        super().__init__(group=group, target=target, name=name, daemon=daemon)

    def run(self):
        while True:
            if scan_event.is_set():
                # Read distance
                dist = ultrasonic.get_distance()
                # logging.debug("Object " + str(dist) + " cm away")

                # # Stop the car if it is 
                # if (dist > MIN_DISTANCE and obstacle_event.is_set()): 
                #     logging.debug("Stop event cleared with distance " + str(dist))
                #     obstacle_event.clear()

                if (dist < MIN_DISTANCE and dist != 0 and not obstacle_event.is_set()):
                    logging.debug("Obstacle detected at " + str(dist))
                    obstacle_event.set()

            time.sleep(0.1)

class IntersectionDetectionThread(Thread):

    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, *, daemon=None):
        super().__init__(group=group, target=target, name=name, daemon=daemon)

    def run(self):
        # Turn on LED
        sensor.driver_led = True
        
        while True:
            # Wait for data to be ready
            while not sensor.data_ready:
                time.sleep(0.01)

            v = sensor.violet
            b = sensor.blue
            g = sensor.green
            y = sensor.yellow
            o = sensor.orange
            r = sensor.red

            illuminance = v+b+g+y+o+r

            if color_detection_event.is_set():
                yellow_detection = y/v
                red_detection = (r*o)/(g*b)
                if (yellow_detection > 5.0 and not yellow_event.is_set()):
                    logging.debug("Detected yellow")
                    yellow_event.set()
                    logging.debug("Yellow/Violet = " + str(yellow_detection))
                    logging.debug("Red/Green = " + str(red_detection))
                    logging.debug("Illuminance = " + str(illuminance))
                elif (red_detection > 6.0 and not red_event.is_set()):
                    red_event.set()
                    logging.debug("Detected red")
                    logging.debug("Yellow/Violet = " + str(yellow_detection))
                    logging.debug("Red/Green = " + str(red_detection))
                    logging.debug("Illuminance = " + str(illuminance))
            
                logging.debug("Yellow/Violet = " + str(yellow_detection))
                logging.debug("Red/Green = " + str(red_detection))
                logging.debug("Illuminance = " + str(illuminance))

            color_event.wait(0.1)

logging.basicConfig(
    level=logging.DEBUG,
    format='(%(threadName)-10s) %(message)s',
)

if __name__ == "__main__":
    
    tDriving = DrivingThread(name="Driver")
    tUltrasonic = UltrasonicThread(name="Distance Sensor")
    tColorSensor = IntersectionDetectionThread(name="Intersection Detection")
    
    try:
        tDriving.start()
        tUltrasonic.start()
        tColorSensor.start()
    except KeyboardInterrupt:
        PWM.setMotorModel(0,0,0,0)

    # PWM.setMotorModel(4000,4000,-2000,-2000)
    # time.sleep(5)
    # PWM.setMotorModel(4000,4000,-4000,-4000)
    # time.sleep(5)
    # PWM.setMotorModel(4000,4000,0,0)
    # time.sleep(5)
    # PWM.setMotorModel(0,0,0,0)