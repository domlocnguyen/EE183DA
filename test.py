import RPi.GPIO as GPIO
import time

# Port def
ARM_UP = 16
ARM_DOWN = 21
ARM_LEFT = 23
ARM_RIGHT = 24
ARM_FINGER_OUT = 17
ARM_FINGER_IN = 27
ARM_CLAMP_CLOSE = 25
ARM_CLAMP_OPEN = 26

# Setup pin mode
GPIO.setmode(GPIO.BCM)
GPIO.setup(ARM_UP,GPIO.OUT)
GPIO.setup(ARM_DOWN,GPIO.OUT)
GPIO.setup(ARM_LEFT,GPIO.OUT)
GPIO.setup(ARM_RIGHT,GPIO.OUT)
GPIO.setup(ARM_FINGER_OUT,GPIO.OUT)
GPIO.setup(ARM_FINGER_IN,GPIO.OUT)
GPIO.setup(ARM_CLAMP_CLOSE,GPIO.OUT)
GPIO.setup(ARM_CLAMP_OPEN,GPIO.OUT)


# Control function
def arm_up():
    GPIO.output(ARM_UP,GPIO.HIGH)
    GPIO.output(ARM_DOWN,GPIO.LOW)

def arm_down():
    GPIO.output(ARM_UP,GPIO.LOW)
    GPIO.output(ARM_DOWN,GPIO.HIGH)

def arm_left():
    GPIO.output(ARM_RIGHT,GPIO.LOW)
    GPIO.output(ARM_LEFT,GPIO.HIGH)

def arm_right():
    GPIO.output(ARM_LEFT,GPIO.LOW)
    GPIO.output(ARM_RIGHT,GPIO.HIGH)

def arm_finger_out():
    GPIO.output(ARM_FINGER_IN,GPIO.LOW)
    GPIO.output(ARM_FINGER_OUT,GPIO.HIGH)
    
def arm_finger_in():
    GPIO.output(ARM_FINGER_OUT,GPIO.LOW)
    GPIO.output(ARM_FINGER_IN,GPIO.HIGH)

def arm_clamp_open():
    GPIO.output(ARM_CLAMP_CLOSE,GPIO.LOW)
    GPIO.output(ARM_CLAMP_OPEN,GPIO.HIGH)
    
def arm_clamp_close():
    GPIO.output(ARM_CLAMP_OPEN,GPIO.LOW)
    GPIO.output(ARM_CLAMP_CLOSE,GPIO.HIGH)
    
def arm_stop():
    GPIO.output(ARM_UP,GPIO.LOW)
    GPIO.output(ARM_DOWN,GPIO.LOW)
    GPIO.output(ARM_RIGHT,GPIO.LOW)
    GPIO.output(ARM_LEFT,GPIO.LOW)
    GPIO.output(ARM_FINGER_IN,GPIO.LOW)
    GPIO.output(ARM_FINGER_OUT,GPIO.LOW)
    GPIO.output(ARM_CLAMP_OPEN,GPIO.LOW)
    GPIO.output(ARM_CLAMP_CLOSE,GPIO.LOW)



def init():
    arm_down()
    arm_left()
    arm_finger_in()
    arm_clamp_open()
    
def getBook():
    arm_right()
    time.sleep(13)
    arm_stop()
    arm_up()
    time.sleep(7.7)  #Max high = 9
    arm_stop()
    arm_finger_out()
    time.sleep(12)
    arm_stop()
    arm_down()
    time.sleep(3)
    arm_stop()
    arm_finger_in()
    time.sleep(12)
    arm_stop()
    arm_clamp_close()
    time.sleep(5)
    arm_stop()
    arm_up()
    time.sleep(2)
    arm_stop()
    arm_left()
    time.sleep(13)
    arm_stop()
    arm_down()
    time.sleep(12)
    arm_stop()
    
# Running ()
init()