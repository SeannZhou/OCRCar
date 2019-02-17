
from subprocess import Popen, PIPE
import json
import threading
import time
import picamera
import RPi.GPIO as GPIO
import serial
#from gpiozero import DistanceSensor


ser = serial.Serial("/dev/ttyACM0",9600)
ser.baudrate=9600
GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)
p = GPIO.PWM(7, 50)
p2 = GPIO.PWM(11, 50)
p.start(7.5)
p2.start(7.5)
current_speed = 5
#while True:
#    print(ser.readline().decode('utf-8'))

def moveForward(seconds):
    try:
        # while True:
        current_time = 0
        while current_time < seconds:
            p.ChangeDutyCycle(5)  # turn towards 90 degree
            p2.ChangeDutyCycle(10)
            time.sleep(1)  # sleep 1 second
            current_time = current_time + 1

    except KeyboardInterrupt:
        p.stop()
        p2.stop()
        GPIO.cleanup()


def moveBackward(seconds):
    try:
        current_time = 0
        while current_time < seconds:
            p.ChangeDutyCycle(10)  # turn towards 90 degree
            p2.ChangeDutyCycle(5)
            time.sleep(1)  # sleep 1 second
            current_time = current_time + 1

    except KeyboardInterrupt:
        p.stop()
        p2.stop()
        GPIO.cleanup()


def stop(seconds):
    p.ChangeDutyCycle(0)
    p2.ChangeDutyCycle(0)
    time.sleep(seconds)

#def increaseSpeed(current_speed):
#    if not current_speed == 5:
#        counter = 0;
#        while()
#        current_speed = current_speed + 1
#        p.ChangeDutyCycle(current_speed)  # turn towards 90 degree
#        p2.ChangeDutyCycle(15 - current_speed)
#        time.sleep(1)  # sleep 1 second


def change_speed(old_speed,seconds):
    print("Changing speed from " + str(old_speed) + " to " + str(current_speed))
    p.ChangeDutyCycle(current_speed)
    p2.ChangeDutyCycle(15-current_speed)
    time.sleep(seconds)

def decreaseSpeed(current_speed):
    if not current_speed == 0:
        current_speed = current_speed - 1
        if current_speed == 0:
            stop(5)
        else:
            p.ChangeDutyCycle(current_speed)  # turn towards 90 degree
            p2.ChangeDutyCycle(15 - current_speed)
            time.sleep(1)  # sleep 1 second

def turnRight():
    for i in range(4):
        p.ChangeDutyCycle(7)
        p2.ChangeDutyCycle(11)
        time.sleep(1)

def turnLeft():
    for i in range(4):
        p.ChangeDutyCycle(5)
        p2.ChangeDutyCycle(10)
        time.sleep(1)
#def detectCollision(active):
#     if active == True:
#         while active == True:
#
#             front_sonic = DistanceSensor(echo=, trigger=, threshold_distance=.2)
#             right_sonic = DistanceSensor(echo=, trigger=, threshold_distance=.2)
#             left_sonic = DistanceSensor(echo=, trigger=, threshold_distance=.2)

#             if front_sonic.distance < front_sonic.threshold_distance:
#                 stop(3)
#             elif left_sonic.distance < left_sonic.threshold_distance:
#                 moveBackward(2)
#             elif right_sonic.distance < right_sonic.threshold_distance:
#                 moveBackward(2)

#     else:
#         time.sleep(1)

def call_instruction(instruction_list):
    global current_speed
    speed_list = [10,20,30,40]
    speed_conv = {}
    speed_conv["10"] = 7
    speed_conv["40"] = 5
    if(instruction_list[0] == "STOP"):
        stop(3)
        moveForward(1)
    elif (instruction_list[0] == "DEAD"):
        moveBackward(2)
        turnRight()
        turnRight()
    elif instruction_list[0] == "SPEED" and int(instruction_list[1]) in speed_list:
        if current_speed == int(instruction_list[1]):
            pass
        elif current_speed != int(instruction_list[1]):
            old_speed = current_speed
            current_speed = speed_conv[instruction_list[1]]
            change_speed(old_speed, 5)
    elif instruction_list[0] == "TURN":
        if instruction_list[1] == "RIGHT":
            turnRight()
            moveForward(1)
        else:
            turnLeft()
            moveForward(1)

def process_road():
    interrupt = True
    camera = picamera.PiCamera()
    camera.resolution = (640,640)
    time.sleep(1)
    print("Camera Initialized")
    moveForward(0.5)
    #with picamera.PiCamera() as camera:
        #camera.resolution = (640,640)
        #time.sleep(2)
    while(interrupt):
        process_image(camera)
        time.sleep(0.5)
    camera.close()

# Take image with raspberry pi camera
# Send request to google cloud API for OCR
# Parse JSON for text in image
def process_image(camera):
#    threading.Timer(2, process_image).start()
    #process = Popen(['raspistill','-w','800','-h','640','-q','100','-o', 'images/road.jpg'])
    camera.capture('images/road.jpg')
    process = Popen(['gcloud', 'ml', 'vision', 'detect-text', '/home/pi/Documents/mlcar/images/road.jpg'], stdout=PIPE)
    time.sleep(3)
    p.ChangeDutyCycle(0)
    p2.ChangeDutyCycle(0)
    output = process.communicate()
    loaded_json = json.loads(output[0].decode('utf-8'))
    if(loaded_json.get("responses")[0] == {}):
        print("Empty")
        if(current_speed == 7):
            change_speed(7, 5)
        else:
            moveForward(1)
    else:
        instruction = loaded_json.get("responses")[0].get("fullTextAnnotation").get("text")
        print(instruction)
        instruction = instruction.split("\n")
        call_instruction(instruction)
    #Call their function
    #print(instruction)

try:
    #detectCollision(True)
    process_road()
finally:
    p.stop()
    p2.stop()
    GPIO.cleanup()
