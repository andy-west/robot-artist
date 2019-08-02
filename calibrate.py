import xbox
import time
import piplates.MOTORplate as MOTOR
import RPi.GPIO as GPIO

DIRECTION_0 = 0
DIRECTION_1 = 1

DEAD_ZONE = 30.0
INPUT_TO_SPEED_CONVERSION_FACTOR = 100.0

previousStart = False
previousJoyA = False

class Motor:
    number = -1
    sensorGpio = -1
    sensorValue = 0
    directions = ['cw', 'ccw']
    direction = DIRECTION_0
    position = 0
    isRunning = False

    def __init__(self, number, sensorGpio, directions):
        self.number = number
        self.sensorGpio = sensorGpio
        self.directions = directions

wristMotor = Motor(1, 16, ['ccw', 'cw'])
elbowMotor = Motor(2, 12, ['cw', 'ccw'])
shoulderMotor = Motor(3, 21, ['ccw', 'cw'])
baseMotor = Motor(4, 20, ['cw', 'ccw'])

def startMotor(motor, direction, speed):
    motor.direction = direction
    MOTOR.dcCONFIG(0, motor.number, motor.directions[direction], speed, 0.0)
    MOTOR.dcSTART(0, motor.number)
    motor.isRunning = True

def moveMotor(motor, direction, speed):
    if not motor.isRunning:
        startMotor(motor, direction, speed)
    else:
        if motor.direction == direction:
            MOTOR.dcSPEED(0, motor.number, speed)
        else:
            MOTOR.dcSTOP(0, motor.number)
            startMotor(motor, direction, speed)

def stopMotor(motor):
    if motor.isRunning:
        MOTOR.dcSTOP(0, motor.number)
        motor.isRunning = False

def updateMotor(motor, condition0, speed0, condition1, speed1):
    if condition0:
        moveMotor(motor, DIRECTION_0, speed0)
    elif condition1:
        moveMotor(motor, DIRECTION_1, speed1)
    else:
        stopMotor(motor)

def updateMotorPosition(motor):
    sensorValue = GPIO.input(motor.sensorGpio)

    if sensorValue != motor.sensorValue:
        motor.sensorValue = sensorValue
        positionChange = 1 if motor.direction == DIRECTION_0 else -1
        motor.position += positionChange

try:
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(wristMotor.sensorGpio, GPIO.IN)
    GPIO.setup(elbowMotor.sensorGpio, GPIO.IN)
    GPIO.setup(shoulderMotor.sensorGpio, GPIO.IN)
    GPIO.setup(baseMotor.sensorGpio, GPIO.IN)

    wristMotor.sensorValue = GPIO.input(wristMotor.sensorGpio)
    elbowMotor.sensorValue = GPIO.input(elbowMotor.sensorGpio)
    shoulderMotor.sensorValue = GPIO.input(shoulderMotor.sensorGpio)
    baseMotor.sensorValue = GPIO.input(baseMotor.sensorGpio)

    joy = xbox.Joystick()

    print('Ready for manual input.')

    while not joy.Back():
        if previousStart and not joy.Start():
            wristMotor.position = 0
            elbowMotor.position = 0
            shoulderMotor.position = 0
            baseMotor.position = 0

            print('Motor positions have been reset.')

        if previousJoyA and not joy.A():
            print('wrist motor: ', wristMotor.position)
            print('elbow motor: ', elbowMotor.position)
            print('shoulder motor: ', shoulderMotor.position)
            print('base motor: ', baseMotor.position)

        updateMotorPosition(wristMotor)
        updateMotorPosition(elbowMotor)
        updateMotorPosition(shoulderMotor)
        updateMotorPosition(baseMotor)

        leftX = joy.leftX() * INPUT_TO_SPEED_CONVERSION_FACTOR
        updateMotor(baseMotor, leftX < -DEAD_ZONE, -leftX, leftX > DEAD_ZONE, leftX)

        leftY = joy.leftY() * INPUT_TO_SPEED_CONVERSION_FACTOR
        updateMotor(shoulderMotor, leftY > DEAD_ZONE, leftY, leftY < -DEAD_ZONE, -leftY)

        rightY = joy.rightY() * INPUT_TO_SPEED_CONVERSION_FACTOR
        updateMotor(elbowMotor, rightY > DEAD_ZONE, rightY, rightY < -DEAD_ZONE, -rightY)

        leftTrigger = joy.leftTrigger() * INPUT_TO_SPEED_CONVERSION_FACTOR
        rightTrigger = joy.rightTrigger() * INPUT_TO_SPEED_CONVERSION_FACTOR
        updateMotor(wristMotor, rightTrigger > DEAD_ZONE, rightTrigger, leftTrigger > DEAD_ZONE, leftTrigger)

        previousStart = joy.Start()
        previousJoyA = joy.A()

        time.sleep(0.001)
finally:
    print('Shutting down.')

    joy.close()
    GPIO.cleanup()
