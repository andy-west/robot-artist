import xbox
import time
import piplates.MOTORplate as MOTOR
import RPi.GPIO as GPIO

# motor 1, sensor GPIO 16, "wrist", cw = down 
# motor 2, sensor GPIO 12, "elbow", cw = up
# motor 3, sensor GPIO 21, "shoulder", cw = down
# motor 4, sensor GPIO 20, "rotate base", cw = counter-clockwise

class Motor:
    number = -1
    sensorGpio = -1
    directions = ['cw', 'ccw']
    isRunning = False

    def __init__(self, number, sensorGpio, directions):
        self.number = number
        self.sensorGpio = sensorGpio
        self.directions = directions

DIRECTION_0 = 0
DIRECTION_1 = 1

DEAD_ZONE = 30.0
INPUT_TO_SPEED_CONVERSION_FACTOR = 100.0

wristMotor = Motor(1, 16, ['ccw', 'cw'])
elbowMotor = Motor(2, 12, ['cw', 'ccw'])
shoulderMotor = Motor(3, 21, ['ccw', 'cw'])
baseMotor = Motor(4, 20, ['cw', 'ccw'])

def moveMotor(motor, direction, speed):
    if not motor.isRunning:
        MOTOR.dcCONFIG(0, motor.number, motor.directions[direction], speed, 0.0)
        MOTOR.dcSTART(0, motor.number)
        motor.isRunning = True
    else:
        MOTOR.dcSPEED(0, motor.number, speed)

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

try:
    #GPIO.setmode(GPIO.BCM)
    #GPIO.setup(sensorGpio, GPIO.IN)

    joy = xbox.Joystick()

    print('Ready for manual input.')

    while not joy.Back():
        #sensorValue = GPIO.input(sensorGpio)
        #print(sensorValue)

        leftX = joy.leftX() * INPUT_TO_SPEED_CONVERSION_FACTOR
        updateMotor(baseMotor, leftX < -DEAD_ZONE, -leftX, leftX > DEAD_ZONE, leftX)

        leftY = joy.leftY() * INPUT_TO_SPEED_CONVERSION_FACTOR
        updateMotor(shoulderMotor, leftY > DEAD_ZONE, leftY, leftY < -DEAD_ZONE, -leftY)

        rightY = joy.rightY() * INPUT_TO_SPEED_CONVERSION_FACTOR
        updateMotor(elbowMotor, rightY > DEAD_ZONE, rightY, rightY < -DEAD_ZONE, -rightY)

        leftTrigger = joy.leftTrigger() * INPUT_TO_SPEED_CONVERSION_FACTOR
        rightTrigger = joy.rightTrigger() * INPUT_TO_SPEED_CONVERSION_FACTOR
        updateMotor(wristMotor, rightTrigger > DEAD_ZONE, rightTrigger, leftTrigger > DEAD_ZONE, leftTrigger)

        time.sleep(0.001)
finally:
    print('Shutting down.')

    joy.close()
    #GPIO.cleanup()
