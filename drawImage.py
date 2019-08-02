import math
import time
import piplates.MOTORplate as MOTOR
import RPi.GPIO as GPIO

# ----------------------------------------
# Constants
# ----------------------------------------

MOTOR_SPEED = 100.0

BASE_OFFSET = 4.0
GRIPPER_OFFSET = 90.0

UPPER_ARM_LENGTH = 91.3
FOREARM_LENGTH = 113.3

DIRECTION_NOT_SET = -1
DIRECTION_NEGATIVE = 0
DIRECTION_POSITIVE = 1

# ----------------------------------------
# Classes
# ----------------------------------------

class Motor:
    previousDirection = DIRECTION_NOT_SET
    direction = DIRECTION_NOT_SET
    position = 0
    targetPosition = 0
    isRunning = False

    def __init__(self, number, sensorGpio, directions, backlashCompensation, unitsPerAngle):
        self.number = number
        self.sensorGpio = sensorGpio
        self.directions = directions
        self.backlashCompensation = backlashCompensation
        self.unitsPerAngle = unitsPerAngle

        GPIO.setup(sensorGpio, GPIO.IN)
        sensorValue = GPIO.input(sensorGpio)
        self.previousSensorValue = sensorValue
        self.sensorValue = sensorValue

    def updateTargetPosition(self, angle):
        self.targetPosition = int(angle * self.unitsPerAngle)

    def setDirection(self):
        self.previousDirection = self.direction
        self.direction = DIRECTION_NEGATIVE if (self.targetPosition - self.position) < 0 else DIRECTION_POSITIVE

    def compensateForBacklash(self):
        if (self.direction != self.previousDirection
            and self.previousDirection != DIRECTION_NOT_SET
            and self.backlashCompensation != 0):
            self.targetPosition += self.backlashCompensation * (1 if self.direction == DIRECTION_POSITIVE else -1)

    def configure(self):
        direction = self.directions[self.direction]
        MOTOR.dcCONFIG(0, self.number, direction, MOTOR_SPEED, 0.0)

    def start(self):
        if not self.isRunning:
            MOTOR.dcSTART(0, self.number)
            self.isRunning = True

    def stop(self):
        if self.isRunning:
            MOTOR.dcSTOP(0, self.number)
            self.isRunning = False

class Point2d:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def getDistanceToOrigin(self):
        distance = math.sqrt((self.x * self.x) + (self.y * self.y))
        return distance

    def getAngleToOrigin(self):
        angleInRadians = math.atan2(self.x, -self.y)
        angleInDegrees = math.degrees(angleInRadians)
        return angleInDegrees

# ----------------------------------------
# Variables
# ----------------------------------------

baseMotor = Motor(4, 20, ['cw', 'ccw'], 10, 4.39)
shoulderMotor = Motor(3, 21, ['ccw', 'cw'], 0, 4.21)
elbowMotor = Motor(2, 12, ['cw', 'ccw'], 0, 4.21)
wristMotor = Motor(1, 16, ['cw', 'ccw'], 0, 4.84)

motors = [baseMotor, shoulderMotor, elbowMotor, wristMotor]

points = [
    #Point2d(-60, -200),
    #Point2d(30, -160)

    # hello-world

    Point2d(5, 22),
    Point2d(7, 18),
    Point2d(8, 14),
    Point2d(9, 10),
    Point2d(8, 6),
    Point2d(6, 9),
    Point2d(5, 14),
    Point2d(3, 18),
    Point2d(3, 23),
    Point2d(2, 27),
    Point2d(1, 31),

    Point2d(2, 29),
    Point2d(3, 26),
    Point2d(5, 23),
    Point2d(7, 22),
    Point2d(9, 24),
    Point2d(9, 28),
    Point2d(10, 31),
    Point2d(13, 29),
    Point2d(15, 27),
    Point2d(17, 25),
    Point2d(20, 24),
    Point2d(20, 22),
    Point2d(17, 22),
    Point2d(14, 25),

    Point2d(14, 30),
    Point2d(17, 31),
    Point2d(20, 31),
    Point2d(22, 29),
    Point2d(24, 26),
    Point2d(26, 22),
    Point2d(28, 18),
    Point2d(29, 14),
    Point2d(29, 11),
    Point2d(28, 6),
    Point2d(26, 10),
    Point2d(25, 14),
    Point2d(24, 19),
    Point2d(24, 22),
    Point2d(24, 26),

    Point2d(25, 30),
    Point2d(28, 31),
    Point2d(30, 28),
    Point2d(32, 24),
    Point2d(34, 19),
    Point2d(36, 14),
    Point2d(36, 10),
    Point2d(35, 6),
    Point2d(33, 10),
    Point2d(32, 15),
    Point2d(31, 19),
    Point2d(30, 24),
    Point2d(30, 28),

    Point2d(31, 30),
    Point2d(34, 31),
    Point2d(37, 28),
    Point2d(38, 24),
    Point2d(40, 22),
    Point2d(44, 22),

    Point2d(40, 23),
    Point2d(38, 26),
    Point2d(38, 30),
    Point2d(41, 31),
    Point2d(44, 29),
    Point2d(45, 27),
    Point2d(45, 25),
    Point2d(43, 22),
    Point2d(42, 25),
    Point2d(45, 27),
    Point2d(48, 27),
    Point2d(52, 26),
    Point2d(56, 24),
    Point2d(59, 23),
    Point2d(62, 22),
    Point2d(61, 26),
    Point2d(61, 29),
    Point2d(64, 31),
    Point2d(67, 29),
    Point2d(68, 25),
    Point2d(69, 22),

    Point2d(69, 26),
    Point2d(69, 30),
    Point2d(71, 31),
    Point2d(74, 29),
    Point2d(75, 27),
    Point2d(75, 25),
    Point2d(73, 22),
    Point2d(72, 25),
    Point2d(75, 27),
    Point2d(78, 27),
    Point2d(79, 24),
    Point2d(82, 22),

    Point2d(80, 24),
    Point2d(79, 28),
    Point2d(80, 31),
    Point2d(83, 31),
    Point2d(86, 29),
    Point2d(86, 25),
    Point2d(85, 22),
    Point2d(83, 23),
    Point2d(85, 26),
    Point2d(88, 27),
    Point2d(90, 25),
    Point2d(91, 22),
    Point2d(94, 23),
    Point2d(93, 26),
    Point2d(93, 29),
    Point2d(95, 31),
    Point2d(97, 28),
    Point2d(99, 25),
    Point2d(101, 21),
    Point2d(103, 17),
    Point2d(104, 13),
    Point2d(104, 10),
    Point2d(103, 6),
    Point2d(101, 10),
    Point2d(99, 14),
    Point2d(99, 18),
    Point2d(98, 22),

    Point2d(99, 27),
    Point2d(100, 31),
    Point2d(102, 31),
    Point2d(104, 28),
    Point2d(106, 24),
    Point2d(109, 22),
    Point2d(112, 22),

    Point2d(108, 23),
    Point2d(106, 25),
    Point2d(105, 28),
    Point2d(107, 31),
    Point2d(110, 30),
    Point2d(112, 26),
    Point2d(113, 22),
    Point2d(114, 18),
    Point2d(115, 14),
    Point2d(116, 10),
    Point2d(117, 6),

    Point2d(116, 11),
    Point2d(115, 15),
    Point2d(114, 19),
    Point2d(113, 23),
    Point2d(112, 27),
    Point2d(112, 31),
    Point2d(115, 30),
    Point2d(116, 27)
    ]

# ----------------------------------------
# Functions
# ----------------------------------------
    
def calculateAngleFromSides(s1, s2, s3):
    # Law of Cosines
    cosAngle = ((s1 * s1) + (s2 * s2) - (s3 * s3)) / (2.0 * s1 * s2)

    angleInRadians = math.acos(cosAngle)
    angleInDegrees = math.degrees(angleInRadians);

    return angleInDegrees

def updateMotorTargetPositions(point):
    a = point.getDistanceToOrigin() - (BASE_OFFSET + GRIPPER_OFFSET)
    b = UPPER_ARM_LENGTH
    c = FOREARM_LENGTH

    C = calculateAngleFromSides(a, b, c)
    A = calculateAngleFromSides(b, c, a)
    B = 180.0 - A - C

    baseMotor.updateTargetPosition(point.getAngleToOrigin())
    shoulderMotor.updateTargetPosition(90.0 - C)
    elbowMotor.updateTargetPosition(180.0 - A)
    wristMotor.updateTargetPosition(B)

def configureMotors(point):
    for motor in motors:
        motor.setDirection()
        motor.compensateForBacklash()
        motor.configure()

def runMotors():
    for motor in motors:
        if motor.position != motor.targetPosition:
            motor.start()

    while any(motor.isRunning for motor in motors):
        for motor in motors:
            if motor.isRunning:
                motor.sensorValue = GPIO.input(motor.sensorGpio)

                if motor.sensorValue != motor.previousSensorValue:
                    motor.previousSensorValue = motor.sensorValue

                    if motor.position < motor.targetPosition:
                        motor.position += 1
                    elif motor.position > motor.targetPosition:
                        motor.position -= 1

                    if motor.position == motor.targetPosition:
                        motor.stop()

            time.sleep(0.00025)

def home():
    baseMotor.targetPosition = 0
    shoulderMotor.targetPosition = 0
    elbowMotor.targetPosition = 0
    wristMotor.targetPosition = 0

def moveToPoint(point):
    updateMotorTargetPositions(point)
    configureMotors(point)
    runMotors()

# ----------------------------------------
# Program start
# ----------------------------------------

try:
    GPIO.setmode(GPIO.BCM)

    print('Drawing image.')

    for point in points:
        point.x -= 60
        point.y -= 200

    for point in points:
        moveToPoint(point)
        #time.sleep(0.5)

    home()
    configureMotors(point)
    runMotors()

finally:
    print('Shutting down.')

    for motor in motors:
       motor.stop()

    GPIO.cleanup()
