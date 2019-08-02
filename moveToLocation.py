import math
import time
import piplates.MOTORplate as MOTOR
import RPi.GPIO as GPIO

# motor 1, sensor GPIO 16, "wrist", cw = down 
# motor 2, sensor GPIO 12, "elbow", cw = up
# motor 3, sensor GPIO 21, "shoulder", cw = down
# motor 4, sensor GPIO 20, "rotate base", cw = counter-clockwise

motorSpeed = 100.0

baseOffset = 4.0
gripperOffset = 90.0

upperArmLength = 91.3
forearmLength = 113.3

baseUnitsPerAngle = 4.39
shoulderUnitsPerAngle = 4.21
elbowUnitsPerAngle = 4.21
wristUnitsPerAngle = 4.84

targetX = -60.0
targetY = -200.0

global baseMotorPosition
global shoulderMotorPosition
global elbowMotorPosition
global wristMotorPosition

baseMotorPosition = 0
shoulderMotorPosition = 0
elbowMotorPosition = 0
wristMotorPosition = 0

def getDistanceToTarget():
    distance = math.sqrt((targetX * targetX) + (targetY * targetY)) - (baseOffset + gripperOffset)

    return distance

def getAngleToTarget():
    angleInRadians = math.atan2(targetX, -targetY)
    angleInDegrees = math.degrees(angleInRadians)

    return angleInDegrees

def updateMotorPositions(distanceToTarget, angleToTarget):
    global baseMotorPosition
    global shoulderMotorPosition
    global elbowMotorPosition
    global wristMotorPosition

    a = distanceToTarget
    b = upperArmLength
    c = forearmLength

    C = calculateAngleFromSides(a, b, c)
    A = calculateAngleFromSides(b, c, a)
    B = 180.0 - A - C

    baseMotorPosition = angleToTarget * baseUnitsPerAngle
    shoulderMotorPosition = (90.0 - C) * shoulderUnitsPerAngle
    elbowMotorPosition = (180.0 - A) * elbowUnitsPerAngle
    wristMotorPosition = B * wristUnitsPerAngle

def calculateAngleFromSides(s1, s2, s3):
    # Law of Cosines
    cosAngle = ((s1 * s1) + (s2 * s2) - (s3 * s3)) / (2.0 * s1 * s2)

    angleInRadians = math.acos(cosAngle)
    angleInDegrees = math.degrees(angleInRadians);

    return angleInDegrees

def moveMotor(motorNumber, motorDirection, sensorGpio, motorPosition):
    GPIO.setup(sensorGpio, GPIO.IN)
    previousSensorValue = GPIO.input(sensorGpio)

    MOTOR.dcCONFIG(0, motorNumber, motorDirection, motorSpeed, 0.0)
    MOTOR.dcSTART(0, motorNumber)

    motorMovement = 0
    
    while motorMovement < abs(motorPosition):
        sensorValue = GPIO.input(sensorGpio)

        if sensorValue != previousSensorValue:
            previousSensorValue = sensorValue
            motorMovement += 1

        time.sleep(0.001)

    MOTOR.dcSTOP(0, motorNumber)

try:
    GPIO.setmode(GPIO.BCM)

    print('Calculating motor positions...')

    distanceToTarget = getDistanceToTarget()
    angleToTarget = getAngleToTarget()
    updateMotorPositions(distanceToTarget, angleToTarget)

    print('Moving motors...')

    motorDirection = 'cw' if baseMotorPosition < 0 else 'ccw'
    motorPosition = abs(baseMotorPosition)
    print('Base motor = direction: ', motorDirection, ', position: ', motorPosition)
    moveMotor(4, motorDirection, 20, motorPosition)

    motorDirection = 'ccw' if shoulderMotorPosition < 0 else 'cw'
    motorPosition = abs(shoulderMotorPosition)
    print('Shoulder motor = direction: ', motorDirection, ', position: ', motorPosition)
    moveMotor(3, motorDirection, 21, motorPosition)

    motorDirection = 'cw' if wristMotorPosition < 0 else 'ccw'
    motorPosition = abs(wristMotorPosition)
    print('Wrist motor = direction: ', motorDirection, ', position: ', motorPosition)
    moveMotor(1, motorDirection, 16, motorPosition)

    motorDirection = 'cw' if elbowMotorPosition < 0 else 'ccw'
    motorPosition = abs(elbowMotorPosition)
    print('Elbow motor = direction: ', motorDirection, ', position: ', motorPosition)
    moveMotor(2, motorDirection, 12, motorPosition)
finally:
    print('Shutting down.')

    MOTOR.dcSTOP(0, 1)
    MOTOR.dcSTOP(0, 2)
    MOTOR.dcSTOP(0, 3)
    MOTOR.dcSTOP(0, 4)

    GPIO.cleanup()
