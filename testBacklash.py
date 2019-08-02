import time
import piplates.MOTORplate as MOTOR
import RPi.GPIO as GPIO

# motor 1, sensor GPIO 16, "wrist", cw = down 
# motor 2, sensor GPIO 12, "elbow", cw = up
# motor 3, sensor GPIO 21, "shoulder", cw = down
# motor 4, sensor GPIO 20, "rotate base", cw = counter-clockwise

motorNumber = 4
sensorGpio = 20
motorSpeed = 100.0
motorDirection = 1

backlashTestValue = 20

def startMotor():
    MOTOR.dcCONFIG(0, motorNumber, 'cw' if motorDirection == 1 else 'ccw', motorSpeed, 0.0)
    MOTOR.dcSTART(0, motorNumber)

try:
    GPIO.setmode(GPIO.BCM)

    print('Moving motor.')

    GPIO.setup(sensorGpio, GPIO.IN)
    previousSensorValue = GPIO.input(sensorGpio)

    motorMovement = 0
    startMotor()

    while true
        sensorValue = GPIO.input(sensorGpio)

        if sensorValue != previousSensorValue:
            previousSensorValue = sensorValue
            motorMovement += 1

            if motorMovement >= backlashTestValue
                MOTOR.dcSTOP(0, motorNumber)
                motorMovement = 0
                motorDirection *= -1
                time.sleep(0.5)
                startMotor()

        time.sleep(0.001)

    MOTOR.dcSTOP(0, motorNumber)
finally:
    print('Shutting down.')

    MOTOR.dcSTOP(0, motorNumber)

    GPIO.cleanup()
