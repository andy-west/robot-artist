import piplates.MOTORplate as MOTOR
import RPi.GPIO as GPIO
import time

# motor 1, sensor GPIO 16, "wrist", cw = down 
# motor 2, sensor GPIO 12, "elbow", cw = up
# motor 3, sensor GPIO 21, "shoulder", cw = down
# motor 4, sensor GPIO 20, "rotate base", cw = counter-clockwise

motor = 1
sensorGpio = 16

GPIO.setmode(GPIO.BCM)
GPIO.setup(sensorGpio, GPIO.IN)

print('Starting main loop.')

# addr, motor, dir, speed, acceleration
MOTOR.dcCONFIG(0, motor, 'cw', 50.0, 0.5)

# addr, motor
MOTOR.dcSTART(0, motor)

try:
    while True:
        sensorValue = GPIO.input(sensorGpio)
        print(sensorValue)
        time.sleep(0.001)
except KeyboardInterrupt:
    pass

MOTOR.dcSTOP(0, motor)

print('Done!')

GPIO.cleanup()
