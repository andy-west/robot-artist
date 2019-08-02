import RPi.GPIO as GPIO
import time

sensorGpio = 16

GPIO.setmode(GPIO.BCM)
GPIO.setup(sensorGpio, GPIO.IN)

print('Starting main loop.')

try:
    while True:
        sensorValue = GPIO.input(sensorGpio)
        print(sensorValue)
        time.sleep(0.001)
except KeyboardInterrupt:
    pass

print('Done!')

GPIO.cleanup()
