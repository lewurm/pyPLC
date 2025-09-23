import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# BCM (WiringPi)
channel_cp = 23 # (4)
GPIO.setup(channel_cp, GPIO.OUT, initial=GPIO.HIGH)
time.sleep(5)

print("break cp")
GPIO.output(channel_cp, GPIO.LOW)
time.sleep(5)

print("enable cp")
GPIO.output(channel_cp, GPIO.HIGH)
time.sleep(5)

# time.sleep(3)
# print("low")
# 
# time.sleep(3)
# GPIO.output(channel, GPIO.HIGH)
# print("high")
# 
# time.sleep(3)
# GPIO.output(channel, GPIO.LOW)
# print("low")
# 
# time.sleep(3)
# GPIO.output(channel, GPIO.HIGH)
# print("high")
# # while True:
# #     pass
# 
# time.sleep(3)
GPIO.cleanup()
print("cleanup")

