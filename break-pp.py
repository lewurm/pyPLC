import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# BCM (WiringPi)
channel_cp = 23 # (4)
channel_pp = 26 # (25)
GPIO.setup(channel_cp, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(channel_pp, GPIO.OUT, initial=GPIO.HIGH)
time.sleep(1)

print("break cp")
GPIO.output(channel_cp, GPIO.LOW)
time.sleep(1)
print("break pp")
GPIO.output(channel_pp, GPIO.LOW)
time.sleep(2)
print("enable pp")
GPIO.output(channel_pp, GPIO.HIGH)

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

