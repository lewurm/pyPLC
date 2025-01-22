import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

stepup_pwm = GPIO.PWM(18, 10)
stepup_pwm.start(0.5)

for i in range(1200, 3060, 100):
    # bei 5.1V Eingangsspannung (USB Netzteil)

    # Not exactly linear this whole thing, but good enough?

    #   25k Hz: 298V
    #   55k Hz: 355V
    #   65k Hz: 369V
    #   80k Hz: 380V
    #  110k Hz: 388V
    #  150k Hz: 401V
    #  180k Hz: 407V
    #  210k Hz: 411V
    #  300k Hz: 415V
    #  400k Hz: 420V
    #  600k Hz: 425V
    #  800k Hz:
    # 2000k Hz: 431V

    freq_k = i * 1000
    stepup_pwm.ChangeFrequency(freq_k)
    print("trying with freq_k=" + str(freq_k) + " Hz")

    # stepup_pwm.ChangeDutyCycle(0.05 * i)
    # print("trying with dc=" + str(0.05 * i) + " %")

    time.sleep(5)

stepup_pwm.ChangeFrequency(0)
print("trying with freq_k=" + str(0) + " Hz")
time.sleep(50)
stepup_pwm.stop()
GPIO.cleanup()
