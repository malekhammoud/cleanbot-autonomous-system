import RPi.GPIO as GPIO
from time import sleep
import serial

GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BOARD)		#set pin numbering system
GPIO.setup(32,GPIO.OUT)
throttle = GPIO.PWM(32,500)		#create PWM instance with frequency
throttle.start(0)				#start PWM of required Duty Cycle 

GPIO.setup(12,GPIO.OUT)
yaw = GPIO.PWM(12,500)		#create PWM instance with frequency
yaw.start(0)				#start PWM of required Duty Cycle 
sleep(0.5)

ser = serial.Serial('/dev/ttyACM0')
print('start')


GPIO.setup(16, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)

"""
    GPIO.output(16, True) 
    GPIO.output(18, False) 
    pi_pwm.ChangeDutyCycle(20) #provide duty cycle in the range 0-100
    pitch.ChangeDutyCycle(20) #provide duty cycle in the range 0-100
    sleep(0.04)
    GPIO.output(16, False) 
    GPIO.output(18, True) 
    pi_pwm.ChangeDutyCycle(2) #provide duty cycle in the range 0-100
    pitch.ChangeDutyCycle(2) #provide duty cycle in the range 0-100
    sleep(0.04)
"""
while True:
    command=str(ser.readline())
    command=command[2:][:-5]
    command=command.split()
    for i in range(len(command)):
        command[i] = int(command[i])
    if command[0]>60000:
        command[0] = 60000
    elif command[0]<500:
        command[0] = 0
    command[0] = round(command[0]/60000*70+0.1, 3)
    command[1] = round(command[1]/60000*70+0.1, 3)
    if command[1] > 65:
        command[1] =0
    if command[0] > 65:
        command[0] =0
    
    print(command)

    if command[2] == 0:
        sleep(0.5)

        for i in range(1):
            GPIO.output(16, True) 
            GPIO.output(18, False) 
            for duty in range(29,0,-1):
                yaw.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
                throttle.ChangeDutyCycle(duty)
                print(duty)
                sleep(0.1)
            """
            for duty in range(0,71,1):
                yaw.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
                throttle.ChangeDutyCycle(duty)
                print(duty)
                sleep(0.1)
            for duty in range(70,-1,-1):
                yaw.ChangeDutyCycle(duty)
                throttle.ChangeDutyCycle(duty)

                print(duty)
                sleep(0.1)
                """
            sleep(2)
            yaw.ChangeDutyCycle(28)
            throttle.ChangeDutyCycle(28)
            sleep(1)
            yaw.ChangeDutyCycle(28)
            throttle.ChangeDutyCycle(28)
            sleep(2)
        """
        GPIO.output(16, True) 
        GPIO.output(18, False) 
        print("arming")
        yaw.ChangeDutyCycle(40)
        throttle.ChangeDutyCycle(40)
        sleep(3)
        yaw.ChangeDutyCycle(1)
        throttle.ChangeDutyCycle(1)
        sleep(3)
        yaw.ChangeDutyCycle(25)
        throttle.ChangeDutyCycle(25)
        sleep(0.5)
        print('done')
        """

    GPIO.output(16, False) 
    GPIO.output(18, True) 
    throttle.ChangeDutyCycle(command[0]) #provide duty cycle in the range 0-100
    yaw.ChangeDutyCycle(command[0])
    """
        GPIO.output(18, False) 
        GPIO.output(16, True) 
        throttle.ChangeDutyCycle(command[1]) #provide duty cycle in the range 0-100
        yaw.ChangeDutyCycle(command[1])
        sleep(0.1)
    """
