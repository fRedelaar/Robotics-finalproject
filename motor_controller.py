import time
from threading import Thread
import Jetson.GPIO as GPIO
from collections import deque
import numpy as np 
import math

GPIO.setmode(GPIO.BOARD)

class Motor_Controller:
    FORWARD = (1,0)
    BACKWARD = (0,1)
    BREAK = (0,0) 

    def __init__(self,ena1,ena2,pwm,enc,name="default_motor", sampling_speed = 0.01, deque_size = 50) -> None:
        self.enable1 = ena1
        self.enable2 = ena2
        self.pwm_pin = pwm
        self.name = name
        
        self.enc = enc
        self.enc_pulse_count = 0
        self.sampling_speed = sampling_speed
        self.deque = deque([0], maxlen=deque_size)
        self.running = False 

        self.forward = True

        print(f"Starting Initialization: {self.name}")
        
        GPIO.setup(self.enable1,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.enable2,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.pwm_pin,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(self.enc,GPIO.IN)
        print(f"Motor Pins Initialized: {self.name}")
        
        self.pwm = GPIO.PWM(self.pwm_pin,50)
        self.pwm.start(0)
        print(f"PWM started: {self.name}")
        time.sleep(0.1)

        time.sleep(0.1)
        print(f"Encoder Initialized: {self.name}")

        time.sleep(0.2)
        print(f"Finished Initialization: {self.name}")

    def count_enc(self,channel):
        if self.forward:
            self.enc_pulse_count +=1
        else:
            self.enc_pulse_count -=1

    def track_speed(self):
        while self.running:
            time.sleep(self.sampling_speed)
            self.deque.appendleft(self.enc_pulse_count)
            self.enc_pulse_count=0

    def get_speed(self):
        return np.mean(self.deque) / self.sampling_speed

    def start(self):
        GPIO.add_event_detect(self.enc,GPIO.RISING,callback=self.count_enc)
        print("Start Tracking Speed")
        self.running = True
        self.thread = Thread(target=self.track_speed)
        self.thread.start()

    def set_pwm(self,forward,pwm):
        dir = self.FORWARD if forward else self.BACKWARD
        self.forward = forward
        GPIO.output(self.enable1,dir[0])
        GPIO.output(self.enable2,dir[1])
        self.pwm.ChangeDutyCycle(pwm)

    def stop(self):
        self.running = False
        self.thread.join()
        print(f"{self.name} Stopped PWM")
        self.pwm.stop()

def main():
    MA = Motor_Controller(37,35,33,16,name="MA")
    MB = Motor_Controller(38,36,32,22,name="MB")
    MA.set_pwm(True,40)
    MB.set_pwm(True,40)
    MA.start()
    MB.start()
    t_start = time.time()
    diff = 0
    
    while diff < 5:
        diff = time.time()-t_start
        MA_s = MA.get_speed()
        MB_s = MB.get_speed()
        print(f"MA: {round(MA_s,2)}\tMB: {round(MB_s,2)}|\t Time elapsed {diff}")

    MA.set_pwm(False,40)
    MB.set_pwm(False,40)
    t_start = time.time()
    diff = 0
    
    while diff < 5:
        diff = time.time()-t_start
        MA_s = MA.get_speed()
        MB_s = MB.get_speed()
        print(f"MA: {round(MA_s,2)}\tMB: {round(MB_s,2)}|\t Time elapsed {diff}")

    MA.set_pwm(True,0)
    MB.set_pwm(True,0)

    MA.stop()
    MB.stop()
    GPIO.cleanup()
    
if __name__ == "__main__":
    print("Testing Motor Controller")
    main()
