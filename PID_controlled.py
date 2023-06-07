import time
from motor_controller import Motor_Controller
import Jetson.GPIO as GPIO
from ICM20948 import ICM20948
class PD:
    def __init__(self, Proportional, Integral, Direction, goal,name):
        scale = 1
        self.Proportional = Proportional *scale
        self.Integral = Integral * scale
        self.Direction = Direction * scale
        self.goal = goal
        self.name = name
        self.last_error = 0
        self.last_time = time.time()
        self.I = 0


    def observe(self, x):
        # now_time = time.time()

        # determine error
        error = self.goal - x

        # compute proportional term
        P = self.Proportional * error

        # compute intergral term
        self.I = self.I + self.Integral * error * 1 # (now_time - self.last_time) # experiment with sample speed correction
        I =  self.I

        # compute differential term
        Direction_error = error - self.last_error
        self.last_error = error
        D = self.Direction * Direction_error / 1 # (now_time - self.last_time) # experiment with sample speed correction

        # self.last_time = now_time
        return P + I + D
    
    def check_settings(self, settings:dict):
        self.Proportional = settings["p"] if settings["p"] is not None else self.Proportional
        self.Integral= settings["i"] if settings["i"] is not None else self.Integral


class Controller:
    def __init__(self):
        self.cart = PD(Proportional=0.0, Integral=0.0, Direction=0.0, goal=0, name="cart") # not tuned yet
        self.pole = PD(Proportional=5, Integral=0.0, Direction=6, goal=0, name= "pole")
        time.sleep(0.2)

    def observe(self, v1, alpha):
        cart = self.cart.observe(v1)
        pole = self.pole.observe(alpha)
        action = cart + pole
        return action
    
def map_control_value(x,min_val=1,max_val=100):
    forward = x > 0
    x = abs(x)
    x = max(x,min_val)
    x = min(x,max_val)
    return x, forward

def print_sensor(m1,m2,IMU_dat,t):
    pitch = round(IMU_dat[0],5) 
    roll = round(IMU_dat[1],5) 
    yaw = round(IMU_dat[2],5) 
    ax = round(IMU_dat[3],5)
    ay = round(IMU_dat[4],5)
    az = round(IMU_dat[5],5)
    gx = round(IMU_dat[6],5)
    gy = round(IMU_dat[7],5)
    gz = round(IMU_dat[8],5)
    print(f"MotorA: {str(round(m1,2)).ljust(10)} | MotorB: {str( round(m2,2)).ljust(10)} | Pitch: {str(pitch).ljust(10)} Roll: {str(roll).ljust(10)} Yaw: {str(yaw).ljust(10)} | Ax: {str(ax).ljust(10)} Ay: {str(ay).ljust(10)} Az: {str(az).ljust(10)} | Gx: {str(gx).ljust(10)} Gy: {str(gy).ljust(10)} Gz: {str(gz).ljust(10)} | Time elapsed {str(t).ljust(10)}")

def main():
    GPIO.setmode(GPIO.BOARD)

    IMU = ICM20948()

    left_controller = Controller()
    right_controller = Controller()
    forward = True

    MA = Motor_Controller(37,35,33,16,name="MA")
    MB = Motor_Controller(38,36,32,22,name="MB")
    MB.set_pwm(forward,10)
    MA.set_pwm(forward,10)
    MA.start() # start the speed controller thread for motor A
    MB.start() # start the speed controller thread for motor B
    
    ### sensor warmup to allow magnetometer to provide accurate readings
    for x in range(100):
        IMU_dat=IMU.getdata()
        MA_s = MA.get_speed()
        MB_s = MB.get_speed()
        print_sensor(MA_s,MB_s,IMU_dat,x)

    print("##### FINISHED WARMUP #####")

    MA.set_pwm(forward,0)
    MB.set_pwm(forward,0)

    t_start = time.time()
    diff = 0
    t = time.time()
    while diff < 20: # run program for 20 seconds
        delta =time.time()-t
        diff = time.time()-t_start
        
        if delta > 0.08: # make sure control loop has a constant time interval
            t=time.time()
            
            IMU_dat = None
            count = 0
            while IMU_dat is None:
                try:
                    IMU_dat=IMU.getdata()
                except:
                    print(f"imu failed {count} times")
                    count +=1
            MA_s = MA.get_speed()
            MB_s = MB.get_speed()
            print_sensor(MA_s,MB_s,IMU_dat,delta)
            left_ctrl = left_controller.observe(MA_s,-(IMU_dat[0]-90))
            right_ctrl = right_controller.observe(MB_s,-(IMU_dat[0]-90))
            left_ctrl = map_control_value(left_ctrl)
            right_ctrl = map_control_value(right_ctrl)

            MA.set_pwm(left_ctrl[1],left_ctrl[0])
            MB.set_pwm(right_ctrl[1],right_ctrl[0])

    MA.set_pwm(MA.FORWARD,1)
    MB.set_pwm(MB.FORWARD,1)

    MA.stop()
    MB.stop()
    GPIO.cleanup()    
    
if __name__ == "__main__":
    main()
