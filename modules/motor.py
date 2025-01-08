import time
from machine import SoftI2C, Pin, PWM

class Motor:
    def __init__(self, L_G, L_B, R_G, R_B, BASE_SPEED=0):
        self.L_GO   = PWM(Pin(L_G), freq=100)
        self.L_BACK = PWM(Pin(L_B), freq=100)
        self.R_GO   = PWM(Pin(R_G), freq=100)
        self.R_BACK = PWM(Pin(R_B), freq=100)
        self.BASE_SPEED = BASE_SPEED

    def l_motor(self, speed):
        
        if speed > 0:
            speed = speed + self.BASE_SPEED
            speed = int(max(-1023, min(speed, 1023)))
            self.L_GO.duty(speed)
            self.L_BACK.duty(0)

        elif speed < 0:
            speed = speed - self.BASE_SPEED
            speed = int(max(-1023, min(speed, 1023)))
            self.L_GO.duty(0)
            self.L_BACK.duty(-speed)

        else:
            self.L_GO.duty(0)

    def r_motor(self, speed):
        
        if speed > 0:
            speed = speed + self.BASE_SPEED
            speed = int(max(-1023, min(speed, 1023)))
            self.R_GO.duty(speed)
            self.R_BACK.duty(0)

        elif speed < 0:
            speed = speed - self.BASE_SPEED
            speed = int(max(-1023, min(speed, 1023)))
            self.R_GO.duty(0)
            self.R_BACK.duty(-speed)
            
        else:
            self.R_GO.duty(0)

    def motor(self, v_l, v_r):
        self.l_motor(v_l)
        self.r_motor(v_r)

    def motion(self, v, w):  # v: linear speed, w: angular speed
        v_l = v - w
        v_r = v + w
        self.l_motor(v_l)
        self.r_motor(v_r)

    def stop(self):
        self.L_GO.duty(0)
        self.L_BACK.duty(0)
        self.R_GO.duty(0)
        self.R_BACK.duty(0)

def test():
    motor = Motor(33,35,18,16)
    while True:
        
        print("l_motor forward")
        motor.l_motor(100)
        time.sleep(2)
        
        print("r_motor forward")
        motor.r_motor(200)
        time.sleep(2)    
        
        print("stop")
        motor.stop()
        time.sleep(1)
        
        print("move")
        motor.move(200, 80) 
        time.sleep(5)

        print("stop")
        motor.stop()
        time.sleep(3)


if __name__ == '__main__':
    
    test()

    from encoder import HallEncoder
    from pid import PID

    encoder_l = HallEncoder(7, 10)
    encoder_r = HallEncoder(6, 5)


    pid_l = PID(0.0000001, 0.0, 0.0, setpoint=0)
    
    pid_r = PID(1.1, 0.1, 0.001, setpoint=0)

    motor = Motor(3,4,2,1)

    pid_l.setpoint = 800
    pid_r.setpoint = 800

    start = time.time()
    while time.time() - start < 100:
        
        if time.time() - start > 5:
            pid_r.setpoint = 1000        

        if time.time() - start > 10:
            pid_r.setpoint = 1500        
        
        if time.time() - start > 15:
            pid_r.setpoint = 2000
        
        if time.time() - start > 15:
            pid_r.setpoint = 1000

        v_l_now = encoder_l.get_speed()
        v_r_now = encoder_r.get_speed()
        
        v_l_pwm = pid_l.update(v_l_now)
        v_r_pwm = pid_r.update(v_r_now)
        
        print(f"trage: {pid_r.setpoint}, v_r_now: {v_r_now}")
        motor.motor(v_l_pwm, v_r_pwm)

        time.sleep(0.1)

    motor.stop()
    