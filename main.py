import time
from machine import UART, Pin, PWM

from imu import Accel
from encoder import HallEncoder
from motor import Motor
from pid import PID

from tools import time_diff

# 初始化参数
IMU_OFFSET = 1 
BASE_PWM = 5

# 初始化各对象
imu = Accel(9, 8)
encoder_l = HallEncoder(4,6)
encoder_r = HallEncoder(2,1)
motor = Motor(33,35,18,16, BASE_PWM)

pid = PID(kp=0.7, ki=0.00, kd=0.01, setpoint=0, output_limits=(-1023, 1023))

Beep = PWM(Pin(15, Pin.OUT), freq=500)

# 初始化全局变量
delay_s = 0
delay_s_max = 0

while True:
    # 获取时间差相关信息
    dt = time_diff()

    ms  = dt
    sec = dt / 1000
    hz  = 1 / (sec)

    # print(f"一个循环运行时间: {ms:.2f} ms, fps: {hz:.2f} Hz")

    roll, pitch = imu.get_angles(sec)

    speed_l = encoder_l.get_speed()
    speed_r = encoder_r.get_speed()

    if abs(pitch) > 90 or abs(roll) > 70:
        motor.stop()
        print(f"检测到跌倒, 关闭电机, roll = {roll}, pitch = {pitch}")
        time.sleep(0.5)
        continue

    angle = roll - IMU_OFFSET

    b_pwm = min(int(abs(angle)/60 * 1023), 1023)

    Beep.duty(b_pwm)

    v_pwm = (angle / 60) * 1023
    w_pwm = 0

    v_pwm_pid = -pid.update(v_pwm)

    motor.motion(v_pwm_pid, w_pwm)

    vofa_msg = f"{angle:.2f}, {v_pwm}, {v_pwm_pid}, {ms}, {speed_l}, {speed_r}, {pid.kp}, {pid.ki}, {pid.kd}"

    debug_msg = f"delay: {ms:.2f} ms, {hz:.2f} Hz, v_pwm: {v_pwm:.2f}, v_pwm_pid: {v_pwm_pid:.2f}, roll: {roll:.2f}, pitch: {pitch:.2f}, speed_l: {speed_l:.2f}, speed_r: {speed_r:.2f}"
    print(debug_msg)

    # time.sleep(0.00001)
    # time.sleep(0.1)

