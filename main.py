import time
from machine import UART, Pin, PWM

from lib.imu import Accel
from lib.encoder import HallEncoder
from lib.motor import Motor
from lib.pid import PID


from tools import time_diff

# 初始化参数
IMU_OFFSET = 1 
BASE_PWM = 5

# 初始化各对象
imu = Accel(9, 8)
encoder_l = HallEncoder(4,6)
encoder_r = HallEncoder(2,1)
motor = Motor(33,35,18,16, BASE_PWM)

pid_angle = PID(kp=10.0, ki=0.0, kd=0.0 , setpoint=0, output_limits=(-1023, 1023))  # 角度环
pid_gyx   = PID(kp=3.0, ki=0.0, kd=0.01, setpoint=0, output_limits=(-1023, 1023))  # 角速度环


Beep = PWM(Pin(15, Pin.OUT), freq=500)

# 初始化全局变量
delay_s = 0
delay_s_max = 0

print("初始化完成, 启动循环")
time.sleep(1)

while True:
    # 获取时间差相关信息
    dt = time_diff()

    ms  = dt
    sec = dt / 1000
    hz  = 1 / (sec)

    # speed_l = encoder_l.get_speed()
    # speed_r = encoder_r.get_speed()

    speed_l = 0
    speed_r = 0

    # 读取imu数据
    roll, pitch = imu.get_angles(sec)
    gyx = imu.vals["GyX"] / 131.0

    if abs(pitch) > 90 or abs(roll) > 70:
        motor.stop()
        print(f"检测到跌倒, 关闭电机, roll = {roll}, pitch = {pitch}")
        time.sleep(0.5)
        continue

    angle = roll - IMU_OFFSET  # 减去偏置

    # 计算PID
    angle_pid = -pid_angle.update(angle)
    gyx_pid   = -pid_gyx.update(gyx)

    # 计算输出
    pwm_all = angle*0.5 + gyx_pid*0.0         # 直立环 = 角度环 + 角速度环
    pwm_all = min(max(pwm_all, -1023), 1023)  # 限制输出

    motor.motion(pwm_all, 0)

    vofa_msg = f"{angle:.2f}, {angle}, {angle_pid}, {ms}, {speed_l}, {speed_r}, {pid_angle.kp}, {pid_angle.ki}, {pid_angle.kd}"

    debug_msg = f"delay: {ms:.2f} ms, {hz:.2f} Hz, angle: {angle:.2f}, angle_pid: {angle_pid:.2f}, \
roll: {roll:.2f}, gyx: {gyx:.2f}, gyx_pid: {gyx_pid:.2f}, speed_l: {speed_l:.2f}, speed_r: {speed_r:.2f}"
    
    print(debug_msg)

    # 蜂鸣器反馈显示
    b_pwm = min(int(abs(gyx)/60 * 1023), 1023)
    Beep.duty(b_pwm)

    # time.sleep(0.00001)
    # time.sleep(0.1)

