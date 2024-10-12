import time
import usocket
import _thread
from machine import UART, Pin, PWM

from imu import Accel
from encoder import HallEncoder
from motor import Motor
from pid import PID

IMU_OFFSET = 2
BASE_PWM = 1

imu = Accel(9, 8)
encoder_l = HallEncoder(4,6)
encoder_r = HallEncoder(2,1)
motor = Motor(33,35,18,16, BASE_PWM)

pid = PID(kp=0.4, ki=0.000, kd=0.00, setpoint=0, output_limits=(-1023, 1023))


Beep = PWM(Pin(15, Pin.OUT), freq=500)

sw = True
def stop_btn_callback(pin):
    global sw
    time.sleep(0.1)
    if pin.value() == 0:
        sw = not sw
        # led.value(not led.value())
        print("停止定时器")  # 不然Thonny无法停止程序

stop_btn = Pin(0, Pin.IN, Pin.PULL_UP)
stop_btn.irq(stop_btn_callback, Pin.IRQ_FALLING)

# try:
#     tcp = usocket.socket()
#     tcp.connect(('192.168.43.240', 1347))  # 服务器IP和端口
#     tcp.send(b'Hello WalnutPi!')
# 
# except Exception as e:
#     print("TCP Error:", e)
# 
# def tcp_thread():
#     global tcp, IMU_OFFSET, BASE_PWM
#     while tcp:
#         time.sleep(0.01)
#         try:
#             text = tcp.recv(128)  # 单次最多接收128字节
#             if text:
#                 print("RX:", text)
#                 # 将字节串解码为字符串
#                 decoded_data = text.decode('utf-8').strip()  # 去掉末尾的换行符
#                 key, value = decoded_data.split(':')  # 分割字符串以提取键和值
#                 tcp.send(b'I got: ' + key + value)
# 
#                 if key == 'kp':
#                     pid.kp = float(value)
#                 elif key == 'ki':
#                     pid.ki = float(value)
#                 elif key == 'kd':
#                     pid.kd = float(value)
#                 elif key == 'offset':
#                     IMU_OFFSET = float(value)
#                 elif key == 'BASE_PWM':
#                     BASE_PWM = int(value)
#                     motor.BASE_SPEED = BASE_PWM         
#                 
#         except Exception as e:
#             print("TCP Error:", e)
#             break
# 
# # 启动TCP线程
# _thread.start_new_thread(tcp_thread, ())

def time_diff(last_time=[None]):
    """计算两次调用之间的时间差，单位为微秒。"""
    current_time = time.ticks_us()  # 获取当前时间（单位：微秒）

    if last_time[0] is None: # 如果是第一次调用，更新last_time
        last_time[0] = current_time
        return 0.000_001 # 防止除零错误
    
    else: # 计算时间差
        diff = time.ticks_diff(current_time, last_time[0])  # 计算时间差
        last_time[0] = current_time  # 更新上次调用时间
        return diff  # 返回时间差us

delay_s = 0
delay_s_max = 0

while sw:
    roll, pitch = imu.get_angles(delay_s/1000_000)

    speed_l = encoder_l.get_speed()
    speed_r = encoder_r.get_speed()

    if abs(pitch) > 15 or abs(roll) > 60:
        motor.stop()
        print(f"检测到跌倒, 关闭电机, roll = {roll}, pitch = {pitch}")
        continue

    angle = roll - IMU_OFFSET

    b_pwm = min(int(abs(angle)/45 * 1023), 1023)

    Beep.duty(b_pwm)

    v_pwm = (angle / 45) * 1023
    w_pwm = 0

    v_pwm_pid = -pid.update(v_pwm)

    motor.motion(v_pwm_pid, w_pwm)

    vofa_msg = f"{angle}, {speed_l}, {speed_r}, {v_pwm}, {delay_s},  {motor.BASE_SPEED}, {pid.kp}, {pid.ki}, {pid.kd}"

    print(vofa_msg)

#     if tcp:
#         try:
#             tcp.send(vofa_msg)#.encode('utf-8'))
#         except Exception as e:
#             print("Send Error:", e)

    delay_s = time_diff()

    delay_s_max = max(delay_s_max, delay_s)
    
    # print(f"delay: {delay_s}, delay_s_max: {delay_s_max}")

    time.sleep(0.00001)
    time.sleep(0.1)

