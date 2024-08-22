import time
import usocket
import _thread
from machine import UART

from imu import Accel
from encoder import HallEncoder
from motor import Motor
from pid import PID

IMU_OFFSET = 4.0
BASE_PWM = 20

imu = Accel(9, 8)
encoder_l = HallEncoder(4,6)
encoder_r = HallEncoder(2,1)
motor = Motor(33,35,18,16, BASE_PWM)

pid = PID(kp=1.0, ki=0.0, kd=0.0, setpoint=0, output_limits=(-1023, 1023))

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

delay_s = 0
delay_s_max = 0
last_time = time.ticks_us()

while True:

    start = time.ticks_us()
    
    delay_s = (start - last_time) / 1000000

    delay_s_max = max(delay_s_max, delay_s)
    
    last_time = start
    
    roll, pitch = imu.get_angles()

    speed_l = encoder_l.get_speed()
    speed_r = encoder_r.get_speed()

    if abs(pitch) > 15 or abs(roll) > 60:
        motor.stop()
        print(f"检测到跌倒, 关闭电机, roll = {roll}, pitch = {pitch}")
        continue

    angle = roll - IMU_OFFSET

    v_pwm = (angle / 90) * 1023
    w_pwm = 0

    v_pwm_pid = -pid.update(v_pwm)

    motor.motion(v_pwm_pid, w_pwm)

    vofa_msg = f"{angle}, {speed_l}, {speed_r}, {v_pwm}, {delay_s},  {motor.BASE_SPEED}, {pid.kp}, {pid.ki}, {pid.kd} \n"

    print(vofa_msg)

#     if tcp:
#         try:
#             tcp.send(vofa_msg)#.encode('utf-8'))
#         except Exception as e:
#             print("Send Error:", e)

    time.sleep(0.00001)

    end = time.ticks_us()


    #print(f"delay: {delay_s}, delay_s_max: {delay_s_max}")
