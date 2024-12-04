from machine import SoftI2C, Pin
import math
import time
import _thread

class KalmanFilter:
    def __init__(self):
        self.angle = 0.0
        self.bias = 0.0
        self.rate = 0.0
        self.P = 0.0
        self.Q = 0.001  # 过程噪声
        self.R = 0.003  # 量测噪声

    def get_angle(self, new_angle, new_rate, dt):
        self.rate = new_rate + self.bias
        self.angle += self.rate * dt

        self.P += self.Q
        S = self.P + self.R
        K = self.P / S
        self.angle += K * (new_angle - self.angle)
        self.P -= K * self.P

        return self.angle

class Accel:
    def __init__(self, scl_pin, sda_pin, addr=0x68):
        self.iic = SoftI2C(scl=Pin(scl_pin), sda=Pin(sda_pin))
        self.addr = addr
        self.iic.start()
        self.iic.writeto(self.addr, bytearray([107, 0]))  # 唤醒 MPU6050
        self.iic.stop()
        
        # 初始化卡尔曼滤波器
        self.kalmanX = KalmanFilter()
        self.kalmanY = KalmanFilter()
        self.kalmanZ = KalmanFilter()

        # 初始化角度值
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0

        self.rolls  = []
        self.pitchs = []

        # 启动线程更新角度
        #_thread.start_new_thread(self.update_angles, ())

        # 初始化时间
        self.dt = 0

    def get_raw_values(self):
        self.iic.start()
        a = self.iic.readfrom_mem(self.addr, 0x3B, 14)
        self.iic.stop()
        return a

    def bytes_toint(self, firstbyte, secondbyte):
        if not firstbyte & 0x80:
            return firstbyte << 8 | secondbyte
        return - (((firstbyte ^ 255) << 8) | (secondbyte ^ 255) + 1)

    def get_values(self):
        raw_ints = self.get_raw_values()
        vals = {}
        vals["AcX"] = self.bytes_toint(raw_ints[0], raw_ints[1])
        vals["AcY"] = self.bytes_toint(raw_ints[2], raw_ints[3])
        vals["AcZ"] = self.bytes_toint(raw_ints[4], raw_ints[5])
        vals["Tmp"] = self.bytes_toint(raw_ints[6], raw_ints[7]) / 340.00 + 36.53
        vals["GyX"] = self.bytes_toint(raw_ints[8], raw_ints[9])
        vals["GyY"] = self.bytes_toint(raw_ints[10], raw_ints[11])
        vals["GyZ"] = self.bytes_toint(raw_ints[12], raw_ints[13])
        return vals  # 返回原始值
    
    def window_filter(self, value, values, window_size=50):
        # 将值添加到列表中
        values.append(value)

        # 如果列表长度超过窗口大小，则移除最旧的元素
        if len(values) > window_size:
            values.pop(0)

        # 返回平均值
        return sum(values) / len(values)
    
    def time_diff(self, last_time=[None]):
        """计算两次调用之间的时间差，单位为微秒。"""
        current_time = time.ticks_us()  # 获取当前时间（单位：微秒）

        if last_time[0] is None: # 如果是第一次调用，更新last_time
            last_time[0] = current_time
            return 0.000_001 # 防止除零错误
        
        else: # 计算时间差
            diff = time.ticks_diff(current_time, last_time[0])  # 计算时间差
            last_time[0] = current_time  # 更新上次调用时间
            return diff  # 返回时间差us

    def update_angles(self):
        while True:
            dt = self.time_diff()
            self.get_angles(dt)
            print(f"Roll: {self.roll:.2f}, Pitch: {self.pitch:.2f}, dt: {dt:.6f}")
            
            time.sleep(0.000005)  # 每次更新间隔

    def get_angles(self, dt):
        vals = self.get_values()
        acc_x = vals["AcX"]
        acc_y = vals["AcY"]
        acc_z = vals["AcZ"]
        gyro_x = vals["GyX"] / 131.0
        gyro_y = vals["GyY"] / 131.0
        gyro_z = vals["GyZ"] / 131.0

        # 计算roll
        if acc_z != 0:
            roll = math.atan2(acc_y, acc_z) * 180 / math.pi
        else:
            roll = 0

        # 计算pitch
        if acc_y**2 + acc_z**2 != 0:
            pitch = math.atan(-acc_x / math.sqrt(acc_y**2 + acc_z**2)) * 180 / math.pi
        else:
            pitch = 0

        # self.roll = roll
        # self.pitch = pitch

        # 计算yaw 
        # if acc_x != 0:
        #     yaw = math.atan2(acc_y, acc_x) * 180 / math.pi
        # else:
        #     yaw = 0

        # self.yaw = self.kalmanZ.get_angle(yaw, gyro_z, dt)

        
        self.roll  = self.kalmanX.get_angle( roll, gyro_x, dt)
        self.pitch = self.kalmanY.get_angle(pitch, gyro_y, dt)
        
        # self.roll  = self.window_filter(roll,  self.rolls, 3)
        # self.pitch = self.window_filter(pitch, self.pitchs, 3)

        return self.roll, self.pitch
    

if __name__ == "__main__":

    mpu = Accel(9, 8)
    
    delay_s = 0
    delay_s_max = 0

    while True:
        roll, pitch = mpu.get_angles(delay_s/1_000_000)

        delay_s = mpu.time_diff()
        delay_s_max = max(delay_s_max, delay_s)
        
        print(f"roll: {roll:.2f}, pitch: {pitch:.2f}, delay_us: {delay_s}, delay_s_max: {delay_s_max}")

        time.sleep(0.00001)