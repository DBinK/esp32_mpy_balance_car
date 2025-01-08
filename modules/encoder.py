from machine import Pin, PWM
import time
import _thread

class HallEncoder:
    def __init__(self, pin_a, pin_b):
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self.position = 0
        self.last_a = self.pin_a.value()
        self.speed = 0  # 初始化速度
        self.last_position = 0  # 上一个位置
        self.last_time = time.ticks_ms()  # 上一次更新时间
        
        # 设置中断
        self.pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.update_position)
        self.pin_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.update_position)

        # 启动线程更新速度
        _thread.start_new_thread(self.calculate_speed, ())

    def update_position(self, pin):
        current_a = self.pin_a.value()
        current_b = self.pin_b.value()
        
        if current_a != self.last_a:  # 只有在A引脚状态变化时才处理
            if abs(self.position) > 100000:  # 超过10000个脉冲，重置位置
                self.position      = 0
                self.last_position = 0
            if current_a == 1:  # A引脚上升沿
                if current_b == 0:  # B引脚为低，顺时针旋转
                    self.position += 1
                else:  # B引脚为高，逆时针旋转
                    self.position -= 1
            self.last_a = current_a

    def calculate_speed(self):
        while True:
            current_time = time.ticks_ms()
            elapsed_time = time.ticks_diff(current_time, self.last_time) / 1000.0  # 转换为秒
            if elapsed_time > 0:
                self.speed = (self.position - self.last_position) / elapsed_time  # 计算速度
                self.last_position = self.position
                self.last_time = current_time
            time.sleep(0.02)  # 每0.02秒更新一次速度

    def get_position(self):
        return self.position

    def get_speed(self):
        return self.speed
    
    def test_max_speed(self):
        while True:
            if self.position > 100000:
                self.position = 0
                self.last_position = 0
                print("Max Speed:", self.speed)

# 示例使用
if __name__ == "__main__":
    
#     PWM = PWM(Pin(5), freq=50, duty=100)
#     # VCC = Pin(5,Pin.OUT,value=1) 
#     GND = Pin(6,Pin.OUT,value=0) 
    

    encoder_l = HallEncoder(4,6)
    encoder_r = HallEncoder(2,1)
    
    try:
        while True:
            print("Encoder L Position:", encoder_l.get_position(), "Speed:", encoder_l.get_speed())
            print("Encoder R Position:", encoder_r.get_position(), "Speed:", encoder_r.get_speed())
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        print("程序结束")