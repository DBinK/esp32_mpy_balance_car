'''
实验名称：连接无线路由器
版本：v1.0
作者：WalnutPi
说明：编程实现连接路由器，将IP地址等相关信息打印出来。
'''
import network,time
from machine import Pin

# 释放所有GPIO, 断电重上电不再失控
def release_all_GPIO():
    for i in range(0, 48):
        try:
            GND = Pin(i, Pin.OUT, value=0)
            print(f"releasing gpio {i}")
        except:
            print(f"skip gpio {i}")
            continue

release_all_GPIO()


LED=Pin(15,Pin.OUT) #构建led对象，GPIO46,输出
LED.value(1) #点亮LED，也可以使用led.on()

