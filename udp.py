'''实验名称：UDP通讯
版本：v1.0
作者：WalnutPi
平台：核桃派PicoW
说明：通过Socket编程实现核桃派PicoW与电脑服务器助手建立UDP连接，发送数据。'''

# 导入相关模块
import network
import usocket
import time
from machine import Pin

# WIFI连接函数
def WIFI_Connect():
    WIFI_LED = Pin(8, Pin.OUT)  # 初始化WIFI指示灯
    wlan = network.WLAN(network.STA_IF)  # STA模式
    wlan.active(True)  # 激活接口
    start_time = time.time()  # 记录时间做超时判断
    if not wlan.isconnected():
        print('Connecting to network...')
        wlan.connect('GD', '00000000')  # 输入WIFI账号密码
        while not wlan.isconnected():
            # LED闪烁提示
            WIFI_LED.value(1)
            time.sleep_ms(300)
            WIFI_LED.value(0)
            time.sleep_ms(300)
            # 超时判断,15秒没连接成功判定为超时
            if time.time() - start_time > 15:
                print('WIFI Connected Timeout!')
                break
    if wlan.isconnected():
        # LED点亮
        WIFI_LED.value(1)
        # 串口打印信息
        print('network information:', wlan.ifconfig())
        return True
    else:
        return False

# 判断WIFI是否连接成功
if WIFI_Connect():
    # 创建UDP socket连接
    # 创建UDP socket连接
    s = usocket.socket(usocket.AF_INET, usocket.SOCK_DGRAM)  # 创建UDP socket
    s.settimeout(2.0)  # 设置超时时间为2秒
    addr = ('192.168.1.16', 1346)  # 服务器的IP和端口

    while True:
        # 发送数据
        s.sendto(b'Hello WalnutPi!\n', addr)  # 发送字节数据
        time.sleep(1)  # 每秒发送一次

        try:
            text = s.recv(128)  # 单次最多接收128字节
            # 打印接收到的信息
            print(text.decode('utf-8'))  # 转成字符串
            # 发送确认信息
            s.sendto(b'I got: ' + text, addr)  # 发送确认消息
        except OSError:
            # 超时处理
            print("未接收到数据，继续发送...")
            

