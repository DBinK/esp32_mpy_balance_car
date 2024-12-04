'''实验名称：UDP通讯
版本：v1.0
作者：WalnutPi
平台：核桃派PicoW
说明：通过Socket编程实现核桃派PicoW与电脑服务器助手建立TCP连接，发送数据。'''

# 导入相关模块
import network
import usocket
import time
from machine import Pin

# WIFI连接函数
def WIFI_Connect():
    WIFI_LED = Pin(15, Pin.OUT)  # 初始化WIFI指示灯
    wlan = network.WLAN(network.STA_IF)  # STA模式
    wlan.active(True)  # 激活接口
    start_time = time.time()  # 记录时间做超时判断
    if not wlan.isconnected():
        print('Connecting to network...')
        # wlan.connect('GD', '00000000')  # 输入WIFI账号密码
        wlan.connect('DT46', '12345678')  # 输入WIFI账号密码
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
        # 串口打印信息
        print('network information:', wlan.ifconfig())
        # LED点亮
        WIFI_LED.value(0)
        return True
    else:
        return False

# 判断WIFI是否连接成功
if WIFI_Connect():
    
    addr = ('192.168.1.116', 1347)  # 服务器的IP和端口

    sock = usocket.socket()  
    # s = usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
    sock.connect(addr)
    sock.settimeout(3.0)  # 设置超时时间为2秒
    
    time.sleep(3)

    while True:
        # 发送数据
        sock.sendto(b'Hello WalnutPi!\n', addr)  # 发送字节数据
        time.sleep(1)  # 每秒发送一次

        try:
            text = sock.recv(128)  # 单次最多接收128字节
            # 打印接收到的信息
            print(text.decode('utf-8'))  # 转成字符串
            # 发送确认信息
            sock.sendto(b'I got: ' + text, addr)  # 发送确认消息
        except OSError:
            # 超时处理
            print("未接收到数据，继续发送...")
            
    sock.close()
