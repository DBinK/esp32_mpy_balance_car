'''
实验名称：Socket通讯
版本：v1.0
作者：WalnutPi
平台：核桃派PicoW
说明：通过Socket编程实现核桃派PicoW与电脑服务器助手建立TCP连接，相互收发数据。
'''

#导入相关模块
import network,usocket,time
from machine import Pin,Timer

#WIFI连接函数
def WIFI_Connect():

    WIFI_LED=Pin(15, Pin.OUT) #初始化WIFI指示灯

    wlan = network.WLAN(network.STA_IF) #STA模式
    wlan.active(True)                   #激活接口
    start_time=time.time()              #记录时间做超时判断

    if not wlan.isconnected():
        print('Connecting to network...')
        wlan.connect('GD', '00000000') #输入WIFI账号密码

        while not wlan.isconnected():

            #LED闪烁提示
            WIFI_LED.value(1)
            time.sleep_ms(300)
            WIFI_LED.value(0)
            time.sleep_ms(300)

            #超时判断,15秒没连接成功判定为超时
            if time.time()-start_time > 5 :
                print('WIFI Connected Timeout!')
                break

    if wlan.isconnected():
        #LED点亮
        WIFI_LED.value(1)

        #串口打印信息
        print('network information:', wlan.ifconfig())

        return True

    else:
        return False

def check_network():
    if network.WLAN(network.STA_IF).isconnected():
        return True
    else:
        return False

if __name__ == '__main__':

    WIFI_Connect()
    
    #判断WIFI是否连接成功
    if check_network():

        #创建socket连接TCP类似，连接成功后发送“Hello WalnutPi！”给服务器。
        s=usocket.socket()
        addr=('192.168.1.15',1234) #服务器IP和端口
        s.connect(addr)
        s.send('Hello WalnutPi!')

    while True:
        
        text=s.recv(128) #单次最多接收128字节
        if text == '':
            pass

        else: #打印接收到的信息为字节，可以通过decode('utf-8')转成字符串
            print(text)
            s.send('I got:'+text.decode('utf-8'))
        
        time.sleep_ms(300)