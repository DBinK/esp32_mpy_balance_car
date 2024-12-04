import time

def time_diff(last_time=[None]):
    """计算两次调用之间的时间差，单位为纳秒。"""
    current_time = time.time_ns()  # 获取当前时间（单位：纳秒）

    if last_time[0] is None:  # 如果是第一次调用，更新last_time
        last_time[0] = current_time  # 使用列表可以在函数调用之间保持状态
        return 1  # 防止除零错误，返回1纳秒

    else:  # 计算时间差
        diff = current_time - last_time[0]  # 计算时间差（单位：纳秒）
        last_time[0] = current_time  # 更新上次调用时间
        return diff / 1e6 # 返回时间差（ms）
    

def time_logger(func):  # 定义装饰器
    def wrapper(*args, **kwargs):  # 包装函数
        start_time = time.time_ns()  # 记录开始时间
        result = func(*args, **kwargs)  # 调用原函数
        end_time = time.time_ns()  # 记录结束时间
        dt = end_time - start_time
        print(f"函数 '{func.__name__}' 的运行时间: {dt/1e6} ms, fps: {1/(dt/1e9)} ")  # 打印运行时间
        return result  # 返回原函数的结果
    return wrapper