# *******************************************************************************
# 更新日期:2022.6.15
# 版本:V2.0.6
# 固件:V1.4.4
# 说明:本模块用于ZMROBO各种底层接口功能实现、版本更新功能等
# 更新内容:
#     1、进/线程机制重新修改调整
#     2、返回值元素修改为int

# *******************************************************************************
# 更新日期:2022.6.13
# 版本:V2.0.5
# 固件:V1.4.4
# 说明:本模块用于ZMROBO各种底层接口功能实现、版本更新功能等
# 更新内容:
#     1、修复2.0.4接口get_motors_code 3端口的编码值永远打印为0，motor_servo_unlock/motors_servo_unlock名称不对应文档
#     2、返回错误码机制重新调整
#     3、增加进/线程锁     

# *******************************************************************************
# 更新日期:2022.6.8
# 版本:V2.0.4
# 说明:本模块用于ZMROBO各种底层接口功能实现、版本更新功能等
# 更新内容:
#     1、修复V2.0.3的bug
#     2、upgrade接口重新构建

# *******************************************************************************
# 更新日期:2022.6.7
# 版本:V2.0.3
# 说明:本模块用于ZMROBO各种底层接口功能实现、版本更新功能等
# 更新内容:
#     更换函数名称;
#     新增功能函数:
#         set_photo_electric_threshold 设置port端口的光电传感器阈值       新增加
#         supplement_light_open 打开光电传感器中的补光灯                  新增加
#         supplement_light_close 关闭光电传感器中的补光灯                 新增加
#         set_motor_speed 异步设置port端口的马达传感器速度                新增加
#         set_motors_speed 异步设置ports端口列表的马达传感器速度          新增加
#         set_motor_speed_pid                                          未实现
#         set_motors_speed_pid                                         未实现
#         set_motor_speed_pid_asyn                                     未实现
#         set_motors_speed_pid_asyn                                    未实现
#         set_motor_servo  同步设置port端口列表的马达传感器伺服角度       未实现
#         set_motors_servo 同步设置ports端口列表的马达传感器伺服角度       未实现
#         set_motor_servo_asyn 异步设置port端口的马达传感器伺服角度        新增加
#         set_motors_servo_asyn 异步设置ports端口列表的马达传感器伺服角度  新增加
#         set_motor_servo_unlock                                        未实现
#         set_motors_servo_unlock                                       未实现
#         motor_servo_lock                                              未实现
#         motors_servo_lock                                             未实现
#         motor_servo_unlock 马达伺服模式解锁单个端口                     新增加
#         motors_servo_unlock 马达伺服模式解锁多个端口                    新增加
#         clear_motor_code 清理port端口列表的马达传感器编码               新增加
#         clear_motors_code 清理ports端口列表的马达传感器编码             新增加
#         get_motor_code 获取port端口的马达传感器编码                     新增加
#         get_motors_code 获取ports端口列表的马达传感器编码               新增加
#         set_steering 异步设置舵机传感器伺服角度                         新增加
#         set_steerings 异步设置舵机传感器伺服角度                        新增加
#         set_steering_unlock 异步设置舵机伺服角度不上锁                  新增加
#         set_steerings_unlock 异步设置多个舵机伺服角度不上锁             新增加 
#         steering_unlock  解锁port端口舵机                              新增加
#         steerings_unlock 同时解锁ports端口列表舵机                      新增加
#         set_color_lights 设置彩灯传感器列表颜色                         新增加
#         error  查看错误代码信息                                         新增加
#         errnos 查看错误代码                                             新增加

# *******************************************************************************
# """
# *******************************************************************************
# 更新日期:2022.4.28
# 版本:V2.0.2
# 说明:本模块用于ZMROBO各种底层接口功能实现、版本更新功能等
# 更新内容:新增更新固件功能、查询单片机型号功能
# 注意事项:
# 1、增加多端口读传感器数值接口,如photo_electric_values等
# 2、查询所有端口是否有指定传感器功能暂未实现,接口都是空接口,请注意,如photoElectricList
# *******************************************************************************
# """
# *******************************************************************************
# 更新日期:2022.4.27
# 版本:V2.0.1
# 说明:本模块用于ZMROBO各种底层接口功能实现、版本更新功能等
# 更新内容:****
# 注意事项:
# 1、增加多端口读传感器数值接口,如photo_electric_values等
# 2、查询所有端口是否有指定传感器功能暂未实现,接口都是空接口,请注意,如photoElectricList
# *******************************************************************************
# 首次创建日期:2022.4.15
# 重构版本:V2.0.0
# 规范:
# 1、接口名称标准格式:按照首字母小写,后续字母大写的命名规则,在每个接口源码前必须注释接口的作用,格式 -- 名称:功能说明
# 2、代码格式规范:为方便阅读代码修复bug
# ⑴发送的数据命令换行必须保持对齐
# ⑵符号两边都必须空格,如"= * + 等" ,","后面必须空一格
# ⑶不同函数之间必须空行再往下写
# ⑷其他标准严格按照python代码规范标准执行
# 3、第一版完成之后新增接口或者修复功能必须注明新增或修复接口说明并注明日期和版本号
# 4、接口新增必须按照文档的顺序和所属类别进行排序,并在接口抬头注释增加进所属类别
# 5、发布文件名称命名格式:ZMSensors_V2.0.0
# 6、其他未尽事宜有需要再补充
# *******************************************************************************
# -------------风扇----------------
# fan_speed:                        查询风扇的速度
# set_fan_speed:                    设置风扇的速度

# -------------光电----------------
# photo_electric_value:             获取端口光电传感器的值
# photo_electric_values:            获取指定多个端口光电传感器的值
# photoElectricList:                查询所有端口是否有光电传感器
# set_photo_electric_threshold:     设置port端口的光电传感器阈值
# supplement_light_open:            打开光电传感器中的补光灯
# supplement_light_close:           关闭光电传感器中的补光灯

# -------------触碰----------------
# touch_value:                      获取端口触碰传感器的值
# touch_values:                     获取指定多个端口触碰传感器的值
# touchList:                        查询所有端口是否有触碰传感器
# set_photo_electric_threshold:     设置port端口的光电传感器阈值

# -------------颜色----------------
# color_value:                      获取端口颜色传感器RGB的值
# color_values:                     获取指定多个端口颜色传感器RGB的值
# colorList:                        查询所有端口是否有颜色传感器

# -------------超声波---------------
# ultrasonic_value:                 获取端口超声波传感器的值
# ultrasonic_values:                获取指定多个端口超声波传感器的值
# ultrasonicList:                   查询所有端口是否有超声波传感器

# --------------湿度----------------
# humidity_value:                   获取端口湿度传感器的值
# humidity_values:                  获取指定多个端口湿度传感器的值
# humidityList:                     查询所有端口是否有湿度传感器

# --------------温度----------------
# temperature_value:                获取端口温度传感器的值
# temperature_values:               获取指定多个端口温度传感器的值
# temperatureList:                  查询所有端口是否有温度传感器

# --------------电机----------------
# set_motor_speed:                  设置电机的速度
# set_motors_speed:                 设置多个电机的速度
# clear_motor_code:                 清除端口电机编码
# clear_motors_code:                清除多个端口电机编码
# get_motor_code:                   获取端口电机的值
# get_motors_code:                  获取指定多个端口电机的值
# motorList:                        查询所有端口是否有电机
# set_motor_servo_asyn:             异步设置port端口的电机传感器伺服角度
# set_motors_servo_asyn:            异步设置ports端口列表的电机传感器伺服角度
# motor_servo_unlock:               电机伺服模式解锁单个端口 
# motors_servo_unlock:              电机伺服模式解锁多个端口

# 未实现：
# set_motor_speed_pid               未实现
# set_motors_speed_pid              未实现
# set_motor_speed_pid_asyn          未实现
# set_motors_speed_pid_asyn         未实现
# set_motor_servo                   未实现
# set_motors_servo                  未实现
# set_motor_servo_unlock            设置单个电机伺服角度不锁 
# set_motors_servo_unlock           设置多个电机伺服角度不锁
# motor_servo_lock                  未实现
# motors_servo_lock                 未实现

# ---------------舵机---------------
# set_steering:                     设置舵机伺服角度
# set_steerings:                    设置多个舵机伺服角度
# steeringList:                     查询所有端口是否有舵机
# set_steering_unlock:              转动舵机不上锁                              
# set_steerings_unlock:             转动多个舵机不上锁
# steering_unlock:                  解锁port端口舵机
# steerings_unlock:                 同时解锁ports端口列表舵机

# --------------指南针--------------
# compass_value:                    获取指南针的值
# compass_values:                   获取指定多个指南针的值
# compassList:                      查询所有端口是否有指南针

# --------------陀螺仪--------------
# gyroscope_value:                  获取陀螺仪传感器的值
# gyroscope_values:                 获取指定多个陀螺仪传感器的值
# gyroscope_init:                   陀螺仪校准
# gyroscopeList:                    查询所有端口是否有陀螺仪传感器

# ---------------彩灯---------------
# 0:关灯 1:红灯 2:绿灯 3:蓝灯 4:黄灯 5:紫灯 6:青灯 7:白灯 
# set_color_light:                  设置彩灯的颜色
# set_color_lights:                 设置彩灯传感器列表颜色

# ---------------固件---------------
# fw_version:                       查询固件版本号

# ---------------电池---------------
# battery:                          查询电池电量

# -------------端口复位-------------
# reset:                            端口复位

# -------------固件更新-------------
# upgrade:                          固件更新

# ----------查询单片机型号----------
# chip:                              查询单片机型号
# *******************************************************************************

from cgitb import reset
import errno
import serial
import time
import threading
import fcntl
from distutils.log import error
from lib2to3.pytree import Base
import serial
import time
import os
import serial
import time
import os
import struct
import math
import requests
import re
import sys
import tarfile
import subprocess

errors_list = []
errors = {
0:"成功",

1001:"获取数值速度失败",
1101:"风扇设置失败",

2001:"获取光电传感器数值失败",
2002:"获取多个光电传感器数值失败",
2101:"设置光电传感器数值失败",
2102:"设置光电传感器阈值失败",
2103:"打开光电传感器中的补光灯失败",
2104:"关闭光电传感器中的补光灯失败",
2201:"查询单个端口是否有光电传感器失败",
2202:"查询多个端口是否有光电传感器失败",

3001:"获取触碰传感器数值失败",
3002:"获取多个端口触碰传感器数值失败",
3102:"设置触碰传感器数值失败",
3201:"查询指定端口是否有触碰传感器失败",
3202:"查询所有端口是否有触碰传感器失败",

4001:"获取颜色传感器数值失败",
4002:"获取多个端口颜色传感器数值失败",
4101:"设置颜色传感器数值失败",
4201:"查询端口是否有颜色传感器失败",
4202:"查询多个端口是否有颜色传感器失败",

5001:"获取超声波传感器数值失败",
5002:"获取多个端口超声波传感器数值失败",
5101:"设置超声波传感器数值失败",
5201:"查询端口是否有超声波传感器失败",
5202:"查询多个端口是否有超声波传感器失败",

6001:"获取温度传感器数值失败",
6002:"获取多个温度传感器数值失败",
6201:"查询端口是否有温度传感器失败",
6202:"查询所有端口是否有温度传感器失败",

7001:"获取湿度传感器数值失败",
7002:"获取多个传感器数值失败",
7201:"查询端口是否有湿度传感器失败",
7202:"查询所有端口是否有湿度传感器失败",

8001:"获取电机编码数值失败",
8002:"获取多个电机编码数值失败",
8101:"设置电机角度数值失败",
8102:"设置电机速度值失败",
8103:"设置多个电机速度值失败",
8104:"同步设置多个端口列表的电机传感器伺服角度失败",
8105:"异步设置port端口的电机传感器伺服角度失败 ",
8106:"异步设置ports端口列表的电机传感器伺服角度失败",
8107:"清理port端口列表的电机传感器编码失败 ",
8108:"清理ports端口列表的电机传感器编码失败 ",
8109:"解锁单个电机失败",
8110:"解锁多个电机失败",
8111:"单个不锁地设置电机角度失败",
8112:"多个不锁地设置电机角度失败",
8201:"查询端口是否有电机传感器失败",
8202:"查询所有端口是否有电机传感器失败",

9101:"异步设置伺服舵机数值失败",
9102:"异步设置多个伺服舵机数值失败",
9103:"不上锁异步设置伺服舵机数值失败",
9104:"不上锁异步设置多个伺服舵机数值失败",
9105:"解锁port端口舵机失败",
9106:"同时解锁ports端口列表舵机失败",
9201:"查询所有端口是否有舵机失败",

10001:"获取罗盘传感器数值失败",
10002:"获取多个罗盘传感器数值失败",
10201:"查询端口是否有罗盘传感器失败",
10202:"查询多个端口是否有罗盘传感器失败",

11000:"陀螺仪传感器校正失败",
11001:"获取陀螺仪传感器数值失败",
11002:"获取多个陀螺仪传感器数值失败",
11201:"查询端口是否有陀螺仪传感器失败",
11202:"查询所有端口是否有陀螺仪传感器失败",

12101:"设置彩灯颜色数值失败",
12102:"设置多个彩灯颜色数值失败",

13201:"查询固件版本号失败",

14201:"查询电池电量数值失败",

15201:"查询充电状态失败",

16100:"重置传感器失败",

17201:"单片机型号查询失败s",

18001:"读取电池电压失败",

20001:"升级固件数值失败",
}
    
class StormSensors(object):
    def __init__(self, serial_name = "/dev/ttyAMA1", serial_bps = 921600, serial_timeout = 0.05):
        super().__init__()
        
        try:
             self.ser = serial.Serial(serial_name, serial_bps, timeout = serial_timeout)
        except BaseException as err:
            self.ser = None
            print('{}串口初始化错误:{}'.format(serial_name, err))

        self.lock = threading.Lock()

        self.port_lock = threading.Lock()

        self.zm_path = "/tmp/rcu.lock"

        self.zm_err = 0

    # 清除串口数据    
    def clear_serial(self):
        self.ser.flushInput()
        self.ser.flushOutput()

    #执行
    def write_serial(self, cmd):
        try:
            self.lock.acquire()#线程锁
            fcntl.lockf(self.ser, fcntl.LOCK_EX)#文件锁
            self.clear_serial()
            self.ser.write(cmd)
        except:
            print('串口异常')
        finally:
            fcntl.lockf(self.ser, fcntl.LOCK_UN)#文件解锁
            self.lock.release()#线程解锁
    #查看
    def write_read_serial(self, cmd, bytes_size):
        RECV = []
        try:
            self.lock.acquire()
            fcntl.lockf(self.ser, fcntl.LOCK_EX)
            self.clear_serial()
            self.ser.write(cmd)
            RECV = self.ser.read(bytes_size)
        except:
            print('串口异常')
        finally:
            fcntl.lockf(self.ser, fcntl.LOCK_UN)
            self.lock.release()
            return RECV

    #异步执行
    def write_serial_asyn(self, cmd, types):
        try:
            self.lock.acquire()#线程锁
            fcntl.lockf(self.ser, fcntl.LOCK_EX)#文件锁
            self.clear_serial()
            self.ser.write(cmd)
            self.ser.flush()
            time.sleep(0.005)
            data = self.ser.inWaiting()
            atime = 0
            # print(types,"****")
            if types == 1:
                while data == 0 and atime < 20:
                    data = self.ser.inWaiting()
                    time.sleep(0.015)
                    atime += 1
                if data != 0:
                    RECV =self.ser.read(data)
                    if RECV[10] == ord("O") and RECV[11] == ord("K"):
                        return 1    
                    else:
                        return 0
                else:
                    return 0
            elif types == 2:
                while data == 0 and atime < 20:
                    data = self.ser.inWaiting()
                    time.sleep(0.015)
                    atime += 1
                if data != 0:
                    RECV =self.ser.read(data)
                    if RECV[10] == ord("O") and RECV[11] == ord("K") and RECV[17] == ord("O") and RECV[18] == ord("K"):
                        return 1    
                    else:
                        return 0
                else:
                    return 0
            elif types == 3:
                while data == 0 and atime < 20:
                    data = self.ser.inWaiting()
                    time.sleep(0.015)
                    atime += 1
                if data != 0:
                    RECV =self.ser.read(data)
                    if RECV[10] == ord("O") and RECV[11] == ord("K") and RECV[17] == ord("O") and RECV[18] == ord("K") and RECV[24] == ord("O") and RECV[25] == ord("K"):
                        return 1    
                    else:
                        return 0
                else:
                    return 0
            elif types == 4:
                while data == 0 and atime < 20:
                    data = self.ser.inWaiting()
                    time.sleep(0.015)
                    atime += 1
                if data != 0:
                    RECV =self.ser.read(data)
                    if RECV[10] == ord("O") and RECV[11] == ord("K") and RECV[17] == ord("O") and RECV[18] == ord("K") and RECV[24] == ord("O") and RECV[25] == ord("K") and RECV[31] == ord("O") and RECV[32] == ord("K"):  
                        return 1    
                    else:
                        return 0
                else:
                    return 0
            elif types == 5:
                while data == 0 and atime < 20:
                    data = self.ser.inWaiting()
                    time.sleep(0.015)
                    atime += 1
                if data != 0:
                    RECV =self.ser.read(data)
                    if RECV[10] == ord("O") and RECV[11] == ord("K") and RECV[17] == ord("O") and RECV[18] == ord("K") and RECV[24] == ord("O") and RECV[25] == ord("K") and RECV[31] == ord("O") and RECV[32] == ord("K") and RECV[38] == ord("O") and RECV[39] == ord("K"):
                        return 1    
                    else:
                        return 0
                else:
                    return 0
            elif types == 6:
                while data == 0 and atime < 20:
                    data = self.ser.inWaiting()
                    time.sleep(0.015)
                    atime += 1
                if data != 0:
                    RECV =self.ser.read(data)
                    if RECV[10] == ord("O") and RECV[11] == ord("K") and RECV[17] == ord("O") and RECV[18] == ord("K") and RECV[24] == ord("O") and RECV[25] == ord("K") and RECV[31] == ord("O") and RECV[32] == ord("K") and RECV[38] == ord("O") and RECV[39] == ord("K") and RECV[45] == ord("O") and RECV[46] == ord("K"):
                        return 1    
                    else:
                        return 0
                else:
                    return 0
            elif types == 7:
                while data == 0 and atime < 20:
                    data = self.ser.inWaiting()
                    time.sleep(0.015)
                    atime += 1
                if data != 0:
                    RECV =self.ser.read(data)
                    if RECV[10] == ord("O") and RECV[11] == ord("K") and RECV[17] == ord("O") and RECV[18] == ord("K") and RECV[24] == ord("O") and RECV[25] == ord("K") and RECV[31] == ord("O") and RECV[32] == ord("K") and RECV[38] == ord("O") and RECV[39] == ord("K") and RECV[45] == ord("O") and RECV[46] == ord("K") and RECV[52] == ord("O") and RECV[53] == ord("K"):
                        return 1    
                    else:
                        return 0
                else:
                    return 0
            elif types == 8:
                while data == 0 and atime < 20:
                    data = self.ser.inWaiting()
                    time.sleep(0.015)
                    atime += 1
                if data != 0:
                    RECV =self.ser.read(data)
                    if RECV[10] == ord("O") and RECV[11] == ord("K") and RECV[17] == ord("O") and RECV[18] == ord("K") and RECV[24] == ord("O") and RECV[25] == ord("K") and RECV[31] == ord("O") and RECV[32] == ord("K") and RECV[38] == ord("O") and RECV[39] == ord("K") and RECV[45] == ord("O") and RECV[46] == ord("K") and RECV[52] == ord("O") and RECV[53] == ord("K") and RECV[59] == ord("O") and RECV[60] == ord("K"):
                        return 1   
                    else:
                        return 0
                else:
                    return 0
        except BaseException as err:
            print(err)
            print('串口异常')
            return 0
        finally:
            fcntl.lockf(self.ser, fcntl.LOCK_UN)#文件解锁
            self.lock.release()#线程解锁

    # fan_speed -- 查询风扇的速度 -- int -- 实时返回当前风扇的速度值
    def fan_speed(self):
        zm_err = 0
        zm_num = None
        zm_result = []
        zm_result.append(zm_err)
        try:
            self.port_lock.acquire()
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            # print("1111")
            fan_speedCmd = [0x86, 0xAB, 0x00, 0x09, 0xFF, 0x0A, 0x01, 0x00, 0xCF]
            RECV = []
            while len(RECV) != 10:
                # print("222")
                RECV = self.write_read_serial(fan_speedCmd, 10)
            zm_num = RECV[6]
        except BaseException as err:
            err_no = 1001
            zm_num = None
            errors_list.append(err_no)
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # set_fan_speed -- 设置风扇的速度 
    def set_fan_speed(self, speed):
        zm_err = 0
        if not isinstance(speed, int):
            return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)  
            set_fan_speedCmd = [0x86, 0xAB, 0x00, 0x0A, 0x02, 0x00, 0x00, 0x00, 0x00, 0xCF]
            speed = max(min(speed, 100), 0)
            set_fan_speedCmd[6] = speed
            # self.write_serial(set_fan_speedCmd)
            num = self.write_serial_asyn(set_fan_speedCmd,1)
            if not num :
                zm_err = 1101
        except BaseException as err:
            #exit(1)
            errors_list.append(1101)
            zm_err = 1101
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    # photo_electric_value -- 获取端口光电传感器的值
    def photo_electric_value(self, port):
        zm_err = 0
        zm_num = None
        zm_result = []
        zm_result.append(zm_err)
        if not isinstance(port, int) or port < 1 or port > 8:
                zm_err = 2001
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            # a = self.photoElectricList()
            # if len(a) == 0:
            #     return None
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            photo_electric_valueCmd = [0x86, 0xAB, 0x00, 0x29, 0x01, 0x02,
                                     0xA1, 0x02, 0x01, 0xBE,
                                     0xA2, 0x02, 0x01, 0xBE,
                                     0xA3, 0x02, 0x01, 0xBE,
                                     0xA4, 0x02, 0x01, 0xBE,
                                     0xA5, 0x02, 0x01, 0xBE,
                                     0xA6, 0x02, 0x01, 0xBE,
                                     0xA7, 0x02, 0x01, 0xBE,
                                     0xA8, 0x02, 0x01, 0xBE, 
                                     0x01, 0x00, 0xCF]
            RECV = []
            while len(RECV) != 65:
                RECV = self.write_read_serial(photo_electric_valueCmd, 65)
            ldata1 = RECV[10] * 256 + RECV[11]
            ldata2 = RECV[17] * 256 + RECV[18]
            ldata3 = RECV[24] * 256 + RECV[25]
            ldata4 = RECV[31] * 256 + RECV[32]
            ldata5 = RECV[38] * 256 + RECV[39]
            ldata6 = RECV[45] * 256 + RECV[46]
            ldata7 = RECV[52] * 256 + RECV[53]
            ldata8 = RECV[59] * 256 + RECV[60]
            data = {1:ldata1, 2:ldata2, 3:ldata3, 4:ldata4, 5:ldata5, 6:ldata6, 7:ldata7, 8:ldata8}
            zm_num = data[port]
        except BaseException as err:
            #exit(1)
            errors_list.append(2001)
            zm_num = None
            zm_err = 2001
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    #获取指定多个端口光电传感器的值_new
    def photo_electric_values(self, ports = []):
        zm_err = 0
        zm_num = None
        zm_result = []
        zm_result.append(zm_err)
        for i in range(len(ports)):
            if not isinstance(ports[i], int) or ports[i] < 1 or ports[i] > 8 :
                zm_err = 2002
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            photoElectricValue_list = []
            for i in ports:photoElectricValue_list.append(i)
            photoElectric = 0
            for i in range(len(ports)):
                if ports[i] < 1 or ports[i] > 8:
                    photoElectricValue_list[i] = None
                if self.photoElectric(ports[i]):
                    photoElectric +=1
            if photoElectric == 0: 
                zm_err = 2002
                zm_result.append(zm_num)
                return zm_err, zm_num
            photoElectricValueCmd = [0x86, 0xAB, 0x00, 0x29, 0x01, 0x02,
                                     0xA1, 0x02, 0x01, 0xBE,
                                     0xA2, 0x02, 0x01, 0xBE,
                                     0xA3, 0x02, 0x01, 0xBE,
                                     0xA4, 0x02, 0x01, 0xBE,
                                     0xA5, 0x02, 0x01, 0xBE,
                                     0xA6, 0x02, 0x01, 0xBE,
                                     0xA7, 0x02, 0x01, 0xBE,
                                     0xA8, 0x02, 0x01, 0xBE, 
                                     0x01, 0x00, 0xCF]
            RECV = []
            while len(RECV) != 65:
                RECV = self.write_read_serial(photoElectricValueCmd, 65)
            ldata1 = RECV[10] * 256 + RECV[11]
            ldata2 = RECV[17] * 256 + RECV[18]
            ldata3 = RECV[24] * 256 + RECV[25]
            ldata4 = RECV[31] * 256 + RECV[32]
            ldata5 = RECV[38] * 256 + RECV[39]
            ldata6 = RECV[45] * 256 + RECV[46]
            ldata7 = RECV[52] * 256 + RECV[53]
            ldata8 = RECV[59] * 256 + RECV[60]
            data = {1:ldata1, 2:ldata2, 3:ldata3, 4:ldata4, 5:ldata5, 6:ldata6, 7:ldata7, 8:ldata8}
            for i in range(len(photoElectricValue_list)):
                if photoElectricValue_list[i] != None:
                    a = int(photoElectricValue_list[i])
                    photoElectricValue_list[i] = data[a]
            zm_num = photoElectricValue_list
        except BaseException as err:
            #exit(1)
            errors_list.append(2002)
            zm_err = 2002
            zm_num = []
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num


    # photoElectricList:查询单个端口是否有光电传感器 返回布尔型:真假 _new
    def photoElectric(self, port):
        if not isinstance(port, int) or port < 1 or port > 8:
                return False
        try:
            # photoElectricListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF]
            # data = []
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(photoElectricListCmd, 65)
            # data = RECV[port+5]
            # if data == 0x02 :
            #     return True
            # else:return False
            return True
        except BaseException as err:
            errors_list.append(2201)
            #exit(1)
            return False

    # photoElectricList:查询所有端口是否有光电传感器
    def photoElectricList(self):
        try:
            # photoElectricListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF]
            # data = []
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(photoElectricListCmd, 65)
            # j = 0
            # for i in RECV[6:14]:
            #     j+=1
            #     if i == 0x02:data.append(j)
            # return data
            return []
        except BaseException as err:
            #exit(1)
            errors_list.append(2202)
            return None
     
    # touch_value:获取端口触碰传感器的值
    def touch_value(self, port):
        zm_err = 0
        zm_num = None
        zm_result = []
        zm_result.append(zm_err)
        if not isinstance(port, int) or port < 1 or port > 8:
                zm_err = 3001
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            touch_valueCmd = [0x86, 0xAB, 0x00, 0x29, 0x01, 0x02,
                             0xA1, 0x03, 0x01, 0xBE,
                             0xA2, 0x03, 0x01, 0xBE,
                             0xA3, 0x03, 0x01, 0xBE,
                             0xA4, 0x03, 0x01, 0xBE,
                             0xA5, 0x03, 0x01, 0xBE,
                             0xA6, 0x03, 0x01, 0xBE,
                             0xA7, 0x03, 0x01, 0xBE,
                             0xA8, 0x03, 0x01, 0xBE,
                             0x01, 0x00, 0xCF]
            RECV = []
            while len(RECV) != 57:
                self.clear_serial()
                self.ser.write(touch_valueCmd)
                RECV = self.ser.read(57)
            ldata1 = RECV[10]
            ldata2 = RECV[16]
            ldata3 = RECV[22]
            ldata4 = RECV[28]
            ldata5 = RECV[34]
            ldata6 = RECV[40]
            ldata7 = RECV[46]
            ldata8 = RECV[52]
            data = {1:ldata1, 2:ldata2, 3:ldata3, 4:ldata4, 5:ldata5, 6:ldata6, 7:ldata7, 8:ldata8}
            zm_num = data[port]
        except BaseException as err:
            #exit(1)
            errors_list.append(3001)
            zm_num = None
            zm_err = 3001
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # touch_values:获取指定多个端口触碰传感器的值 
    def touch_values(self, ports = []):
        zm_err = 0
        zm_num = []
        zm_result = []
        zm_result.append(zm_err)
        for i in range(len(ports)):
            if not isinstance(ports[i], int) or ports[i] < 1 or ports[i] > 8 :
                zm_err = 3002
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            touchValue_list = []
            for i in ports:touchValue_list.append(i)
            touch = 0
            for i in range(len(ports)):
                if ports[i] < 1 or ports[i] > 8:
                    touchValue_list[i] = None
                if self.touch(ports[i]):
                    touch += 1
            if touch == 0: 
                zm_err = 3002
                zm_result.append(zm_num)
                # 解锁文件锁
                fcntl.lockf(zm_fd, fcntl.LOCK_UN)
                # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
                if self.port_lock.locked():
                    self.port_lock.release()
                # 关闭文件
                os.close(zm_fd)
                # 返回返回值
                return zm_err, zm_num
            touchValueCmd = [0x86, 0xAB, 0x00, 0x29, 0x01, 0x02,
                             0xA1, 0x03, 0x01, 0xBE,
                             0xA2, 0x03, 0x01, 0xBE,
                             0xA3, 0x03, 0x01, 0xBE,
                             0xA4, 0x03, 0x01, 0xBE,
                             0xA5, 0x03, 0x01, 0xBE,
                             0xA6, 0x03, 0x01, 0xBE,
                             0xA7, 0x03, 0x01, 0xBE,
                             0xA8, 0x03, 0x01, 0xBE,
                             0x01, 0x00, 0xCF]
            RECV = []
            while len(RECV) != 57:
                self.clear_serial()
                self.ser.write(touchValueCmd)
                RECV = self.ser.read(57)
            ldata1 = RECV[10]
            ldata2 = RECV[16]
            ldata3 = RECV[22]
            ldata4 = RECV[28]
            ldata5 = RECV[34]
            ldata6 = RECV[40]
            ldata7 = RECV[46]
            ldata8 = RECV[52]
            data = {1:ldata1, 2:ldata2, 3:ldata3, 4:ldata4, 5:ldata5, 6:ldata6, 7:ldata7, 8:ldata8}
            for i in range(len(touchValue_list)):
                if touchValue_list[i] != None:
                    a = int(touchValue_list[i])
                    touchValue_list[i] = data[a]
            zm_num = touchValue_list
        except BaseException as err:
            #exit(1)
            errors_list.append(3002)
            zm_err = 3002
            zm_num = []
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # touch:查询指定端口是否有触碰传感器 返回布尔型:真假
    def touch(self, port):
        try:
            if port < 1 or port > 8:
                return False
            # touchListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF]
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(touchListCmd, 65)
            # data = RECV[port+5]
            # if data == 0x03:
            #     return True
            # else:
                # return False
            return True
        except BaseException as err:
            #exit(1)
            errors_list.append(3201)
            return False

    # touchList:查询所有端口是否有触碰传感器
    def touchList(self):
        try:
            # touchListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF]
            # data = []
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(touchListCmd, 65)
            # j = 0
            # for i in RECV[6:14]:
            #     j+=1
            #     if i == 0x03:data.append(j)
            # return data
            return []
        except BaseException as err:
            #exit(1)
            errors_list.append(3202)
            return None

    def set_photo_electric_threshold(self,port, threshold):#设置光电阈值，默认1200
        zm_err = 0
        try:
            if not isinstance(port, int) or port < 1 or port > 8:
                zm_err = 2102
                return zm_err
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            self.ser.flushInput()
            self.ser.flushOutput()
            cmd = [0x86, 0xAB, 0x00, 0x00, 0x01, 0x02,
                0xA1, 0x02, 0x04, 0x81, 0x00, 0x00, 0xBE,
                0x01, 0x00, 0xCF]
            cmd[6] = port + 0xA0  #Set port
            cmd[10] = (threshold >> 8) & 0xFF
            cmd[11] = threshold & 0xFF
            num = self.write_serial_asyn(cmd,1)
            if not num :
                zm_err = 2102
        except BaseException as err:
            #exit(1)
            errors_list.append(2102)
            zm_err = 2102
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    # color_value:获取端口颜色传感器RGB的值
    def color_value(self, port):
        zm_err = 0
        zm_num = None
        zm_result = []
        zm_result.append(zm_err)
        if not isinstance(port, int) or port < 1 or port > 8:
                zm_err = 4001
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            color_valueCmd = [0x86, 0xAB, 0x00, 0x29, 0x01, 0x02,
                             0xA1, 0x04, 0x02, 0xBE,
                             0xA2, 0x04, 0x02, 0xBE,
                             0xA3, 0x04, 0x02, 0xBE,
                             0xA4, 0x04, 0x02, 0xBE,
                             0xA5, 0x04, 0x02, 0xBE,
                             0xA6, 0x04, 0x02, 0xBE,
                             0xA7, 0x04, 0x02, 0xBE,
                             0xA8, 0x04, 0x02, 0xBE,
                             0x01, 0x00, 0xCF]
            RECV = []
            while len(RECV) != 81:
                RECV = self.write_read_serial(color_valueCmd, 81)
            ldata1 = [RECV[10], RECV[11], RECV[12], RECV[13]]
            ldata2 = [RECV[19], RECV[20], RECV[21], RECV[22]]
            ldata3 = [RECV[28], RECV[29], RECV[30], RECV[31]]
            ldata4 = [RECV[37], RECV[38], RECV[39], RECV[40]]
            ldata5 = [RECV[46], RECV[47], RECV[48], RECV[49]]
            ldata6 = [RECV[55], RECV[56], RECV[57], RECV[58]]
            ldata7 = [RECV[64], RECV[65], RECV[66], RECV[67]]
            ldata8 = [RECV[73], RECV[74], RECV[75], RECV[76]]
            data = {1:ldata1, 2:ldata2, 3:ldata3, 4:ldata4, 5:ldata5, 6:ldata6, 7:ldata7, 8:ldata8}
            # return 0, data[port]
            zm_num = data[port]
        except BaseException as err:
            #exit(1)
            errors_list.append(4001)
            zm_num = None
            zm_err = 4001
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # color_values:获取指定多个端口颜色传感器RGB的值 _new 
    def color_values(self, ports = []):
        zm_err = 0
        zm_num = []
        zm_result = []
        zm_result.append(zm_err)
        for i in range(len(ports)):
            if not isinstance(ports[i], int) or ports[i] < 1 or ports[i] > 8 :
                zm_err = 2002
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)   
            colorValue_list = []
            for i in ports:colorValue_list.append(i)
            color = 0
            for i in range(len(ports)):
                if ports[i] < 1 or ports[i] > 8:
                    colorValue_list[i] = None
                if self.color(ports[i]):
                    color += 1
            if color == 0: 
                zm_err = 2002
                zm_result.append(zm_num)
                return zm_err, zm_num
            colorValueCmd = [0x86, 0xAB, 0x00, 0x29, 0x01, 0x02,
                             0xA1, 0x04, 0x02, 0xBE,
                             0xA2, 0x04, 0x02, 0xBE,
                             0xA3, 0x04, 0x02, 0xBE,
                             0xA4, 0x04, 0x02, 0xBE,
                             0xA5, 0x04, 0x02, 0xBE,
                             0xA6, 0x04, 0x02, 0xBE,
                             0xA7, 0x04, 0x02, 0xBE,
                             0xA8, 0x04, 0x02, 0xBE,
                             0x01, 0x00, 0xCF]
            RECV = []
            while len(RECV) != 81:
                RECV = self.write_read_serial(colorValueCmd, 81)
            ldata1 = [RECV[10], RECV[11], RECV[12], RECV[13]]
            ldata2 = [RECV[19], RECV[20], RECV[21], RECV[22]]
            ldata3 = [RECV[28], RECV[29], RECV[30], RECV[31]]
            ldata4 = [RECV[37], RECV[38], RECV[39], RECV[40]]
            ldata5 = [RECV[46], RECV[47], RECV[48], RECV[49]]
            ldata6 = [RECV[55], RECV[56], RECV[57], RECV[58]]
            ldata7 = [RECV[64], RECV[65], RECV[66], RECV[67]]
            ldata8 = [RECV[73], RECV[74], RECV[75], RECV[76]]
            data = {1:ldata1, 2:ldata2, 3:ldata3, 4:ldata4, 5:ldata5, 6:ldata6, 7:ldata7, 8:ldata8}
            for i in range(len(colorValue_list)):
                if colorValue_list[i] != None:
                    a = int(colorValue_list[i])
                    colorValue_list[i] = data[a]
            zm_num = colorValue_list
        except BaseException as err:
            #exit(1)
            errors_list.append(4002)
            zm_err = 4002
            zm_num = []
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # color:查询指定端口是否有颜色传感器 _new 布尔型,真假 
    def color(self, port):
        try:
            if port < 1 or port > 8:
                return False
            # colorListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF]
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(colorListCmd, 65)
            # data = RECV[port+5]
            # if data == 0x04:
            #     return True
            # else:
            #     return False
            return True
        except BaseException as err:
            #exit(1)
            errors_list.append(4201)
            return False

    # colorList:查询所有端口是否有颜色传感器
    def colorList(self):
        try:
            # colorListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF]
            # data = []
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(colorListCmd, 65)
            # j = 0
            # for i in RECV[6:14]:
            #     j+=1
            #     if i == 0x04:
            #         data.append(j)
            # return data
            return []
        except BaseException as err:
            #exit(1)
            errors_list.append(4202)
            return None


    # ultrasonic_value:获取端口超声波传感器的值 , port 端口数字
    def ultrasonic_value(self, port):
        zm_err = 0
        zm_num = None
        zm_result = []
        zm_result.append(zm_err)
        if not isinstance(port, int) or port < 1 or port > 8:
                zm_err = 5001
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            ultrasonic_valueCmd = [0x86, 0xAB, 0x00, 0x29, 0x01, 0x02, 
                                  0xA1, 0x06, 0x01, 0xBE,
                                  0xA2, 0x06, 0x01, 0xBE,
                                  0xA3, 0x06, 0x01, 0xBE,
                                  0xA4, 0x06, 0x01, 0xBE,
                                  0xA5, 0x06, 0x01, 0xBE,
                                  0xA6, 0x06, 0x01, 0xBE,
                                  0xA7, 0x06, 0x01, 0xBE,
                                  0xA8, 0x06, 0x01, 0xBE, 
                                  0x01, 0x00, 0xCF]
            RECV = []
            while len(RECV) != 65:
                RECV = self.write_read_serial(ultrasonic_valueCmd, 65)
            ldata1 = RECV[10]*256+RECV[11]
            ldata2 = RECV[17]*256+RECV[18]
            ldata3 = RECV[24]*256+RECV[25]
            ldata4 = RECV[31]*256+RECV[32]
            ldata5 = RECV[38]*256+RECV[39]
            ldata6 = RECV[45]*256+RECV[46]
            ldata7 = RECV[52]*256+RECV[53]
            ldata8 = RECV[59]*256+RECV[60]
            data = {1:ldata1, 2:ldata2, 3:ldata3, 4:ldata4, 5:ldata5, 6:ldata6, 7:ldata7, 8:ldata8}
            zm_num = data[port]   
        except BaseException as err:
            #exit(1)
            errors_list.append(5001)
            zm_num = None
            zm_err = 5001
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # ultrasonic_values:获取指定多个端口超声波传感器的值 , ports 端口数字 _new
    def ultrasonic_values(self, ports = []):
        zm_err = 0
        zm_num = []
        zm_result = []
        zm_result.append(zm_err)
        for i in range(len(ports)):
            if not isinstance(ports[i], int) or ports[i] < 1 or ports[i] > 8 :
                zm_err = 5002
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX) 
            ultrasonicValue_list = []
            for i in ports:ultrasonicValue_list.append(i)
            ultrasonic = 0
            for i in range(len(ports)):
                if ports[i] < 1 or ports[i] > 8:
                    ultrasonicValue_list[i] = None
                if self.ultrasonic(ports[i]):
                    ultrasonic += 1
            if ultrasonic == 0: 
                return 5002, None
            ultrasonicValueCmd = [0x86, 0xAB, 0x00, 0x29, 0x01, 0x02, 
                                  0xA1, 0x06, 0x01, 0xBE,
                                  0xA2, 0x06, 0x01, 0xBE,
                                  0xA3, 0x06, 0x01, 0xBE,
                                  0xA4, 0x06, 0x01, 0xBE,
                                  0xA5, 0x06, 0x01, 0xBE,
                                  0xA6, 0x06, 0x01, 0xBE,
                                  0xA7, 0x06, 0x01, 0xBE,
                                  0xA8, 0x06, 0x01, 0xBE, 
                                  0x01, 0x00, 0xCF]
            # s_t = time.time()
            RECV = []
            while len(RECV) != 65:
                RECV = self.write_read_serial(ultrasonicValueCmd, 65)
            ldata1 = RECV[10]*256+RECV[11]
            ldata2 = RECV[17]*256+RECV[18]
            ldata3 = RECV[24]*256+RECV[25]
            ldata4 = RECV[31]*256+RECV[32]
            ldata5 = RECV[38]*256+RECV[39]
            ldata6 = RECV[45]*256+RECV[46]
            ldata7 = RECV[52]*256+RECV[53]
            ldata8 = RECV[59]*256+RECV[60]
            data = {1:ldata1, 2:ldata2, 3:ldata3, 4:ldata4, 5:ldata5, 6:ldata6, 7:ldata7, 8:ldata8}
            for i in range(len(ultrasonicValue_list)):
                if ultrasonicValue_list[i] != None:
                    a = int(ultrasonicValue_list[i])
                    ultrasonicValue_list[i] = data[a]
            zm_num = ultrasonicValue_list     
        except BaseException as err:
            #exit(1)
            errors_list.append(5002)
            zm_err = 5002
            zm_num = []
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # ultrasonic:查询指定端口是否有超声波传感器 _new 返回布尔型:真假
    def ultrasonic(self, port):    
        try:
            if port < 1 or port > 8:
                return False
            # ultrasonicListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF]
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(ultrasonicListCmd, 65)
            # data = RECV[port+5]
            # if data == 0x06:
            #     return True
            # else:
            #     return False
            return True
        except BaseException as err:
            #exit(1)   
            errors_list.append(5201)
            return False

    # ultrasonicList:查询所有端口是否有超声波传感器
    def ultrasonicList(self):    
        try:
            # ultrasonicListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF]
            # data = []
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(ultrasonicListCmd, 65)
            # j = 0
            # for i in RECV[6:14]:
            #     j+=1
            #     if i == 0x06:data.append(j)
            # return data
            return []
        except BaseException as err:
            #exit(1)              
            errors_list.append(5202)
            return None
        
    # humidity_value:获取端口湿度传感器的值
    def humidity_value(self, port):
        zm_err = 0
        zm_num = None
        zm_result = []
        zm_result.append(zm_err)
        if not isinstance(port, int) or port < 1 or port > 8:
                zm_err = 7001
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            humidity_valueCmd = [0x86, 0xAB, 0x00, 0x39, 0x01, 0x02,
                                0xA1, 0x0C, 0x01, 0x71, 0x01, 0xBE,
                                0xA2, 0x0C, 0x01, 0x71, 0x01, 0xBE,
                                0xA3, 0x0C, 0x01, 0x71, 0x01, 0xBE,
                                0xA4, 0x0C, 0x01, 0x71, 0x01, 0xBE,
                                0xA5, 0x0C, 0x01, 0x71, 0x01, 0xBE,
                                0xA6, 0x0C, 0x01, 0x71, 0x01, 0xBE,
                                0xA7, 0x0C, 0x01, 0x71, 0x01, 0xBE,
                                0xA8, 0x0C, 0x01, 0x71, 0x01, 0xBE,
                                0x01, 0x00, 0xCF]
            RECV = []
            while len(RECV) != 57:
                RECV = self.write_read_serial(humidity_valueCmd, 57)
            ldata1 = RECV[10]
            ldata2 = RECV[16]
            ldata3 = RECV[22]
            ldata4 = RECV[28]
            ldata5 = RECV[34]
            ldata6 = RECV[40]
            ldata7 = RECV[46]
            ldata8 = RECV[52]
            data = {1:ldata1, 2:ldata2, 3:ldata3, 4:ldata4, 5:ldata5, 6:ldata6, 7:ldata7, 8:ldata8}
            zm_num = data[port]
        except BaseException as err:
            #exit(1)
            errors_list.append(7001)
            zm_num = None
            zm_err = 7001
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # humidity_values:获取指定多个端口湿度传感器的值 _new
    def humidity_values(self, ports = []):
        zm_err = 0
        zm_num = []
        zm_result = []
        zm_result.append(zm_err)
        for i in range(len(ports)):
            if not isinstance(ports[i], int) or ports[i] < 1 or ports[i] > 8 :
                zm_err = 7002
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            humidityValue_list = []
            for i in ports:humidityValue_list.append(i)
            humidity = 0
            for i in range(len(ports)):
                if ports[i] < 1 or ports[i] > 8:
                    humidityValue_list[i] = None
                if self.humidity(ports[i]):
                    humidity += 1
            if humidity == 0: 
                zm_err = 7002
                zm_result.append(zm_num)
                return zm_err, zm_num
            humidityValueCmd = [0x86, 0xAB, 0x00, 0x39, 0x01, 0x02,
                                0xA1, 0x0C, 0x01, 0x71, 0x01, 0xBE,
                                0xA2, 0x0C, 0x01, 0x71, 0x01, 0xBE,
                                0xA3, 0x0C, 0x01, 0x71, 0x01, 0xBE,
                                0xA4, 0x0C, 0x01, 0x71, 0x01, 0xBE,
                                0xA5, 0x0C, 0x01, 0x71, 0x01, 0xBE,
                                0xA6, 0x0C, 0x01, 0x71, 0x01, 0xBE,
                                0xA7, 0x0C, 0x01, 0x71, 0x01, 0xBE,
                                0xA8, 0x0C, 0x01, 0x71, 0x01, 0xBE,
                                0x01, 0x00, 0xCF]
            RECV = []
            while len(RECV) != 57:
                RECV = self.write_read_serial(humidityValueCmd, 57)
            ldata1 = RECV[10]
            ldata2 = RECV[16]
            ldata3 = RECV[22]
            ldata4 = RECV[28]
            ldata5 = RECV[34]
            ldata6 = RECV[40]
            ldata7 = RECV[46]
            ldata8 = RECV[52]
            data = {1:ldata1, 2:ldata2, 3:ldata3, 4:ldata4, 5:ldata5, 6:ldata6, 7:ldata7, 8:ldata8}
            for i in range(len(humidityValue_list)):
                if humidityValue_list[i] != None:
                    a = int(humidityValue_list[i])
                    humidityValue_list[i] = data[a]
            zm_num = humidityValue_list
        except BaseException as err:
            #exit(1)
            errors_list.append(7002)
            zm_err = 7002
            zm_num = []
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # humidity:查询指定端口是否有湿度传感器 返回布尔型:真假 _new
    def humidity(self, port):
        try:
            if port < 1 or port > 8:
                return False
            # humidityListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF]
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(humidityListCmd, 65)
            # data = RECV[port+5]
            # if data == 0x0C:
            #     return True
            # else:
            #     return False
            return True
        except BaseException as err:
            #exit(1)
            errors_list.append(7201)
            return False

    # humidityList:查询所有端口是否有湿度传感器
    def humidityList(self):
        try:
            # humidityListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF]
            # data = []
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(humidityListCmd, 65)
            # j = 0
            # for i in RECV[6:14]:
            #     j+=1
            #     if i == 0x0C:data.append(j)
            # return data

            return []
        except BaseException as err:
            #exit(1)
            errors_list.append(7202)
            return None


    # temperature_value:获取端口温度传感器的值
    def temperature_value(self, port):
        zm_err = 0
        zm_num = None
        zm_result = []
        zm_result.append(zm_err)
        if not isinstance(port, int) or port < 1 or port > 8:
                zm_err = 6001
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            temperature_valueCmd = [0x86, 0xAB, 0x00, 0x39, 0x01, 0x02,
                                   0xA1, 0x0C, 0x01, 0x71, 0x00, 0xBE,
                                   0xA2, 0x0C, 0x01, 0x71, 0x00, 0xBE,
                                   0xA3, 0x0C, 0x01, 0x71, 0x00, 0xBE,
                                   0xA4, 0x0C, 0x01, 0x71, 0x00, 0xBE,
                                   0xA5, 0x0C, 0x01, 0x71, 0x00, 0xBE,
                                   0xA6, 0x0C, 0x01, 0x71, 0x00, 0xBE,
                                   0xA7, 0x0C, 0x01, 0x71, 0x00, 0xBE,
                                   0xA8, 0x0C, 0x01, 0x71, 0x00, 0xBE,
                                   0x01, 0x00, 0xCF]
            RECV = []
            while len(RECV) != 57:
                RECV = self.write_read_serial(temperature_valueCmd, 57)
            ldata1 = 0 if RECV[10] == 129 else RECV[10]
            ldata2 = 0 if RECV[16] == 129 else RECV[16]
            ldata3 = 0 if RECV[22] == 129 else RECV[22]
            ldata4 = 0 if RECV[28] == 129 else RECV[28]
            ldata5 = 0 if RECV[34] == 129 else RECV[34]
            ldata6 = 0 if RECV[40] == 129 else RECV[40]
            ldata7 = 0 if RECV[46] == 129 else RECV[46]
            ldata8 = 0 if RECV[52] == 129 else RECV[52]
            data = {1:ldata1, 2:ldata2, 3:ldata3, 4:ldata4, 5:ldata5, 6:ldata6, 7:ldata7, 8:ldata8}
            zm_num = data[port]
        except BaseException as err:
            #exit(1)
            errors_list.append(6001)
            zm_num = None
            zm_err = 6001
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # temperature_values:获取指定多个端口温度传感器的值 
    def temperature_values(self, ports = []):
        zm_err = 0
        zm_num = []
        zm_result = []
        zm_result.append(zm_err)
        for i in range(len(ports)):
            if not isinstance(ports[i], int) or ports[i] < 1 or ports[i] > 8 :
                zm_err = 6002
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX) 
            Value_list = []
            for i in ports:Value_list.append(i)
            Empty = 0
            for i in range(len(ports)):
                if ports[i]<1 or ports[i]>8:
                    Value_list[i] = None
                if self.temperature(ports[i]):
                    Empty += 1
            if Empty == 0: 
                zm_err = 6002
                zm_result.append(zm_num)
                return zm_err, zm_num
            temperatureValueCmd = [0x86, 0xAB, 0x00, 0x39, 0x01, 0x02,
                                   0xA1, 0x0C, 0x01, 0x71, 0x00, 0xBE,
                                   0xA2, 0x0C, 0x01, 0x71, 0x00, 0xBE,
                                   0xA3, 0x0C, 0x01, 0x71, 0x00, 0xBE,
                                   0xA4, 0x0C, 0x01, 0x71, 0x00, 0xBE,
                                   0xA5, 0x0C, 0x01, 0x71, 0x00, 0xBE,
                                   0xA6, 0x0C, 0x01, 0x71, 0x00, 0xBE,
                                   0xA7, 0x0C, 0x01, 0x71, 0x00, 0xBE,
                                   0xA8, 0x0C, 0x01, 0x71, 0x00, 0xBE,
                                   0x01, 0x00, 0xCF]
            RECV = []
            while len(RECV) != 57:
                RECV = self.write_read_serial(temperatureValueCmd, 57)
            ldata1 = 0 if RECV[10] == 129 else RECV[10]
            ldata2 = 0 if RECV[16] == 129 else RECV[16]
            ldata3 = 0 if RECV[22] == 129 else RECV[22]
            ldata4 = 0 if RECV[28] == 129 else RECV[28]
            ldata5 = 0 if RECV[34] == 129 else RECV[34]
            ldata6 = 0 if RECV[40] == 129 else RECV[40]
            ldata7 = 0 if RECV[46] == 129 else RECV[46]
            ldata8 = 0 if RECV[52] == 129 else RECV[52]
            data = {1:ldata1, 2:ldata2, 3:ldata3, 4:ldata4, 5:ldata5, 6:ldata6, 7:ldata7, 8:ldata8}
            for i in range(len(Value_list)):
                if Value_list[i] != None:
                    a = int(Value_list[i])
                    Value_list[i] = data[a]
            zm_num = Value_list
        except BaseException as err:
            #exit(1)
            errors_list.append(6002)
            zm_err = 6002
            zm_num = []
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # temperature:查询指定端口是否有温度传感器 返回布尔型:真假 
    def temperature(self, port):
        try:
            if port < 1 or port > 8:
                return False
            # temperatureListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF]
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(temperatureListCmd, 65)
            # data = RECV[port+5]
            # if data == 0x0C:
            #     return True
            # else:
            #     return False
            return True
        except BaseException as err:
            #exit(1)
            errors_list.append(6201)
            return False

    # temperatureList:查询所有端口是否有温度传感器
    def temperatureList(self):
        try:
            # temperatureListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF]
            # data = []
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(temperatureListCmd, 65)
            # j = 0
            # for i in RECV[6:14]:
            #     j+=1
            #     if i == 0x0C:data.append(j)
            # return data

            return []
        except BaseException as err:
            #exit(1)
            errors_list.append(6202)
            return None

    # get_motor_code:获取端口电机的值
    def get_motor_code(self, port):
        zm_err = 0
        zm_num = None
        zm_result = []
        zm_result.append(zm_err)
        if not isinstance(port, int) or port < 1 or port > 4:
                zm_err = 8001
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            get_motor_codeCmd =[0x86,0xAB,0x00,0x00,0x01,0x02,
                                0xA1,0x00,0x01,0xBE,
                                0x01,0x00,0xCF]
            get_motor_codeCmd[6] = port + 0xA0  #Set port
            get_motor_codeCmd[7] = 0x01  #set cmd
            get_motor_codeCmd[8] = 0x01
            self.ser.write(get_motor_codeCmd)
            time.sleep(0.015)
            data = self.ser.inWaiting()
            if data != 0:
                RECV = self.ser.read(data)
                Mdata = RECV[10]*16777216+RECV[11]*65536+RECV[12]*256+RECV[13]
                if Mdata > 0x7FFFFFFF:
                    Mdata = Mdata - 0xFFFFFEFF
                #print("MotorCode:", int(Mdata))
                self.ser.flushInput
                zm_num = Mdata
            else:
                zm_err = 8001
        except BaseException as err:
            #exit(1)
            errors_list.append(8001)
            zm_num = None
            zm_err = 8001
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # get_motors_code:获取指定多个端口电机的值 _new
    def get_motors_code(self, ports = []):
        zm_err = 0
        zm_num = []
        zm_result = []
        zm_result.append(zm_err)
        for i in range(len(ports)):
            if not isinstance(ports[i], int) or ports[i] < 1 or ports[i] > 4 :
                zm_err = 8002
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)   
            Value_list = []
            for i in ports:Value_list.append(i)
            Empty = 0
            for i in range(len(ports)):
                if ports[i] < 1 or ports[i] > 4:
                    Value_list[i] = None
                if self.motor(ports[i]):
                    Empty += 1
            if Empty == 0: 
                zm_err = 8002
                zm_result.append(zm_num)
                return zm_err, zm_num
            motorValueCmd = [0x86, 0xAB, 0x00, 0x00, 0x01, 0x02,
                             0xA1, 0x01, 0x01, 0xBE,
                             0xA2, 0x01, 0x01, 0xBE,
                             0xA3, 0x01, 0x01, 0xBE,
                             0xA4, 0x01, 0x01, 0xBE, 
                             0x01, 0x00, 0xCF]
            RECV = []
            while len(RECV) != 45:
                RECV = RECV = self.write_read_serial(motorValueCmd, 45)
            Mdata1 = RECV[10] * 16777216 + RECV[11] * 65536 + RECV[12] * 256 + RECV[13]
            if Mdata1 > 0x7FFFFFFF:
                Mdata1 = Mdata1 - 0xFFFFFFFF

            Mdata2 = RECV[19] * 16777216 + RECV[20] * 65536 + RECV[21] * 256 + RECV[22]
            if Mdata2 > 0x7FFFFFFF:
                Mdata2 = Mdata2 - 0xFFFFFFFF

            Mdata3 = RECV[28] * 16777216 + RECV[29] * 65536 + RECV[30] * 256 + RECV[31]
            if Mdata3 > 0x7FFFFFFF:
                Mdata3 = Mdata3 - 0xFFFFFFFF

            Mdata4 = RECV[37] * 16777216 + RECV[38] * 65536 + RECV[39] * 256 + RECV[40]
            if Mdata4 > 0x7FFFFFFF:
                Mdata4 = Mdata4 - 0xFFFFFFFF

            data = {1:Mdata1, 2:Mdata2, 3:Mdata3, 4:Mdata4}

            for i in range(len(Value_list)):
                if Value_list[i] != None:
                    num = Value_list[i]
                    Value_list[i] = data[num]
            zm_num = Value_list
        except BaseException as err:
            #exit(1)
            errors_list.append(8002)
            zm_err = 8002
            zm_num = []
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # motor:查询指定端口是否有电机 _new 返回 布尔型:真假
    def motor(self, port):
        try:
            if port < 1 or port > 4:
                return False
            # motorListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF]
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(motorListCmd, 65)
            # data = RECV[port+13]
            # if data == 0x01:
            #     return True
            # else:
            #     return False
            return True
        except BaseException as err:
            #exit(1)
            errors_list.append(8201)
            return False

    # motorList:查询所有端口是否有电机
    def motorList(self):
        try:
            # motorListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF]
            # data = []
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(motorListCmd, 65)
            # j = 0
            # for i in RECV[14:18]:
            #     j+=1
            #     if i == 0x01:data.append(j)
            # return data
            return []
        except BaseException as err:
            #exit(1)
            errors_list.append(8202)
            return None

    # set_motor_speed:设置电机的速度
    def set_motor_speed(self, port, speed):
        zm_err = 0
        if not isinstance(port, int) or not isinstance(speed, int) or port < 1 or port > 4 or speed < -100 or speed > 100 :
                zm_err = 8102
                return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX) 
            set_motor_speedCmd = [0x86, 0XAB, 0x00, 0x0F, 0x01, 0x02,
                                0xA1, 0x01, 0x02, 0x00, 0x00, 0xBE,
                                0x01, 0x00, 0xCF]
            speed = (speed & 0xFF)
            set_motor_speedCmd[6] = port + 0xA0  # Set port
            set_motor_speedCmd[8] = 0x02
            set_motor_speedCmd[10] = speed  # set speed
            num = self.write_serial_asyn(set_motor_speedCmd,1)
            if not num :
                zm_err = 8102
        except BaseException as err:
            #exit(1)
            errors_list.append(8102)
            zm_err = 8102
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err
            
    # set_motors_speed:设置多个电机的速度 
    def set_motors_speed(self, ports = [], speeds = []):
        zm_err = 0
        if len(ports) != len(speeds):
            zm_err = 8103
            return zm_err
        for i in range(len(ports)):
            if not isinstance(ports[i], int) or ports[i] < 1 or ports[i] > 4 :
                zm_err = 8103
                return zm_err
        for j in range(len(speeds)):
            if not isinstance(speeds[j], int) or speeds[j] < -100 or speeds[j] > 100:
                zm_err = 8103
                return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX) 
            L = len(ports)
            cmdstart = [0x86,0XAB,0x00,0x00,0x01,0x02] 
            cmdA1 = [0xA1,0x01,0x02,0x00,0x00,0xBE]
            cmdA2 = [0xA2,0x01,0x02,0x00,0x00,0xBE]
            cmdA3 = [0xA3,0x01,0x02,0x00,0x00,0xBE]
            cmdA4 = [0xA4,0x01,0x02,0x00,0x00,0xBE]
            cmdover = [0x01,0x00,0xCF]   
            dict_cmd = {1:cmdA1,2:cmdA2,3:cmdA3,4:cmdA4} 
            set_motor_speedCmd = cmdstart
            for k in range(L):
                cmd = dict_cmd[ports[k]]
                cmd[4] = speeds[k] & 0xFF
                set_motor_speedCmd += cmd
            set_motor_speedCmd += cmdover 
            num = self.write_serial_asyn(set_motor_speedCmd,L)
            if not num :
                zm_err = 8103
        except BaseException as err:
            #exit(1)
            errors_list.append(8103)
            zm_err = 8103
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    # set_motor_servo:设置电机伺服角度
    def set_motor_servo(self, port, angle , speed):
        try:
            # if port < 1 or port > 4 or speed < -100 or speed > 100 or angle < 0:
            #     return  False
            # set_motor_servoCmd = [0x86, 0xAB, 0x00, 0x12, 0x01, 0x02, 
            #                     0xA1, 0x01, 0x04, 0x81, 0x00, 0x81, 0x00, 0x00, 0xBE, 
            #                     0x01, 0x00, 0xCF]
            # speed = (speed & 0xFF)
            # set_motor_servoCmd[6] = port + 0xA0  # port
            # set_motor_servoCmd[10] = speed  # set speed
            # set_motor_servoCmd[12] = (angle >> 8) & 0xFF
            # set_motor_servoCmd[13] = angle & 0xFF
            # # self.write_serial(set_motor_servoCmd)
            # num = self.write_serial_asyn(set_motor_servoCmd,1)
            # #print(num)
            # # return num
            # # #print(num)
            # time.sleep(0.01)
            return True
        except BaseException as err:
            #exit(1)
            errors_list.append(8101)
            return False

    # set_motors_servo:设置多个电机伺服角度
    def set_motors_servo(self, ports = [], angles = [], speeds = [] ):
        try:
            # if len(ports) == 0 or len(angles) == 0 or len(speeds) == 0 :
            #     return 0
            # for p in ports:
            #     if p <1 or p >4:
            #         print("输入的端口值不符合要求")
            #         return 0
            # for i in speeds:
            #     if i <-100 or i >100:
            #         print("输入的速度不符合要求")
            #         return 0
            # for a in angles:
            #     if a < 0 :
            #         print("输入的角度不符合要求")
            #         return 0
            # set_motor_servoCmd = [0x86, 0xAB, 0x00, 0x12, 0x01, 0x02, 
            #                     0xA1, 0x01, 0x04, 0x81, 0x00, 0x81, 0x00, 0x00, 0xBE, 
            #                     0xA2, 0x01, 0x04, 0x81, 0x00, 0x81, 0x00, 0x00, 0xBE,
            #                     0xA3, 0x01, 0x04, 0x81, 0x00, 0x81, 0x00, 0x00, 0xBE,
            #                     0xA4, 0x01, 0x04, 0x81, 0x00, 0x81, 0x00, 0x00, 0xBE,
            #                     0x01, 0x00, 0xCF]
            # CMD_dict = {1:[10,12,13],2:[19,21,22],3:[28,30,31],4:[37,39,40]}
            # for k in range(len(ports)):
            #     num = ports[k]
            #     s = CMD_dict[num]
            #     set_motor_servoCmd[s[0]] = speeds[k] & 0xFF  #set speed
            #     set_motor_servoCmd[s[1]] = (angles[k] >> 8) & 0xFF
            #     set_motor_servoCmd[s[2]] = angles[k] & 0xFF
            # # speed = (speed & 0xFF)
            # # set_motor_servoCmd[6] = port + 0xA0  # port
            # # set_motor_servoCmd[10] = speed  # set speed
            # # set_motor_servoCmd[12] = (angle >> 8) & 0xFF
            # # set_motor_servoCmd[13] = angle & 0xFF
            # # self.write_serial(set_motor_servoCmd)
            # num = self.write_serial_asyn_2(set_motor_servoCmd)
            # #print(num)
            # # return num
            # time.sleep(0.01)
            return True
        except BaseException as err:
            #exit(1)
            errors_list.append(8104)
            return False

    #motor_servo_unlock 单个解锁电机 
    def motor_servo_unlock(self, port):
        zm_err = 0
        try:
            speed = 0
            if not isinstance(port, int) or not isinstance(speed, int) or port < 1 or port > 4 or speed < -100 or speed > 100 :
                zm_err = 8109
                return zm_err
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            set_motor_speedCmd = [0x86, 0XAB, 0x00, 0x0F, 0x01, 0x02,
                                0xA1, 0x01, 0x02, 0x00, 0x00, 0xBE,
                                0x01, 0x00, 0xCF]
            speed = (speed & 0xFF)
            set_motor_speedCmd[6] = port + 0xA0  # Set port
            set_motor_speedCmd[8] = 0x02
            set_motor_speedCmd[10] = speed  # set speed
            num = self.write_serial_asyn(set_motor_speedCmd,1)
            if not num :
                zm_err = 8109
        except BaseException as err:
            #exit(1)
            errors_list.append(8109)
            zm_err = 8109
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    # motors_servo_unlock 解锁多个电机
    def motors_servo_unlock(self, ports = []):
        zm_err = 0
        for i in range(len(ports)):
            if not isinstance(ports[i], int) or ports[i] < 1 or ports[i] > 4 :
                zm_err = 8110
                return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)        
            cmdstart = [0x86,0XAB,0x00,0x00,0x01,0x02] 
            cmdA1 = [0xA1,0x01,0x02,0x00,0x00,0xBE,]
            cmdA2 = [0xA2,0x01,0x02,0x00,0x00,0xBE,]
            cmdA3 = [0xA3,0x01,0x02,0x00,0x00,0xBE,]
            cmdA4 = [0xA4,0x01,0x02,0x00,0x00,0xBE,]
            cmdover = [0x01,0x00,0xCF]        
            dict_cmd = {1:cmdA1,2:cmdA2,3:cmdA3,4:cmdA4}
            set_motor_speedCmd = cmdstart
            L = len(ports)
            for k in range(L):
                set_motor_speedCmd += dict_cmd[ports[k]]
            set_motor_speedCmd += cmdover
            num = self.write_serial_asyn(set_motor_speedCmd,L)
            if not num :
                zm_err = 8110
        except BaseException as err:
            #exit(1)
            errors_list.append(8110)
            zm_err = 8110
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    #设置电机编码归零 
    def clear_motor_code(self, port):
        zm_err = 0
        if not isinstance(port, int) or port < 1 or port > 4 :
            zm_err = 8107
            return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            SetMotorCmd = [0x86, 0XAB, 0x00, 0x0F, 0x01, 0x02, 
                            0xA1, 0x01,0x02, 0x00, 0x32, 0xBE, 
                            0x01, 0x00, 0xCF]
            SetMotorCmd[6] = port + 0xA0  # Set port
            SetMotorCmd[8] = 0x03
            SetMotorCmd[10] = 0x01
            # self.write_serial(SetMotorCmd)
            num = self.write_serial_asyn(SetMotorCmd,1)
            if not num :
                zm_err = 8107
        except BaseException as err:
            #exit(1)
            errors_list.append(8107)
            zm_err = 8107
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    #设置全部电机编码为0
    def clear_motors_code(self,ports = []):
        zm_err = 0
        zm_num = []
        zm_result = []
        zm_result.append(zm_err)
        for i in range(len(ports)):
            if not isinstance(ports[i], int) or ports[i] < 1 or ports[i] > 4 :
                zm_err = 8108
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)  
            cmd = [0x86, 0XAB, 0x00, 0x00, 0x01, 0x02,
                0xA1, 0x01, 0x03, 0xBE,
                0xA2, 0x01, 0x03, 0xBE,
                0xA3, 0x01, 0x03, 0xBE,
                0xA4, 0x01, 0x03, 0xBE,
                0x01, 0x00, 0xCF]#SetMotor
            num = self.write_serial_asyn(cmd,4)
            if not num :
                zm_err = 8108
        except BaseException as err:
            #exit(1)
            errors_list.append(8108)
            zm_err = 8108
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    # steeringList:查询所有端口是否有舵机
    def steeringList(self):
        try:
            # steeringListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF]
            # data = []
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(steeringListCmd, 65)
            # j = 0
            # for i in RECV[6:14]:
            #     j+=1
            #     if i == 0x0E:data.append(j)
            # return data
            return []
        except BaseException as err:
            #exit(1)
            errors_list.append(9201)
            return None
 
    # set_steering:设置舵机伺服角度 P6-P8
    def set_steering(self, port, angle):
        zm_err = 0
        if not isinstance(port, int) or not isinstance(angle, int) or port < 6 or port > 8 or angle < 0 or angle > 270 :
                zm_err = 9101
                return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            set_steeringCmd = [0x86, 0xAB, 0x00, 0x10, 0x01, 0x02, 
                              0xA1, 0x0E, 0x01, 0x72, 0x00, 0x00, 0xBE,
                              0x01, 0x00, 0xCF]
            if angle >= 270:
                angle = 270
            set_steeringCmd[6] = port + 0xA0  # set port
            set_steeringCmd[7] = 0x0E  # set cmd
            set_steeringCmd[8] = 0x01  # set cmd
            set_steeringCmd[10] = angle // 0x100  #set cmd
            set_steeringCmd[11] = angle % 0x100  #set cmd
            num = self.write_serial_asyn(set_steeringCmd,1)
            if not num :
                zm_err = 9101
        except BaseException as err:
            #exit(1)
            errors_list.append(9101)
            zm_err = 9101
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    # set_steerings:设置多个舵机伺服角度 P6-P8
    def set_steerings(self, ports = [], angles = []):
        zm_err = 0
        if len(ports) != len(angles):
            zm_err = 9102
            return zm_err
        for i in range(len(ports)):
            if not isinstance(ports[i], int) or ports[i] < 6 or ports[i] > 8 :
                zm_err = 9102
                return zm_err
        for j in range(len(angles)):
            if not isinstance(angles[j], int) or angles[j] < 0 or angles[j] > 270:
                zm_err = 9102
                return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            L = len(ports)
            cmdstart = [0x86, 0xAB, 0x00, 0x10, 0x01, 0x02] 
            cmdA6 = [0xA6, 0x0E, 0x01, 0x72, 0x00, 0x00, 0xBE]
            cmdA7 = [0xA7, 0x0E, 0x01, 0x72, 0x00, 0x00, 0xBE]
            cmdA8 = [0xA8, 0x0E, 0x01, 0x72, 0x00, 0x00, 0xBE]
            cmdover = [0x01,0x00,0xCF]   
            dict_cmd = {6:cmdA6,7:cmdA7,8:cmdA8}     
            set_steeringCmd = cmdstart
            for k in range(L):
                cmd = dict_cmd[ports[k]]
                cmd[4] = angles[k] // 0x100
                cmd[5] = angles[k] % 0x100
                set_steeringCmd += cmd
            set_steeringCmd += cmdover
            num = self.write_serial_asyn(set_steeringCmd,L)
            if not num :
                zm_err = 9102
        except BaseException as err:
            #exit(1)
            errors_list.append(9102)
            zm_err = 9102
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    def set_steering_unlock(self, port, angle):#设置一个舵机,不锁
        zm_err = 0
        if not isinstance(port,int) or not isinstance(angle,int) or port < 6 or port > 8 or angle < 0 or angle > 270:
            zm_err = 9103
            return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            self.ser.flushInput()
            self.ser.flushOutput()
            cmd = [0x86, 0xAB, 0x00, 0x10, 0x01, 0x02,
                0xA6, 0x0E, 0x01, 0x72, 0x00, 0x00, 0xBE,
                0x01, 0x00, 0xCF]
            if angle >= 270:
                angle = 270
            cmd[6] = port + 0xA0  #Set port
            cmd[10] = (angle >> 8) & 0xFF
            cmd[11] = angle & 0xFF
            num = self.write_serial_asyn(cmd,1)
            # time.sleep(0.1)
            # self.steering_unlock(port)
            self.ser.flushInput()
            self.ser.flushOutput()
            cmd = [0x86, 0xAB, 0x00, 0x0D, 0x01, 0x02,
                0xA0, 0x0F, 0x01, 0xBE,
                0x01, 0x00, 0xCF]
            cmd[6] = port + 0xA0  #Set port
            num = self.write_serial_asyn(cmd,1)
            if not num :
                zm_err = 9103
        except BaseException as err:
            #exit(1)
            errors_list.append(9103)
            zm_err = 9103
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    #设置P6,P7,P8舵机，不锁
    def set_steerings_unlock(self, ports = [], angles = []):
        zm_err = 0
        if len(ports) != len(angles):
            zm_err = 9104
            return zm_err
        for i in range(len(ports)):
            if not isinstance(ports[i], int) or ports[i] < 6 or ports[i] > 8 :
                zm_err = 9104
                return zm_err
        for j in range(len(angles)):
            if not isinstance(angles[j], int) or angles[j] < 0 or angles[j] > 270:
                zm_err = 9104
                return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX) 
            self.ser.flushInput()
            self.ser.flushOutput()
            L = len(ports)
            cmdstart = [0x86, 0xAB, 0x00, 0x1E, 0x01, 0x02] 
            cmdA6 = [0xA6, 0x0E, 0x01, 0x72, 0x00, 0x00, 0xBE]
            cmdA7 = [0xA7, 0x0E, 0x01, 0x72, 0x00, 0x00, 0xBE]
            cmdA8 = [0xA8, 0x0E, 0x01, 0x72, 0x00, 0x00, 0xBE]
            cmdover = [0x01,0x00,0xCF]   
            dict_cmd = {6:cmdA6,7:cmdA7,8:cmdA8}     
            set_steeringCmd = cmdstart
            for k in range(L):
                cmd = dict_cmd[ports[k]]
                cmd[4] = (angles[k] >> 8) & 0xFF
                cmd[5] = angles[k] & 0xFF 
                set_steeringCmd += cmd
            set_steeringCmd += cmdover
            num = self.write_serial_asyn(set_steeringCmd,L)
            time.sleep(0.1)
            # steerings_unlock(ports)
            self.ser.flushInput()
            self.ser.flushOutput()
            L = len(ports)
            cmdstart = [0x86, 0xAB, 0x00, 0x0D, 0x01, 0x02] 
            cmdA6 = [0xA6, 0x0F, 0x00, 0xBE]
            cmdA7 = [0xA7, 0x0F, 0x00, 0xBE]
            cmdA8 = [0xA8, 0x0F, 0x00, 0xBE]
            cmdover = [0x01,0x00,0xCF]   
            dict_cmd = {6:cmdA6,7:cmdA7,8:cmdA8}     
            set_steeringCmd = cmdstart
            for k in range(L):
                cmd = dict_cmd[ports[k]]
                cmd[2] = 0x01
                set_steeringCmd += cmd
            set_steeringCmd += cmdover
            num = self.write_serial_asyn(set_steeringCmd,L)
            if not num :
                zm_err = 9104
        except BaseException as err:
            #exit(1)
            errors_list.append(9104)
            zm_err = 9104
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    def steering_unlock(self, port):#释放一个舵机，异步
        zm_err = 0
        if not isinstance(port,int) or port < 6 or port > 8 :
            zm_err = 9105
            return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            self.ser.flushInput()
            self.ser.flushOutput()
            cmd = [0x86, 0xAB, 0x00, 0x0D, 0x01, 0x02,
                0xA0, 0x0F, 0x01, 0xBE,
                0x01, 0x00, 0xCF]
            cmd[6] = port + 0xA0  #Set port
            num = self.write_serial_asyn(cmd,1)
            if not num :
                zm_err = 9105
        except BaseException as err:
            #exit(1)
            errors_list.append(9105)
            zm_err = 9105
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    #释放P6,P7,P8舵机，异步    
    def steerings_unlock(self, ports = []):
        zm_err = 0
        for i in range(len(ports)):
            if not isinstance(ports[i], int) or ports[i] < 6 or ports[i] > 48:
                zm_err = 9106
                return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            self.ser.flushInput()
            self.ser.flushOutput()
            L = len(ports)
            cmdstart = [0x86, 0xAB, 0x00, 0x0D, 0x01, 0x02] 
            cmdA6 = [0xA6, 0x0F, 0x00, 0xBE]
            cmdA7 = [0xA7, 0x0F, 0x00, 0xBE]
            cmdA8 = [0xA8, 0x0F, 0x00, 0xBE]
            cmdover = [0x01,0x00,0xCF]   
            dict_cmd = {6:cmdA6,7:cmdA7,8:cmdA8}     
            set_steeringCmd = cmdstart
            for k in range(L):
                cmd = dict_cmd[ports[k]]
                cmd[2] = 0x01
                set_steeringCmd += cmd
            set_steeringCmd += cmdover
            num = self.write_serial_asyn(set_steeringCmd,L)
            if not num :
                zm_err = 9106
        except BaseException as err:
            #exit(1)
            errors_list.append(9106)
            zm_err = 9106
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    #设置伺服电机，异步        
    def set_motor_servo_asyn(self, port, angle, speed):
        zm_err = 0
        if not isinstance(port, int) or not isinstance(speed, int) or port < 1 or port > 4 or speed < -100 or speed > 100 :
                zm_err = 8105
                return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX) 
            self.ser.flushInput()
            self.ser.flushOutput()
            cmd = [0x86, 0xAB, 0x00, 0x00, 0x01, 0x02,
                0xA1, 0x01, 0x04, 0x71, 0x00, 0x81, 0x00, 0x00, 0xBE,
                0x01, 0x00, 0xCF]#SetMotor speed
            speed = (speed & 0xFF)
            cmd[6] = port + 0xA0  #Set port
            cmd[10] = speed  #set speed
            cmd[12] = (angle >> 8) & 0xFF
            cmd[13] = angle & 0xFF
            num = self.write_serial_asyn(cmd,1)
            if not num :
                zm_err = 8105
        except BaseException as err:
            #exit(1)
            errors_list.append(8105)
            zm_err = 8105
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    #设置多个伺服电机，异步
    def set_motors_servo_asyn(self,ports = [],angles = [], speeds = []):
        zm_err = 0
        if len(ports) != len(speeds):
            zm_err = 8106
            return zm_err
        for i in range(len(ports)):
            if not isinstance(ports[i], int) or ports[i] < 1 or ports[i] > 4 :
                zm_err = 8106
                return zm_err
        for j in range(len(speeds)):
            if not isinstance(speeds[j], int) or speeds[j] < -100 or speeds[j] > 100:
                zm_err = 8106
                return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            self.ser.flushInput()
            self.ser.flushOutput()
            L = len(ports)
            cmdstart = [0x86, 0xAB, 0x00, 0x00, 0x01, 0x02] 
            cmdA1 = [0xA1, 0x01, 0x04, 0x71, 0x00, 0x81, 0x00, 0x00, 0xBE]
            cmdA2 = [0xA2, 0x01, 0x04, 0x71, 0x00, 0x81, 0x00, 0x00, 0xBE]
            cmdA3 = [0xA3, 0x01, 0x04, 0x71, 0x00, 0x81, 0x00, 0x00, 0xBE]
            cmdA4 = [0xA4, 0x01, 0x04, 0x71, 0x00, 0x81, 0x00, 0x00, 0xBE]
            cmdover = [0x01,0x00,0xCF]   
            dict_cmd = {1:cmdA1,2:cmdA2,3:cmdA3,4:cmdA4}     
            set_motor_speedCmd = cmdstart
            for k in range(L):
                cmd = dict_cmd[ports[k]]
                cmd[4] = speeds[k] & 0xFF  #set speed
                cmd[6] = (angles[k] >> 8) & 0xFF
                cmd[7] = angles[k] & 0xFF
                set_motor_speedCmd += cmd
            set_motor_speedCmd += cmdover
            num = self.write_serial_asyn(set_motor_speedCmd,L)
            if not num :
                zm_err = 8106
        except BaseException as err:
            #exit(1)
            errors_list.append(8106)
            zm_err = 8106
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    #设置伺服电机，不锁        
    def set_motor_servo_unlock(self, port, angle, speed):
        zm_err = 0
        if not isinstance(port, int) or not isinstance(speed, int) or port < 1 or port > 4 or speed < -100 or speed > 100 :
                zm_err = 8111
                return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)    
            self.ser.flushInput()
            self.ser.flushOutput()
            cmd = [0x86, 0xAB, 0x00, 0x00, 0x01, 0x02,
                0xA1, 0x01, 0x04, 0x71, 0x00, 0x81, 0x00, 0x00, 0xBE,
                0x01, 0x00, 0xCF]#SetMotor speed
            speed = (speed & 0xFF)
            cmd[6] = port + 0xA0  #Set port
            cmd[10] = speed  #set speed
            cmd[12] = (angle >> 8) & 0xFF
            cmd[13] = angle & 0xFF
            num = self.write_serial_asyn(cmd,1)
            time.sleep(1)
            motor_servo_unlock(port)
            if not num :
                zm_err = 8111
        except BaseException as err:
            #exit(1)
            errors_list.append(8111)
            zm_err = 8111
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    #设置多个伺服电机，不锁
    def set_motors_servo_unlock(self,ports = [],angles = [], speeds = []):
        zm_err = 0
        if len(ports) != len(speeds) or len(ports) == 0 or len(angles) == 0 or len(speeds) == 0 :
            zm_err = 8112
            return zm_err
        for i in range(len(ports)):
            if not isinstance(ports[i], int) or ports[i] < 1 or ports[i] > 4 :
                zm_err = 8112
                return zm_err
        for j in range(len(speeds)):
            if not isinstance(speeds[j], int) or speeds[j] < -100 or speeds[j] > 100:
                zm_err = 8112
                return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            self.ser.flushInput()
            self.ser.flushOutput()
            L = len(ports)
            cmdstart = [0x86, 0xAB, 0x00, 0x00, 0x01, 0x02] 
            cmdA1 = [0xA1, 0x01, 0x04, 0x71, 0x00, 0x81, 0x00, 0x00, 0xBE]
            cmdA2 = [0xA2, 0x01, 0x04, 0x71, 0x00, 0x81, 0x00, 0x00, 0xBE]
            cmdA3 = [0xA3, 0x01, 0x04, 0x71, 0x00, 0x81, 0x00, 0x00, 0xBE]
            cmdA4 = [0xA4, 0x01, 0x04, 0x71, 0x00, 0x81, 0x00, 0x00, 0xBE]
            cmdover = [0x01,0x00,0xCF]   
            dict_cmd = {1:cmdA1,2:cmdA2,3:cmdA3,4:cmdA4}     
            set_motor_speedCmd = cmdstart
            for k in range(L):
                cmd = dict_cmd[ports[k]]
                cmd[4] = speeds[k] & 0xFF
                cmd[6] = (angles[k] >> 8)& 0xFF
                cmd[7] = angles[k] & 0xFF
                set_motor_speedCmd += cmd
            set_motor_speedCmd += cmdover
            num = self.write_serial_asyn(set_motor_speedCmd,L)
            time.sleep(0.01)
            motors_servo_unlock(ports)
            if not num :
                zm_err = 8112
        except BaseException as err:
            #exit(1)
            errors_list.append(8112)
            zm_err = 8112
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    # compass_value:获取指南针的值
    def compass_value(self, port):
        zm_err = 0
        zm_num = None
        zm_result = []
        zm_result.append(zm_err)
        if port < 1 or port > 8:
                zm_err = 10001
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX) 
            compass_valueCmd = [0x86, 0xAB, 0x00, 0x29, 0x01, 0x02,
                               0xA1, 0x0D, 0x01, 0xBE,
                               0xA2, 0x0D, 0x01, 0xBE,
                               0xA3, 0x0D, 0x01, 0xBE,
                               0xA4, 0x0D, 0x01, 0xBE,
                               0xA5, 0x0D, 0x01, 0xBE,
                               0xA6, 0x0D, 0x01, 0xBE,
                               0xA7, 0x0D, 0x01, 0xBE,
                               0xA8, 0x0D, 0x01, 0xBE,
                               0x01, 0x00, 0xCF]
            RECV = []
            while len(RECV) != 65:
                RECV = self.write_read_serial(compass_valueCmd, 65)
            udata1 = RECV[10] * 256+RECV[11]
            udata2 = RECV[17] * 256+RECV[18]
            udata3 = RECV[24] * 256+RECV[25]
            udata4 = RECV[31] * 256+RECV[32]
            udata5 = RECV[38] * 256+RECV[39]
            udata6 = RECV[45] * 256+RECV[46]
            udata7 = RECV[52] * 256+RECV[53]
            udata8 = RECV[59] * 256+RECV[60]
            data = {1:udata1, 2:udata2, 3:udata3, 4:udata4, 5:udata5, 6:udata6, 7:udata7, 8:udata8}
            if data[port] < 0 or data[port] > 360:
                zm_num = None
            else:
                zm_num = data[port]
        except BaseException as err:
            #exit(1)
            errors_list.append(10001)
            zm_num = None
            zm_err = 10001
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # compass_values:获取指定多个指南针的值
    def compass_values(self, ports = []):
        zm_err = 0
        zm_num = []
        zm_result = []
        zm_result.append(zm_err)
        for i in range(len(ports)):
            if not isinstance(ports[i], int) or ports[i] < 1 or ports[i] > 8 :
                zm_err = 10002
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX) 
            Value_list = []
            for i in ports:Value_list.append(i)
            Empty = 0
            for i in range(len(ports)):
                if ports[i] < 1 or ports[i] > 8:
                    Value_list[i] = None
                if self.compass(ports[i]):
                    Empty += 1
            if Empty == 0: 
                zm_err = 10002
                zm_result.append(zm_num)
                return zm_err, zm_num
            compass_valueCmd = [0x86, 0xAB, 0x00, 0x29, 0x01, 0x02,
                               0xA1, 0x0D, 0x01, 0xBE,
                               0xA2, 0x0D, 0x01, 0xBE,
                               0xA3, 0x0D, 0x01, 0xBE,
                               0xA4, 0x0D, 0x01, 0xBE,
                               0xA5, 0x0D, 0x01, 0xBE,
                               0xA6, 0x0D, 0x01, 0xBE,
                               0xA7, 0x0D, 0x01, 0xBE,
                               0xA8, 0x0D, 0x01, 0xBE,
                               0x01, 0x00, 0xCF]
            RECV = []
            while len(RECV) != 65:
                RECV = self.write_read_serial(compass_valueCmd, 65)
            udata1 = RECV[10] * 256+RECV[11]
            udata2 = RECV[17] * 256+RECV[18]
            udata3 = RECV[24] * 256+RECV[25]
            udata4 = RECV[31] * 256+RECV[32]
            udata5 = RECV[38] * 256+RECV[39]
            udata6 = RECV[45] * 256+RECV[46]
            udata7 = RECV[52] * 256+RECV[53]
            udata8 = RECV[59] * 256+RECV[60]
            data = {1:udata1, 2:udata2, 3:udata3, 4:udata4, 5:udata5, 6:udata6, 7:udata7, 8:udata8}
            for i in range(len(Value_list)):
                if Value_list[i] != None:
                    a = int(Value_list[i])
                    if data[a] < 0 or data[a] > 360:
                        Value_list[i] = None
                    else:
                        Value_list[i] = data[a]
            zm_num = Value_list
        except BaseException as err:
            #exit(1)
            errors_list.append(10002)
            zm_err = 10002
            zm_num = []
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # compass:查询指定端口是否有指南针 返回布尔类型:真假 _new
    def compass(self, port):
        try:
            if port < 1 or port > 8:
                return False
            # compassListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF] 
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(compassListCmd, 65)
            # data = RECV[port+5]
            # if data == 0x0D:
            #     return True
            # else:
            #     return False
            return True
        except BaseException as err:
            #exit(1)
            errors_list.append(10201)
            return False
            
    # compassList:查询所有端口是否有指南针
    def compassList(self):
        try:
            # compassListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF] 
            # data = []
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(compassListCmd, 65)
            # j = 0
            # for i in RECV[6:14]:
            #     j+=1
            #     if i == 0x0D:data.append(j)
            # return data
            return []
        except BaseException as err:
            #exit(1)
            errors_list.append(10202)
            return None

    # gyroscope_init:陀螺仪传感器校正 --自加
    def gyroscope_init(self, port):
        zm_err = 0
        if not isinstance(port, int) or port < 1 or port > 8:
                zm_err = 11000
                return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            self.ser.flushInput()
            self.ser.flushOutput()
            gyroscope_initCmd = [0x86, 0xAB, 0x00, 0x0F, 0x01, 0x02,
                                0xA1, 0x08, 0x02, 0x71, 0x00, 0xBE,
                                0x01, 0x00, 0xCF]
            gyroscope_initCmd[6] = port + 0xA0
            # self.write_serial(gyroscope_init)
            num = self.write_serial_asyn(gyroscope_initCmd,1)
            # print(num)
            if num :
                zm_err = 11000
        except BaseException as err:
            #exit(1)
            errors_list.append(11000)
            zm_err = 11000
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    # gyroscope_value:获取陀螺仪传感器的值
    def gyroscope_value(self, port):
        zm_err = 0
        zm_num = None
        zm_result = []
        zm_result.append(zm_err)
        if not isinstance(port, int) or port < 1 or port > 8:
                zm_err = 11001
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            gyroscope_valueCmdX = [0x86, 0xAB, 0x00, 0x39, 0x01, 0x02,
                                  0xA1, 0x08, 0x01, 0x71, 0x00, 0xBE,
                                  0xA2, 0x08, 0x01, 0x71, 0x01, 0xBE,
                                  0xA3, 0x08, 0x01, 0x71, 0x01, 0xBE,
                                  0xA4, 0x08, 0x01, 0x71, 0x01, 0xBE,
                                  0xA5, 0x08, 0x01, 0x71, 0x01, 0xBE,
                                  0xA6, 0x08, 0x01, 0x71, 0x01, 0xBE,
                                  0xA7, 0x08, 0x01, 0x71, 0x01, 0xBE,
                                  0xA8, 0x08, 0x01, 0x71, 0x01, 0xBE,
                                  0x01, 0x00, 0xCF]
            gyroscope_valueCmdY = [0x86, 0xAB, 0x00, 0x39, 0x01, 0x02,
                                  0xA1, 0x08, 0x01, 0x71, 0x02, 0xBE,
                                  0xA2, 0x08, 0x01, 0x71, 0x02, 0xBE,
                                  0xA3, 0x08, 0x01, 0x71, 0x02, 0xBE,
                                  0xA4, 0x08, 0x01, 0x71, 0x02, 0xBE,
                                  0xA5, 0x08, 0x01, 0x71, 0x02, 0xBE,
                                  0xA6, 0x08, 0x01, 0x71, 0x02, 0xBE,
                                  0xA7, 0x08, 0x01, 0x71, 0x02, 0xBE,
                                  0xA8, 0x08, 0x01, 0x71, 0x02, 0xBE,
                                  0x01, 0x00, 0xCF]
            gyroscope_valueCmdZ = [0x86, 0xAB, 0x00, 0x39, 0x01, 0x02,
                                  0xA1, 0x08, 0x01, 0x71, 0x03, 0xBE,
                                  0xA2, 0x08, 0x01, 0x71, 0x03, 0xBE,
                                  0xA3, 0x08, 0x01, 0x71, 0x03, 0xBE,
                                  0xA4, 0x08, 0x01, 0x71, 0x03, 0xBE,
                                  0xA5, 0x08, 0x01, 0x71, 0x03, 0xBE,
                                  0xA6, 0x08, 0x01, 0x71, 0x03, 0xBE,
                                  0xA7, 0x08, 0x01, 0x71, 0x03, 0xBE,
                                  0xA8, 0x08, 0x01, 0x71, 0x03, 0xBE,
                                  0x01, 0x00, 0xCF]
            arglist = []
            RECV = []
            while len(RECV) != 65:
                RECV = self.write_read_serial(gyroscope_valueCmdX, 65)
            x = self.gyroscopeGetList(RECV)
            arglist.append(x[port-1])

            RECV = []
            while len(RECV) != 65:
                RECV = self.write_read_serial(gyroscope_valueCmdY, 65)
            y = self.gyroscopeGetList(RECV)
            arglist.append(y[port-1])

            RECV = []
            while len(RECV) != 65:
                RECV = self.write_read_serial(gyroscope_valueCmdZ, 65)
            z = self.gyroscopeGetList(RECV)
            arglist.append(z[port-1])
            zm_num = arglist     

        except BaseException as err:
            # print(err)
            #exit(1)
            errors_list.append(11001)
            zm_num = None
            zm_err = 11001
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # gyroscope_values:获取指定多个陀螺仪传感器的值 
    def gyroscope_values(self, ports = []):
        zm_err = 0
        zm_num = []
        zm_result = []
        zm_result.append(zm_err)
        for i in range(len(ports)):
            if not isinstance(ports[i], int) or ports[i] < 1 or ports[i] > 8 :
                zm_err = 11002
                zm_result.append(zm_num)
                return zm_err, zm_num
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            gyroscope_valueCmdX = [0x86, 0xAB, 0x00, 0x39, 0x01, 0x02,
                                  0xA1, 0x08, 0x01, 0x71, 0x00, 0xBE,
                                  0xA2, 0x08, 0x01, 0x71, 0x01, 0xBE,
                                  0xA3, 0x08, 0x01, 0x71, 0x01, 0xBE,
                                  0xA4, 0x08, 0x01, 0x71, 0x01, 0xBE,
                                  0xA5, 0x08, 0x01, 0x71, 0x01, 0xBE,
                                  0xA6, 0x08, 0x01, 0x71, 0x01, 0xBE,
                                  0xA7, 0x08, 0x01, 0x71, 0x01, 0xBE,
                                  0xA8, 0x08, 0x01, 0x71, 0x01, 0xBE,
                                  0x01, 0x00, 0xCF]
            gyroscope_valueCmdY = [0x86, 0xAB, 0x00, 0x39, 0x01, 0x02,
                                  0xA1, 0x08, 0x01, 0x71, 0x02, 0xBE,
                                  0xA2, 0x08, 0x01, 0x71, 0x02, 0xBE,
                                  0xA3, 0x08, 0x01, 0x71, 0x02, 0xBE,
                                  0xA4, 0x08, 0x01, 0x71, 0x02, 0xBE,
                                  0xA5, 0x08, 0x01, 0x71, 0x02, 0xBE,
                                  0xA6, 0x08, 0x01, 0x71, 0x02, 0xBE,
                                  0xA7, 0x08, 0x01, 0x71, 0x02, 0xBE,
                                  0xA8, 0x08, 0x01, 0x71, 0x02, 0xBE,
                                  0x01, 0x00, 0xCF]
            gyroscope_valueCmdZ = [0x86, 0xAB, 0x00, 0x39, 0x01, 0x02,
                                  0xA1, 0x08, 0x01, 0x71, 0x03, 0xBE,
                                  0xA2, 0x08, 0x01, 0x71, 0x03, 0xBE,
                                  0xA3, 0x08, 0x01, 0x71, 0x03, 0xBE,
                                  0xA4, 0x08, 0x01, 0x71, 0x03, 0xBE,
                                  0xA5, 0x08, 0x01, 0x71, 0x03, 0xBE,
                                  0xA6, 0x08, 0x01, 0x71, 0x03, 0xBE,
                                  0xA7, 0x08, 0x01, 0x71, 0x03, 0xBE,
                                  0xA8, 0x08, 0x01, 0x71, 0x03, 0xBE,
                                  0x01, 0x00, 0xCF]
            Value_list = []
            for i in ports:Value_list.append(i)
            Empty = 0
            for i in range(len(ports)):
                if ports[i] < 1 or ports[i] > 8:
                    Value_list[i] = None
                if self.gyroscope(ports[i]):
                    Empty += 1
            if Empty == 0: 
                zm_err = 11002
                zm_result.append(zm_num)
                return zm_err, zm_num
            for i in range(len(Value_list)):
                a = int(Value_list[i])
                if Value_list[i] != None:
                    arglist = []
                    RECV = []
                    while len(RECV) != 65:
                        RECV = self.write_read_serial(gyroscope_valueCmdX, 65)
                    x = self.gyroscopeGetList(RECV)
                    arglist.append(x[a-1])

                    RECV = []
                    while len(RECV) != 65:
                        RECV = self.write_read_serial(gyroscope_valueCmdY, 65)
                    y = self.gyroscopeGetList(RECV)
                    arglist.append(y[a-1])

                    RECV = []
                    while len(RECV) != 65:
                        RECV = self.write_read_serial(gyroscope_valueCmdZ, 65)
                    z = self.gyroscopeGetList(RECV)
                    arglist.append(z[a-1])
                    Value_list[i] = arglist
            zm_num = Value_list
        except BaseException as err:
            # print(err)
            #exit(1)
            errors_list.append(11002)
            zm_err = 11002
            zm_num = []
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    def gyroscopeGetList(self, RECV):
        try:
            arg1 = RECV[10] * 256 + RECV[11]
            if arg1 > 0x7FFF:
                arg1 = arg1 - 0x10000
            arg1 = int(arg1 / 10)
            if arg1 == 999:
                arg1 = 0

            arg2 = RECV[17] * 256 + RECV[18]
            if arg2 > 0x7FFF:
                arg2 = arg2 - 0x10000
            arg2 = int(arg2 / 10)
            if arg2 == 999:
                arg2 = 0

            arg3 = RECV[24] * 256 + RECV[25]
            if arg3 > 0x7FFF:
                arg3 = arg3 - 0x10000
            arg3 = int(arg3 / 10)
            if arg3 == 999:
                arg3 = 0

            arg4 = RECV[31] * 256 + RECV[32]
            if arg4 > 0x7FFF:
                arg4 = arg4 - 0x10000
            arg4 = int(arg4 / 10)
            if arg4 == 999:
                arg4 = 0

            arg5 = RECV[38] * 256 + RECV[39]
            if arg5 > 0x7FFF:
                arg5 = arg5 - 0x10000
            arg5 = int(arg5 / 10)
            if arg5 == 999:
                arg5 = 0

            arg6 = RECV[45] * 256 + RECV[46]
            if arg6 > 0x7FFF:
                arg6 = arg6 - 0x10000
            arg6 = int(arg6 / 10)
            if arg6 == 999:
                arg6 = 0

            arg7 = RECV[52] * 256 + RECV[53]
            if arg7 > 0x7FFF:
                arg7 = arg7 - 0x10000
            arg7 = int(arg7 / 10)
            if arg7 == 999:
                arg7 = 0

            arg8 = RECV[59] * 256 + RECV[60]
            if arg8 > 0x7FFF:
                arg8 = arg8 - 0x10000
            arg8 = int(arg8 / 10)
            if arg8 == 999:
                arg8 = 0
                
            return [arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8] 
        except BaseException as err:
            print("error: 11001.5")
            #exit(1)
            # errors_list.append(11001.5)
            return None
            
    # gyroscope:查询指定端口是否有陀螺仪传感器 返回布尔类型 : 真假 _new
    def gyroscope(self, port):
        try:
            if port < 1 or port > 8:
                return False
            # gyroscopeListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF]
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(gyroscopeListCmd, 65)
            # data = RECV[port+5]
            # if data == 0x03:
            #     return True
            # else:
            #     return False
            return True
        except BaseException as err:
            #exit(1)
            errors_list.append(11201)
            return False

    # gyroscopeList:查询所有端口是否有陀螺仪传感器
    def gyroscopeList(self):
        try:
            # gyroscopeListCmd = [0x86, 0xAB, 0x00, 0x09, 0x01, 0x01, 0x01, 0x00, 0xCF]
            # data = []
            # RECV = []
            # while len(RECV) != 21:
            #     RECV = self.write_read_serial(gyroscopeListCmd, 65)
            # j = 0
            # for i in RECV[6:14]:
            #     j+=1
            #     if i == 0x08:data.append(j)
            # return data
            return []
        except BaseException as err:
            #exit(1)
            errors_list.append(11202)
            return None

    #设置姿态,暂时没有功能  
    def setGyroscope(self, port, attitude = [0,0,0]):
        zm_err = 0
        if not isinstance(port, int) or port < 1 or port > 8:
                zm_err = "****陀螺仪"
                return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            SensorCmd = [0x86, 0xAB, 0x00, 0x0E, 0x01, 0x02,
                         0xA0, 0x08, 0x02, 0x71, 0x01, 0xBE,
                         0x01, 0x00, 0xCF]
            SensorCmd[6] = port + 0xA0
            SensorCmd[10] = attitude
            self.write_serial(SensorCmd)
            time.sleep(0.01)
        except BaseException as err:
            print("error: ***陀螺仪")
            #exit(1)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    # set_color_light:设置彩灯的颜色
    def set_color_light(self, port, color):
        zm_err = 0
        if not isinstance(port, int) or port < 1 or port > 8:
                zm_err = 12101
                return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX) 
            set_color_lightCmd = [0x86,0xAB,0x00,0x0D,0x01,0x02,
                                0xA1,0x00,0x01,0xBE,
                                0x01,0x00,0xCF]
            set_color_lightCmd[6] = port + 0xA0  # set port
            set_color_lightCmd[7] = 0x05  # set cmd
            set_color_lightCmd[8] = color  # set color
            num = self.write_serial_asyn(set_color_lightCmd,1)
            if not num :
                zm_err = 12101
        except BaseException as err:
            #exit(1)
            errors_list.append(12101)
            zm_err = 12101
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    # set_color_lights:设置多个彩灯的颜色
    def set_color_lights(self, ports = [], colors = []):
        zm_err = 0
        if len(ports) != len(colors):
            zm_err = 12102
            return zm_err
        for i in range(len(ports)):
            if not isinstance(ports[i], int) or ports[i] < 1 or ports[i] > 8 :
                zm_err = 12102
                return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX) 
            L = len(ports)
            cmdstart = [0x86,0xAB,0x00,0x0D,0x01,0x02] 
            cmdA1 = [0xA1,0x00,0x01,0xBE]
            cmdA2 = [0xA2,0x00,0x01,0xBE]
            cmdA3 = [0xA3,0x00,0x01,0xBE]
            cmdA4 = [0xA4,0x00,0x01,0xBE]
            cmdA5 = [0xA5,0x00,0x01,0xBE]
            cmdA6 = [0xA6,0x00,0x01,0xBE]
            cmdA7 = [0xA7,0x00,0x01,0xBE]
            cmdA8 = [0xA8,0x00,0x01,0xBE]
            cmdover = [0x01,0x00,0xCF]   
            dict_cmd = {1:cmdA1,2:cmdA2,3:cmdA3,4:cmdA4,5:cmdA5,6:cmdA6,7:cmdA7,8:cmdA8}     
            set_color_lightCmd = cmdstart
            for k in range(L):
                cmd = dict_cmd[ports[k]]
                cmd[1] = 0x05
                cmd[2] = colors[k]
                set_color_lightCmd += cmd
            set_color_lightCmd += cmdover
            num = self.write_serial_asyn(set_color_lightCmd,L)
            if not num :
                zm_err = 12102
        except BaseException as err:
            #exit(1)
            errors_list.append(12102)
            zm_err = 12102
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    # fw_version:查询固件版本号
    def fw_version(self):
        zm_err = 0
        zm_num = None
        zm_result = []
        zm_result.append(zm_err)
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            fw_versionCmd = [0x86, 0xAB, 0x00, 0x09, 0xFF, 0x02, 0x01, 0x0B, 0xCF]
            RECV = []
            while len(RECV) != 15:
                RECV = self.write_read_serial(fw_versionCmd, 15)
            # return 0, str(RECV[6:-3], encoding="utf-8")
            zm_num = str(RECV[6:-3], encoding="utf-8")
        except BaseException as err:
            #exit(1)
            errors_list.append(13201)
            # return 13201, None
            zm_err = 13201
            zm_num =None
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # battery:查询电池电量
    def battery(self):
        zm_err = 0
        zm_num = None
        zm_result = []
        zm_result.append(zm_err)
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            batteryCmd = [0x86, 0xAB, 0x00, 0x09, 0xFF, 0x09, 0x01, 0x0A, 0xCF] 
            RECV = []
            while len(RECV) != 11:
                RECV = self.write_read_serial(batteryCmd, 11)
            # print(RECV)    
            zm_num = RECV[6]
        except BaseException as err:
            print(err)
            #exit(1)
            errors_list.append(14201)
            zm_err = 14201
            zm_num =None
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # charging: 查询充电状态
    def charging(self):
        zm_err = 0
        zm_num = None
        zm_result = []
        zm_result.append(zm_err)
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX) 
            chargingCmd = [0x86, 0xAB, 0x00, 0x09, 0xFF, 0x09, 0x01, 0x0A, 0xCF]
            RECV = []
            while len(RECV) != 11:
                RECV = self.write_read_serial(chargingCmd, 11)
            if RECV[-4] == 1:
                zm_num = True
            if RECV[-4] == 0:
                zm_num = False
        except BaseException as err:
            errors_list.append(15201)
            zm_err = 15201
            zm_num =None
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    # reset:端口复位
    def reset(self):
        zm_err = 0    
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            resetCmd = [0x86, 0xAB, 0x00, 0x0A, 0x02, 0x01, 0x01, 0x00, 0x00, 0xCF]
            self.write_serial(resetCmd)
            time.sleep(0.01)
            set_motor_speedCmd = [0x86, 0XAB, 0x00, 0x0F, 0x01, 0x02,
                                0xA1, 0x01, 0x02, 0x71, 0x00, 0xBE,
                                0xA2, 0x01, 0x02, 0x71, 0x00, 0xBE,
                                0xA3, 0x01, 0x02, 0x71, 0x00, 0xBE,
                                0xA4, 0x01, 0x02, 0x71, 0x00, 0xBE,
                                0x01, 0x00, 0xCF]
            # self.write_serial(set_motor_speedCmd)
            num = self.write_serial_asyn(set_motor_speedCmd, 1)
            if not num :
                zm_err = 16100
        except BaseException as err:
            #exit(1)
            errors_list.append(16100)
            zm_err = 16100
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    def error(self,errcode):
        errtxet = errors[errcode]
        return errtxet

    def errnos(self):
        err_list = []
        for i in errors_list:
            err_list.append(errors[i])
        return err_list

    def chip(self):
        zm_err = 0
        zm_num = None
        zm_result = []
        zm_result.append(zm_err)
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX) 
            chipCmd = [0x86, 0xAB, 0x00, 0x09, 0xFF, 0x0B, 0x01, 0x00, 0xCF]
            RECV = []
            while len(RECV) != 10:
                RECV = self.write_read_serial(chipCmd, 11)
            if RECV[-4] == 0:
                zm_num = "STM32"
            elif RECV[-4] == 1:
                zm_num = "GD32"
            else:
                zm_err = 17201
                zm_num = None      
        except BaseException as err:
            # print(err)
            #exit(1)
            errors_list.append(17201)
            zm_num = None
            zm_err = 17201
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

    def supplement_light_open(self,port):#开光补光灯
        zm_err = 0
        if not isinstance(port, int) or port < 1 or port > 8:
                zm_err = 2103
                return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)
            self.ser.flushInput()
            self.ser.flushOutput()
            cmd = [0x86, 0xAB, 0x00, 0x00, 0x01, 0x02, 0xA1, 0x02, 0x02, 0xBE, 0x01, 0x00, 0xCF]
            cmd[6] = port + 0xA0  #Set port
            num = self.write_serial_asyn(cmd, 1)
            if not num :
                zm_err = 2103
        except BaseException as err:
            #exit(1)
            errors_list.append(2103)
            zm_err = 2103
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    def supplement_light_close(self,port):#关光补光灯
        zm_err = 0
        if not isinstance(port, int) or port < 1 or port > 8:
                zm_err = 2104
                return zm_err
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)   
            self.ser.flushInput()
            self.ser.flushOutput()
            cmd = [0x86, 0xAB, 0x00, 0x00, 0x01, 0x02, 0xA1, 0x02, 0x03, 0xBE, 0x01, 0x00, 0xCF]
            cmd[6] = port + 0xA0  #Set port
            num = self.write_serial_asyn(cmd, 1)
            if not num :
                zm_err = 2104
        except BaseException as err:
            #exit(1)
            errors_list.append(2104)
            zm_err = 2104
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err

    def getBatteryData2(self):#读电池电压，返回值有小数
        zm_err = 0
        zm_num = None
        zm_result = []
        zm_result.append(zm_err)
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)   
            cmd = [0x86, 0xAB, 0x00, 0x09, 0xFF, 0x0C, 0x01, 0x0A, 0xCF]
            self.ser.write(cmd)
            time.sleep(0.02)
            data = self.ser.inWaiting()
            if data != 0:
                RECV = self.ser.read(data)
                zm_num = (RECV[6] * 256 + RECV[5]) * 305 / 1000000 * 2
            else:
                zm_err = 18001      
        except BaseException as err:
            #exit(1)
            errors_list.append(18001)
            zm_num = None
            zm_err = 18001
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num
#-------------------------------------------------------------------
#固件更新模块


import serial
import time
import os
import struct
import math
import requests
import re
import sys


def getFileData(filename):
    file = open(filename, 'rb')
    data = []
    size = os.path.getsize(filename)
    for i in range(size):
        d = file.read(1)
        d = struct.unpack('B', d)[0]
        # print(datas)
        data.append(d)

    file.close()
    return data

class FirmwareUpgrade:
    ReqVersionCmd = [
        0x86, 0xAB, 0x00, 0x09, 0xFF,
        0x02, 0x01, 0x0B,
        0xCF]

    ReqDownLoadStartCmd = [
        0x86, 0xAB, 0x00, 0x09, 0xFF,
        0x10, 0x01, 0x0A,
        0xCF]

    ReqMaxCommLenCmd = [
        0x86, 0xAB, 0x00, 0x09, 0xFF,
        0x11, 0x01, 0x00,
        0xCF]

    ReqDownLoadInfoCmd = [
        0x86, 0xAB, 0x00, 0x1C, 0xFF,
        0x12]

    ReqStatusCmd = [
        0x86, 0xAB, 0x00, 0x09, 0xFF,
        0x13, 0x01, 0x00,
        0xCF]

    ReqDownLoadCmd = [
        0x86, 0xAB, 0x00, 0x0E, 0xFF,
        0x14]

    ReqPageCmd = [
        0x86, 0xAB, 0x00, 0x09, 0xFF,
        0x15]

    def __init__(self, start_step, file_path, file_name):
        self._serial = serial.Serial("/dev/ttyAMA1", 921600)
        self._filepath = file_path
        self._filename = file_name
        self._step = start_step
        

    # serial sendE:
    def send(self, cmd, flag = 1):
        send_bytes = self._serial.write(cmd)
        time.sleep(0.5)
        data = self._serial.inWaiting()
        if data == 0:
            self._serial.close()
            self._serial.open()
            self._serial.flushInput()
            time.sleep(0.5)
            self._serial.write(cmd)
            time.sleep(0.03)
            data = self._serial.inWaiting()

        if data == 0:
            self._serial.close()
            self._serial.open()
            self._serial.flushInput()
            time.sleep(0.5)
            self._serial.write(cmd)
            time.sleep(0.03)
            data = self._serial.inWaiting()

        # if data == 0:
        #     print("error data==0)
        #     exit(1)

        strs = self._serial.read(data)
        self._serial.flushInput()
        if flag == 1:
            return
        if strs[6] == ord("O") and strs[7] == ord("K"): #查找并修改处
            return

        print("error, return invalid")
        # exit(1)

    # download firmware
    def download(self):
        size = os.path.getsize(self._filepath)
        pcount = math.ceil(size/2048)
        zm_fdata = getFileData(self._filepath)
        for i in range(pcount):
            start = i * 2048
            end = (i + 1) * 2048
            pdata = zm_fdata[start:end]
            psize = len(pdata)

            pcmd = [0x00, i]
            cmd = self.ReqDownLoadCmd + pcmd

            pscmd = [(psize >> 8), (psize & 0xFF)]
            cmd = cmd + pscmd

            num = 0
            for x in cmd:
                num = x + num
            cmd.append(num & 0xFF)
            cmd = cmd + [0x01, 0x00, 0xCF]

            # send page info command
            self.send(cmd)

            cmd = self.ReqPageCmd + pdata

            num = 0
            for x in cmd:
                num += x

            # send page command
            cmd = cmd + [0x01, 0x00, 0xCF]
            cmd[2] = ((9 + psize) >> 8) & 0xFF
            cmd[3] = (9 + psize) & 0xFF
            cmd[-2] = num & 0xFF
            self.send(cmd)

            self._step += 0.037037
            print("\rrunning [{:.2%}] [{}{}]".format(self._step,'>'*int(self._step*100)*1,'-'*(99-int(self._step*100)*1)),end="")

    # send download info
    def sendDownloadInfo(self):
        fsize = os.path.getsize(self._filepath)
        #cmd = [0x86, 0xAB, 0x00, 0x1C, 0xFF, 0x12]
        fsize1 = (fsize >> 8 >> 8 >> 8) & 255
        fsize2 = (fsize >> 8 >> 8) & 255
        fsize3 = (fsize >> 8) & 255
        fsize4 = fsize & 255
        cmd = self.ReqDownLoadInfoCmd + [fsize1, fsize2, fsize3, fsize4]

        ct = time.time()
        lct = time.localtime(ct)
        cmd = cmd + [lct.tm_year >> 8, lct.tm_year & 0xff, lct.tm_mon, lct.tm_mday, lct.tm_hour, lct.tm_min, lct.tm_sec]

        for x in self._filename:
            cmd.append(ord(x))

        cmd = cmd + [0x01, 0x00, 0xCF]

        self.send(cmd)
        self._step += 0.037037
        print("\rrunning [{:.2%}] [{}{}]".format(self._step,'>'*int(self._step*100)*1,'-'*(99-int(self._step*100)*1)),end="")

    def getStatus(self):
        self.send(self.ReqStatusCmd)
        self._step += 0.037037
        print("\rrunning [{:.2%}] [{}{}]".format(self._step,'>'*int(self._step*100)*1,'-'*(99-int(self._step*100)*1)),end="")

    def maxCommLen(self):
        self.send(self.ReqMaxCommLenCmd)
        self._step += 0.037037
        print("\rrunning [{:.2%}] [{}{}]".format(self._step,'>'*int(self._step*100)*1,'-'*(99-int(self._step*100)*1)),end="")

    def upgrade(self):
        self.send(self.ReqDownLoadStartCmd)
        self._step += 0.037037
        print("\rrunning [{:.2%}] [{}{}]".format(self._step,'>'*int(self._step*100)*1,'-'*(99-int(self._step*100)*1)),end="")

    def version(self):
        self.send(self.ReqVersionCmd)
        self._step += 0.037037
        print("\rrunning [{:.2%}] [{}{}]".format(self._step,'>'*int(self._step*100)*1,'-'*(99-int(self._step*100)*1)),end="")

    def steps(self):
        size = os.path.getsize(self._filepath)
        print(math.ceil(size/2048) + 3)

# upgrade flow
# 1. update()
# 2. maxCommLen()
# 3. sendDownloadInfo()
# 4. getStatus()
# 5. download()
def getVersion():
    ser = serial.Serial("/dev/ttyAMA1", 921600)
    cmd = [0x86, 0xAB, 0x00, 0x09, 0xFF, 0x02, 0x01, 0x0B, 0xCF]
    ser.write(cmd)
    time.sleep(0.02)
    data = ser.inWaiting()
    
    if data == 0:
        ser.close()
        ser.open()
        ser.flushInput() # 清空缓冲区
        time.sleep(0.2)#等待
        ser.write(cmd)
        time.sleep(0.03)
        data = ser.inWaiting()
        
    if data == 0:
        ser.close()
        ser.open()
        ser.flushInput() # 清空缓冲区
        time.sleep(0.2)
        ser.write(cmd)
        time.sleep(0.03)
        data = ser.inWaiting()
        
    if data != 0:
        strs = ser.read(data)
        ser.flushInput()
        texts = strs[6:-3]
        # print(texts.decode('utf-8'))
        return texts.decode('utf-8')
    else:
        # print("None")
        return "None"

def upgrade_OS(file_path):
    file_name = file_path.split("/")[-1]
    version_start = getVersion()
    print("Current version: %s\nPrepare to upgrade to %s \nStart..." % (version_start,file_name[5:11]))
    start_step = 0.0
    if len(sys.argv) >= 2:
        if sys.argv[1] == "steps":
            fu = FirmwareUpgrade(0)
            fu.steps()
            exit(0)
        start_step = int(sys.argv[1])

    fu = FirmwareUpgrade(start_step,file_path, str(file_name))
    time.sleep(1)
    fu.upgrade()
    fu.sendDownloadInfo()
    fu.getStatus()
    fu.download()
    version_over = getVersion()
    success = "\nUpgrade succeeded!\nUpgrade version: %s"%version_over
    fail = "\nUpgrade failed!\nPlease upgrade again"
    if version_over == file_name[5:11]:
        print(success)
        return 0
    else:
        print(fail)
        return 20001


# if __name__ == "__main__":

#     file_path = "/home/pi/RCU/main_V1.4.2.bin"
#     update_OS(file_path)
        



class ZM_Upgrade(object):
    def __init__(self,path):
        super().__init__()
        self.path = path
        self.port_lock = threading.Lock()
        self.zm_path = "/tmp/rcu.lock"
    def upgrade(self):
        zm_err = 0
        zm_num = None
        zm_result = []
        zm_result.append(zm_err)
        try:
            zm_fd = os.open(self.zm_path, os.O_CREAT|os.O_RDWR)
            #进行文件和线程上锁
            self.port_lock.acquire()
            fcntl.lockf(zm_fd, fcntl.LOCK_EX)   
            num = upgrade_OS(self.path)
            zm_num = num
            return num
        except BaseException as err:
            # print(err)
            # return 20001
            zm_num = None
            zm_err = 20001
        zm_result.append(zm_num)
        # 解锁文件锁
        fcntl.lockf(zm_fd, fcntl.LOCK_UN)
        # 线程锁是否锁住？如果锁住，解锁；如果没有，就不需要解锁
        if self.port_lock.locked():
            self.port_lock.release()
        # 关闭文件
        os.close(zm_fd)
        # 返回返回值
        return zm_err, zm_num

# ------------------------------------- 函数封装 -----------------------------------
from concurrent.futures import ThreadPoolExecutor
import threading

ZM_RCU = StormSensors()

def upgrade(path):
    ZM_robot = ZM_Upgrade(path)
    return ZM_robot.upgrade()

#查看错误代码信息
def error(errcode):
    return ZM_RCU.error(errcode)

#查看错误代码
def errnos():
    return ZM_RCU.errnos()

# fan_speed:查询风扇的速度 
def fan_speed():
    return ZM_RCU.fan_speed()

# set_fan_speed:设置风扇的速度 
def set_fan_speed(speed):
    return ZM_RCU.set_fan_speed(speed)

# photo_electric_value:获取端口光电传感器的值
def photo_electric_value(port):
    return ZM_RCU.photo_electric_value(port)

# photo_electric_values:获取指定多个端口光电传感器的值
def photo_electric_values(ports = []):
    return ZM_RCU.photo_electric_values(ports)

# photoElectricList:查询所有端口是否有光电传感器
def photoElectricList():
    return ZM_RCU.photoElectricList()

# touch_value:获取端口触碰传感器的值
def touch_value(port):
    return ZM_RCU.touch_value(port)

# touch_values:获取指定多个端口触碰传感器的值
def touch_values( ports = []):
    return ZM_RCU.touch_values(ports)

# touchList:查询所有端口是否有触碰传感器
def touchList():
    return ZM_RCU.touchList()

# color_value:获取端口颜色传感器RGB的值
def color_value(port):
    return ZM_RCU.color_value(port)

# color_values:获取指定多个端口颜色传感器RGB的值
def color_values(ports = []):
    return ZM_RCU.color_values(ports)

# colorList:查询所有端口是否有颜色传感器
def colorList():
    return ZM_RCU.colorList()

# ultrasonic_value:获取端口超声波传感器的值,port端口数字
def ultrasonic_value(port):
    return ZM_RCU.ultrasonic_value(port)

# ultrasonic_values:获取指定多个端口超声波传感器的值,ports端口数字
def ultrasonic_values(ports = []):
    return ZM_RCU.ultrasonic_values(ports)    

# ultrasonicList:查询所有端口是否有超声波传感器
def ultrasonicList():     
    return ZM_RCU.ultrasonicList()            

# humidity_value:获取端口湿度传感器的值
def humidity_value(port):
    return ZM_RCU.humidity_value(port)

# humidity_values:获取指定多个端口湿度传感器的值
def humidity_values(ports = []):
    return ZM_RCU.humidity_values(ports)    

# humidityList:查询所有端口是否有湿度传感器
def humidityList():
    return ZM_RCU.humidityList()

# temperature_value:获取端口温度传感器的值
def temperature_value(port):
    return ZM_RCU.temperature_value(port)

# temperature_values:获取指定多个端口温度传感器的值
def temperature_values(ports = []):
    return ZM_RCU.temperature_values(ports)    

# temperatureList:查询所有端口是否有温度传感器
def temperatureList():
    return ZM_RCU.temperatureList()

# set_motor_speed:设置电机的速度
def set_motor_speed(port, speed):
    return ZM_RCU.set_motor_speed(port, speed)

# set_motors_speed:设置多个电机的速度
def set_motors_speed(ports = [], speeds = []):
    return ZM_RCU.set_motors_speed(ports, speeds)

# set_motor_servo:设置电机伺服角度
def set_motor_servo(port, angle , speed):
    return ZM_RCU.set_motor_servo(port, angle, speed)

# set_motors_servo:设置多个电机伺服角度
def set_motors_servo(ports = [], angles = [], speeds = [] ):
    return ZM_RCU.set_motors_servo(ports, angles, speeds)

# clear_motor_code:清除端口电机编码
def clear_motor_code(port):
    return ZM_RCU.clear_motor_code(port)

# clear_motors_code:清除全部端口电机编码
def clear_motors_code(ports = []):
    return ZM_RCU.clear_motors_code(ports)

# get_motor_code:获取端口电机的值
def get_motor_code(port):
    return ZM_RCU.get_motor_code(port)

# get_motors_code:获取指定多个端口电机的值
def get_motors_code(ports = []):
    return ZM_RCU.get_motors_code(ports)    
                
# motorList:查询所有端口是否有电机
def motorList():
    return ZM_RCU.motorList()

# set_steering:设置舵机伺服角度 P6-P8
def set_steering(port, angle):
    return ZM_RCU.set_steering(port, angle)

# set_steerings:设置舵机伺服角度 P6-P8
def set_steerings(ports = [], angles = []):
    return ZM_RCU.set_steerings(ports, angles)

#设置舵机，异步
def set_steering_unlock(port,angle):
    return ZM_RCU.set_steering_unlock(port,angle)

#设置多个舵机，异步
def set_steerings_unlock(ports = [],angles = []):
    return ZM_RCU.set_steerings_unlock(ports,angles)

#释放单个舵机，异步
def steering_unlock(port):
    return ZM_RCU.steering_unlock(port)

#释放多个舵机，异步
def steerings_unlock(ports = []):
    return ZM_RCU.steerings_unlock(ports)

# steeringList:查询所有端口是否有舵机
def steeringList():
    return ZM_RCU.steeringList()
                   
# compass_value:获取指南针的值
def compass_value(port):
    return ZM_RCU.compass_value(port)

# compass_values:获取指定多个指南针的值
def compass_values(ports = []):
    return ZM_RCU.compass_values(ports)    

# compassList:查询所有端口是否有指南针
def compassList():
    return ZM_RCU.compassList()

# gyroscope_value:获取陀螺仪传感器的值
def gyroscope_value(port):
    return ZM_RCU.gyroscope_value(port)

# gyroscope_values:获取指定多个陀螺仪传感器的值
def gyroscope_values(ports = []):
    return ZM_RCU.gyroscope_values(ports)

def gyroscope_init(port):
    return ZM_RCU.gyroscope_init(port)       
                    
# gyroscopeList:查询所有端口是否有陀螺仪传感器
def gyroscopeList():
    return ZM_RCU.gyroscopeList()

# set_color_light:设置彩灯的颜色
def set_color_light(port, color):
    return ZM_RCU.set_color_light(port, color)

# set_color_lights:设置多个彩灯的颜色
def set_color_lights(port = [],color = []):
    return ZM_RCU.set_color_lights(port, color)

# fw_version:查询固件版本号
def fw_version():
    return ZM_RCU.fw_version()
    
# battery:查询电池电量
def battery():
    return ZM_RCU.battery()
    
# charging: 查询充电状态
def charging():
    return ZM_RCU.charging()

# reset:端口复位
def reset():
    return ZM_RCU.reset()

#chip:单片机型号查询
def chip():
    return ZM_RCU.chip()

#set_photo_electric_threshold 设置光电传感器阈值
def set_photo_electric_threshold(port, threshold):
    return ZM_RCU.set_photo_electric_threshold(port, threshold)

#打开补光灯
def supplement_light_open(port):
    return ZM_RCU.supplement_light_open(port)

#关闭补光灯
def supplement_light_close(port):
    return ZM_RCU.supplement_light_close(port)

#设置伺服电机，异步
def set_motor_servo_asyn(port, angle, speed):
    return ZM_RCU.set_motor_servo_asyn(port, angle, speed)

#设置多个伺服电机，异步
def set_motors_servo_asyn(ports=[], angles=[], speeds=[]):
    return ZM_RCU.set_motors_servo_asyn(ports, angles, speeds)

#设置伺服电机，不锁
def set_motor_servo_unlock(port, angle, speed):
    return ZM_RCU.set_motor_servo_unlock(port, angle, speed)

#设置多个伺服电机，不锁
def set_motors_servo_unlock(ports=[], angles=[], speeds=[]):
    return ZM_RCU.set_motors_servo_unlock(ports, angles, speeds)

#查看电压
def getBatteryData2():
    return ZM_RCU.getBatteryData2()

#解开单个电机
def motor_servo_unlock(port):
    return ZM_RCU.motor_servo_unlock(port)

#解开多个电机
def motors_servo_unlock(ports = []):
    return ZM_RCU.motors_servo_unlock(ports)