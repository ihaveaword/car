# Untitled - By: mengfei - Sun Mar 17 2024

import sensor, image, time,pyb,omv,math,utime,tf,gc,lcd
from pyb import UART,Pin,Timer,Servo,LED
from machine import SPI
from ubluetooth import CARSTATE,UBLUETOOTH
from umotor import UMOTOR
from pid import PID
from ultrasonic import ULTRASONIC
led=LED(1)                #LED指示
led.off()
object_s=0                #目标距离
count=0                   #目标到达指定区域的计数
max_object=(0,0,0,0,0.0)  #目标信息元组
flag_lost=0               #目标丢失计数
pan_offset=-5             #底部舵机安装偏移参数（与中心的偏差角）
pan_start_angle=0         #底部舵机初始位置角度
tilt_start_angle=-10      #上部舵机初始位置
#配合视频小车app，实现小车寻迹，跟随，避障三种功能
#寻迹，小车沿着黑线前进，可以自己标定一下线的阈值
#跟随，小车颜色识别跟随，默认是跟随黄色乒乓球，可自行标定其他阈值
#避障，小车超声波测距避障，找到目标则跟随
#前面三种模式下，按下小车前后左右键可以控制小车
#开灯，关灯按钮暂时用来作为云台控制功能，即按下这两个按钮时，上下左右键可以控制云台，
#以上仅为参考功能设计，可以在ubluetooth.py库自行定义其他功能
global ultradis,carstate,cardir,left_speed,right_speed,img,THRESHOLD
ultradis=0.0
motor=UMOTOR()
ble=UBLUETOOTH(uart_port=1,baudrate=9600)
ultrawave=ULTRASONIC(trig_pin="B13",echo_pin="B12")
objects = list()
last_objects=objects

pan_servo=Servo(3)      #左右控制
tilt_servo=Servo(4)     #上下控制
#pan_servo.calibration(500, 2500, 1500)
#tilt_servo.calibration(500, 2500, 1500)
pan_servo.angle(0)
tilt_servo.angle(0)
#舵机PID
pan_pid = PID(p=0.06, i=0.0 ,imax=90)#在线调试使用这个PID
tilt_pid = PID(p=0.07, i=0.0, imax=90)#在线调试使用这个PID
THRESHOLD  =[(90, 100, 0, 21, -15, 15),(26, 63, 12, 71, 19, 68)]  #火焰识别阈值，灯光也可能被识别到，所以用火灾识别模型进行预检
DIS_RATE=20000          #测距离系数，使用图像像素多少估算距离，具体要根据目标的大小进行调试,
#由于火焰大小不固定，测试的话最好找个固定的装置产生火源，或者用图片测试
#加载火灾识别模型
s_flag=0 #是否需要进行火灾识别的标志
net = tf.load("fire_uint8.tflite")
labels = ['fire','no_fire']
print(net)
#功能： 寻找最大的目标，计算方式为像素面积排序
#输入： 链表objects，坐标X,坐标y,像素宽,像素高,mode=0是色块，1是人脸坐标
#输出： 最大目标元组,
def find_max_object(objects,mode=0):
    max_size=0
    if mode==0:#色块元组
        max_object=None
    if mode==1:#人脸框坐标格式和色块不同，单独定义
        max_object=(0,0,0,0,0.0)
    for object in objects:
        if object[2]*object[3] > max_size:
            max_object=object
            max_size = object[2]*object[3]
    return max_object

#小车根据目标信息进行追踪和灭火操作
def car_traking(object_s,pan_error):
    #############################小车追踪############################
    global count,s_flag
    #小车追踪PID
    dis_pid  = PID(p=0.65, i=0.01) #小车前后追球PID
    len_pid  = PID(p=0.3 ,i=0.01) #小车左右追踪pid
    power_s,power_l=0,0
    if ((object_s>100) and (object_s<2000)):
        count=0
        if object_s>300 and object_s<=2000:
            dis_error=object_s-300
            power_s=int(dis_pid.get_pid(dis_error,1)/2)
        elif object_s>100 and object_s<=300:
            dis_error=object_s-100
            power_s=int(dis_pid.get_pid(dis_error,1)/2)
            #电机PWM限幅，防止最后一点距离走不到
        else:
            power_s =50
        print("power_s:",power_s)
        if(power_s>100):
            power_s=100
        if(power_s<30):
            power_s=30
        power_l=int(len_pid.get_pid(pan_error,1))
        motor.run(power_s-power_l,power_s+power_l)
    if(object_s>=50 and object_s<=100):
        motor.run(0,0)
        count=count+1
        led.on()
        time.sleep(200)
        led.off()
        time.sleep(200)
        if count >=10:
            count=0
            motor.run(0,0)
            print("Open fire extinguishing !!!!")
            #此处可进行灭火操作，通过串口给灭火单元发送指令
            time.sleep(1000)
            s_flag=0 #灭火结束和目标丢失都要重新进行火灾识别


def fire_recognization(img):
    global s_flag
    fire_flag=False
    score_str = ""
    for obj in tf.classify(net, img, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
        print(obj)
        out = obj.output()
        max_idx = out.index(max(out))
        score = int(out[max_idx]*100)
        if (score < 85):
            score_str = "no_fire"
            fire_flag=False
            s_flag=0
        if (score >= 99):
            s_flag+=5
            fire_flag=True
            score_str = "%s:%d%% "%(labels[max_idx], score)
        if (score < 99) and score >= 85:
            score_str = "%s:%d%% "%(labels[max_idx], score)
            fire_flag=True
        print(score_str)
        img.draw_string(0, 0, score_str, color=(255, 0, 0))
        return fire_flag

#功能： 小球颜色识别
#输入： 图像
#输出： ball_s,max_blob
def color_detect(img):
    #全局变量
    global objects,flag_lost
    object_s=0
    max_object=None
    objects = img.find_blobs(THRESHOLD,pixels_threshold=100,merge=True)
    #step1 有目标，就进行追踪和抓取操作
    if objects:  #如果找到了色块，就计算最大的色块位置，并做舵机追踪
        max_object = find_max_object(objects=objects,mode=0)
        #色块的位置和大小需要满足图像本身大小的要求，以免越界
        object_s=DIS_RATE/(max_object[2]*2) #计算距离
        img.draw_rectangle(max_object.rect()) # rect 目标区域画矩形
        img.draw_cross(max_object.cx(), max_object.cy()) # cx, cy画中心点
        img.draw_string(max_object.cx(), max_object.cy(), "%.2f mm"%(object_s))  #显示目标的距离参数
    return object_s,max_object

#采用间隔时间获取测距值，在跟随目标时实现近距离避障
ultime=time.ticks()
last_time= time.ticks()
def update_ultradis(stime=500):
    global  ultradis,ultime,last_time
    ultime=time.ticks()-last_time
    last_time= time.ticks()
    if (ultime>=stime):
        ultradis=ultrawave.get_distance()
#函数功能：自动避障，延时策略很关键
#输入：目标距离 单位：cm
#输出：无
def auto_avoidance(objdis):
    car_speed=30
    if(objdis>=35):
        motor.run(car_speed,car_speed)      #前进
        time.sleep(100) #前进时间不能长，否则正前方行驶容易一直前进后退
    if objdis<=25 and (objdis>2.0):
        motor.run(-car_speed,-car_speed)    #后退
        time.sleep(600)
    if objdis< 35 and (objdis>25):    #右转
        motor.run(car_speed,-car_speed)
        time.sleep(600)#转弯时间要稍微长一点，特别是墙角位置转时间短了不容易过去
#功能： 目标追踪
#输入： 图像，目标类型，控制模式
#输出： 无
def color_traking(img,mode=0):
    global flag_lost,ultradis,s_flag
    object_s=0
    max_object=None
    object_s,max_object=color_detect(img)
    if(object_s>0):
        flag_lost=0 #丢失清空
        if mode>0:
            update_ultradis(stime=500)	#只在避障模式下更新测距
        else :  ultradis=200#赋一个安全值单位cm
        cx =int(max_object[0]+max_object[2]/2)
        cy =int(max_object[1]+max_object[3]/2)
        print("object_s: ", object_s)
        pan_error =img.width()/2-cx  #由于镜像水平方向偏差要反过来
        tilt_error =img.height()/2-cy #计算目标位置偏差
        print("pan_error: ", pan_error)
        print("tilt_error: ", tilt_error)
        pan_output=pan_pid.get_pid(pan_error,1)/2 #计算PID跟随
        tilt_output=tilt_pid.get_pid(tilt_error,1)/2
        pan_angle=pan_servo.angle()+pan_output
        if pan_angle>30:pan_angle=30
        if pan_angle<-30:pan_angle=-30
        #pan_servo.angle(pan_angle) #关闭云台左右追踪
        tilt_angle=tilt_servo.angle()-tilt_output
        if tilt_angle>60:tilt_angle=60
        if tilt_angle<-60:tilt_angle=-60
        tilt_servo.angle(tilt_angle) #云台开上下追踪
        if(ultradis>=25):
            car_traking(object_s,pan_error)
        else:
            auto_avoidance(ultradis)
    else: #目标丢失计数
        flag_lost=flag_lost+1
        if mode>0:
            ultradis=ultrawave.get_distance()
            auto_avoidance(ultradis)
        if flag_lost>10:#连续10帧没有
            flag_lost=0
            s_flag=0 #清空检测标志，重新进行火灾识别
            pan_servo.angle(0,1000) #舵机回中
            tilt_servo.angle(0,1000)
            if mode==0: #非避障模式这里会停止
                motor.run(0,0)

    return object_s

def fire_traking_autocontrol(img,mode=0):
    global s_flag
    if s_flag<10:
        sfire=fire_recognization(img)
        if sfire:
            s_flag=s_flag+1
        else :
            s_flag=0
        if mode>0:
            ultradis=ultrawave.get_distance()
            auto_avoidance(ultradis)
    else: #连续5次检测都是相同结果
        color_traking(img,mode)#根据mode判断是否启用避障

def car_state_deal(img):
    carstate,cardir,left_speed,right_speed,s_flag=ble.bluetooth_deal()
    if (carstate==CARSTATE.enMANUAL): #手动控制小车
        motor.run(left_speed,right_speed)
    elif (carstate==CARSTATE.enFLITING): #控制舵机
        if(cardir==CARSTATE.enRUN):  #舵机向上
            tilt_servo.angle(tilt_servo.angle()-2)
        elif(cardir==CARSTATE.enBACK): #舵机向下
            tilt_servo.angle(tilt_servo.angle()+2)
        elif(cardir==CARSTATE.enLEFT): #舵机向左
            pan_servo.angle(pan_servo.angle()+2)
        elif(cardir==CARSTATE.enRIGHT):#舵机向右
            pan_servo.angle(pan_servo.angle()-2)
        elif(cardir==CARSTATE.enRELEASE):
            led.on() #可以换成开启灭火指令，方便app控制灭火
        elif(cardir==CARSTATE.enCATCH):
            led.off() #可以换成关闭灭火指令，方便app控制
    elif(carstate==CARSTATE.enTRACING):#寻迹
        color_detect(img)
    elif(carstate==CARSTATE.enTRAKING):
        fire_traking_autocontrol(img,mode=0) #跟随模式，不带避障
    elif(carstate==CARSTATE.enAVOIDING):
        fire_traking_autocontrol(img,mode=1) #避障模式，追踪目标时支持近距离避障


sensor.reset()
#sensor.set_contrast(1)
#sensor.set_brightness(1)
#sensor.set_framesize(sensor.QVGA)
sensor.set_framesize(sensor.QVGA)  # Set frame size to QVGA (320x240)
sensor.set_pixformat(sensor.RGB565)
#sensor.set_hmirror(True) #水平镜像，方便调试
#sensor.set_vflip(True) #不同的摄像头需要设置一下旋转镜像才能正常跟踪
sensor.skip_frames(10)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
#sensor.set_pixformat(sensor.JPEG) #OV2640 SET
clock = time.clock()
lcd.init()

while(True):
  clock.tick()
  img = sensor.snapshot()
  car_state_deal(img=img)
  print(clock.fps(), "fps", end="\n\n")
  lcd.display(img)
  gc.collect()
