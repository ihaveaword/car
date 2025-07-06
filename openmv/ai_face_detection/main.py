# Edge Impulse - OpenMV Object Detection Example

import sensor, image, time, os, tf, math, uos, gc
from pid import PID
from pyb import Servo,UART,Pin,Timer
from centroidtracking import CentroidTracking
objtraking=CentroidTracking(maxDisappeared=5) #目标追踪算法，质心追踪maxDisappeared表示最大丢失5帧代表目标消失
object_s=0                #目标距离
max_object=(0,0,0,0,0.0)  #目标信息元组
flag_lost=0               #目标丢失计数
pan_offset=-5             #底部舵机安装偏移参数（与中心的偏差角）
pan_start_angle=0         #底部舵机初始位置角度
tilt_start_angle=-30      #上部舵机初始位置，人脸，行人都是朝上看的
##############################电机控制部分#####################
pwmA1 = Pin('B0')   #TIM3_CH3
pwmA2 = Pin('B1')   #TIM3_CH4
pwmB1 = Pin('B4')   #TIM3_CH1
pwmB2 = Pin('B5')   #TIM3_CH2
# 定义定时器3，频率为100Hz，即10ms一次定时器中断
tim = Timer(3, freq=100)
# 初始化PWM，通过调节PWM控制左右轮速度，初试占空比为0（车静止）
Left1 = tim.channel(4, Timer.PWM, pin=pwmA2,pulse_width_percent=0)
Left2 = tim.channel(3, Timer.PWM, pin=pwmA1,pulse_width_percent=0)
Ritht1 = tim.channel(1, Timer.PWM, pin=pwmB1,pulse_width_percent=0)
Ritht2 = tim.channel(2, Timer.PWM, pin=pwmB2,pulse_width_percent=0)
#左右电机控制函数
def run(left_speed, right_speed):
    #print("left_speed",left_speed)
    #print("right_speed",right_speed)
    if left_speed < -100:
        left_speed = -100
    elif left_speed > 100:
        left_speed = 100
    if right_speed < -100:
        right_speed = -100
    elif right_speed > 100:
        right_speed = 100
    if left_speed > 0:
        Left1.pulse_width_percent(abs(left_speed))
        Left2.pulse_width_percent(abs(0))
    else:
        Left1.pulse_width_percent(abs(0))
        Left2.pulse_width_percent(abs(left_speed))

    if right_speed > 0:
        Ritht1.pulse_width_percent(abs(right_speed))
        Ritht2.pulse_width_percent(abs(0))
    else:
        Ritht1.pulse_width_percent(abs(0))
        Ritht2.pulse_width_percent(abs(right_speed))
#功能： 寻找最大的目标，计算方式为像素面积排序
#输入： 链表objects，坐标X,坐标y,像素宽,像素高
#输出： 最大目标元组
def find_max_object(objects):
    max_size=0
    global max_object
    for object in objects:
        if object[2]*object[3]> max_size:
            max_object=object
            max_size = object[2]*object[3]
    return max_object

def object_traking(objects):
    global flag_lost
    print("flag_lost:",flag_lost)
    max_object=find_max_object(objects)
    if(max_object[2]>0):
        flag_lost=0
        #云台pid追踪，行人要追踪头部位置
        x =int(max_object[0]+max_object[2]/2)
        y =int(max_object[1])
        object_s=60000/(max_object[2]*2) #计算距离
        #img.draw_string(x, y, "%d"%(object_s))
        print("object_s: ", object_s)
        pan_error =img.width()/2-x
        tilt_error =100-y
        #tilt_error =img.height()/2-y

        print("pan_error: ", pan_error)
        print("tilt_error: ", tilt_error)
        #小车追踪关闭左右舵机追踪
        #pan_output=pan_pid.get_pid(pan_error,1)/2
        tilt_output=tilt_pid.get_pid(tilt_error,1)/2
        #pan_servo.angle(pan_servo.angle()+pan_output)
        tilt_angle=tilt_servo.angle()-tilt_output
        if(tilt_angle>=-10):tilt_angle=-10
        tilt_servo.angle(tilt_angle)
        #############################小车追踪############################
        power,power_s,power_l=0,0,0
        #前后PID
        if object_s>20 and object_s<=1000:
            dis_error=object_s-100 #跟踪距离，单位cm
            power_s=int(dis_pid.get_pid(dis_error,1)/2)
            print("power_s:",power_s)
            if power_s>0:
                if power_s>100:
                    power_s=100
                if power_s<50:
                    power_s=50
                power_l=int(len_pid.get_pid(pan_error,1))
            if power_s<0:
                if power_s<-100:
                  power_s=-100
                if power_s>-50:
                  power_s=-50
                  power_l=0
            print("power_l:",power_l)
            run(power_s+power_l,power_s-power_l)
    else:
        run(0,0)
        #pan_servo.angle(pan_servo.angle())
        #tilt_servo.angle(tilt_servo.angle())
        flag_lost=flag_lost+1
        if flag_lost>=10:
            flag_lost=0
            pan_servo.angle(pan_start_angle+pan_offset,1000)
            tilt_servo.angle(tilt_start_angle,1000)


#控制左右接D14
#控制上下接D15
pan_servo=Servo(3)
tilt_servo=Servo(4)
pan_servo.angle(pan_start_angle+pan_offset)
tilt_servo.angle(tilt_start_angle)
#舵机PID
pan_pid = PID(p=0.06, i=0.0 ,imax=90)#在线调试使用这个PID
tilt_pid = PID(p=0.07, i=0.0, imax=90)#在线调试使用这个PID

#小车追踪PID
dis_pid = PID(p=0.5, i=0.01)
len_pid = PID(p=0.2, i=0.01)


sensor.reset()                         # Reset and initialize the sensor.
sensor.set_pixformat(sensor.GRAYSCALE)    # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.VGA)      # Set frame size to QVGA (320x240)
sensor.set_hmirror(1)
#sensor 自动曝光控制
print("Initial exposure == %d" % sensor.get_exposure_us())
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time = 2000)
current_exposure_time_in_microseconds = sensor.get_exposure_us()
print("Current Exposure ==%d" % current_exposure_time_in_microseconds)
EXPOSURE_TIME_SCALE = 1.0
sensor.set_auto_exposure(False, \
    exposure_us = int(current_exposure_time_in_microseconds * EXPOSURE_TIME_SCALE))

sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
net = None
labels = None
confidence=0.7
rid=640/480
try:
    # load the model, alloc the model file on the heap if we have at least 64K free after loading
    labels,net = tf.load_builtin_model('yoloface')
    #net = tf.load("yolo_person_int8.tflite")
except Exception as e:
    raise Exception('Failed to load "yoloface", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

print(net)


colors = [ # Add more colors if you are detecting more than 7 types of classes at once.
    (255,   0,   0),
    (  0, 255,   0),
    (255, 255,   0),
    (  0,   0, 255),
    (255,   0, 255),
    (  0, 255, 255),
    (255, 255, 255),
]
clock = time.clock()
detection_list=list()

while(True):
    clock.tick()
    img = sensor.snapshot()
    max_object=(0,0,0,0,0.0)
    object_s=0
    person_num=0
    xrect=[]
    objects=net.detect_yolo(img, confidence=0.75, anchors=[21,28,34,49,61,77],nms=0.2)
    #if (len(objects) == 0): continue # no detections for this class?
    if objects:
        for d in objects :
            rect=(int(d.x()/rid), d.y(), int(d.w()/rid), d.h())
            img.draw_rectangle(rect,color=(255,255,0))
            img.draw_string(d.x(), d.y()-10, "face %.3f"%(d.output()))
            xrect.append(rect)
    objects=objtraking.update(xrect)
    #if (len(objects) == 0): continue # no detections for this class?
    object_traking(objects)
    #lcd.display(img)
    print(clock.fps(), "fps", end="\n\n")

