'''
@file:    find_face_yolo.py
@company: Hiwonder
@author:  CuZn
@date:    2023-08-26
@description: 人脸检测，并将识别后的数据发送给 开发板（STM32等）
        # K210视觉模块 会将识别到的方框数据和识别信息 通过串口 发送给 开发板（STM32等）

'''

#载入相关模块
import sensor, image, time, lcd
from maix import KPU
from hiwonder import hw_uart
import binascii

#定义串口对象
serial = hw_uart()



##################################### send_func begin #####################################

#CRC校验参数，不可修改，否则数据通讯会出错
crc8_table = [
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
]

#CRC校验函数
def checksum_crc8(data):
    # 校验
    check = 0
    for b in data:
        check = crc8_table[check ^ b]
    return check & 0x00FF

#功能号
class FuncTag:
    START = 0
    FIND_FACE_YOLO  = 5


#字符串转int
def str_2_int(data_str):
    bb = binascii.hexlify(data_str)
    bb = str(bb)[2:-1]
    int_h = int(bb[0])*16
    int_l = int(bb[1],16)
    return int_h+int_l


def send_data(x,y,w,h,msg):
    '''
    0xAA  0x55  功能号  数据长度  data  CRC
    '''
    START_1 = 0xAA
    START_2 = 0x55
    FUNC_num = FuncTag.FIND_FACE_YOLO #功能编号
    Length = 0  #数据长度
    crc = 0 #校验位
    data = [] #数据组

    #参数都为0
    if x==0 and y==0 and w==0 and h ==0:
        pass
    else:
        #x(小端模式)
        low = x & 0xFF #低位
        high = x >> 8& 0xFF #高位
        data.append(low)
        data.append(high)

        #y(小端模式)
        low = y & 0xFF #低位
        high = y >> 8& 0xFF #高位
        data.append(low)
        data.append(high)

        #w(小端模式)
        low = w & 0xFF #低位
        high = w >> 8& 0xFF #高位
        data.append(low)
        data.append(high)

        #h(小端模式)
        low = h & 0xFF #低位
        high = h >> 8& 0xFF #高位
        data.append(low)
        data.append(high)

    #msg
    if msg != None:
        for i in range(len(msg)):
            msg_int = str_2_int(msg[i])
            data.append(msg_int)

    Length += len(data)
    send_buf = [FUNC_num,Length]
    for i in range(len(data)):
        send_buf.append(data[i])

    #进行CRC运算
    crc = checksum_crc8(send_buf)

    send_buf.insert(0,START_1) #插入协议头1
    send_buf.insert(1,START_2) #插入协议头2
    send_buf.append(crc) #加入CRC校验码

    print(send_buf) #打印数据
    serial.send_bytearray(send_buf) #发送数据

##################################### send_func end #####################################

#定义发送数据
send_x = 0
send_y = 0
send_w = 0
send_h = 0
send_msg = ""



#初始化LCD
lcd.init()
#以下是初始化传感器
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 100)

#帧率时钟
clock = time.clock()


#创建图像对象
img_obj = image.Image(size=(320,256))

#创建锚框的尺寸列表，用于目标检测
anchor = (0.893, 1.463, 0.245, 0.389, 1.55, 2.58, 0.375, 0.594, 3.099, 5.038, 0.057, 0.090, 0.567, 0.904, 0.101, 0.160, 0.159, 0.255)

#创建 KPU 模型对象
kpu = KPU()

#加载 YOLO 目标检测模型文件（.kmodel 格式）
kpu.load_kmodel("/sd/KPU/yolo_face_detect/yolo_face_detect.kmodel")

#使用 init_yolo2 初始化 YOLO 模型参数
#anchor: 锚框的尺寸列表，用于目标检测
#anchor_num: 锚框的数量
#img_w, img_h: 输入图像的宽度和高度
#net_w, net_h: 模型输入的宽度和高度
#layer_w, layer_h: 模型最终层的宽度和高度
#threshold: 检测目标的置信度阈值
#nms_value: 非最大抑制的 IOU 阈值
#classes: 目标类别数量
kpu.init_yolo2(anchor, anchor_num=9, img_w=320, img_h=240, net_w=320, net_h=256, layer_w=10, layer_h=8, threshold=0.7, nms_value=0.3, classes=1)


try:
    #loop
    while True:
        clock.tick() #计算每秒帧率
        img = sensor.snapshot() #从相机获取图像
        #将图像数据复制到 img_obj 对象中，以便传递给 KPU 运行
        img_obj.draw_image(img, 0,0)
        img_obj.pix_to_ai()
        #使用 KPU 运行目标检测模型
        kpu.run_with_output(img_obj)
        #获取检测结果
        dect = kpu.regionlayer_yolo2()
        #计算帧率
        fps = clock.fps()
        #如果检测到目标
        if len(dect) > 0:
            for l in dect :
                #在图像上绘制检测到的目标框
                img.draw_rectangle(l[0],l[1],l[2],l[3], color=(0, 255, 0))
                #将方框数据与消息赋值
                send_x = l[0]
                send_y = l[1]
                send_w = l[2]
                send_h = l[3]
            #发送数据
            send_data(send_x,send_y,send_w,send_h,None)

        #在图像上显示帧率
        img.draw_string(0, 0, "%2.1ffps" %(fps), color=(0, 60, 128), scale=2.0)
        #在 LCD 上显示处理后的图像
        lcd.display(img)


#捕获错误并处理
except Exception as e:
    raise e
finally:
    #若出现错误，则释放 KPU 资源
    kpu.deinit()


