#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import cv2
import gc
import numpy as np
from multiprocessing import Process, Manager

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

# https://lukeyalvin.blog.csdn.net/article/details/116052377

def write(stack, cam, top):
    """
    :param cam: 摄像头参数
    :param stack: Manager.list对象
    :param top: 缓冲栈容量
    :return: None
    """
    print('Process to write: %s' % os.getpid())
    cap = cv2.VideoCapture(cam)
    while True:
        _, img = cap.read()
        if _:
            stack.append(img)
            # 每到一定容量清空一次缓冲栈
            # 利用gc库，手动清理内存垃圾，防止内存溢出
            if len(stack) >= top:
                del stack[:]
                gc.collect()


def img_resize(image):
    height, width = image.shape[0], image.shape[1]
    # 设置新的图片分辨率框架 640x369 1280×720 1920×1080
    width_new = 1280
    height_new = 720
    # 判断图片的长宽比率
    if width / height >= width_new / height_new:
        img_new = cv2.resize(image, (width_new, int(height * width_new / width)))
    else:
        img_new = cv2.resize(image, (int(width * height_new / height), height_new))
    return img_new


# 在缓冲栈中读取数据:
def read(stack):
    print('Process to read: %s' % os.getpid())
    pub = rospy.Publisher("/camera/hik_image", Image, queue_size=1)
    rospy.init_node("HIK", anonymous=True)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if len(stack) != 0:
            frame = stack.pop()
            frame_new = img_resize(frame)

            image_temp = Image()
            header = Header(stamp=rospy.Time.now())
            header.frame_id = 'map'
            image_temp.height = 720
            image_temp.width = 1280
            image_temp.encoding = 'bgr8'
            image_temp.data = np.array(frame_new).tostring()

            image_temp.header = header
            image_temp.step = 1280*3
            pub.publish(image_temp)
            rate.sleep()

# 在缓冲栈中读取数据:
# def read(stack):
#     print('Process to read: %s' % os.getpid())
#     while True:
#         if len(stack) != 0:
#             value = stack.pop()
#             # 对获取的视频帧分辨率重处理
#             img_new = img_resize(value)
#             # 使用yolo模型处理视频帧
#             # yolo_img = yolo_deal(img_new)
#             # 显示处理后的视频帧
#             cv2.imshow("img", img_new)
#             # 将处理的视频帧存放在文件夹里
#             # save_img(img_new)
#             key = cv2.waitKey(1) & 0xFF
#             if key == ord('q'):
#                 break


if __name__ == '__main__':
    

    # 父进程创建缓冲栈，并传给各个子进程：
    q = Manager().list()
    url = "rtsp://admin:js123456@192.168.50.64:554/h264/ch1/main/av_stream"
    pw = Process(target=write, args=(q, url, 100))
    pr = Process(target=read, args=(q,))
    # 启动子进程pw，写入:
    pw.start()
    # 启动子进程pr，读取:
    pr.start()

    # 等待pr结束:
    pr.join()

    # pw进程里是死循环，无法等待其结束，只能强行终止:
    pw.terminate()
