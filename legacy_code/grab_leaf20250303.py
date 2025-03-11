#!/usr/bin/env python3
#coding=utf-8
import time
from Arm_Lib import Arm_Device
import numpy as np
import ikpy.chain

import argparse
import os
import platform
import shutil
import time
from pathlib import Path

import pyrealsense2 as rs
import numpy as np
import cv2

import torch
import torch.backends.cudnn as cudnn
from numpy import random

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import (
    check_img_size, non_max_suppression, apply_classifier, scale_coords, 
    xyxy2xywh, plot_one_box, strip_optimizer, set_logging, xywh2xyxy)
from utils.torch_utils import select_device, load_classifier, time_synchronized
from matplotlib import pyplot as plt

SERVO_SPEED = 3.5
LINK_LIST = [0,30,83,83,80,90]
TOMATO_SIZE = (0.033, 0.037)  # (单位：米)

GRABBER = {
    'open':80,
    'open_leaf':50,
    'close':160,
    'close_leaf':180,
}
DROP_POSITION = [-0.05,-0.2,0.15]
LEAF_POSITION = [-0.10,0.02,0.15]

def read_servolines(Arm):
    angle = []
    time.sleep(0.02)
    for i in range(6):
        aa = Arm.Arm_serial_servo_read(i+1)
        if aa:
            angle.append(aa)
        else:
            angle.append(0)
        time.sleep(.002)
    time.sleep(.002)
    return angle
    


def servo_write(Arm,angle,servo_speed,s_time=None):
    calculate_time = calculate_servotime(Arm,angle,servo_speed)
    # s_time = 1500
    if s_time:
        Arm.Arm_serial_servo_write6(angle[0], angle[1], angle[2], angle[3], angle[4], angle[5], s_time)
        time.sleep(s_time/1000)
        return s_time
    else:
        Arm.Arm_serial_servo_write6(angle[0], angle[1], angle[2], angle[3], angle[4], angle[5], calculate_time)
        time.sleep(calculate_time/1000)
        return calculate_time
    

def calculate_servotime(Arm,target,servo_speed=3):
    servotime = np.array(read_servolines(Arm))-np.array(target)
    return int(max(max(np.abs(servotime)) *servo_speed*5,500))

def one_step(model,pipeline,align,names,img_save=True):
    
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    # 将帧转换为numpy数组
    color_image = np.asanyarray(color_frame.get_data())
    color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
    depth_intrinsics = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
    device = select_device('')
    # path, img, im0s, vid_cap = LoadStreams(source, img_size=320)
    
    img = torch.tensor(color_image)
    img = np.transpose(img, (2, 0, 1))
    half = device.type != 'cpu'  # half precision only supported on CUDA
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)
    
    img = img.to(device)
    t1 = time_synchronized()
    pred = model(img, augment=True)[0]

    # Apply NMS
    pred = non_max_suppression(pred, 0.88, 0.3, classes=None, agnostic=True)
    t2 = time_synchronized()

    # Process detections
    img = np.array(img.squeeze(0).cpu())
    img = np.transpose(img, (1, 2, 0))
    img = img * 255
    img = img.astype(np.uint8)
    for i, det in enumerate(pred):  # detections per image
        # Print time (inference + NMS)
        print('Done. ({:.3f}s)'.format((t2 - t1)))    
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    if det is not None:
        for *xyxy, conf, cls in reversed(det):
            label = '%s %.2f' % (names[int(cls)], conf)
            print(xyxy)
            plot_one_box(xyxy, img, label=label, line_thickness=3)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    plt.imshow(img)
    plt.show()
    if img_save:
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imwrite("a.png",img)
    return det,aligned_depth_frame,depth_intrinsics



def camera_to_world(P_cam, T_cam, R_cam):
    """
    将相机坐标系下的点转换为世界坐标系。
    
    参数：
        P_cam (numpy.array): 相机坐标系下的点 (3x1)。
        T_cam (numpy.array): 相机在世界坐标系中的位置 (3x1)。
        R_cam (numpy.array): 相机的旋转矩阵 (3x3)。
    
    返回：
        numpy.array: 世界坐标系下的点 (3x1)。
    """
    P_world = R_cam @ P_cam + T_cam
    return P_world

def calculate_frame_coor(det,object_index=2):
    res = []
    for i,(*xyxy, conf, cls) in enumerate(det):
        # 番茄=2,叶子=0,茎=1
        if cls == object_index:
            x = int(xyxy[2]-xyxy[0])
            y = int(xyxy[3]-xyxy[1])
            c_x = int((xyxy[0]+ xyxy[2])/2)
            c_y = int((xyxy[1]+ xyxy[3])/2)
            res.append([i,c_x,c_y,x,y,conf,cls])
    return res

def select_tomato():
    # 根据坐标计算得分并选择番茄,现在是用得分最高的
    # 可以用最靠近一侧的番茄
    pass

def control_arm_coor(Arm,my_chain,target_position,grabber,rotation=90,compensate=2):
    converted_position = np.array(target_position)
    print("target coor:",converted_position)
    joints = my_chain.inverse_kinematics(converted_position,initial_position = np.radians([0,90,120,30,40,90,30]))
    joint_list = joints[1:-2]
    print("servos should be",np.degrees(joint_list))
    joint_deg_list = [int(x)for x in np.degrees(joint_list)]
    joint_deg_list.append(rotation)
    joint_deg_list.append(grabber)
    print("The angles of each joints should be:" , joint_deg_list)
    real_frame = my_chain.forward_kinematics(joints)
    error = np.abs(np.linalg.norm(list(real_frame[:3,3]),ord=2)-np.linalg.norm(converted_position,ord=2))
    print("Error:{:.2f}%".format(error*100))
    print("The position is:\n", real_frame)
    if "{:.2f}%".format(error*0.05) != "0.00%":
        print("out of range")
    else:
        print(joint_deg_list)
        if compensate & all([(x - compensate) < 179 for x in joint_deg_list[1:4]]):
            
            joint_deg_list[1:4] = [(x+compensate) for x in joint_deg_list[1:4]]
            print(f"final joint:{joint_deg_list}")
        return servo_write(Arm,joint_deg_list,SERVO_SPEED),joint_deg_list


def object_axis_angle(xyxy,depth_image, padding=5):
    obj_depth_image = depth_image[(xyxy[1]-padding):(xyxy[3]+padding),(xyxy[0]-padding):(xyxy[2]+padding)]
    hist, bin_edges = np.histogram(obj_depth_image,120,range=(100,600))

    min_val = bin_edges[np.argmax(hist)-3]
    max_val = bin_edges[np.argmax(hist)+3]

    mask = (obj_depth_image > min_val) & (obj_depth_image < max_val)
    # 找到 mask 中为 True 的坐标
    y_coords, x_coords = np.where(mask)
    points = np.column_stack((x_coords, y_coords))  # 转换为点集

    # 计算旋转最小包围矩形
    rect = cv2.minAreaRect(points.astype(np.float32))  # (center, (width, height), angle)

    # 提取矩形参数
    (center_x, center_y), (width, height), angle = rect

    # 短轴的倾斜角计算
    if width < height:
        short_axis_angle = (angle + 90)  # 宽是短轴，直接取 angle
    else:
        short_axis_angle = angle  # 高是短轴，角度加 90°
    print("short_axis_angle:",short_axis_angle)
    return short_axis_angle




def object_position(det,aligned_depth_frame,depth_intrinsics,halfsize=0.007,compen = True,object_index = 2):
    res = calculate_frame_coor(det,object_index)
    
    point_list = []
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    for obj in res:
        *xyxy, conf, cls = det[obj[0]]
        xyxy = [int(x) for x in xyxy]
        depth_value = aligned_depth_frame.get_distance(obj[1], obj[2]) + halfsize
        point = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [obj[1], obj[2]], depth_value)
        axis_angle = object_axis_angle(xyxy,depth_image, padding=20)
        point = [-point[2],point[0],point[1],int(axis_angle)]
        point_list.append(point)
        print(f"Index:{obj[0]}, Pixel: ({obj[1]}, {obj[2]}) -> Point: {point}")
    
    point_list = np.array(point_list)
    

    
    
    
    try:
        filtered_point_list = point_list[point_list[:, 0] > -0.4]

        sorted_point_list = filtered_point_list[np.argsort(filtered_point_list[:, 0])][::-1]
        # 这里只取最近的
        *P_cam,obj_axis_angle = sorted_point_list[0]
    except:
        return None,None
    # 计算旋转矩阵
    T_cam,R_cam = camera_position(Arm,my_chain)

    # 将相机坐标系的点转换为世界坐标系
    P_world = camera_to_world(P_cam, T_cam, R_cam)
    print("Position of the most confident object to be tomato :", np.array(P_world))
    if compen:
        z = P_world[2]
        z = z + (0.35-z)*0.15
        P_world[2] = z
    return np.array(P_world),obj_axis_angle


def camera_position(Arm,my_chain):
    servos = read_servolines(Arm)
    servos = np.radians([0,servos[0],servos[1],servos[2],servos[3],servos[4],servos[5]])
    transformations = my_chain.forward_kinematics(servos, full_kinematics=True)
    servo4_transform = transformations[4]
    
    print(f"关节 {4} 的变换: {servo4_transform},\n坐标:{servo4_transform[:3,3]}")

    camera_offset_z_rotated = np.array([
    [0,  0, -1, 0.04],
    [1,  0,  0, -0.06],
    [0, 1,  0, -0.03],
    [0, 0, 0, 1]
    ])   
    camera_transform = np.dot(servo4_transform, camera_offset_z_rotated)
    camera_position = camera_transform[:3, 3]
    camera_rotation = camera_transform[:3, :3]
    print("摄像头的位置（相对于世界坐标系）：", camera_position)
    return camera_position,camera_rotation

my_chain = ikpy.chain.Chain.from_urdf_file("/home/jetson/code/yolov5/yolov5-3.0/arm_real copy.URDF",active_links_mask=[False,True,True,True,True,True,False])
Arm = Arm_Device()
Arm.Arm_serial_set_torque(1)
servo_write(Arm,[90,90,90,0,90,0],SERVO_SPEED,1500)
# object_position(det,aligned_depth_frame,depth_intrinsics)
camera_position(Arm,my_chain)
# 初始化RealSense管道
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# 启动管道
pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)
control_arm_coor(Arm,my_chain,[-0.05,0.05,0.23],GRABBER['open'])
# target = [135, 96, 29, 22, 90, 150]
# servo_write(Arm,target,SERVO_SPEED)
model = attempt_load('/home/jetson/code/yolov5/yolov5-3.0/0214.pt', map_location=select_device('')).half()  # load FP32 model
# Run inference
img = torch.zeros((1, 3, 640, 640), device=select_device(''))  # init img
_ = model(img.half())
names = ['leaf','stem','tomato',]
import json, time
json_file_path = "/home/jetson/code/data.json"
flag_file_path = "/home/jetson/code/flag.log"
with open(json_file_path, "w") as f:
    json.dump({"Status": "Awaiting"},f)
with open(flag_file_path, "r") as f:
    data = f.readlines()
# 循环等待GET请求
flag_leave = False
while True:
    time.sleep(1)
    with open(flag_file_path, "r") as f:
        new_data = f.readlines()
    if len(new_data) != len(data):
        # 抓叶子并往旁边提
        task_list = [[-0.04,0.07,0.28],[-0.04,0.07,0.23],[-0.04,0.07,0.17]] # !!!!!!modify here to set the origin position
        memory_position_leaf = []
        padding = 10
        while task_list:
            target = task_list[0]
            s_time,target_joint_degree_list = control_arm_coor(Arm,my_chain,target,GRABBER['open'])
            time.sleep(1)
            flag = True

            det,aligned_depth_frame,depth_intrinsics = one_step(model,pipeline,align,names)


            res = calculate_frame_coor(det,object_index=0)
            point_list = []
            depth_image = np.asanyarray(aligned_depth_frame.get_data())

            for obj in res:
                *xyxy, conf, cls = det[obj[0]]
                xyxy = [int(x) for x in xyxy]
                
                # 3. 找到 mask 的边缘
                try:
                    obj_depth_image = depth_image[(xyxy[1]-padding):(xyxy[3]+padding), (xyxy[0]-padding):(xyxy[2]+padding)]
                    hist, bin_edges = np.histogram(obj_depth_image, 120, range=(100, 600))
                    min_val = bin_edges[np.argmax(hist)-2]
                    max_val = bin_edges[np.argmax(hist)+4]

                    mask = (obj_depth_image > min_val) & (obj_depth_image < max_val)
                    edges = cv2.Canny(mask.astype(np.uint8) * 255, 100, 200)  # Canny 检测边缘
                    edge_coords = np.column_stack(np.where(edges > 0))  # 获取边缘像素坐标 (y, x)
                    

                # 4. 在边缘上找到深度最小的点
                    min_depth = float('inf')
                    min_point = None

                    for (y, x) in edge_coords:
                        depth = obj_depth_image[y, x]
                        if depth < min_depth and depth != 0:
                            min_depth = depth
                            min_point = (x, y)  # (x, y) 形式
                    print(f"边缘最小深度点: {min_point}，深度值: {min_depth}")
                except:
                    pass
                if min_point:
                    min_point_original = (min_point[0] + xyxy[0], min_point[1] + xyxy[1])
                    print(f"原图中的最小深度点像素坐标: {min_point_original}")

                    # **2. 获取该像素的深度值**
                    depth_value = aligned_depth_frame.get_distance(min_point_original[0], min_point_original[1]) +0.005
                    print(f"深度值: {depth_value} m")

                    # **3. 计算真实世界坐标**
                    point_3d = rs.rs2_deproject_pixel_to_point(depth_intrinsics, list(min_point_original), depth_value)
                    print(f"3D 坐标: {point_3d}")
                else:
                    print("未找到有效的最小深度点")
                        

                
                print(f"Index:{obj[0]}, Pixel: ({obj[1]}, {obj[2]}) -> Point: {point_3d}")
                axis_angle = object_axis_angle(xyxy,depth_image, padding=0)
                point = [-point_3d[2],point_3d[0],point_3d[1],int(axis_angle)]
                print(f"point:{point}")
                point_list.append(point)

            point_list = np.array(point_list)
            filtered_point_list = point_list[point_list[:, 0] > -0.4]
            print(filtered_point_list)
            sorted_point_list = filtered_point_list[np.argsort(filtered_point_list[:, 0])][::-1]
            # 这里只取最近的
            *P_cam,obj_axis_angle = sorted_point_list[0]
            # 计算旋转矩阵
            T_cam,R_cam = camera_position(Arm,my_chain)

            # 将相机坐标系的点转换为世界坐标系
            P_world = camera_to_world(P_cam, T_cam, R_cam)
            # 保存结果并继续寻找
            memory_position_leaf.append([P_world,axis_angle])
            
            print(f"Leaf on task point:{task_list.pop(0)} is finished")


        print(memory_position_leaf)

        if memory_position_leaf is not None:
            control_arm_coor(Arm,my_chain,memory_position_leaf[-1][0],GRABBER['open_leaf'],obj_axis_angle)
            control_arm_coor(Arm,my_chain,memory_position_leaf[-1][0],GRABBER['close_leaf'],obj_axis_angle)
            control_arm_coor(Arm,my_chain,LEAF_POSITION,GRABBER['close_leaf'],obj_axis_angle)
            flag_leave = True
        with open(json_file_path, "w") as f:
            json.dump({"Status": "OK"},f)
        
        break
    if flag_leave:
        break

with open(json_file_path, "w") as f:
    json.dump({"Status": "Awaiting"},f)