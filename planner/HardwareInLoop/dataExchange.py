#!/usr/bin/env python
# -*- coding: utf-8 -*-
import pandas as pd
import json
import socket
from planner.plannerBase import PlannerBase

class DataExchange(PlannerBase):
    def __init__(self):
        """跟socket通信相关的参数及变量设置
        """
        
        SENDER_IP = '192.168.0.124'
        SENDER_PORT = 12345
        RECIVER_IP = '192.168.0.123'
        RECIVER_PORT = 6668
        
        self.sender_address = (SENDER_IP, SENDER_PORT)
        self.receiver_address = (RECIVER_IP, RECIVER_PORT)
        
        # 创建UDP套接字
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # 绑定到接收方的地址和端口
        self.udp_socket.bind(self.receiver_address)

    def act(self, observation):
        frame = pd.DataFrame()
        print("observation:",observation)
        print("vehicle_info:",observation.vehicle_info)

        # 从act方法传入参数observation中获取当前车辆状态信息
        data = observation.vehicle_info

        # 发送数据到UDP服务器
        json_str = json.dumps(data)
        self.udp_socket.sendto(json_str.encode('utf-8'), self.sender_address)
        print("已发送数据:", json_str)

        # 从UDP服务器获取控制信息
        response_data, _ = self.udp_socket.recvfrom(1024)
        # 解码JSON数据
        response_json = json.loads(response_data.decode('utf-8'))
        # 提取加速度和方向盘转角数据
        acceleration = response_json.get("acceleration", 0.0)
        steering_angle = response_json.get("steering_angle", 0.0)

        print("接收到加速度:", acceleration)
        print("接收到方向盘转角:", steering_angle)
        for key, value in observation.vehicle_info.items():
            sub_frame = pd.DataFrame(value, columns=['x', 'y', 'v', 'yaw', 'length', 'width'],index=[key])

            frame = pd.concat([frame, sub_frame])
        state = frame.to_numpy()

        return acceleration, steering_angle
