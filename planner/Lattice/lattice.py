#! /usr/bin/env python3
# _*_ coding: utf-8 _*_

import math
import numpy as np
import matplotlib.pyplot as plt
import time
import scipy.signal
import scipy.optimize as opt
from scipy.integrate import quad
from scipy.interpolate import splrep, splev
from planner.plannerBase import PlannerBase
import pandas as pd

M_PI = 3.141593

# 车辆属性, global const. and local var. check!
VEH_L = 1  # length
VEH_W = 0.5  # width
MAX_V = 200
MIN_V = -2000
'''
MAX_A = 10
MIN_A = -20
MAX_LAT_A = 100 #参考apollo，横向约束应该是给到向心加速度，而不是角速度
'''

# cost权重
SPEED_COST_WEIGHT = 1  # 速度和目标速度差距，暂时不用
DIST_TRAVEL_COST_WEIGHT = 1  # 实际轨迹长度，暂时不用
LAT_COMFORT_COST_WEIGHT = 1  # 横向舒适度
LAT_OFFSET_COST_WEIGHT = 1  # 横向偏移量

# 前四个是中间计算时用到的权重，后三个是最终合并时用到的
LON_OBJECTIVE_COST_WEIGHT = 1  # 纵向目标cost，暂时不用
LAT_COST_WEIGHT = 1  # 横向约束，包括舒适度和偏移量
LON_COLLISION_COST_WEIGHT = 1  # 碰撞cost
DESTINATION_WEIGHT = 1

def NormalizeAngle(angle_rad):
    # to normalize an angle to [-pi, pi]
    a = math.fmod(angle_rad + M_PI, 2.0 * M_PI)
    if a < 0.0:
        a = a + 2.0 * M_PI
    return a - M_PI

def Dist(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

class PathPoint:
    def __init__(self, pp_list):
        # pp_list: from CalcRefLine, [rx, ry, rs, rtheta, rkappa, rdkappa] x y 路程 角度 角度变化量/路程变化量 (角度变化量/路程变化量)/路程变化量
        self.rx = pp_list[0]
        self.ry = pp_list[1]
        self.rs = pp_list[2]
        self.rtheta = pp_list[3]
        self.rkappa = pp_list[4]
        self.rdkappa = pp_list[5]

class TrajPoint:
    def __init__(self, tp_list):
        # tp_list: from sensors, [x, y, v, a, theta, kappa]
        self.x = tp_list[0]
        self.y = tp_list[1]
        self.v = tp_list[2]
        self.a = tp_list[3]
        self.theta = tp_list[4]
        self.kappa = tp_list[5]

    def MatchPath(self, path_points):
        '''
        find the closest/projected point on the reference path
        the deviation is not large; the curvature is not large
        '''

        def DistSquare(traj_point, path_point):
            dx = path_point.rx - traj_point.x
            dy = path_point.ry - traj_point.y
            return (dx ** 2 + dy ** 2)

        dist_all = []
        for path_point in path_points:
            dist_all.append(DistSquare(self, path_point))
        dist_min = DistSquare(self, path_points[0])
        index_min = 0
        for index, path_point in enumerate(path_points):
            dist_temp = DistSquare(self, path_point)
            if dist_temp < dist_min:
                dist_min = dist_temp
                index_min = index
        path_point_min = path_points[index_min]
        if index_min == 0 or index_min == len(path_points) - 1:
            self.matched_point = path_point_min
        else:
            path_point_next = path_points[index_min + 1]
            path_point_last = path_points[index_min - 1]
            vec_p2t = np.array([self.x - path_point_min.rx, self.y - path_point_min.ry])
            vec_p2p_next = np.array([path_point_next.rx - path_point_min.rx, path_point_next.ry - path_point_min.ry])
            vec_p2p_last = np.array([path_point_last.rx - path_point_min.rx, path_point_last.ry - path_point_min.ry])
            if np.dot(vec_p2t, vec_p2p_next) * np.dot(vec_p2t, vec_p2p_last) >= 0:
                self.matched_point = path_point_min
            else:
                if np.dot(vec_p2t, vec_p2p_next) >= 0:
                    rs_inter = path_point_min.rs + np.dot(vec_p2t, vec_p2p_next / np.linalg.norm(vec_p2p_next))
                    self.matched_point = LinearInterpolate(path_point_min, path_point_next, rs_inter)
                else:
                    rs_inter = path_point_min.rs - np.dot(vec_p2t, vec_p2p_last / np.linalg.norm(vec_p2p_last))
                    self.matched_point = LinearInterpolate(path_point_last, path_point_min, rs_inter)
        return self.matched_point

    def LimitTheta(self, theta_thr=M_PI / 6):
        # limit the deviation of traj_point.theta from the matched path_point.rtheta within theta_thr
        if self.theta - self.matched_point.rtheta > theta_thr:
            self.theta = NormalizeAngle(self.matched_point.rtheta + theta_thr)  # upper limit of theta
        elif self.theta - self.matched_point.rtheta < -theta_thr:
            self.theta = NormalizeAngle(self.matched_point.rtheta - theta_thr)  # lower limit of theta
        else:
            pass  # maintained, actual theta should not deviate from the path rtheta too much

    def IsOnPath(self, dist_thr=0.5):
        # whether the current traj_point is on the path
        dx = self.matched_point.rx - self.x
        dy = self.matched_point.ry - self.y
        dist = math.sqrt(dx ** 2 + dy ** 2)
        if dist <= dist_thr:
            return True
        else:
            return False

# 障碍物类
class Obstacle():
    def __init__(self, obstacle_info):
        self.x = obstacle_info[0]
        self.y = obstacle_info[1]
        self.v = obstacle_info[2]
        self.length = obstacle_info[3]
        self.width = obstacle_info[4]
        self.heading = obstacle_info[5]  # 这里设定朝向是length的方向，也是v的方向
        self.type = obstacle_info[6]
        self.corner = self.GetCorner()

    def GetCorner(self):
        cos_o = math.cos(self.heading)
        sin_o = math.sin(self.heading)
        dx3 = cos_o * self.length / 2
        dy3 = sin_o * self.length / 2
        dx4 = sin_o * self.width / 2
        dy4 = -cos_o * self.width / 2
        return [self.x - (dx3 - dx4), self.y - (dy3 - dy4)]

    def MatchPath(self, path_points):
        '''
        find the closest/projected point on the reference path
        the deviation is not large; the curvature is not large
        '''

        def DistSquare(traj_point, path_point):
            dx = path_point.rx - traj_point.x
            dy = path_point.ry - traj_point.y
            return (dx ** 2 + dy ** 2)

        dist_all = []
        for path_point in path_points:
            dist_all.append(DistSquare(self, path_point))  # 求障碍物到reference line的各个点距
        dist_min = DistSquare(self, path_points[0])  # 与第一个参考点的距离
        index_min = 0
        for index, path_point in enumerate(path_points):  # 求最近的参考点
            dist_temp = DistSquare(self, path_point)
            if dist_temp < dist_min:
                dist_min = dist_temp
                index_min = index
        path_point_min = path_points[index_min]  # 得到障碍物到reference line的最短距离
        if index_min == 0 or index_min == len(path_points) - 1:
            self.matched_point = path_point_min
        else:
            path_point_next = path_points[index_min + 1]  # 上一时刻参考点和下一时刻参考点
            path_point_last = path_points[index_min - 1]
            vec_p2t = np.array([self.x - path_point_min.rx, self.y - path_point_min.ry])
            vec_p2p_next = np.array([path_point_next.rx - path_point_min.rx, path_point_next.ry - path_point_min.ry])
            vec_p2p_last = np.array([path_point_last.rx - path_point_min.rx, path_point_last.ry - path_point_min.ry])
            if np.dot(vec_p2t, vec_p2p_next) * np.dot(vec_p2t, vec_p2p_last) >= 0:
                self.matched_point = path_point_min
            else:
                if np.dot(vec_p2t, vec_p2p_next) >= 0:
                    rs_inter = path_point_min.rs + np.dot(vec_p2t, vec_p2p_next / np.linalg.norm(vec_p2p_next))
                    self.matched_point = LinearInterpolate(path_point_min, path_point_next, rs_inter)
                else:
                    rs_inter = path_point_min.rs - np.dot(vec_p2t, vec_p2p_last / np.linalg.norm(vec_p2p_last))
                    self.matched_point = LinearInterpolate(path_point_last, path_point_min, rs_inter)
        return self.matched_point

def CartesianToFrenet(path_point, traj_point):
    ''' from Cartesian to Frenet coordinate, to the matched path point
    copy Apollo cartesian_frenet_conversion.cpp'''
    rx, ry, rs, rtheta, rkappa, rdkappa = path_point.rx, path_point.ry, path_point.rs, \
                                          path_point.rtheta, path_point.rkappa, path_point.rdkappa
    x, y, v, a, theta, kappa = traj_point.x, traj_point.y, traj_point.v, \
                               traj_point.a, traj_point.theta, traj_point.kappa

    s_condition = np.zeros(3)
    d_condition = np.zeros(3)

    dx = x - rx
    dy = y - ry

    cos_theta_r = math.cos(rtheta)
    sin_theta_r = math.sin(rtheta)

    cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx
    d_condition[0] = math.copysign(math.sqrt(dx ** 2 + dy ** 2), cross_rd_nd)

    delta_theta = theta - rtheta
    tan_delta_theta = math.tan(delta_theta)
    cos_delta_theta = math.cos(delta_theta)

    one_minus_kappa_r_d = 1 - rkappa * d_condition[0]
    d_condition[1] = one_minus_kappa_r_d * tan_delta_theta

    kappa_r_d_prime = rdkappa * d_condition[0] + rkappa * d_condition[1]

    d_condition[2] = -kappa_r_d_prime * tan_delta_theta + one_minus_kappa_r_d / (cos_delta_theta ** 2) * \
                     (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa)

    s_condition[0] = rs
    s_condition[1] = v * cos_delta_theta / one_minus_kappa_r_d

    delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa
    s_condition[2] = (a * cos_delta_theta - s_condition[1] ** 2 * \
                      (d_condition[1] * delta_theta_prime - kappa_r_d_prime)) / one_minus_kappa_r_d

    return s_condition, d_condition

def FrenetToCartesian(path_point, s_condition, d_condition):
    ''' from Frenet to Cartesian coordinate
    copy Apollo cartesian_frenet_conversion.cpp'''
    rx, ry, rs, rtheta, rkappa, rdkappa = path_point.rx, path_point.ry, path_point.rs, \
                                          path_point.rtheta, path_point.rkappa, path_point.rdkappa
    if math.fabs(rs - s_condition[0]) >= 1.0e-6:
        pass
        # print("the reference point s and s_condition[0] don't match")

    cos_theta_r = math.cos(rtheta)
    sin_theta_r = math.sin(rtheta)

    x = rx - sin_theta_r * d_condition[0]
    y = ry + cos_theta_r * d_condition[0]

    one_minus_kappa_r_d = 1 - rkappa * d_condition[0]
    tan_delta_theta = d_condition[1] / one_minus_kappa_r_d
    delta_theta = math.atan2(d_condition[1], one_minus_kappa_r_d)
    cos_delta_theta = math.cos(delta_theta)
    theta = NormalizeAngle(delta_theta + rtheta)

    kappa_r_d_prime = rdkappa * d_condition[0] + rkappa * d_condition[1]
    kappa = ((d_condition[2] + kappa_r_d_prime * tan_delta_theta) * cos_delta_theta ** 2 / one_minus_kappa_r_d \
             + rkappa) * cos_delta_theta / one_minus_kappa_r_d

    d_dot = d_condition[1] * s_condition[1]
    v = math.sqrt((one_minus_kappa_r_d * s_condition[1]) ** 2 + d_dot ** 2)

    delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa
    a = s_condition[2] * one_minus_kappa_r_d / cos_delta_theta + s_condition[1] ** 2 / cos_delta_theta * \
        (d_condition[1] * delta_theta_prime - kappa_r_d_prime)

    tp_list = [x, y, v, a, theta, kappa]
    return TrajPoint(tp_list)

def CalcRefLine(cts_points):  # 输入参考轨迹的x y 计算rs/rtheta/rkappa/rdkappa 此时是笛卡尔坐标系 rs为已走路程 rtheta为角度
    '''
    deal with reference path points 2d-array
    to calculate rs/rtheta/rkappa/rdkappa according to cartesian points
    '''
    rx = cts_points[0]  # the x value
    ry = cts_points[1]  # the y value
    rs = np.zeros_like(rx)
    rtheta = np.zeros_like(rx)
    rkappa = np.zeros_like(rx)
    rdkappa = np.zeros_like(rx)
    for i, x_i in enumerate(rx):
        # y_i = ry[i]
        if i != 0:
            dx = rx[i] - rx[i - 1]
            dy = ry[i] - ry[i - 1]
            rs[i] = rs[i - 1] + math.sqrt(dx ** 2 + dy ** 2)
        if i < len(ry) - 1:
            dx = rx[i + 1] - rx[i]
            dy = ry[i + 1] - ry[i]
            ds = math.sqrt(dx ** 2 + dy ** 2)
            rtheta[i] = math.copysign(math.acos(dx / ds),
                                      dy)  # acos求角度 copysign功能为返回第一个输入的值和第二个输入的符号(即dy>0在0-pi dy<0在-pi-0)
    if len(rtheta) > 2:
        rtheta[-1] = rtheta[-2]  # 最后一个时刻的角度没法求就直接等于倒数第二个时刻
        rkappa[:-1] = np.diff(rtheta) / np.diff(rs)  # 角度变化量/路程变化量
        rdkappa[:-1] = np.diff(rkappa) / np.diff(rs)
        rkappa[-1] = rkappa[-2]
        rdkappa[-1] = rdkappa[-3]
        rdkappa[-2] = rdkappa[-3]
    if len(rkappa) > 333:
        window_length = 333
    elif len(rkappa) % 2 == 0:
        window_length = len(rkappa) - 1
    else:
        window_length = len(rkappa)
    polyorder = 5
    if window_length <= polyorder:
        polyorder = window_length - 1
    rkappa = scipy.signal.savgol_filter(rkappa, window_length, polyorder)  # 平滑
    rdkappa = scipy.signal.savgol_filter(rdkappa, window_length, polyorder)
    path_points = []
    for i in range(len(rx)):
        path_points.append(PathPoint([rx[i], ry[i], rs[i], rtheta[i], rkappa[i], rdkappa[i]]))  # 生成笛卡尔坐标系下的参考轨迹点
    return path_points

def LinearInterpolate(path_point_0, path_point_1, rs_inter):
    ''' path point interpolated linearly according to rs value
    path_point_0 should be prior to path_point_1'''

    def lerp(x0, x1, w):
        return x0 + w * (x1 - x0)

    def slerp(a0, a1, w):
        # angular, for theta
        a0_n = NormalizeAngle(a0)
        a1_n = NormalizeAngle(a1)
        d = a1_n - a0_n
        if d > M_PI:
            d = d - 2 * M_PI
        elif d < -M_PI:
            d = d + 2 * M_PI
        a = a0_n + w * d
        return NormalizeAngle(a)

    rs_0 = path_point_0.rs
    rs_1 = path_point_1.rs
    weight = (rs_inter - rs_0) / (rs_1 - rs_0)
    if weight < 0 or weight > 1:
        print("weight error, not in [0, 1]")
        quit()
    rx_inter = lerp(path_point_0.rx, path_point_1.rx, weight)
    ry_inter = lerp(path_point_0.ry, path_point_1.ry, weight)
    rtheta_inter = slerp(path_point_0.rtheta, path_point_1.rtheta, weight)
    rkappa_inter = lerp(path_point_0.rkappa, path_point_1.rkappa, weight)
    rdkappa_inter = lerp(path_point_0.rdkappa, path_point_1.rdkappa, weight)
    return PathPoint([rx_inter, ry_inter, rs_inter, rtheta_inter, rkappa_inter, rdkappa_inter])

def TrajObsFree(xoy_traj, obstacle, delta_t):  ### 输入为路径点 障碍物类 帧长
    dis_sum = 0
    for point in xoy_traj:
        if isinstance(point, PathPoint):  # 如果是原来路径点，就只按圆形计算。因为每点的车辆方向难以获得
            ### isinstance 当参数1和参数2是同一类型时返回True
            if ColliTestRough(point, obstacle) > 0:  ### 返回point与obstacle的距离
                continue
            return 0, False
        else:
            dis = ColliTestRough(point,
                                 obstacle)  ### isinstance执行时是路径点与障碍物的距离(是否碰撞) 而else里point不再是路径点(与PathPoint不同类)因此是车辆的位置
            dis_sum += dis
            if dis > 0:
                continue
            if ColliTest(point, obstacle):  ### 对于车辆与障碍物是否碰撞 ColliTestRough不足以(将两者视为圆形) 要用更准确的ColliTest检测是否碰撞
                # print("不满足实际碰撞检测")
                return 0, False
    if len(xoy_traj) != 0:
        dis_mean = dis_sum / len(xoy_traj)
    else:
        return 0, False
    # print("满足实际碰撞检测")
    return dis_mean, True

# 粗略的碰撞检测(视作圆形)  如果此时不碰撞，就无需按矩形检测。返回的距离作为该点车到障碍物的大致距离（无碰撞时也可能为负）
def ColliTestRough(point, obs):
    if isinstance(point, PathPoint):
        dis = math.sqrt((point.rx - obs.x) ** 2 + (point.ry - obs.y) ** 2)
    else:
        dis = math.sqrt((point.x - obs.x) ** 2 + (point.y - obs.y) ** 2)
    global VEH_L, VEH_W
    max_veh = max(VEH_L, VEH_W)
    max_obs = max(obs.length, obs.width)
    return dis - (max_veh + max_obs) / 2

# 碰撞检测 (这部分参考apollo代码)
def ColliTest(point, obs):
    shift_x = obs.x - point.x
    shift_y = obs.y - point.y

    global VEH_L, VEH_W
    cos_v = math.cos(point.theta)
    sin_v = math.sin(point.theta)
    cos_o = math.cos(obs.heading)
    sin_o = math.sin(obs.heading)
    half_l_v = VEH_L / 2
    half_w_v = VEH_W / 2
    half_l_o = obs.length / 2
    half_w_o = obs.width / 2

    dx1 = cos_v * VEH_L / 2
    dy1 = sin_v * VEH_L / 2
    dx2 = sin_v * VEH_W / 2
    dy2 = -cos_v * VEH_W / 2
    dx3 = cos_o * obs.length / 2
    dy3 = sin_o * obs.length / 2
    dx4 = sin_o * obs.width / 2
    dy4 = -cos_o * obs.width / 2

    # 使用分离轴定理进行碰撞检测
    return ((abs(shift_x * cos_v + shift_y * sin_v) <=
             abs(dx3 * cos_v + dy3 * sin_v) + abs(dx4 * cos_v + dy4 * sin_v) + half_l_v)
            and (abs(shift_x * sin_v - shift_y * cos_v) <=
                 abs(dx3 * sin_v - dy3 * cos_v) + abs(dx4 * sin_v - dy4 * cos_v) + half_w_v)
            and (abs(shift_x * cos_o + shift_y * sin_o) <=
                 abs(dx1 * cos_o + dy1 * sin_o) + abs(dx2 * cos_o + dy2 * sin_o) + half_l_o)
            and (abs(shift_x * sin_o - shift_y * cos_o) <=
                 abs(dx1 * sin_o - dy1 * cos_o) + abs(dx2 * sin_o - dy2 * cos_o) + half_w_o))

# 对符合碰撞和约束限制的轨迹对进行cost排序，目前只保留了碰撞和横向两个cost ### 取cost min作为opt traj
def CostSorting(traj_pairs):
    cost_dict = {}
    num = 0
    global LAT_COST_WEIGHT, LON_COLLISION_COST_WEIGHT
    global destination, DESTINATION_WEIGHT
    for i in traj_pairs:  # traj_pairs [0]是poly_traj [1]是到障碍物的dis_mean
        traj = i[0]
        lat_cost = traj.lat_cost  # 横向偏移和横向加速度
        lon_collision_cost = -i[1]  # 碰撞风险：apollo的较复杂，这里直接用轨迹上各点到障碍物圆的平均距离表示 这里
        x = np.mean([tp.x for tp in traj.tp_all])
        y = np.mean([tp.y for tp in traj.tp_all])
        lon_destination_cost = math.sqrt((x - destination[0]) ** 2 + (y - destination[1]) ** 2)
        cost_dict[
            num] = lat_cost * LAT_COST_WEIGHT + lon_collision_cost * LON_COLLISION_COST_WEIGHT + lon_destination_cost * DESTINATION_WEIGHT
        num += 1
    cost_list_sorted = sorted(cost_dict.items(), key=lambda d: d[1], reverse=False)
    return cost_list_sorted  # [0]表示原编号num [1]表示该traj的损失函数值

class PolyTraj:
    def __init__(self, s_cond_init, d_cond_init, total_t):
        self.s_cond_init = s_cond_init
        self.d_cond_init = d_cond_init
        self.total_t = total_t  # to plan how long in seconds
        self.delta_s = 0

    def __QuinticPolyCurve(self, y_cond_init, y_cond_end, x_dur):  ### 五次多项式拟合
        '''
        form the quintic polynomial curve: y(x) = a0 + a1 * delta_x + ... + a5 * delta_x ** 5, x_dur = x_end - x_init
        y_cond = np.array([y, y', y'']), output the coefficients a = np.array([a0, ..., a5])
        '''
        a0 = y_cond_init[0]
        a1 = y_cond_init[1]
        a2 = 1.0 / 2 * y_cond_init[2]
        T = x_dur
        if T != 0:
            h = y_cond_end[0] - y_cond_init[0]
            v0 = y_cond_init[1]
            v1 = y_cond_end[1]
            acc0 = y_cond_init[2]
            acc1 = y_cond_end[2]
            # print(x_dur)
            a3 = 1.0 / (2 * T ** 3) * (20 * h - (8 * v1 + 12 * v0) * T - (3 * acc0 - acc1) * T ** 2)
            a4 = 1.0 / (2 * T ** 4) * (-30 * h + (14 * v1 + 16 * v0) * T + (3 * acc0 - 2 * acc1) * T ** 2)
            a5 = 1.0 / (2 * T ** 5) * (12 * h - 6 * (v1 + v0) * T + (acc1 - acc0) * T ** 2)
        else:  # 有时由于delta_s(采样距离=0) 导致total_t = delta_s/v_tgt 使T=0
            a3 = 0
            a4 = 0
            a5 = 0
        return np.array([a0, a1, a2, a3, a4, a5])

    def GenLongTraj(self, s_cond_end):
        self.long_coef = self.__QuinticPolyCurve(self.s_cond_init, s_cond_end,
                                                 self.total_t)  ### self.long_coef为五次多项式的参数
        self.delta_s = self.long_coef[1] * self.total_t + self.long_coef[2] * self.total_t ** 2 + \
                       self.long_coef[3] * self.total_t ** 3 + self.long_coef[4] * self.total_t ** 4 + \
                       self.long_coef[5] * self.total_t ** 5
        # return self.long_coef

    def GenLatTraj(self, d_cond_end):
        # GenLatTraj should be posterior to GenLongTraj
        self.lat_coef = self.__QuinticPolyCurve(self.d_cond_init, d_cond_end, self.delta_s)
        # return self.lat_coef

    # 求各阶导数
    def Evaluate(self, coef, order, t):
        if order == 0:
            return ((((coef[5] * t + coef[4]) * t + coef[3]) * t
                     + coef[2]) * t + coef[1]) * t + coef[0]
        if order == 1:
            return (((5 * coef[5] * t + 4 * coef[4]) * t + 3 *
                     coef[3]) * t + 2 * coef[2]) * t + coef[1]
        if order == 2:
            return (((20 * coef[5] * t + 12 * coef[4]) * t)
                    + 6 * coef[3]) * t + 2 * coef[2]
        if order == 3:
            return (60 * coef[5] * t + 24 * coef[4]) * t + 6 * coef[3]
        if order == 4:
            return 120 * coef[5] * t + 24 * coef[4]
        if order == 5:
            return 120 * coef[5]

    # 纵向速度&加速度约束
    def LongConsFree(self, delta_t):
        size = int(self.total_t / delta_t)
        global MAX_V, MIN_V
        for i in range(size):
            v = self.Evaluate(self.long_coef, 1, i * delta_t)
            # print(v)
            if v > MAX_V or v < MIN_V:
                print(v, "纵向速度超出约束")
                return False
            '''
            加速度约束暂时删去
            a = self.Evaluate(self.long_coef,2, i*delta_t)
            if a > MAX_A or a < MIN_A:
                print("纵向加速度超出约束")
                return False
            '''
        return True

    # 横向加速度约束，参考apollo。这里把横向的cost一块算了
    # 横向偏移量和横向加速度cost同样参考apollo，数学上做了一些简化，如省略了偏移量绝对值，只计算平方；忽略和起点之间的偏移量关系等
    def LatConsFree(self, delta_t):
        size = int(self.total_t / delta_t)
        lat_offset_cost = 0
        lat_comfort_cost = 0
        global LAT_COMFORT_COST_WEIGHT, LAT_OFFSET_COST_WEIGHT
        for i in range(size):
            s = self.Evaluate(self.long_coef, 0, i * delta_t)
            d = self.Evaluate(self.lat_coef, 0, s)
            dd_ds = self.Evaluate(self.lat_coef, 1, s)
            ds_dt = self.Evaluate(self.long_coef, 1, i * delta_t)
            d2d_ds2 = self.Evaluate(self.lat_coef, 2, s)
            d2s_dt2 = self.Evaluate(self.long_coef, 2, i * delta_t)

            lat_a = d2d_ds2 * ds_dt * ds_dt + dd_ds * d2s_dt2
            '''
            向心加速度暂时删去
            if abs(lat_a) > MAX_LAT_A:
                print(lat_a, "不满足横向约束")
                return False
            '''
            lat_comfort_cost += lat_a * lat_a
            lat_offset_cost += d * d

        self.lat_cost = lat_comfort_cost * LAT_COMFORT_COST_WEIGHT
        + lat_offset_cost * LAT_OFFSET_COST_WEIGHT
        # print("满足横向约束")
        return True

    def GenCombinedTraj(self, path_points, delta_t):
        '''
        combine long and lat traj together
        F2C function is used to output future traj points in a list to follow
        '''
        a0_s, a1_s, a2_s, a3_s, a4_s, a5_s = self.long_coef[0], self.long_coef[1], self.long_coef[2], \
                                             self.long_coef[3], self.long_coef[4], self.long_coef[5]
        a0_d, a1_d, a2_d, a3_d, a4_d, a5_d = self.lat_coef[0], self.lat_coef[1], self.lat_coef[2], \
                                             self.lat_coef[3], self.lat_coef[4], self.lat_coef[5]

        rs_pp_all = []  # the rs value of all the path points
        for path_point in path_points:
            rs_pp_all.append(path_point.rs)
        rs_pp_all = np.array(rs_pp_all)
        num_points = math.floor(self.total_t / delta_t)  ### 规划时长/帧长 = 规划点数
        s_cond_all = []  # possibly useless
        d_cond_all = []  # possibly useless
        pp_inter = []  # possibly useless
        tp_all = []  # all the future traj points in a list
        t, s = 0, 0  # initialize variables, s(t), d(s) or l(s)
        for i in range(int(num_points)):
            s_cond = np.zeros(3)
            d_cond = np.zeros(3)

            t = t + delta_t
            s_cond[0] = a0_s + a1_s * t + a2_s * t ** 2 + a3_s * t ** 3 + a4_s * t ** 4 + a5_s * t ** 5  # 路程
            s_cond[1] = a1_s + 2 * a2_s * t + 3 * a3_s * t ** 2 + 4 * a4_s * t ** 3 + 5 * a5_s * t ** 4  # 速度(d路程/dt)
            s_cond[2] = 2 * a2_s + 6 * a3_s * t + 12 * a4_s * t ** 2 + 20 * a5_s * t ** 3  # a
            s_cond_all.append(s_cond)

            s = s_cond[0] - a0_s
            d_cond[0] = a0_d + a1_d * s + a2_d * s ** 2 + a3_d * s ** 3 + a4_d * s ** 4 + a5_d * s ** 5
            d_cond[1] = a1_d + 2 * a2_d * s + 3 * a3_d * s ** 2 + 4 * a4_d * s ** 3 + 5 * a5_d * s ** 4
            d_cond[2] = 2 * a2_d + 6 * a3_d * s + 12 * a4_d * s ** 2 + 20 * a5_d * s ** 3
            d_cond_all.append(d_cond)

            index_min = np.argmin(np.abs(rs_pp_all - s_cond[0]))
            path_point_min = path_points[index_min]  ### 现在到哪个位置了
            if index_min == 0 or index_min == len(path_points) - 1:
                path_point_inter = path_point_min
            else:
                if s_cond[0] >= path_point_min.rs:
                    path_point_next = path_points[index_min + 1]
                    path_point_inter = LinearInterpolate(path_point_min, path_point_next, s_cond[0])
                else:
                    path_point_last = path_points[index_min - 1]
                    path_point_inter = LinearInterpolate(path_point_last, path_point_min, s_cond[0])
            pp_inter.append(path_point_inter)
            traj_point = FrenetToCartesian(path_point_inter, s_cond, d_cond)
            # traj_point.v = v_tgt
            tp_all.append(traj_point)
        self.tp_all = tp_all
        return tp_all


class SampleBasis:
    # the basis of sampling: theta, dist, d_end (, v_end); normally for the planning_out cruising case
    def __init__(self, traj_point, theta_thr, ttcs):
        global v_tgt  ### 目标速度
        traj_point.LimitTheta(theta_thr)
        self.theta_samp = [NormalizeAngle(traj_point.theta - theta_thr),
                           NormalizeAngle(traj_point.theta - theta_thr / 2),
                           traj_point.theta, NormalizeAngle(traj_point.theta + theta_thr / 2),
                           NormalizeAngle(traj_point.theta + theta_thr)]
        planning_horizon = 2  # 2s
        ### NormalizeAngle将角度转化为[-pi,pi] 角度的采样区间为原轨迹点theta下[-theta_thr,-theta_thr/2,0,theta_thr/2,theta_thr]即最大转向角为theta_thr
        self.dist_samp = [v_tgt * ttc for ttc in ttcs]
        # self.dist_samp = [(traj_point.v + 0.5 * ttc) * planning_horizon for ttc in
        #                   ttcs]  ### 距离的采样区间为目标速度*ttcs区间(这里为3s 4s 5s)
        # for i in range(len(self.dist_samp)):
        #     if self.dist_samp[i] <= 0:
        #         self.dist_samp[i] = 0
        # self.dist_samp = [v_tgt*ttc for ttc in ttcs]
        self.dist_prvw = 5  # self.dist_samp[0]  # 最小的距离采样
        self.d_end_samp = [0]
        self.v_end = v_tgt  # v_tgt  # for cruising


class LocalPlanner:
    def __init__(self, traj_point, path_points, obstacles, samp_basis):
        self.traj_point_theta = traj_point.theta  # record the current heading
        self.traj_point = traj_point
        self.path_points = path_points
        self.obstacles = obstacles
        self.theta_samp = samp_basis.theta_samp
        self.dist_samp = samp_basis.dist_samp
        self.d_end_samp = samp_basis.d_end_samp
        self.v_end = samp_basis.v_end
        self.polytrajs = []
        self.__JudgeStatus(traj_point, path_points, obstacles, samp_basis)

    def __JudgeStatus(self, traj_point, path_points, obstacles,
                      samp_basis):  ### 赋值self.status和self.dist_prvw和self.to_Stop# 分别表示车辆的位置关系 最小的采样距离
        colli = 0
        global delta_t, sight_range  ### 每帧时长 可视距离(sight_range下有无障碍物)
        path_point_end = self.path_points[-1]
        if path_point_end.rs - self.traj_point.matched_point.rs <= samp_basis.dist_prvw:  ### 如果快到参考轨迹的末端了(小于最小采样空间的距离) 则准备停车
            self.to_stop = True  # stopping
            self.dist_prvw = path_point_end.rs - traj_point.matched_point.rs  ### 最小距离赋值
        else:
            self.to_stop = False  # cruising
            self.dist_prvw = samp_basis.dist_prvw
        for obstacle in self.obstacles:
            if obstacle.matched_point.rs < self.traj_point.matched_point.rs - 2:  ### 障碍物的match_point小于车辆当前match_point
                continue
            if Dist(obstacle.x, obstacle.y, self.traj_point.x, self.traj_point.y) > sight_range:  ### 距离大于可视距离
                # 只看眼前一段距离
                continue
            temp = TrajObsFree(self.path_points, obstacle, delta_t)
            if not temp[1]:  ### 有碰撞 指的是障碍物与参考轨迹是否有碰撞(即是否重合)
                colli = 1
                colli_obs = obstacle
                colli_match_point = obstacle.matched_point
                break
        if colli == 0:
            if traj_point.IsOnPath():  ### 车辆现在在不在参考轨迹上(标准为traj的match_point与当前traj的距离是否小于0.5m)
                self.status = "following_path"
            else:
                self.status = "planning_back"  ### 离得远且没有碰撞即planning_back
        else:
            if colli_obs.type == 'static':  ### 如果是静止障碍物在参考轨迹上 则需要绕行
                #     self.status = "planning_out"  ### 碰撞即planning_out
                # else:  ### 如果是运动障碍物 则停下等他过去
                self.status = 'following_path'
                self.dist_prvw = colli_match_point.rs - traj_point.matched_point.rs
                if self.dist_prvw < sight_range:  # 距离小于可视距离(10m)进入减速
                    self.status = 'wait'
                if self.dist_prvw < 5:  # 距离小于5m进入刹车
                    self.status = 'brake'

    def __LatticePlanner(self, traj_point, path_points, obstacles, samp_basis):
        global delta_t, v_tgt, sight_range
        colli_free_traj_pairs = []  # PolyTraj object with corresponding trajectory's cost
        for theta in self.theta_samp:  # theta (heading) samping  ### 航向角采样空间
            self.traj_point.theta = theta
            s_cond_init, d_cond_init = CartesianToFrenet(self.traj_point.matched_point,
                                                         self.traj_point)  ### 转化坐标系 s d分别速度方向和垂直于速度方向
            s_cond_init[2], d_cond_init[2] = 0, 0  # [0]为该坐标系下路程 [1]为速度

            for delta_s in self.dist_samp:  # s_cond_end[0] sampling
                total_t = delta_s / v_tgt
                poly_traj = PolyTraj(s_cond_init, d_cond_init, total_t)  ##### 报错由于total+t =0
                s_cond_end = np.array([s_cond_init[0] + delta_s, self.v_end, 0])  ### v_end = v_tgt
                poly_traj.GenLongTraj(s_cond_end)  ### GenLongTraj GenLatTraj分别得到五次多项式的系数
                if not poly_traj.LongConsFree(delta_t):  # 先看纵向轨迹s是否满足纵向运动约束
                    pass
                else:
                    for d_end in self.d_end_samp:  ### d_end[0] sampling self.d_end_samp = [0] 横为0
                        d_cond_end = np.array([d_end, 0, 0])
                        poly_traj.GenLatTraj(d_cond_end)
                        tp_all = poly_traj.GenCombinedTraj(self.path_points,
                                                           delta_t)  ### 生成规划轨迹 tp_all是笛卡尔坐标系 完成由frenet到笛卡尔转换
                        self.polytrajs.append(poly_traj)
                        colli = 0
                        dis_to_obs = 0
                        for obstacle in self.obstacles:
                            if obstacle.matched_point.rs < self.traj_point.matched_point.rs - 2:
                                continue
                            if Dist(obstacle.x, obstacle.y, traj_point.x, traj_point.y) > sight_range:
                                # 只看眼前一段距离
                                continue
                            plt.gca().add_patch(
                                plt.Rectangle((obstacle.corner[0], obstacle.corner[1]), obstacle.length, obstacle.width,
                                              color='y', angle=obstacle.heading * 180 / M_PI))
                            tp_x, tp_y = [], []
                            for tp in tp_all:
                                tp_x.append(tp.x)
                                tp_y.append(tp.y)

                            temp = TrajObsFree(tp_all, obstacle, delta_t)  ### 返回距离 是否碰撞
                            if not temp[1]:  # 有碰撞
                                colli = 1
                                break
                            dis_to_obs += temp[0]
                        if colli == 0:
                            if poly_traj.LatConsFree(delta_t):  # 满足横向约束
                                # print("available trajectory found")
                                colli_free_traj_pairs.append(
                                    [poly_traj, dis_to_obs])  ### 把在theta delta_s采样下的轨迹收集 并且同时保留与障碍物的距离dis_to_obs
                            tp_x, tp_y, tp_v, tp_a = [], [], [], []
                            for tp in tp_all:
                                tp_x.append(tp.x)
                                tp_y.append(tp.y)
                                tp_v.append(tp.v)
                                tp_a.append(tp.a)

        if colli_free_traj_pairs:  # selecting the best one
            cost_list = CostSorting(colli_free_traj_pairs)  ### 得到一个traj_pairs(规划轨迹)的升序损失函数序列
            cost_min_traj = colli_free_traj_pairs[cost_list[0][0]][0]
            traj_points_opt = cost_min_traj.tp_all
            tpo_x = []
            tpo_y = []
            for tpo in traj_points_opt:
                tpo_x.append(tpo.x)
                tpo_y.append(tpo.y)
            # plt.plot(tpo_x, tpo_y, '.g')
            # plt.show()
            return traj_points_opt  ### 从一堆无碰撞轨迹中得到一个损失最小的可行轨迹(笛卡尔坐标系下)
        else:  # emergency stop
            # print("没找到可行解, 需要扩大范围或紧急停车")
            return False

    def __PathFollower(self, traj_point, path_points, obstacles, samp_basis):  # 无障碍物且在原轨迹上时的循迹,认为从matched_point开始
        global delta_t
        # print(f'v_end:{self.v_end},traj_point.v:{self.traj_point.v},dist_prvw:{self.dist_prvw},delta_t:{delta_t}')
        acc = ((self.v_end ** 2 - self.traj_point.v ** 2) / (
                2 * self.dist_prvw) + 10e-10)  ### 以此加速度向v_end靠拢(在to_stop=False时=v_tgt)
        if self.dist_prvw < 2:  ### 到2m还没停下 管不了舒适减速度了直接刹死
            acc = -3 * self.traj_point.v
        total_t = 2 * self.dist_prvw / (self.v_end + self.traj_point.v)
        num_points = math.floor(total_t / delta_t)
        tp_all = []  # all the future traj points in a list
        rs_pp_all = []  # the rs value of all the path points
        tp_x = []
        tp_y = []
        for path_point in path_points:
            rs_pp_all.append(path_point.rs)
        rs_pp_all = np.array(rs_pp_all)
        for i in range(int(num_points)):
            s_cond = np.zeros(3)
            d_cond = np.zeros(3)
            s_cond[0] = (self.traj_point.matched_point.rs
                         + self.traj_point.v * i * delta_t + (1 / 2) * acc * ((i * delta_t) ** 2))
            s_cond[1] = self.traj_point.v + acc * i * delta_t
            s_cond[2] = acc  ### 此时的路程 速度 加速度 由于沿着参考轨迹行驶 所以在frenet下没有横向数据
            index_min = np.argmin(np.abs(rs_pp_all - s_cond[0]))
            path_point_min = path_points[index_min]
            if index_min == 0 or index_min == len(path_points) - 1:
                path_point_inter = path_point_min
            else:
                if s_cond[0] >= path_point_min.rs:
                    path_point_next = path_points[index_min + 1]
                    path_point_inter = LinearInterpolate(path_point_min, path_point_next, s_cond[0])
                else:
                    path_point_last = path_points[index_min - 1]
                    path_point_inter = LinearInterpolate(path_point_last, path_point_min, s_cond[0])

            traj_point = FrenetToCartesian(path_point_inter, s_cond, d_cond)
            tp_all.append(traj_point)
            tp_x.append(traj_point.x)
            tp_y.append(traj_point.y)
        return tp_all  ### 同样转化回笛卡尔坐标系

    def __FollowingPath(self, traj_point, path_points, obstacles, samp_basis):
        if traj_point.v < 0.5:  ### 如果处于刚起步状态 特别是v几乎等于0时 采样dis_sample很容易为[0,....] 走不动道了就 所以跟wait一样给赋一个dis_sample 渡过起步的难关 这样ego初速度也能为0了 不然之前ego初速度不能设为0.05以下
            self.dist_samp = [0.2 * v_tgt, 0.5 * v_tgt, v_tgt, 2 * v_tgt]
        if self.to_stop:
            self.v_end = 0  ### else v_end = v_tgt
        return self.__LatticePlanner(traj_point, path_points, obstacles, samp_basis)

    def __PlanningOut(self, traj_point, path_points, obstacles, samp_basis):
        if self.to_stop:  # stopping
            # self.dist_samp = [self.dist_prvw]
            self.v_end = 0
        return self.__LatticePlanner(traj_point, path_points, obstacles, samp_basis)  ### 规划出损失最小的轨迹

    def __PlanningBack(self, traj_point, path_points, obstacles, samp_basis):
        self.theta_samp = [self.traj_point_theta]  # just use the current heading, is it necessary?
        self.dist_samp = [self.dist_prvw]  # come back asap, is it necessary?
        self.d_end_samp = [0]
        if self.to_stop:  # stopping
            self.v_end = 0
        return self.__LatticePlanner(traj_point, path_points, obstacles, samp_basis)

    def __PlanningStop(self, traj_point, path_points, obstacles, samp_basis):
        self.dist_samp = [0, 0.5 * self.dist_prvw, self.dist_prvw]
        self.v_end = 0
        return self.__LatticePlanner(traj_point, path_points, obstacles, samp_basis)

    def LocalPlanning(self, traj_point, path_points, obstacles, samp_basis):
        # print(self.status)
        if self.status == "following_path":  ### 在参考轨迹上且无碰撞的风险
            return self.__FollowingPath(traj_point, path_points, obstacles, samp_basis)
        elif self.status == "planning_out":  ### 与障碍物有冲突 需要离开参考轨迹
            return self.__PlanningOut(traj_point, path_points, obstacles, samp_basis)
        elif self.status == "planning_back":  ### 无冲突且不在参考轨迹上 要回到参考轨迹
            return self.__PlanningBack(traj_point, path_points, obstacles, samp_basis)
        elif self.status == 'wait':
            return self.__PlanningStop(traj_point, path_points, obstacles, samp_basis)
        elif self.status == 'brake':  # 除brake外都是lattice规划器
            self.v_end = 0
            return self.__PathFollower(traj_point, path_points, obstacles, samp_basis)
        else:
            quit()


delta_t = 0.04 * 1  # fixed time between two consecutive trajectory points, sec
v_tgt = 20  # fixed target speed, m/s
sight_range = 20  # 判断有无障碍物的视野距离
# ttcs = [3, 4, 5]  # static ascending time-to-collision, sec
ttcs = [-1, -0.5, 0, 0.5]  ### change this u should also change line 682 605 878 now means acc
theta_thr = M_PI / 6  # delta theta threshold, deviation from matched path

def detail_xy(xy):  # 将原车道中心线上少量的点加密为0.1m间隔的点
    [direct, add_length] = get_lane_feature(xy)
    dist_interval = 1
    new_xy = [[], []]
    new_direct = []
    new_add_len = [0]
    temp_length = dist_interval
    for k in range(0, len(xy[0]) - 1):
        new_xy[0].append(xy[0][k])
        new_xy[1].append(xy[1][k])
        new_add_len.append(temp_length)
        new_direct.append(direct[k])
        while temp_length < add_length[k + 1]:
            temp_length += dist_interval
            new_xy[0].append(new_xy[0][-1] + dist_interval * math.cos(direct[k]))
            new_xy[1].append(new_xy[1][-1] + dist_interval * math.sin(direct[k]))
            new_add_len.append(temp_length)
            new_direct.append(direct[k])
    return [new_xy, new_direct, new_add_len]

def get_lane_feature(xy):
    xy = np.array(xy)
    # n为中心点个数，2为x,y坐标值
    x_prior = xy[0][:-1]
    y_prior = xy[1][:-1]
    x_post = xy[0][1:]
    y_post = xy[1][1:]
    # 根据前后中心点坐标计算【行驶方向】
    dx = x_post - x_prior
    dy = y_post - y_prior

    direction = list(map(lambda d: d > 0 and d or d + 2 * np.pi, np.arctan2(dy, dx)))

    length = np.sqrt(dx ** 2 + dy ** 2)
    length = length.tolist()
    for i in range(len(length) - 1):
        length[i + 1] += length[i]
    length.insert(0, 0)
    return direction, length

class KnotState:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.h = 0.0
        self.curvature = 0.0

    def SetValue(self, state1):
        self.x = state1.x
        self.y = state1.y
        self.h = state1.h

    def Rotate(self, theta):
        rotation_matrix = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
        position_matrix = np.array([self.x, self.y])
        rotated_postion = np.dot(position_matrix, rotation_matrix)  # 坐标系旋转
        self.x = rotated_postion[0]
        self.y = rotated_postion[1]
        self.h += theta

def smooth_cv(cv_init, point_num=1000):
    cv = cv_init
    list_x = cv[:, 0]
    list_y = cv[:, 1]
    if type(cv) is not np.ndarray:
        cv = np.array(cv)
    delta_cv = cv[1:, ] - cv[:-1, ]
    s_cv = np.linalg.norm(delta_cv, axis=1)

    s_cv = np.array([0] + list(s_cv))
    s_cv = np.cumsum(s_cv)

    bspl_x = splrep(s_cv, list_x, s=0.1, k=1)
    bspl_y = splrep(s_cv, list_y, s=0.1, k=1)
    # values for the x axis
    s_smooth = np.linspace(0, max(s_cv), point_num)
    # get y values from interpolated curve
    x_smooth = splev(s_smooth, bspl_x)
    y_smooth = splev(s_smooth, bspl_y)
    new_cv = np.array([x_smooth, y_smooth]).T

    delta_new_cv = new_cv[1:, ] - new_cv[:-1, ]
    s_accumulated = np.cumsum(np.linalg.norm(delta_new_cv, axis=1))
    s_accumulated = np.concatenate(([0], s_accumulated), axis=0)
    return new_cv, s_accumulated

def RefLine(origion, destination):
    origion_knotstate = KnotState()
    origion_knotstate.x, origion_knotstate.y, origion_knotstate.h = origion
    destination_knotstate = KnotState()
    destination_knotstate.x, destination_knotstate.y, destination_knotstate.h = destination
    if abs(origion_knotstate.y - destination_knotstate.y) < 1:  # 该段为直线段
        # print('该段为直线段')
        new_xy, new_direct, new_add_len = detail_xy([[origion[0], destination[0]], [origion[1], destination[1]]])
        return new_xy[0], new_xy[1]
    else:
        # planned_curve = CurvePlaner(origion_knotstate, destination_knotstate)
        planned_curve, _ = smooth_cv(np.array([origion, destination]))
        return planned_curve[:, 0], planned_curve[:, 1]

class LATTICE(PlannerBase):
    def __init__(self):
        pass

    def init(self, scenario_dict):
        print("----------------------------LATTICE INIT----------------------------")
        global origion, destination
        origion = np.array([scenario_dict['task_info']['startPos'][0], scenario_dict['task_info']['startPos'][1], 0])
        destination = np.array([scenario_dict['task_info']['targetPos'][0][0], scenario_dict['task_info']['targetPos'][0][1], 0])

    def act(self, observation):
        control = self.alg(observation.ego_info, observation.object_info)
        return control

    def alg(self,ego, obs):
        # 主车和障碍物位置信息
        ego_info = ego
        # 初始化主车
        x_ego, y_ego, v_ego, heading_ego = ego_info.x, ego_info.y, ego_info.v, ego_info.yaw

        tp_list = [x_ego, y_ego, v_ego, 0, heading_ego, 0]  # from sensor actually, an example here
        traj_point = TrajPoint(tp_list)  ### [x, y, v, a, theta, kappa]

        # 测试
        rx, ry = RefLine(origion, destination)
        cts_points = np.array([rx, ry])
        path_points = CalcRefLine(cts_points)
        static_obstacles = []
        for key in obs:
            ego_obstacles = obs[key]
            for id in ego_obstacles:
                obstacles = ego_obstacles[id]

                static_obstacles.append(Obstacle(
                    [obstacles.x, obstacles.y, obstacles.v, obstacles.length, obstacles.width, obstacles.yaw,
                     'static']))

        for obstacle in static_obstacles:
            obstacle.MatchPath(path_points)  ### 同样match障碍物与参考轨迹

        traj_point.MatchPath(path_points)  # matching once is enough 将traj_point(单点)与path_points(序列)中最近的点匹配
        samp_basis = SampleBasis(traj_point, theta_thr, ttcs)  ### 采样区间(类动作空间)
        local_planner = LocalPlanner(traj_point, path_points, static_obstacles,
                                     samp_basis)  ### 规划器 输入为目前位置 参考轨迹点 障碍物位置 采样空间
        traj_points_opt = local_planner.LocalPlanning(traj_point, path_points, static_obstacles, samp_basis)
        # 如果采样较少的情况未找到可行解，考虑扩大采样范围
        if not traj_points_opt:
            # print("扩大范围")
            theta_thr_ = M_PI / 3
            ttcs_ = [2, 3, 4, 5, 6, 7, 8]
            # ttcs_ = [-3, -2, -1, 0, 1, 2]
            samp_basis = SampleBasis(traj_point, theta_thr_, ttcs_)
            local_planner = LocalPlanner(traj_point, path_points, static_obstacles, samp_basis)
            traj_points_opt = local_planner.LocalPlanning(traj_point, path_points, static_obstacles, samp_basis)
        ### 扩大范围还不行就准备紧急停车 结果表明这个模块有问题 紧急停车总是不能完全停下
        if not traj_points_opt:
            # print("紧急停车")
            local_planner = LocalPlanner(traj_point, path_points, static_obstacles, samp_basis)
            local_planner.status = 'brake'
            traj_points_opt = local_planner.LocalPlanning(traj_point, path_points, static_obstacles, samp_basis)
        else:  ### 正常情况下在正常采样空间内如果有opt 就将opt规划出的点作为下一时刻的traj
            traj_points = []
            for tp_opt in traj_points_opt:
                traj_points.append([tp_opt.x, tp_opt.y, tp_opt.v, tp_opt.a, tp_opt.theta, tp_opt.kappa])

        if v_ego < 2:
            acc_target = 1
        else:
            try:
                acc_target = traj_points_opt[1].a  # 加速度
            except:
                acc_target = 0
        try:
            wheel_target = traj_points_opt[1].theta / 180 * 3.14  # 方向盘转角
        except:
            wheel_target = 0

        if acc_target < -3:
            acc_target = -3
        if acc_target > 3:
            acc_target = 3

        # wheel_target = traj_points_opt[1].theta - heading_ego  # 方向盘的转角是轨迹的航向角的差值

        drivecontrol = [acc_target, wheel_target]  # 最终返回控制信息
        return drivecontrol