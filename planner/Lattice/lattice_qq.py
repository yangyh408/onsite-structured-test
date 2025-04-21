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
from planner.pathFinder import DFSPathFinder
import pandas as pd
from utils.myvisualizer import TrajVisualizer
from planner.tracking_controller import PIDController

class P:
    """
    参数类
    """
    M_PI = 3.141593

    # 车辆属性, global const. and local var. check!
    VEH_L = 5  # length
    VEH_W = 1  # width
    MAX_V = 20
    MIN_V = 0
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
    LON_COST_WEIGHT = 1  # 纵向目标cost，暂时不用
    LAT_COST_WEIGHT = 1  # 横向约束，包括舒适度和偏移量
    COLLISION_COST_WEIGHT = 0.1  # 碰撞cost
    DESTINATION_COST_WEIGHT = 1

    # 障碍物急刹车阈值
    BRAKE_DIST = 8.5
    # 仿真属性
    delta_t = 0.1       # fixed time between two consecutive trajectory points, sec
    v_tgt = 20          # fixed target speed, m/s
    sight_range = 13    # 判断有无障碍物的视野距离

    # 起点和终点
    origin = None
    destination = None


def NormalizeAngle(angle_rad):
    # to normalize an angle to [-pi, pi]
    a = math.fmod(angle_rad + P.M_PI, 2.0 * P.M_PI)
    if a < 0.0:
        a = a + 2.0 * P.M_PI
    return a - P.M_PI


def Dist(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


class PathPoint:
    def __init__(self, pp_list):
        """
        路径点类
        Args:
            pp_list:输出的路径， [rx, ry, rs, rtheta, rkappa, rdkappa]，分别为xy坐标 路程 角度 角度变化量/路程变化量 (角度变化量/路程变化量)/路程变化量
        """
        # pp_list: from CalcRefLine, [rx, ry, rs, rtheta, rkappa, rdkappa] x y 路程 角度 角度变化量/路程变化量 (角度变化量/路程变化量)/路程变化量
        self.rx = pp_list[0]
        self.ry = pp_list[1]
        self.rs = pp_list[2]
        self.rtheta = pp_list[3]
        self.rkappa = pp_list[4]
        self.rdkappa = pp_list[5]


class TrajPoint:
    def __init__(self, tp_list):
        """
        轨迹点类
        Args:
            tp_list: from sensors, [x, y, v, a, theta, kappa]
        """
        self.x = tp_list[0]
        self.y = tp_list[1]
        self.v = tp_list[2]
        self.a = tp_list[3]
        self.theta = tp_list[4]
        self.kappa = tp_list[5]

    def MatchPath(self, path_points):
        '''
        计算轨迹点到路径上的投影点
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

    def LimitTheta(self, theta_thr=P.M_PI / 6):
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
        # print(f'Distance to path: {dist}')
        if dist <= dist_thr:
            return True
        else:
            return False


class Obstacle:
    """
    障碍物类
    """

    def __init__(self, obstacle_info):
        self.x = obstacle_info[0]
        self.y = obstacle_info[1]
        self.v = obstacle_info[2]
        self.length = obstacle_info[3]
        self.width = obstacle_info[4]
        self.heading = obstacle_info[5]  # 这里设定朝向是length的方向，也是v的方向
        self.type = obstacle_info[6]
        self.id = obstacle_info[7]
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
        """
        find the closest/projected point on the reference path
        the deviation is not large; the curvature is not large
        """

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
    s_condition[2] = (a * cos_delta_theta - s_condition[1] ** 2 *
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


def CalcRefLine(cts_points):
    """
    输入参考轨迹的x y 计算rs/rtheta/rkappa/rdkappa 此时是笛卡尔坐标系 rs为已走路程 rtheta为角度
    deal with reference path points 2d-array
    to calculate rs/rtheta/rkappa/rdkappa according to cartesian points
    """
    rx = cts_points[0]
    ry = cts_points[1]

    # 预处理：移除相邻重复点（避免 ds=0）
    unique_indices = []
    for i in range(len(rx)):
        if i == 0 or (rx[i] != rx[i - 1] or ry[i] != ry[i - 1]):
            unique_indices.append(i)
    rx = rx[unique_indices]
    ry = ry[unique_indices]

    # 其他参数初始化
    rs = np.zeros_like(rx)
    rtheta = np.zeros_like(rx)
    rkappa = np.zeros_like(rx)
    rdkappa = np.zeros_like(rx)
    n_points = rx.shape[0]

    # 计算累计弧长rs
    dx = np.diff(rx)
    dy = np.diff(ry)
    ds = np.sqrt(dx**2 + dy**2)
    ds = np.insert(ds, 0, 0)
    rs = np.cumsum(ds)

    # 计算航向角theta
    eps = 1e-8  # 防止零除
    valid_ds = ds > eps
    # 使用向量化的atan2计算角度
    rtheta[:-1] = np.arctan2(dy[:], dx[:])  # dy和dx对应下一个点的方向向量
    rtheta[-1] = rtheta[-2]  # 最后一个点用前一个点的角度

    # 计算曲率（deepseek)
    if n_points >= 5:
        filter_window = 51
        filter_order = 5
        # 先平滑数值以计算导数
        smooth_x = scipy.signal.savgol_filter(rx, filter_window, filter_order, deriv=0)
        smooth_y = scipy.signal.savgol_filter(ry, filter_window, filter_order, deriv=0)

        # 一阶导数 (中心差分)
        dx_ds = np.gradient(smooth_x, rs)
        dy_ds = np.gradient(smooth_y, rs)

        # 二阶导数 (d²x/ds²)
        d2x_ds2 = np.gradient(dx_ds, rs)
        d2y_ds2 = np.gradient(dy_ds, rs)

        # 几何曲率公式: κ = (dx*d²y - dy*d²x) / (dx² + dy²)^(3/2)
        numerator = dx_ds * d2y_ds2 - dy_ds * d2x_ds2
        denominator = (dx_ds ** 2 + dy_ds ** 2 + 1e-16) ** 1.5  # 防止零除
        rkappa = np.divide(numerator, denominator, out=np.zeros_like(numerator), where=denominator > 1e-8)
    else:
        rkappa.fill(0.0)

        ############# Step 4: 曲率导数计算 #############
    if n_points >= 3:
        # 使用高斯滤波后的梯度
        rdkappa = np.gradient(rkappa, rs)
    else:
        rdkappa.fill(0.0)

        ############# 后处理流程 #############
        # 中值滤波消除离群值
    if n_points > 11:
        rkappa = scipy.signal.medfilt(rkappa, kernel_size=5)
        rdkappa = scipy.signal.medfilt(rdkappa, kernel_size=5)

    # 构造输出
    path_points = []
    for i in range(len(rx)):
        path_points.append(PathPoint([rx[i], ry[i], rs[i], rtheta[i], rkappa[i], rdkappa[i]]))

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
        if d > P.M_PI:
            d = d - 2 * P.M_PI
        elif d < -P.M_PI:
            d = d + 2 * P.M_PI
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


def TrajObsFree(xoy_traj, obstacle):  ### 输入为路径点 障碍物类 帧长
    """
    判断经过了delta_t之后，轨迹与障碍物是否碰撞
    假设障碍物是按照初始的heading匀速直线运动
    该函数既可以判断采样的轨迹与障碍物是否碰撞，也可以判断参考路径与障碍物是否碰撞
    Args:
        xoy_traj: 测试的轨迹
        obstacle: 障碍物

    Returns:
        dis_mean: 若不碰撞，则为轨迹上各点到障碍物的平均距离; 否则为0
    """
    dis_sum = 0
    for point in xoy_traj:
        obstacle.x += obstacle.v * P.delta_t * math.cos(obstacle.heading)
        obstacle.y += obstacle.v * P.delta_t * math.sin(obstacle.heading)
        if isinstance(point, PathPoint):  # 如果是原来路径点，就只按圆形计算。因为每点的车辆方向难以获得
            if ColliTestRough(point, obstacle) > 0:  # 返回point与obstacle的距离
                continue
            return 0, False
        else:
            # 否则是采样的轨迹
            dis = ColliTestRough(point, obstacle)
            dis_sum += dis
            if dis > 0:
                continue
            if ColliTest(point, obstacle):  # 对于车辆与障碍物是否碰撞 ColliTestRough不足以(将两者视为圆形) 要用更准确的ColliTest检测是否碰撞
                # print("不满足实际碰撞检测")
                return 0, False
    if len(xoy_traj) != 0:
        dis_mean = dis_sum / len(xoy_traj)
    else:
        return 0, False
    # print("满足实际碰撞检测")
    return dis_mean, True


# 粗略的碰撞检测(视作圆形)  如果此时不碰撞，就无需按矩形检测。返回的距离作为该点车到障碍物的大致距离（无碰撞时也可能为负）
def ColliTestRough(point, obs: Obstacle):
    if isinstance(point, PathPoint):
        dis = math.sqrt((point.rx - obs.x) ** 2 + (point.ry - obs.y) ** 2)
    else:
        dis = math.sqrt((point.x - obs.x) ** 2 + (point.y - obs.y) ** 2)
    max_veh = max(P.VEH_L, P.VEH_W)
    max_obs = max(obs.length, obs.width)
    return dis - (max_veh + max_obs) / 2


# 碰撞检测 (这部分参考apollo代码)
def ColliTest(point, obs: Obstacle):
    shift_x = obs.x - point.x
    shift_y = obs.y - point.y

    cos_v = math.cos(point.theta)
    sin_v = math.sin(point.theta)
    cos_o = math.cos(obs.heading)
    sin_o = math.sin(obs.heading)
    half_l_v = P.VEH_L / 2
    half_w_v = P.VEH_W / 2
    half_l_o = obs.length / 2
    half_w_o = obs.width / 2

    dx1 = cos_v * P.VEH_L / 2
    dy1 = sin_v * P.VEH_L / 2
    dx2 = sin_v * P.VEH_W / 2
    dy2 = -cos_v * P.VEH_W / 2
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


class PolyTraj:
    """
    采样的轨迹类
    五次多项式共包含6个方程，其中初始状态占据3个方程，针对剩下三个方程进行采样，决定一条轨迹
    一组参数[total_t, v_end, s_end, d_end]对应一条轨迹
    其中，total_t, v_end, s_end决定纵向方程。d_end决定横向方程
    Args:
        s_cond_init: 初始s轴状态(位置、速度、加速度)
        d_cond_init: 初始d轴状态(位置、速度、加速度)
        total_t: 轨迹总时长
    """

    def __init__(self, s_cond_init, d_cond_init, total_t, path_points, obstacles):
        # 轨迹参数
        self.long_coef = None
        self.lat_coef = None

        # 初始状态
        self.s_cond_init = s_cond_init
        self.d_cond_init = d_cond_init

        # 采样参数：目标时间
        self.total_t = total_t  # to plan how long in seconds
        self.delta_s = 0

        # 采样参数：目标状态
        self.s_cond_end = None
        self.d_cond_end = None

        # 参考轨迹
        self.path_points = path_points

        # 障碍物
        self.obstacles = obstacles

        # 笛卡尔坐标系下的轨迹
        self.traj_cartesian = None
        self.traj_frenet = None

        # 轨迹的花费(评分)
        self.cost = None

    def __QuinticPolyCurve(self, y_cond_init, y_cond_end, x_dur):  # 五次多项式拟合
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
        self.s_cond_end = s_cond_end
        self.long_coef = self.__QuinticPolyCurve(self.s_cond_init, self.s_cond_end,
                                                 self.total_t)  ### self.long_coef为五次多项式的参数
        self.delta_s = self.long_coef[1] * self.total_t + self.long_coef[2] * self.total_t ** 2 + \
                       self.long_coef[3] * self.total_t ** 3 + self.long_coef[4] * self.total_t ** 4 + \
                       self.long_coef[5] * self.total_t ** 5
        # return self.long_coef

    def GenLatTraj(self, d_cond_end):
        # GenLatTraj should be posterior to GenLongTraj
        self.d_cond_end = d_cond_end
        self.lat_coef = self.__QuinticPolyCurve(self.d_cond_init, self.d_cond_end, self.delta_s)
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
    def LongConsFree(self):
        size = int(self.total_t / P.delta_t)
        for i in range(size):
            v = self.Evaluate(self.long_coef, 1, i * P.delta_t)
            # print(v)
            if v > P.MAX_V or v < P.MIN_V:
                # print(v, "纵向速度超出约束")
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
    # def LatConsFree(self):
    #     size = int(self.total_t / P.delta_t)
    #     lat_offset_cost = 0
    #     lat_comfort_cost = 0
    #     for i in range(size):
    #         s = self.Evaluate(self.long_coef, 0, i * P.delta_t)
    #         d = self.Evaluate(self.lat_coef, 0, s)
    #         dd_ds = self.Evaluate(self.lat_coef, 1, s)
    #         ds_dt = self.Evaluate(self.long_coef, 1, i * P.delta_t)
    #         d2d_ds2 = self.Evaluate(self.lat_coef, 2, s)
    #         d2s_dt2 = self.Evaluate(self.long_coef, 2, i * P.delta_t)
    #
    #         lat_a = d2d_ds2 * ds_dt * ds_dt + dd_ds * d2s_dt2
    #
    #         # 向心加速度暂时删去
    #         # if abs(lat_a) > P.MAX_LAT_A:
    #         #     print(lat_a, "不满足横向约束")
    #         #     return False
    #         lat_comfort_cost += lat_a * lat_a
    #         lat_offset_cost += d * d
    #
    #     self.lat_cost = lat_comfort_cost * P.LAT_COMFORT_COST_WEIGHT + lat_offset_cost * P.LAT_OFFSET_COST_WEIGHT
    #     # print("满足横向约束")
    #     return True

    def __GenCombinedTraj(self):
        '''
        combine long and lat traj together
        F2C function is used to output future traj points in a list to follow
        '''
        a0_s, a1_s, a2_s, a3_s, a4_s, a5_s = self.long_coef[0], self.long_coef[1], self.long_coef[2], \
            self.long_coef[3], self.long_coef[4], self.long_coef[5]
        a0_d, a1_d, a2_d, a3_d, a4_d, a5_d = self.lat_coef[0], self.lat_coef[1], self.lat_coef[2], \
            self.lat_coef[3], self.lat_coef[4], self.lat_coef[5]

        rs_pp_all = []  # the rs value of all the path points
        for path_point in self.path_points:
            rs_pp_all.append(path_point.rs)
        rs_pp_all = np.array(rs_pp_all)
        num_points = math.floor(self.total_t / P.delta_t)  # 规划时长/帧长 = 规划点数
        s_cond_all = []  # possibly useless
        d_cond_all = []  # possibly useless
        pp_inter = []  # possibly useless
        tp_all = []  # all the future traj points in a list
        t, s = 0, 0  # initialize variables, s(t), d(s) or l(s)
        self.traj_frenet = []
        for i in range(int(num_points)):
            s_cond = np.zeros(3)
            d_cond = np.zeros(3)

            t = t + P.delta_t
            s_cond[0] = a0_s + a1_s * t + a2_s * t ** 2 + a3_s * t ** 3 + a4_s * t ** 4 + a5_s * t ** 5  # 路程
            s_cond[1] = a1_s + 2 * a2_s * t + 3 * a3_s * t ** 2 + 4 * a4_s * t ** 3 + 5 * a5_s * t ** 4  # 速度(d路程/dt)
            s_cond[2] = 2 * a2_s + 6 * a3_s * t + 12 * a4_s * t ** 2 + 20 * a5_s * t ** 3  # a
            s_cond_all.append(s_cond)

            s = s_cond[0] - a0_s  # TODO: 这里为什么要减去a0_s
            d_cond[0] = a0_d + a1_d * s + a2_d * s ** 2 + a3_d * s ** 3 + a4_d * s ** 4 + a5_d * s ** 5
            d_cond[1] = a1_d + 2 * a2_d * s + 3 * a3_d * s ** 2 + 4 * a4_d * s ** 3 + 5 * a5_d * s ** 4
            d_cond[2] = 2 * a2_d + 6 * a3_d * s + 12 * a4_d * s ** 2 + 20 * a5_d * s ** 3
            d_cond_all.append(d_cond)

            self.traj_frenet.append([s_cond[0], s_cond[1], s_cond[2], d_cond[0], d_cond[1], d_cond[2]])

            index_min = np.argmin(np.abs(rs_pp_all - s_cond[0]))
            path_point_min = self.path_points[index_min]
            if index_min == 0 or index_min == len(self.path_points) - 1:
                path_point_inter = path_point_min
            else:
                if s_cond[0] >= path_point_min.rs:
                    path_point_next = self.path_points[index_min + 1]
                    path_point_inter = LinearInterpolate(path_point_min, path_point_next, s_cond[0])
                else:
                    path_point_last = self.path_points[index_min - 1]
                    path_point_inter = LinearInterpolate(path_point_last, path_point_min, s_cond[0])
            pp_inter.append(path_point_inter)
            traj_point = FrenetToCartesian(path_point_inter, s_cond, d_cond)
            # traj_point.v = v_tgt
            tp_all.append(traj_point)
        self.traj_cartesian = tp_all

    def CalCost(self):
        self.cost = 0

        # 计算轨迹平均点到目标点的距离
        x = np.mean([tp.x for tp in self.traj_cartesian])
        y = np.mean([tp.y for tp in self.traj_cartesian])
        destination_cost = math.sqrt((x - P.destination[0]) ** 2 + (y - P.destination[1]) ** 2)

        # 计算横向cost
        lat_cost = self._CalLatCost()
        lon_cost = self._CalLonCost()
        collision_cost = self._CalCollisionCost()

        self.cost = lat_cost * P.LAT_COST_WEIGHT \
                    + collision_cost * P.COLLISION_COST_WEIGHT \
                    + destination_cost * P.DESTINATION_COST_WEIGHT \
                    + lon_cost * P.LON_COST_WEIGHT

    def _CalLatCost(self):
        # 计算横向的cost，目前是计算车辆在轨迹中的横向平均距离
        lat_cost = np.mean([i[3] for i in self.traj_frenet])
        return lat_cost

    def _CalLonCost(self):
        # 计算纵向的cost，目前是计算车辆在轨迹中的平均速度（取相反数）
        speed_cost = np.mean([i[1] for i in self.traj_frenet])
        return -speed_cost

    def _CalCollisionCost(self):
        # 计算碰撞风险，现在直接计算轨迹到达最近的障碍物的平均距离（取相反数）
        front_obstacle_dist = P.sight_range
        front_obstacle = None

        # 遍历所有障碍物，获取最近的一个
        for obstacle in self.obstacles:
            obstacle_dist = obstacle.matched_point.rs - self.s_cond_init[0]  # 障碍物距本车距离
            if 0 < obstacle_dist < P.sight_range:
                # 只要较近范围内的障碍物才会被考虑
                if front_obstacle_dist > obstacle_dist:
                    front_obstacle = obstacle
                    front_obstacle_dist = obstacle_dist

        if front_obstacle is not None:
            # 若存在较近的障碍物，则计算平均距离并取相反数(距离越近，cost越大)
            temp = TrajObsFree(self.traj_cartesian, front_obstacle)
            return -temp[0]
        else:
            # 否则返回常数
            return -P.sight_range

    def CollideWithObstacles(self, obstacles=None):
        """
        判断当前轨迹是否与障碍物碰撞
        Returns:
            True: 碰撞
            False: 不碰撞
        """
        # 计算当前轨迹在笛卡尔坐标系下的轨迹
        self.__GenCombinedTraj()
        if obstacles is None:
            obstacles = self.obstacles
        # 遍历障碍物，在笛卡尔坐标系下判断是否碰撞
        for obstacle in obstacles:
            obstacle_dist = obstacle.matched_point.rs - self.s_cond_init[0]  # 障碍物距本车距离
            if obstacle_dist < -2:
                # 若障碍物在车辆后方2m开外，则不考虑该障碍物
                continue
            if obstacle_dist > P.sight_range:
                # 若障碍物在车辆前方较远处，则不考虑该障碍物
                continue
            temp = TrajObsFree(self.traj_cartesian, obstacle)
            if not temp[1]:
                # 若存在碰撞
                return True
        return False


class LocalPlanner:
    def __init__(self, traj_point, path_points, obstacles):
        """
        Lattice算法
        Args:
            traj_point: 车辆当前状态
            path_points: 参考路径点
            obstacles: 所有障碍物
        """
        # 初始化状态
        self.traj_point = traj_point  # 车辆当前状态
        self.path_points = path_points  # 路径点
        self.obstacles = obstacles  # 障碍物状态

        # 初始化采样参数
        self.t_end_samp = None
        self.v_end_samp = None
        self.s_end_samp = None
        self.d_end_samp = None

        # 初始化采样轨迹
        self.sample_poly_trajs = []

        # 初始化车辆状态
        self.status = None

        # 初始化其他参数
        self.front_veh_speed = None
        self.front_veh_dist = None
        self.front_veh_id = None

    def __JudgeStatus(self):
        """
        判断当前状态
        当前状态分为三类：
            emergency_brake: 急刹车，目标速度为0
            cruise: 正常巡航，目标速度采样
            car_following: 跟车，目标速度根据前车状态采样
        """
        # 判断障碍物状态(目前暂未考虑行人和自行车)
        for obstacle in self.obstacles:
            if Dist(self.traj_point.x, self.traj_point.y, obstacle.x, obstacle.y) > P.sight_range * 1.5:
                continue
            obstacle_dist = obstacle.matched_point.rs - self.traj_point.matched_point.rs  # 障碍物距本车距离
            if obstacle_dist > 0 and (self.front_veh_dist is None or self.front_veh_dist > obstacle_dist):
                self.front_veh_dist = obstacle_dist
                self.front_veh_speed = obstacle.v
                self.front_veh_id = obstacle.id
            if obstacle_dist < -2:
                # 若障碍物在车辆后方2m开外，则不考虑该障碍物
                continue
            if obstacle_dist > P.sight_range:
                # 若障碍物在车辆前方较远处，则不考虑该障碍物
                continue
            if 0 < obstacle_dist < P.BRAKE_DIST:
                # 若障碍物距离车辆较近
                self.status = "emergency_brake"
                break
            if P.BRAKE_DIST <= obstacle_dist < P.sight_range:
                # 若障碍物距离前车较近但是没有那么近
                self.status = "car_following"

        # 遍历完障碍物，若既没有急刹，也没有跟车，说明应该是cruise
        if self.status is None:
            self.status = "cruise"

    def __LatticePlanner(self):
        # 将初始状态转化到Frenet坐标系中
        s_cond_init, d_cond_init = CartesianToFrenet(self.traj_point.matched_point, self.traj_point)
        s_cond_init[2], d_cond_init[2] = 0, 0  # 初始状态下横纵向加速度都设为0

        # 直接遍历所有参数开始采样
        for v_samp in self.v_end_samp:
            for d_samp in self.d_end_samp:
                for s_samp in self.s_end_samp:
                    for t_samp in self.t_end_samp:
                        poly_traj = PolyTraj(s_cond_init=s_cond_init,
                                             d_cond_init=d_cond_init,
                                             total_t=t_samp,
                                             path_points=self.path_points,
                                             obstacles=self.obstacles
                                             )
                        # 拟合纵向轨迹
                        s_cond_end = np.array([s_cond_init[0] + s_samp, v_samp, 0])  # 设定纵向目标状态
                        poly_traj.GenLongTraj(s_cond_end)
                        # 拟合横向轨迹
                        if poly_traj.LongConsFree():  # 如果纵向轨迹可行再进行下一步
                            # 拟合横向轨迹
                            d_cond_end = np.array([d_samp, 0, 0])
                            poly_traj.GenLatTraj(d_cond_end)

                            # 若无碰撞，则将该轨迹储存下来
                            if not poly_traj.CollideWithObstacles():
                                poly_traj.CalCost()  # 计算分数
                                self.sample_poly_trajs.append(poly_traj)  # 储存轨迹

        # 若存在可行采样轨迹，则进行排序
        if len(self.sample_poly_trajs) > 0:
            self.sample_poly_trajs.sort(key=lambda x: x.cost, reverse=False)
            return self.sample_poly_trajs
        else:
            return None

    def LocalPlanning(self):
        # 首先判断当前状态
        self.__JudgeStatus()
        print('*************')
        print(self.status)
        print(f'Front vehicle info: id = {self.front_veh_id}, dist = {self.front_veh_dist}, speed = {self.front_veh_speed}')

        # 根据状态设定采样参数
        if self.status == "cruise":
            self.v_end_samp = np.arange(0, P.v_tgt + 0.01, P.v_tgt * 0.25)
            # self.d_end_samp = np.array([-0.5, 0, 0.5])
            self.d_end_samp  = np.array([0])
            self.s_end_samp = np.arange(3, 10, 1)
            self.t_end_samp = np.arange(3, 5, 0.5)
        elif self.status == "emergency_brake":
            self.v_end_samp = [0]
            # self.d_end_samp = np.array([-0.5, 0, 0.5])
            self.d_end_samp = np.array([0])
            self.s_end_samp = np.arange(3, 5, 1)
            self.t_end_samp = np.arange(3, 4, 0.5)
        elif self.status == "car_following":
            self.v_end_samp = np.arange(0, self.front_veh_speed, P.v_tgt * 0.25)
            # self.d_end_samp = np.array([-0.5, 0, 0.5])
            self.d_end_samp = np.array([0])
            self.s_end_samp = np.arange(3, 10, 1)
            self.t_end_samp = np.arange(2.5, 4, 0.5)

        return self.__LatticePlanner()


class LATTICEQQ(PlannerBase):
    def __init__(self):
        super().__init__()
        self.theta_tracker = None
        self.speed_tracker = None
        self.traj_visualizer = None
        self.path_points = None  # 路径点
        self.scenario_dict = None  # 场景信息
        self.obstacles = None  # 障碍物信息

    def init(self, scenario_dict):
        """
        初始化
        Args:
            scenario_dict: 场景信息
        """
        print("----------------------------LATTICE_QQ INIT----------------------------")
        P.origin = np.array([scenario_dict['task_info']['startPos'][0], scenario_dict['task_info']['startPos'][1], 0])
        P.destination = np.array(
            [scenario_dict['task_info']['targetPos'][0][0], scenario_dict['task_info']['targetPos'][0][1], 0])
        self.scenario_dict = scenario_dict

        # 通过DFS寻找路径
        dfs_path_finder = DFSPathFinder(scenario_dict)
        destination_rect = scenario_dict['task_info']['targetPos']
        start_point = scenario_dict['task_info']['startPos']
        _, path = dfs_path_finder.find_shortest_path(start_point, destination_rect)

        # 初始化轨迹跟踪控制器
        self.speed_tracker = PIDController(kp=1/P.delta_t, ki=0, kd=0, max_output=3, min_output=-3)
        self.theta_tracker = PIDController(kp=8, ki=0, kd=1, max_output=0.66, min_output=-0.66)

        if path is not None:
            # 若能够找到最优路径
            cts_points = np.array(path).T
            path_points = CalcRefLine(cts_points)
            self.path_points = path_points

            # # 路径可视化
            # self.traj_visualizer = TrajVisualizer(scenario_dict)    # 初始化Visualizer
            # self.traj_visualizer.show_path(path_points)
        else:
            # 若不能找到路径
            pass

    def act(self, observation):
        # 根据当前车辆信息和周围物体车辆信息决策动作
        control = self.alg(observation.ego_info, observation.object_info)  # 暂未考虑其他信息，例如信号灯
        return control

    def alg(self, ego_info, object_info):
        # 初始化当前车辆信息
        x_ego, y_ego, v_ego, heading_ego = ego_info.x, ego_info.y, ego_info.v, ego_info.yaw
        # 当前轨迹点信息
        traj_point = TrajPoint([x_ego, y_ego, v_ego, 0, heading_ego, 0])
        traj_point.MatchPath(self.path_points)  # 获取当前状态到路径点的投影

        # 初始化障碍物信息
        self.obstacles = []
        for obs_type in object_info:
            # 遍历各种障碍物类型
            ego_obstacles = object_info[obs_type]
            for id in ego_obstacles:
                # 遍历各种
                obstacle_info = ego_obstacles[id]
                new_obstacle = Obstacle(
                    [obstacle_info.x, obstacle_info.y, obstacle_info.v, obstacle_info.length, obstacle_info.width,
                     obstacle_info.yaw, 'obs_type', id])
                new_obstacle.MatchPath(self.path_points)
                self.obstacles.append(new_obstacle)

        # 通过Lattice算法计算轨迹
        local_planner = LocalPlanner(traj_point=traj_point, path_points=self.path_points, obstacles=self.obstacles)
        sorted_sampled_poly_trajs = local_planner.LocalPlanning()

        if sorted_sampled_poly_trajs is not None:
            optimal_trajectory = sorted_sampled_poly_trajs[0].traj_cartesian
            # 对加速度和转向角进行赋值

            try:
                acc_target = self.speed_tracker.update(state=v_ego, target=optimal_trajectory[1].v)
                # acc_target = optimal_trajectory[1].a  # 加速度
                if acc_target < -3:
                    acc_target = -3
                if acc_target > 3:
                    acc_target = 3
            except:
                acc_target = 0
            try:
                wheel_target = self.theta_tracker.update(state=heading_ego, target=optimal_trajectory[1].theta)
                # wheel_target = optimal_trajectory[1].theta - heading_ego  # 方向盘转角
            except:
                wheel_target = 0
            # # 显示最优轨迹
            # self.traj_visualizer.show_traj(traj_points=optimal_trajectory,
            #                                position=[x_ego, y_ego],
            #                                path_points=self.path_points,
            #                                linewidth=2
            #                                )
            # 显示其它轨迹
            # for i, poly_traj in enumerate(sorted_sampled_poly_trajs):
            #     if i > 0:
            #         self.traj_visualizer.show_traj(traj_points=poly_traj.traj_cartesian, color='y', linewidth=0.5,
            #                                        add_traj=True)
        else:
            acc_target = -3
            wheel_target = 0

        # 输出轨迹
        return [acc_target, wheel_target]
