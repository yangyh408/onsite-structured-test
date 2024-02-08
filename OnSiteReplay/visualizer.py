import re
import numpy as np

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

from utils.observation import Observation


class Visualizer():
    def __init__(self, visualize=False):
        self.visualize = visualize
        self.control_info = None

    def init(self, control_info):
        self.control_info = control_info
        if not self.visualize:
            return
        # 测试信息
        self.scenario_name = control_info.test_setting['scenario_name']
        self.scenario_type = control_info.test_setting['scenario_type']
        
        plt.ion()
        plt.rcParams['axes.unicode_minus'] = False

        self.fig = plt.figure(figsize=[6.4, 4.8])
        self.axbg = self.fig.add_subplot()
        self.axobject = self.axbg.twiny()
        if control_info.road_info:  # 对于有地图文件的测试场景，如回放测试
            # 获得目标区域
            self.x_target = list(control_info.test_setting['goal']['x'])
            self.y_target = list(control_info.test_setting['goal']['y'])
            # 获得该场景车辆行驶的边界
            self.x_max = max(control_info.test_setting['goal']['x'][-1], control_info.ego_info['x']) + 50
            self.x_min = min(control_info.test_setting['goal']['x'][-1], control_info.ego_info['x']) - 50
            self.y_max = (max(control_info.test_setting['goal']['y'][-1], control_info.ego_info['y']) + min(control_info.test_setting['goal']['y'][-1], control_info.ego_info['y']))/2 + (self.x_max-self.x_min)/2
            self.y_min = (max(control_info.test_setting['goal']['y'][-1], control_info.ego_info['y']) + min(control_info.test_setting['goal']['y'][-1], control_info.ego_info['y']))/2 - (self.x_max-self.x_min)/2
            # 固定GIF图中路网范围
            self.axbg.set_xlim(self.x_min, self.x_max)
            plt.ylim(self.y_min, self.y_max)
            self.axobject.set_xlim(self.x_min, self.x_max)
        
            self._plot_roads()

    def update(self, observation: Observation) -> None:
        if not self.visualize:
            return
        # 如果测试结束，则结束绘图，关闭绘图模块
        if observation.test_info['end'] != -1:
            plt.ioff()
            plt.close()
            return
        self.axobject.cla()
        self.axobject.set_xlim(self.x_min, self.x_max)
        # 绘制车辆
        # ---------------------------------------------
        self._plot_objects(observation)
        # 若发生碰撞，则绘制碰撞警告标志
        if observation.test_info['end'] == 3:
            self._plot_warning_signal(observation)

        # GIF显示各类信息
        # ---------------------------------------------
        # 显示测试相关信息
        ts_text = f"Time Stamp: {observation.test_info['t']}"
        name_text = f"<{self.control_info.test_setting['scenario_name']}>"
        type_text = f"MODE: {self.control_info.test_setting['scenario_type']}"
        self.axobject.text(0.02, 0.95, name_text, transform=self.axobject.transAxes, fontdict={'size': '10', 'color': 'black'})
        self.axobject.text(0.02, 0.90, type_text, transform=self.axobject.transAxes, fontdict={'size': '10', 'color': 'black'})
        self.axobject.text(0.75, 0.95, ts_text, transform=self.axobject.transAxes, fontdict={'size': '10', 'color': 'black'})
        # 显示所有车辆运行信息
        observation_objects = observation.object_info()
        colLabels = list(observation_objects.keys())
        colLabels_text=[]
        for colLabel in colLabels:
            if colLabel == "ego":
                colLabels_text.append(colLabel)
            elif colLabel[0:3] == "car":
                match = re.search(r'\d+', colLabel)
                extracted_number = match.group()
                colLabels_text.append("A"+str(extracted_number))
            elif colLabel[0:3] == "bic":
                match = re.search(r'\d+', colLabel)
                extracted_number = match.group()
                colLabels_text.append("B" + str(extracted_number))
            else:
                match = re.search(r'\d+', colLabel)
                extracted_number = match.group()
                colLabels_text.append("P" + str(extracted_number))

        rowLabels = ['v (m/s)', 'a (m/s2)']
        v = np.array([round(observation_objects[key]['v'], 4) for key in colLabels]).reshape(1, -1)
        a = np.array([round(observation_objects[key]['a'], 4) for key in colLabels]).reshape(1, -1)
        cellTexts = np.vstack((v, a))
        info_table = self.axobject.table(cellText=cellTexts, colLabels=colLabels_text , rowLabels=rowLabels,
                                      rowLoc='center', colLoc='center', cellLoc='center', loc='bottom')
        info_table.auto_set_font_size(False)
        info_table.set_fontsize(10)


        # 刷新当前帧画布
        # ---------------------------------------------
        plt.subplots_adjust()
        plt.pause(1e-7)
        # plt.show()

    def _plot_objects(self, observation: Observation) -> None:
        for key, values in observation.object_info().items():
            if key == 'ego':
                self._plot_single_object(key, values, c='green')
            elif key.startswith('bicycle'):
                self._plot_single_object(key, values, c='orange')
            elif key.startswith('pedestrian'):
                self._plot_single_object(key, values, c='blue')
            else:
                self._plot_single_object(key, values, c='cornflowerblue')

    def extract_numeric_part(key):
        # 使用正则表达式提取字符串中的数字部分
        match = re.search(r'\d+', key)
        if match:
            return match.group()
        else:
            return None

    def _plot_single_object(self, key: str, objecti: dict, c='cornflowerblue'):
        """利用 matplotlib 和 patches 绘制小汽车，以 x 轴为行驶方向

        """
        x, y, yaw, width, length = [float(objecti[i])
                                    for i in ['x', 'y', 'yaw', 'width', 'length']]

        angle = np.arctan(width / length) + yaw
        diagonal = np.sqrt(length ** 2 + width ** 2)
        self.axobject.add_patch(
            patches.Rectangle(
                xy=(x - diagonal / 2 * np.cos(angle),
                    y - diagonal / 2 * np.sin(angle)),
                width=length,
                height=width,
                angle=yaw / np.pi * 180,
                color=c,
                fill=True,
                zorder=3
            ))
        # 使用正则表达式提取数字部分
        numeric_part = Visualizer.extract_numeric_part(key)

        if key[0:3] == 'car':
            self.axobject.annotate("A" + numeric_part, (x, y))
        elif key[0:3] == 'bic':
            self.axobject.annotate("B" + numeric_part, (x, y))
        elif key[0:3] == 'ped':
            self.axobject.annotate("P" + numeric_part, (x, y))


    def _plot_warning_signal(self, observation: Observation, c='red'):
        '''绘制主车碰撞时的提醒标志

        '''
        for key, values in observation.object_info().items():
            if key == 'ego':
                x, y = [float(values[i]) for i in ['x', 'y']]
                self.axobject.scatter(x, y, s=60, c=c, alpha=1.0, marker=(8, 1, 30), zorder=4)
        collision_text = 'A collision has occurred'
        self.axobject.text(0.02, 0.85, collision_text, transform=self.axobject.transAxes, fontdict={'size': '10', 'color': 'red'})

    def _plot_roads(self) -> None:
        road_data_for_plot = self.control_info.road_info

        xlim1 = float("Inf")
        xlim2 = -float("Inf")
        ylim1 = float("Inf")
        ylim2 = -float("Inf")
        color = "gray"
        label = None
        draw_arrow = True

        for discrete_lane in road_data_for_plot.discretelanes:
            verts = []
            codes = [Path.MOVETO]

            for x, y in np.vstack(
                [discrete_lane.left_vertices, discrete_lane.right_vertices[::-1]]
            ):
                verts.append([x, y])
                codes.append(Path.LINETO)

                # if color != 'gray':
                xlim1 = min(xlim1, x)
                xlim2 = max(xlim2, x)

                ylim1 = min(ylim1, y)
                ylim2 = max(ylim2, y)

            verts.append(verts[0])
            codes[-1] = Path.CLOSEPOLY

            path = Path(verts, codes)

            self.axbg.add_patch(
                patches.PathPatch(
                    path,
                    facecolor=color,
                    edgecolor="black",
                    lw=0.0,
                    alpha=0.5,
                    zorder=0,
                    label=label,
                )
            )

            self.axbg.plot(
                [x for x, _ in discrete_lane.left_vertices],
                [y for _, y in discrete_lane.left_vertices],
                color="black",
                lw=0.3,
                zorder=1,
            )
            self.axbg.plot(
                [x for x, _ in discrete_lane.right_vertices],
                [y for _, y in discrete_lane.right_vertices],
                color="black",
                lw=0.3,
                zorder=1,
            )

            self.axbg.plot(
                [x for x, _ in discrete_lane.center_vertices],
                [y for _, y in discrete_lane.center_vertices],
                color="white",
                alpha=0.5,
                lw=0.8,
                zorder=1,
            )

            if draw_arrow:
                mc = discrete_lane.center_vertices
                total_len = ((mc[0][0] - mc[-1][0]) ** 2 + (mc[0][1] - mc[-1][1]) ** 2) ** 0.5
                if total_len > 30:
                    index_ = list(map(int, np.linspace(start=10, stop=mc.shape[0] - 10, num=4)))
                else:
                    index_ = []
                for i in range(len(index_)):
                    start_c, end_c = mc[index_[i]], mc[index_[i] + 1]
                    self.axbg.arrow(
                        start_c[0], start_c[1], end_c[0] - start_c[0], end_c[1] - start_c[1],
                        shape='full',
                        color='white',
                        alpha=0.5,
                        head_width=1,
                        head_length=2,
                        length_includes_head=True,
                        zorder=1,
                    )
        # 绘制目标区域
        if self.x_target and self.y_target:
            x, y = self.x_target, self.y_target
            codes_box = [Path.MOVETO] + [Path.LINETO] * 3 + [Path.CLOSEPOLY]  # 路径连接方式
            vertices_box = [(x[0], y[0]), (x[1], y[0]), (x[1], y[1]), (x[0], y[1]), (0, 0)]  # 路径连接点
            path_box = Path(vertices_box, codes_box)  # 定义对应Path
            pathpatch_box = patches.PathPatch(path_box, facecolor='tomato', edgecolor='orangered', zorder=2)
            self.axbg.add_patch(pathpatch_box)
        return
