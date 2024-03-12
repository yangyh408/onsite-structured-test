from utils.opendrive2discretenet import parse_opendrive
from utils.ScenarioManager import select_scenario_manager
from utils.ScenarioManager.ScenarioInfo import ScenarioInfo
from utils.observation import Observation, EgoStatus, ObjectStatus

import os
import numpy as np
import pandas as pd
from typing import Dict

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib.gridspec import GridSpec
from matplotlib.animation import FuncAnimation
import matplotlib.colors as colors
import mpl_toolkits.axes_grid1


class Player(FuncAnimation):
    def __init__(self, fig, func, playerax, frames=None, init_func=None, fargs=None,
                 save_count=None, mini=0, maxi=100, **kwargs):
        self.i = 0
        self.min=mini
        self.max=maxi
        self.runs = True
        self.forwards = True
        self.fig = fig
        self.func = func
        self.setup(playerax)
        if not save_count:
            save_count = self.max + 1
        FuncAnimation.__init__(self,self.fig, self.update, frames=self.play(), 
                                           init_func=init_func, fargs=fargs,
                                           save_count=save_count, **kwargs )    

    def play(self):
        while self.runs:
            self.i = np.clip(self.i + self.forwards - (not self.forwards), self.min, self.max)
            if self.i > self.min and self.i < self.max:
                yield self.i
            else:
                self.stop()
                yield self.i

    def start(self):
        self.runs=True
        self.event_source.start()

    def stop(self, event=None):
        self.runs = False
        self.event_source.stop()

    def forward(self, event=None):
        self.forwards = True
        self.start()
    def backward(self, event=None):
        self.forwards = False
        self.start()
    def oneforward(self, event=None):
        self.forwards = True
        self.onestep()
    def onebackward(self, event=None):
        self.forwards = False
        self.onestep()

    def onestep(self):
        if self.i > self.min and self.i < self.max:
            self.i = self.i+self.forwards-(not self.forwards)
        elif self.i == self.min and self.forwards:
            self.i+=1
        elif self.i == self.max and not self.forwards:
            self.i-=1
        self.func(self.i)
        self.slider.set_val(self.i)
        self.fig.canvas.draw_idle()

    def setup(self, playerax):
        divider = mpl_toolkits.axes_grid1.make_axes_locatable(playerax)
        bax = divider.append_axes("right", size="80%", pad=0.05)
        sax = divider.append_axes("right", size="80%", pad=0.05)
        fax = divider.append_axes("right", size="80%", pad=0.05)
        ofax = divider.append_axes("right", size="100%", pad=0.05)
        sliderax = divider.append_axes("right", size="500%", pad=0.07)
        blank = divider.append_axes("right", size="30%", pad=0.05, zorder=-1)
        blank.axis('off')
        self.button_oneback = matplotlib.widgets.Button(playerax, label='$\u29CF$')
        self.button_back = matplotlib.widgets.Button(bax, label='$\u25C0$')
        self.button_stop = matplotlib.widgets.Button(sax, label='$\u25A0$')
        self.button_forward = matplotlib.widgets.Button(fax, label='$\u25B6$')
        self.button_oneforward = matplotlib.widgets.Button(ofax, label='$\u29D0$')
        self.button_oneback.on_clicked(self.onebackward)
        self.button_back.on_clicked(self.backward)
        self.button_stop.on_clicked(self.stop)
        self.button_forward.on_clicked(self.forward)
        self.button_oneforward.on_clicked(self.oneforward)
        self.slider = matplotlib.widgets.Slider(sliderax, '', 
                                                self.min, self.max, valinit=self.i, valfmt='%d')
        self.slider.on_changed(self.set_pos)

    def set_pos(self, i):
        self.i = int(self.slider.val)
        self.func(self.i)

    def update(self, i):
        self.slider.set_val(i)


class Visualizer():
    def __init__(self):
        self.result_df = None       # 输出文件对应的DataFrame
        self.scene_info = None      # 待可视化的场景信息
        self.road_info = None       # 通过parse_opendrive模块解析出的opendrive路网信息
        self.fig_width = 9          # 画布宽度
        self.fig_height = 6         # 画布高度
        self.fig = None             # 画布实例
        self.vis_distance = 30      # 测试详情区域主车周围可视化范围
        self.object_color = {          # 不同类型物体的颜色映射
            'ego': 'orange',
            'vehicle': 'cornflowerblue',
            'bicycle': 'lightgreen',
            'pedestrian': 'lightcoral',
        }
    
    def replay_result(self, result_path: str, save_path: str=None):
        """可视化回放接口"""
        # 解析结果文件
        self.result_df = pd.read_csv(result_path)
        # 加载场景信息
        result_file = os.path.basename(result_path)
        mode = result_file.split('_')[0]
        task = '_'.join(result_file.split('_')[2:-1])
        self.scene_info = self._load_result_scene(mode, task)
        self.scene_info.task_info['dt'] = f"{self.result_df.iloc[1, 0] - self.result_df.iloc[0, 0]:.2f}"
        # 解析opendrive路网文件
        self.road_info = parse_opendrive(self.scene_info.source_file['xodr'])
        # 进行可视化回放
        self.replay_create_ax()
        ani = Player(self.fig, self.replay_update, playerax=self.ax_player, init_func=self.plot_static, interval=100, repeat=False, maxi=len(self.result_df)-1)
        if save_path:
            ani.save(save_path, writer='pillow', fps=10)
            # ani.save(save_path, writer='imagemagick', fps=int(1/self.scene_info.task_info['dt']))
        else:
            # self.plot_static()
            # self.update(2)
            plt.show()
    
    def replay_create_ax(self):
        """进行可视化回放的视图创建和划分"""
        # 创建画布
        self.fig_width = 9
        self.fig_height = 6
        self.fig = plt.figure(figsize=(self.fig_width, self.fig_height))
        # 划分网格
        gs = GridSpec(3, 2, width_ratios=[2, 5], height_ratios=[10, 5, 1])
        self.ax_table = plt.subplot(gs[0, 0])       # 左上角表格区域
        self.ax_map_bg = plt.subplot(gs[1, 0])      # 左下角地图区域
        self.ax_detail_bg = plt.subplot(gs[:2, 1])  # 右边测试详情区域
        self.ax_player = plt.subplot(gs[2, :])      # 底部播放控制区域
        # 创建动态元素图层
        self.ax_detail_obj = self.ax_detail_bg.twinx()
        self.ax_map_obj = self.ax_map_bg.twinx()
        # 网格区域初始设置
        self.ax_map_bg.set_xticks([])
        self.ax_map_bg.set_yticks([])
        self.ax_detail_obj.set_yticks([])
        self.ax_map_obj.set_yticks([])
        self.ax_table.axis('off')

    def plot_static(self):
        """进行可视化回放的静态信息绘制"""
        # 绘制左上角表格区域
        self.table = self._plot_table(self.ax_table, self.scene_info)
        # 绘制右边测试详情区域
        self._plot_roads(self.ax_detail_bg, self.road_info, draw_arrow=True)
        self._plot_goal(self.ax_detail_bg, self.scene_info.task_info['targetPos'])
        
        ax_detail_range = self._update_ax_limit(self.ax_detail_bg, 
                                               [self.scene_info.task_info['startPos'][0]-self.vis_distance, self.scene_info.task_info['startPos'][0]+self.vis_distance], 
                                               [self.scene_info.task_info['startPos'][1]-self.vis_distance, self.scene_info.task_info['startPos'][1]+self.vis_distance])
        self.ax_detail_obj.set_ylim(ax_detail_range[1][0], ax_detail_range[1][1])
        if self.scene_info.task_info['waypoints']:
            self._plot_waypoints(self.ax_detail_bg, self.scene_info.task_info['waypoints'])
        # 绘制左下角地图区域
        self._plot_roads(self.ax_map_bg, self.road_info)
        self._plot_goal(self.ax_map_bg, self.scene_info.task_info['targetPos'])
        ax_map_range = self._update_ax_limit(self.ax_map_bg, *self._get_road_boundary(self.road_info))
        self.ax_map_obj.set_ylim(ax_map_range[1][0], ax_map_range[1][1])
        self.position_box = self._plot_position_box(self.ax_map_obj, ax_detail_range)
        plt.tight_layout()

    def replay_update(self, frame: int):
        """可视化回放的界面更新"""
        test_info, observation = self._get_frame_info_from_result(frame)
        self.update_dynamic(test_info, observation.ego_info, observation.object_info)

    def update_dynamic(self, test_info: dict, ego_info: EgoStatus, object_info: Dict[str, ObjectStatus]):
        """对每一帧进行画布更新"""
        self.ax_detail_obj.cla()
        self.ax_detail_obj.set_yticks([])

        # 更新表格信息
        self._update_table(test_info)
        # 更新右边测试详情区域物体信息
        new_range = self._update_ax_limit(self.ax_detail_bg, 
                                                  [ego_info.x-self.vis_distance, ego_info.x+self.vis_distance], 
                                                  [ego_info.y-self.vis_distance, ego_info.y+self.vis_distance])
        self.ax_detail_obj.set_ylim(new_range[1][0], new_range[1][1])
        self._plot_single_object(self.ax_detail_obj, 'ego', ego_info, c=self.object_color['ego'])
        for obj_type in object_info.keys():
            for obj_id, obj_state in object_info[obj_type].items():
                self._plot_single_object(self.ax_detail_obj, obj_id, obj_state, c=self.object_color[obj_type])
        # 更新左下角地图区域定位框位置
        if new_range:
            self.position_box.set_xy([new_range[0][0], new_range[1][0]])

    def live_init(self, scene_info=None, road_info=None):
        """REPLAY模式下进行可视化的静态信息绘制"""
        self.scene_info = scene_info
        self.road_info = road_info
        # 创建画布
        self.fig_width = 9
        self.fig_height = 6
        self.fig = plt.figure(figsize=(self.fig_width, self.fig_height))
        # 划分网格
        gs = GridSpec(2, 2, width_ratios=[2, 5], height_ratios=[2, 1])
        self.ax_table = plt.subplot(gs[0, 0])       # 左上角表格区域
        self.ax_map_bg = plt.subplot(gs[1, 0])      # 左下角地图区域
        self.ax_detail_bg = plt.subplot(gs[:, 1])   # 右边测试详情区域
        # 创建动态元素图层
        self.ax_detail_obj = self.ax_detail_bg.twinx()
        self.ax_map_obj = self.ax_map_bg.twinx()
        # 网格区域初始设置
        self.ax_map_bg.set_xticks([])
        self.ax_map_bg.set_yticks([])
        self.ax_detail_obj.set_yticks([])
        self.ax_map_obj.set_yticks([])
        self.ax_table.axis('off')
        self.plot_static()
        plt.ion()

    def live_update(self, observation: Observation):
        """REPLAY模式下进行可视化界面更新"""
        test_info = {
            't': observation.test_info['t'],
            'end': observation.test_info['end'],
            'acc': observation.ego_info.a,
            'rot': observation.ego_info.rot,
            'ego_x': observation.ego_info.x,
            'ego_y': observation.ego_info.y,
            'ego_v': observation.ego_info.v,
            'ego_yaw': observation.ego_info.yaw,
        }
        self.update_dynamic(test_info, observation.ego_info, observation.object_info)
        if observation.test_info['end'] != -1:
            plt.ioff()
            plt.close()
        plt.pause(1e-7)
        
    def show_task(self, mode: str, task: str):
        """展示指定任务的场景信息"""
        self.fig = plt.figure(figsize=(self.fig_width, self.fig_height))
        self.ax = self.fig.add_subplot(111)
        # 加载场景信息
        self.scene_info = self._load_result_scene(mode, task)
        # 解析opendrive路网文件
        self.road_info = parse_opendrive(self.scene_info.source_file['xodr'])
        # 绘制路网信息
        self._plot_roads(self.ax, self.road_info, draw_arrow=True)
        ax_map_range = self._update_ax_limit(self.ax, *self._get_road_boundary(self.road_info))
        self.ax.set_ylim(ax_map_range[1][0], ax_map_range[1][1])
        # 绘制起终点
        self.ax.scatter(self.scene_info.task_info['startPos'][0], self.scene_info.task_info['startPos'][1], color='green', marker='o', s=15, zorder=3)
        self.ax.annotate('START', (self.scene_info.task_info['startPos'][0]-3, self.scene_info.task_info['startPos'][1]+2), fontsize=10)
        self._plot_goal(self.ax, self.scene_info.task_info['targetPos'])

        if self.scene_info.task_info['waypoints']:
            self._plot_waypoints(self.ax, self.scene_info.task_info['waypoints'])
            waypoints = np.array(list(self.scene_info.task_info['waypoints'].values()))
            self._update_ax_limit(self.ax,
                                  [np.min(waypoints[:, 0])-10, np.max(waypoints[:, 0])+10],
                                  [np.min(waypoints[:, 1])-10, np.max(waypoints[:, 1])+10])
        plt.show()

    def show_map(self, xodr_path):
        """展示指定opendrive路网文件的地图"""
        self.fig = plt.figure(figsize=(self.fig_width, self.fig_height))
        self.ax = self.fig.add_subplot(111)
        self.road_info = parse_opendrive(xodr_path)
        self._plot_roads(self.ax, self.road_info, draw_arrow=True)
        plt.show()

    def _plot_roads(self, ax, road_info, draw_arrow: bool=False) -> None:
        """根据parse_opendrive模块解析出的opendrive路网信息绘制道路"""
        xlim1 = float("Inf")
        xlim2 = -float("Inf")
        ylim1 = float("Inf")
        ylim2 = -float("Inf")
        color = "gray"
        label = None

        for discrete_lane in road_info.discretelanes:
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

            ax.add_patch(
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

            ax.plot(
                [x for x, _ in discrete_lane.left_vertices],
                [y for _, y in discrete_lane.left_vertices],
                color="black",
                lw=0.3,
                zorder=1,
            )
            ax.plot(
                [x for x, _ in discrete_lane.right_vertices],
                [y for _, y in discrete_lane.right_vertices],
                color="black",
                lw=0.3,
                zorder=1,
            )

            ax.plot(
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
                    ax.arrow(
                        start_c[0], start_c[1], end_c[0] - start_c[0], end_c[1] - start_c[1],
                        shape='full',
                        color='white',
                        alpha=0.5,
                        head_width=1,
                        head_length=2,
                        length_includes_head=True,
                        zorder=1,
                    )
        return

    def _plot_goal(self, ax, goal) -> None:
        """绘制目标区域"""
        format_goal = {
            'x': sorted([item[0] for item in goal]),
            'y': sorted([item[1] for item in goal])
        }
        ax.add_patch(
            patches.Rectangle(
                xy=(format_goal['x'][0], format_goal['y'][0]),
                width=format_goal['x'][1]-format_goal['x'][0],
                height=format_goal['y'][1]-format_goal['y'][0],
                angle=0,
                color="tomato",
                fill=True,
                alpha=0.8,
                zorder=3
            ))

    def _plot_waypoints(self, ax, waypoints) -> None:
        """绘制路径点"""
        x = [point[0] for point in waypoints.values()]
        y = [point[1] for point in waypoints.values()]
        color = np.linspace(0.5, 1, len(x))
        color_map = colors.LinearSegmentedColormap.from_list('new_blue', plt.get_cmap('Blues')(np.linspace(0.5, 1, len(x))),)
        ax.scatter(x, y, c=color, cmap=color_map, marker='o', s=2, zorder=2)

    def _plot_position_box(self, ax, range):
        """绘制地图区域的定位框"""
        position_box = patches.Rectangle(
                xy=(range[0][0], range[1][0]),
                width=range[0][1]-range[0][0],
                height=range[1][1]-range[1][0],
                angle=0,
                color="blue",
                fill=True,
                alpha=0.4,
                zorder=5
            )
        ax.add_patch(position_box)
        return position_box

    def _plot_single_object(self, ax, key: str, objecti: ObjectStatus, c='cornflowerblue'):
        """利用 matplotlib 和 patches 绘制小汽车，以 x 轴为行驶方向"""
        x, y, yaw, width, length = [float(objecti.__getattribute__(i)) for i in ['x', 'y', 'yaw', 'width', 'length']]

        angle = np.arctan(width / length) + yaw
        diagonal = np.sqrt(length ** 2 + width ** 2)
        ax.add_patch(
            patches.Rectangle(
                xy=(x - diagonal / 2 * np.cos(angle),
                    y - diagonal / 2 * np.sin(angle)),
                width=length,
                height=width,
                angle=yaw / np.pi * 180,
                color=c,
                fill=True,
                zorder=4
            ))
        ax.annotate(key, (x, y), fontsize=8, zorder=5)

    def _plot_table(self, ax, scene_info: ScenarioInfo):
        """绘制初始化表格信息"""
        data = [['scene', scene_info.name],
                ['mode', scene_info.type],
                ['dt', scene_info.task_info['dt']],
                ['t', None],
                ['end', None],
                ['acc', None],
                ['rot', None],
                ['x_ego', None],
                ['y_ego', None],
                ['v_ego', None],
                ['yaw_ego', None]]

        table = ax.table(cellText=data, loc='upper left', cellLoc='center', fontsize=10, colWidths=[0.3, 0.65])

        table.auto_set_font_size(False)
        table.auto_set_column_width([0])
        table.scale(1, 1.4) 

        table[(0, 1)].set_fontsize(8)
        table[(5, 1)].set_text_props(fontweight='bold', color='red')
        table[(6, 1)].set_text_props(fontweight='bold', color='red')

        return table

    def _load_result_scene(self, mode: str, task: str) -> ScenarioInfo:
        """加载输出文件对应的测试场景"""
        sm = select_scenario_manager(mode, {'tasks': [task]})
        if sm.next():
            return sm.cur_scene
        else:
            raise ValueError(f"Failed to load scenario: {task}")

    def _get_road_boundary(self, road_info: dict) -> list:
        """获取地图边界信息"""
        points = np.empty(shape=[0, 2])
        for discrete_lane in road_info.discretelanes:
            points = np.concatenate(
                [discrete_lane.left_vertices, discrete_lane.right_vertices, discrete_lane.center_vertices, points],
                axis=0)
        return [np.min(points[:, 0]), np.max(points[:, 0])], [np.min(points[:, 1]), np.max(points[:, 1])]    
        
    def _get_frame_info_from_result(self, frame: int) -> dict:
        """从输出文件中获取指定帧的信息"""
        new_observation = Observation()

        obj_states = ['x', 'y', 'v', 'a', 'yaw', 'width', 'length']

        frame_series = self.result_df.iloc[frame].dropna()
        objects_info = {}
        for col_name in frame_series.keys():
            if col_name.startswith('x_') and not col_name.endswith('ego'):
                objects_info[col_name[2:]] = {}

        new_observation.update_ego_info(
            x=frame_series['x_ego'],
            y=frame_series['y_ego'],
            v=frame_series['v_ego'],
            a=frame_series['a_ego'],
            yaw=frame_series['yaw_ego'],
            rot=frame_series['rot_ego'],
            length=frame_series['length_ego'],
            width=frame_series['width_ego']
        )
        
        for obj in objects_info.keys():
            if obj == 'ego':
                continue
            for state in obj_states:
                objects_info[obj][state] = frame_series[f"{state}_{obj}"]
            new_observation.update_object_info("vehicle", obj, **objects_info[obj])
        
        test_info = {
            't': frame_series[0],
            'end': self._format_number(frame_series['end'], 0),
            'acc': f"{new_observation.ego_info.a} ({round(frame_series['acc'], 3)})",
            'rot': f"{new_observation.ego_info.rot} ({round(frame_series['rot'], 3)})",
            'ego_x': new_observation.ego_info.x,
            'ego_y': new_observation.ego_info.y,
            'ego_v': new_observation.ego_info.v,
            'ego_yaw': new_observation.ego_info.yaw,
        }
        return test_info, new_observation

    def _update_table(self, data: dict) -> None:
        """更新表格中的动态信息"""
        for i, val in enumerate(data.values()):
            self.table[(3+i, 1)].get_text().set_text(val)
    
    def _update_ax_limit(self, ax, range_x, range_y):
        """更新子图的坐标轴范围"""
        aspect_ratio = self._cal_aspect_ratio(ax, self.fig_width, self.fig_height)
        new_range_x, new_range_y = self._cal_proportional_range(range_x, range_y, aspect_ratio)
        ax.set_xlim(new_range_x[0], new_range_x[1])
        ax.set_ylim(new_range_y[0], new_range_y[1])
        return [new_range_x, new_range_y]

    @staticmethod
    def _format_number(number: float, digit: int) -> str:
        if number is not None:
            return f"{number:.{digit}f}"
        return ""

    @staticmethod
    def _cal_proportional_range(range_x, range_y, aspect_ratio):
        """计算比例范围
        Args:
            range_x (list): x轴范围
            range_y (list): y轴范围
            aspect_ratio (float): 长宽比
        Returns:
            list: 比例范围
        """
        len_x = range_x[1] - range_x[0]
        len_y = range_y[1] - range_y[0]
        center = [(range_x[0] + range_x[1]) / 2, (range_y[0] + range_y[1]) / 2]

        if len_x > len_y * aspect_ratio:
            len_y = len_x / aspect_ratio
        else:
            len_x = len_y * aspect_ratio
        return [center[0] - len_x / 2, center[0] + len_x / 2], [center[1] - len_y / 2, center[1] + len_y / 2]

    @staticmethod
    def _cal_aspect_ratio(ax, fig_width, fig_height):
        """计算子图的宽高比"""
        width = ax.get_position().width * fig_width
        height = ax.get_position().height * fig_height
        aspect_ratio = width / height
        return aspect_ratio

