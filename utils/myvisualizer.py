import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib.collections import LineCollection
import matplotlib.colors as colors
import mpl_toolkits.axes_grid1
import numpy as np
from utils.opendrive2discretenet import parse_opendrive
from utils.ScenarioManager import select_scenario_manager
from utils.ScenarioManager.ScenarioInfo import ScenarioInfo

class TrajVisualizer():
    def __init__(self, scene_info: dict):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.scene_info = scene_info
        self.legend_flag = False

    def show_path(self, path_points, highlight_lanes=None):
        xodr_path = self.scene_info['source_file']['xodr']
        road_info = parse_opendrive(xodr_path)
        discreate_map = road_info.discretelanes
        # 绘制所有车道
        for lane in discreate_map:
            self._draw_lane(lane, highlight_lanes=highlight_lanes)

        # 设置显示范围
        self.ax.autoscale_view()
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        plt.title("Road Network Visualization")

        path = np.zeros([len(path_points), 2])
        for i in range(len(path_points)):
            path[i][0] = path_points[i].rx
            path[i][1] = path_points[i].ry

        self._draw_path(path)

        plt.show()

    def _draw_lane(self, lane, highlight_lanes=None):
        """绘制单个车道"""
        # 车道颜色逻辑
        color = 'gray'
        linewidth = 1.5
        if highlight_lanes and lane.lane_id in highlight_lanes:
            color = 'red'
            linewidth = 3.0

        # 绘制车道边界
        left_line = LineCollection(
            [lane.left_vertices[:, :2]],
            colors=color,
            linewidths=linewidth,
            linestyle='--'
        )

        right_line = LineCollection(
            [lane.right_vertices[:, :2]],
            colors=color,
            linewidths=linewidth,
            linestyle='--'
        )

        # 绘制中心线
        center_line = LineCollection(
            [lane.center_vertices[:, :2]],
            colors='blue',
            linewidths=1.0,
            alpha=0.6
        )

        self.ax.add_collection(left_line)
        self.ax.add_collection(right_line)
        self.ax.add_collection(center_line)

    def _draw_path(self, path):
        start_point = self.scene_info['task_info']['startPos']
        target_rect = self.scene_info['task_info']['targetPos']
        # 绘制轨迹线
        self.ax.plot(
            path[:, 0],
            path[:, 1],
            'g-',
            linewidth=3.5,
            alpha=0.8,
            label='Planned Path'
        )

        # 标记起点
        self.ax.plot(
            start_point[0],
            start_point[1],
            'bo',
            markersize=10,
            label='Start Point'
        )

        # 绘制目标区域
        (x1, y1), (x2, y2) = target_rect
        self.ax.add_patch(plt.Rectangle(
            (min(x1, x2), min(y1, y2)),
            abs(x2 - x1),
            abs(y2 - y1),
            edgecolor='orange',
            facecolor='yellow',
            alpha=0.3,
            label='Target Area'
        ))

        # 添加图例
        # if not self.legend_flag:
        self.ax.legend(loc='upper left')
            # self.legend_flag = True

    def show_traj(self, traj_points,
                  position=None,
                  path_points=None,
                  add_traj=False,
                  color='r',
                  linewidth=1,
                  linestyle='-'):

        if not add_traj:
            self.ax.cla()
        if path_points is not None:
            self.show_path(path_points, highlight_lanes=None)
        if position is not None:
            self.ax.plot(position[0], position[1], 'og')
        tx = [x.x for x in traj_points]
        ty = [y.y for y in traj_points]
        self.ax.plot(tx, ty, color=color, linewidth=linewidth, linestyle=linestyle)
