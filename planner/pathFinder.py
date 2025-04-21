import numpy as np
from collections import defaultdict
from utils.opendrive2discretenet import parse_opendrive


class DFSPathFinder:
    def __init__(self, scenario_dict):
        xodr_path = scenario_dict['source_file']['xodr']
        road_info = parse_opendrive(xodr_path)
        lanes = road_info.discretelanes
        self.lanes = {lane.lane_id: lane for lane in lanes}
        self.adj_list = self._build_adjacency_list()

    def _build_adjacency_list(self):
        """构建邻接表"""
        adj = defaultdict(list)
        for lane in self.lanes.values():
            adj[lane.lane_id].extend(lane.successor)
        return adj

    def find_shortest_path(self, start_point: np.ndarray, target_rect: tuple):
        """主入口函数"""
        start_lane_id = self._find_start_lane(start_point)
        target_lane_ids = self._find_target_lanes(target_rect)

        if not start_lane_id or not target_lane_ids:
            return None, None

        # 使用DFS找所有可能路径
        all_paths = self._dfs_search(start_lane_id, target_lane_ids)

        if not all_paths:
            return None, None

        # 选择最短路径
        shortest_path = min(all_paths, key=len)
        trajectory = self._generate_trajectory(shortest_path)
        return shortest_path, trajectory

    def _dfs_search(self, start_id, target_ids):
        """迭代式DFS搜索所有路径"""
        stack = [(start_id, [start_id])]
        valid_paths = []

        while stack:
            current_id, path = stack.pop()

            if current_id in target_ids:
                valid_paths.append(path)
                continue  # 继续搜索可能更短的路径

            for successor in self.adj_list.get(current_id, []):
                if successor not in path:  # 防止循环
                    stack.append((successor, path + [successor]))

        return valid_paths

    def _generate_trajectory(self, path):
        """生成轨迹点"""
        trajectory = []
        for i, lane_id in enumerate(path):
            lane = self.lanes[lane_id]

            # # 处理连接方向
            # if i > 0 and lane_id in self.lanes[path[i-1]].successor:
            #     trajectory.extend(lane.center_vertices)
            # else:
            #     trajectory.extend(lane.center_vertices[::-1])  # 反向
            trajectory.extend(lane.center_vertices)

        return np.array(trajectory)

    def _find_start_lane(self, point):
        """通过坐标点查找起始车道（简化实现）"""
        for lane in self.lanes.values():
            if self._is_point_in_lane(point, lane):
                return lane.lane_id
        return None

    def _find_target_lanes(self, rect):
        """查找与目标区域相交的车道（简化实现）"""
        target_lanes = []
        for lane in self.lanes.values():
            if self._is_lane_in_rect(lane, rect):
                target_lanes.append(lane.lane_id)
        return target_lanes

    def _is_point_in_lane(self, point, lane):
        """判断点是否在车道内（使用边界多边形判断）"""
        # 此处实现需要几何计算，以下是示意代码
        left = lane.left_vertices
        right = lane.right_vertices[::-1]
        polygon = np.vstack([left, right])
        return self._point_in_polygon(point, polygon)

    def _is_lane_in_rect(self, lane, rect):
        """判断车道是否与矩形相交（使用中心线判断）"""
        # rect格式为((xmin, ymin), (xmax, ymax))
        return any(self._point_in_rect(p, rect) for p in lane.center_vertices)

    @staticmethod
    def _point_in_polygon(point, polygon):
        """射线法判断点是否在多边形内"""
        x, y = point
        n = len(polygon)
        inside = False
        p1x, p1y = polygon[0]
        for i in range(n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xints:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    @staticmethod
    def _point_in_rect(point, rect):
        """判断点是否在矩形内"""
        (x1, y1), (x2, y2) = rect
        x, y = point
        return (min(x1, x2) <= x <= max(x1, x2)) and (min(y1, y2) <= y <= max(y1, y2))