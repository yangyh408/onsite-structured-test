netTopo_table = dict()  # 地图中路段与连接器的拓朴关系
connectionRelationship = []
# 路网图
graph = {}
startLinkID = []  # 找到所有起始Link，上下行方向各一条
endLinkId = []

paintPos = {"pos": []}
outSide = {"outSide": False}
startEndPos = {"startPos": None, "endPos": None}
waypoints = {"waypoints": None}
crash = {"crash": False}

def findPaths(graph, start_node, end_nodes, path=None):
    # 将当前节点添加到路径
    if path is None:
        path = []
    path = path + [start_node]

    # 如果当前节点是终点节点之一，输出路径
    if start_node in end_nodes:
        yield path

    # 对于当前节点的每个邻居节点，递归查找路径
    if start_node in graph:
        for neighbor in graph[start_node]:
            if neighbor not in path:
                yield from findPaths(graph, neighbor, end_nodes, path)
