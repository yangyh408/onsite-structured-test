# 场景比例尺
sceneScale = 1

# opendrive —> tess 车道, 不在此映射表中的车道不予显示
# tess 中车道类型定义 机动车道/机非共享/非机动车道/公交专用道
LANE_TYPE_MAPPING = {
    'driving': '机动车道',
    'biking': '非机动车道',
    'sidewalk': '人行道',  # 行人道实际无意义
    'stop': '应急车道',

    'onRamp': '机动车道',
    'offRamp': '机动车道',
    'entry': '机动车道',
    'exit': '机动车道',
    'connectingRamp': '机动车道',
    'shoulder': '应急车道',
    'parking': '停车带',
}

# 当前后几个点的向量夹角小于 default_angle 且点距小于 max_length(除非夹角为0 ) 时，抹除过渡点
default_angle = 1
max_length = 50
# 如果是opendrive导入的路网，会主动进行简化(仅路段)，避免创建过慢
simplify_network_force = True


# 连续次数后可视为正常车道，或者连续次数后可视为连接段,最小值为2
point_require = 2
POINT_REQUIRE = max(2, point_require)

# 当 opendrive 连接段的首尾连接长度低于此值时，抛弃原有的点序列，使用自动连接
MIN_CONNECTOR_LENGTH = None

# 是否将路网移至画面中心
is_center = False

# 需要被处理的车道类型及处理参数
WIDTH_LIMIT = {
    '机动车道': {
        'split': 2,  # 作为正常的最窄距离
        'join': 1.5,  # 被忽略时的最宽距离
    },
    '非机动车道': {
        'split': 2,
        'join': 0.5,
    },
}

# 拓宽连接段时的路段裁剪长度
SPLIT_LENGTH = 2

