# waypoint_parser.py
"""
路点文件解析器
解析比赛提供的 waypoint.txt,支持两种坐标格式:
  (1) 经纬度格式（默认）：序号(int)  经度(double)  纬度(double)  属性(int)
  (2) 直角坐标格式：      序号(int)  X坐标(int/cm)  Y坐标(int/cm)  属性(int)

注意：比赛规则表头为"经度 纬度"顺序（先经度后纬度），
      与常见"纬度/经度"习惯相反，解析时已按实际格式处理。
"""

import os


# ── 路点属性常量（与 waypoint.txt 中第4列属性值对应） ──────────────────────────
# 用于局部规划器判断当前路点应执行的驾驶行为
ATTR_UNKNOWN     = 0   # 未知，默认按直行处理
ATTR_STRAIGHT    = 1   # 直行（保持车道）
ATTR_TURN_RIGHT  = 2   # 右转（在路口向右转向）
ATTR_TURN_LEFT   = 3   # 左转（在路口向左转向）
ATTR_LANE_LEFT   = 4   # 左换道（向左侧车道变道）
ATTR_LANE_RIGHT  = 5   # 右换道（向右侧车道变道）
ATTR_OVERTAKE    = 6   # 超车（从左侧绕过前方慢速/停止车辆）
ATTR_U_TURN      = 7   # 掉头（180°转向）
ATTR_PARK        = 8   # 泊车（减速靠边停车）

# 属性描述字典，方便日志打印
ATTR_NAMES = {
    ATTR_UNKNOWN:    '未知',
    ATTR_STRAIGHT:   '直行',
    ATTR_TURN_RIGHT: '右转',
    ATTR_TURN_LEFT:  '左转',
    ATTR_LANE_LEFT:  '左换道',
    ATTR_LANE_RIGHT: '右换道',
    ATTR_OVERTAKE:   '超车',
    ATTR_U_TURN:     '掉头',
    ATTR_PARK:       '泊车',
}


def parse_waypoint_file(file_path: str) -> list:
    """
    解析经纬度路点文件（比赛默认格式）

    文件格式（空格或制表符分隔）：
      序号(int)  经度(double)  纬度(double)  属性(int)

    注意：
      - 序号从 0 开始，最后一个路点为终点
      - 经纬度精确到小数点后5位(普通民用GPS精度,误差约2-10米)
      - 经度或纬度为 0 表示该路点数据无效，应忽略
      - 属性值含义见 ATTR_* 常量

    :param file_path: waypoint.txt 文件路径
    :return: 路点字典列表，每项含 index, longitude, latitude, attribute
    :raises FileNotFoundError: 文件不存在时抛出
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"路点文件未找到: {file_path}")

    waypoints = []
    with open(file_path, 'r', encoding='utf-8') as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            # 跳过空行和注释行
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue  # 数据不足，跳过该行

            try:
                index     = int(parts[0])
                longitude = float(parts[1])   # 经度（东经，如 116.xxxxx）
                latitude  = float(parts[2])   # 纬度（北纬，如  39.xxxxx）
                attribute = int(parts[3])

                # 过滤无效路点（经纬度均为0视为无效占位）
                if longitude == 0.0 and latitude == 0.0:
                    continue

                waypoints.append({
                    'index':     index,
                    'longitude': longitude,
                    'latitude':  latitude,
                    'attribute': attribute,
                })
            except (ValueError, IndexError):
                continue  # 格式错误行静默跳过

    return waypoints


def parse_cartesian_waypoint_file(file_path: str) -> list:
    """
    解析直角坐标路点文件

    文件格式（空格分隔）：
      序号(int)  X坐标(int,cm)  Y坐标(int,cm)  属性(int)

    说明：
      - 坐标原点 (0, 0) 为无人车出发位置，初始姿态由工作人员指定
      - X、Y 坐标为整数,单位厘米(cm)
      - 最后一个路点为终点

    :param file_path: waypoint.txt 文件路径
    :return: 路点字典列表，每项含 index, x(cm), y(cm), attribute
    :raises FileNotFoundError: 文件不存在时抛出
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"路点文件未找到: {file_path}")

    waypoints = []
    with open(file_path, 'r', encoding='utf-8') as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue

            try:
                index     = int(parts[0])
                x         = int(parts[1])     # X 坐标（厘米）
                y         = int(parts[2])     # Y 坐标（厘米）
                attribute = int(parts[3])

                waypoints.append({
                    'index':     index,
                    'x':         x,
                    'y':         y,
                    'attribute': attribute,
                })
            except (ValueError, IndexError):
                continue

    return waypoints
