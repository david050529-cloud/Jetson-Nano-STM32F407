# waypoint_parser.py
import os

def parse_waypoint_file(file_path):
    """
    解析经纬度航点文件
    文件格式：每行 "序号 纬度 经度 属性"
    返回字典列表，包含 index, latitude, longitude, attribute
    """
    waypoints = []
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Waypoint file not found: {file_path}")

    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if len(parts) == 4:
                index, latitude, longitude, attribute = parts
                waypoints.append({
                    'index': int(index),
                    'latitude': float(latitude),
                    'longitude': float(longitude),
                    'attribute': int(attribute)
                })
    return waypoints

def parse_cartesian_waypoint_file(file_path):
    """
    解析笛卡尔坐标航点文件
    文件格式：每行 "序号 x y 属性"
    返回字典列表，包含 index, x, y, attribute
    """
    waypoints = []
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Waypoint file not found: {file_path}")

    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if len(parts) == 4:
                index, x, y, attribute = parts
                waypoints.append({
                    'index': int(index),
                    'x': int(x),
                    'y': int(y),
                    'attribute': int(attribute)
                })
    return waypoints