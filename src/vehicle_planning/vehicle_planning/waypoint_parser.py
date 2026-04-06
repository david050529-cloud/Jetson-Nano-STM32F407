import os

def parse_waypoint_file(file_path):
    """Parse a waypoint file with latitude and longitude."""
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
    """Parse a waypoint file with Cartesian coordinates."""
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