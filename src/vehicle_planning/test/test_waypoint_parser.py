import unittest
from vehicle_planning.waypoint_parser import parse_waypoint_file, parse_cartesian_waypoint_file

class TestWaypointParser(unittest.TestCase):

    def test_parse_waypoint_file_valid(self):
        content = """0 116.33547 39.20677 1
1 116.33547 39.20699 2"""
        with open('test_waypoint.txt', 'w') as f:
            f.write(content)

        waypoints = parse_waypoint_file('test_waypoint.txt')
        self.assertEqual(len(waypoints), 2)
        self.assertEqual(waypoints[0]['index'], 0)
        self.assertEqual(waypoints[0]['latitude'], 116.33547)
        self.assertEqual(waypoints[0]['longitude'], 39.20677)
        self.assertEqual(waypoints[0]['attribute'], 1)

    def test_parse_waypoint_file_invalid(self):
        content = """0 116.33547 39.20677
1 116.33547 39.20699 2"""
        with open('test_invalid_waypoint.txt', 'w') as f:
            f.write(content)

        with self.assertRaises(ValueError):
            parse_waypoint_file('test_invalid_waypoint.txt')

    def test_parse_cartesian_waypoint_file_valid(self):
        content = """0 0 0 1
1 100 200 2"""
        with open('test_cartesian_waypoint.txt', 'w') as f:
            f.write(content)

        waypoints = parse_cartesian_waypoint_file('test_cartesian_waypoint.txt')
        self.assertEqual(len(waypoints), 2)
        self.assertEqual(waypoints[0]['index'], 0)
        self.assertEqual(waypoints[0]['x'], 0)
        self.assertEqual(waypoints[0]['y'], 0)
        self.assertEqual(waypoints[0]['attribute'], 1)

if __name__ == '__main__':
    unittest.main()