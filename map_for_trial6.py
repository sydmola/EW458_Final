import roslibpy
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import math
import time

class RobotMapper:
    def __init__(self, ip='192.168.8.104', port=9012, robot_name='juliet', map_topic_name='mapCev', reset_pose=True):
        self.ip = ip
        self.port = port
        self.robot_name = robot_name
        self.map_topic_name = map_topic_name
        self.reset_pose = reset_pose

        self.x = 0
        self.y = 0
        self.yaw = 0
        self.trail = []

        self.ranges = []
        self.angs = []

        self.resolution = 0.1
        self.width = 300
        self.height = 300
        self.origin = (-self.width * self.resolution / 2, -self.height * self.resolution / 2)
        #self.origin = (-15, 15)

        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)
        self.odom_data = deque(maxlen=100)
        self.scan_data = deque(maxlen=100)
        self.odomMsgs = 0
        self.scanMsgs = 0
        self.last_synced_time = 0

        self.ros = roslibpy.Ros(host=self.ip, port=self.port)
        self.ros.run()

        self.odom_topic = roslibpy.Topic(self.ros, f'/{self.robot_name}/odom', 'nav_msgs/Odometry')
        self.scan_topic = roslibpy.Topic(self.ros, f'/{self.robot_name}/scan', 'sensor_msgs/LaserScan')
        self.map_topic = roslibpy.Topic(self.ros, f'/{self.robot_name}/{self.map_topic_name}', 'nav_msgs/OccupancyGrid')

        self.odom_topic.subscribe(self._odom_callback)
        self.scan_topic.subscribe(self._scan_callback)

        if self.reset_pose:
            self.reset_odometry()

    def reset_odometry(self):
        reset_service = roslibpy.Service(self.ros, f'/{self.robot_name}/reset_pose', 'irobot_create_msgs/ResetPose')
        reset_service.call(roslibpy.ServiceRequest())

    def world_to_grid(self, x, y):
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        return gx, gy

    def ensure_within_bounds(self, gx, gy): # to expand the grid if the scan, or robot are out of bounds
        pad_top = pad_bottom = pad_left = pad_right = 0
        expand = False

        if gx < 0:
            pad_left = abs(gx) + 10
            expand = True
        elif gx >= self.width:
            pad_right = gx - self.width + 10
            expand = True

        if gy < 0:
            pad_bottom = abs(gy) + 10
            expand = True
        elif gy >= self.height:
            pad_top = gy - self.height + 10
            expand = True

        if expand:
            self.grid = np.pad(self.grid, ((pad_bottom, pad_top), (pad_left, pad_right)), constant_values=-1)
            self.height, self.width = self.grid.shape
            self.origin = (
                self.origin[0] - pad_left * self.resolution,
                self.origin[1] - pad_bottom * self.resolution
            )
            self.trail = [(x + pad_left, y + pad_bottom) for (x, y) in self.trail]

    def _odom_callback(self, message): #to extract robot's position data
        pos = message['pose']['pose']['position']
        orientation = message['pose']['pose']['orientation']
        stamp = message['header']['stamp']
        timestamp = stamp['sec'] + stamp['nanosec'] * 1e-9
        self.odomMsgs += 1
        self.odom_data.append({
            'timestamp': timestamp,
            'position': {'x': pos['x'], 'y': pos['y']}, #appending dictionaries to the list containing position data and timestamps
            'orientation': orientation
        })

    def _scan_callback(self, message): #to grab lidar data
        stamp = message['header']['stamp']
        timestamp = stamp['sec'] + stamp['nanosec'] * 1e-9
        angle_min = message['angle_min']
        angle_increment = message['angle_increment']
        ranges = message['ranges']
        angles = [angle_min + i * angle_increment for i in range(len(ranges))]
        self.scanMsgs += 1
        self.scan_data.append({
            'timestamp': timestamp,       #appending dictionaries to the list containing scan data and timestamps
            'ranges': ranges,
            'angles': angles
        })
    
    def get_synchronized_messages(self, tolerance=0.02): #self explanatory. Extremely important to grab syncrhonized timestamps to plot correctly
        synchronized = []
        for odom in self.odom_data:
            for scan in self.scan_data:
                if abs(odom['timestamp'] - scan['timestamp']) <= tolerance:
                    synchronized.append((odom, scan))
        return synchronized

    def bresenham(self, x0, y0, x1, y1): #this algorithim returns line of sight tracing essential for us to detect obstacles
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x1, y1))
        return points

    def publish_map(self, odom, scan): # here we 
        self.x = odom['position']['x']
        self.y = odom['position']['y']
        orientation = odom['orientation']
        siny_cosp = 2 * (orientation['w'] * orientation['z'] + orientation['x'] * orientation['y'])
        cosy_cosp = 1 - 2 * (orientation['y'] ** 2 + orientation['z'] ** 2)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.ranges = scan['ranges']
        self.angs = scan['angles']

        robot_gx, robot_gy = self.world_to_grid(self.x, self.y)
        self.ensure_within_bounds(robot_gx, robot_gy)

        for r, a in zip(self.ranges, self.angs):
            if math.isinf(r):
                continue
            wx = self.x + r * math.cos(self.yaw + a)
            wy = self.y + r * math.sin(self.yaw + a)
            gx, gy = self.world_to_grid(wx, wy)
            self.ensure_within_bounds(gx, gy)
            for fx, fy in self.bresenham(robot_gx, robot_gy, gx, gy)[:-1]:
                if 0 <= fx < self.width and 0 <= fy < self.height and self.grid[fy, fx] == -1:
                    self.grid[fy, fx] = 0
            
            if 0 <= gx < self.width and 0 <= gy < self.height and self.grid[gy, gx] != 0:
                self.grid[gy, gx] = 100

        for dx in range(-2, 3):
            for dy in range(-2, 3):
                x = robot_gx + dx
                y = robot_gy + dy
                if 0 <= x < self.width and 0 <= y < self.height and self.grid[y, x] == -1:
                    self.grid[y, x] = 0

        plt.clf()
        cmap = ListedColormap(['gray', 'white', 'black'])
        mapped_grid = np.zeros_like(self.grid)
        mapped_grid[self.grid == -1] = 0
        mapped_grid[self.grid == 0] = 1
        mapped_grid[self.grid == 100] = 2

        plt.imshow(mapped_grid, origin='lower', cmap=cmap, vmin=0, vmax=2)
        #plt.plot(robot_gx, robot_gy, 'ko')
        plt.plot(robot_gx, robot_gy, 'ko', markersize=6, markeredgecolor='black')
        plt.arrow(robot_gx, robot_gy, 5 * math.cos(self.yaw), 5 * math.sin(self.yaw), head_width=2, fc='green')
        plt.title('Robot Occupancy Grid Map')
        plt.pause(0.01)

        map_msg = {
            'header': {
                'frame_id': f'{self.robot_name}/odom',
                'stamp': {
                    'secs': int(time.time()),
                    'nsecs': 0
                }
            },
            'info': {
                'map_load_time': {
                    'secs': int(time.time()),
                    'nsecs': 0
                },
                'resolution': self.resolution,
                'width': self.width,
                'height': self.height,
                'origin': {
                    'position': {'x': self.origin[0], 'y': self.origin[1], 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                }
            },
            'data': self.grid.flatten().tolist()
        }

        self.map_topic.publish(roslibpy.Message(map_msg))
        print(f"Published OccupancyGrid map. Robot position: ({self.x:.2f}, {self.y:.2f})")

    def start(self):
        print("RobotMapper is running. Press Ctrl+C to stop.")
        try:
            while self.ros.is_connected:
                synced = self.get_synchronized_messages()    #we make sure we get our sync data
                new_pairs = [pair for pair in synced if pair[0]['timestamp'] > self.last_synced_time]
                if new_pairs:
                    latest_odom, latest_scan = new_pairs[-1]
                    self.last_synced_time = latest_odom['timestamp']
                    self.publish_map(latest_odom, latest_scan) #publish with synced data
                time.sleep(5)
        except KeyboardInterrupt:
            print("Interrupted by user.")
            self.cleanup()

    def cleanup(self):
        self.odom_topic.unsubscribe()
        self.scan_topic.unsubscribe()
        self.map_topic.unadvertise()
        self.ros.terminate()
        print("ROS connection closed.")

if __name__ == '__main__':
    plt.ion()
    mapper = RobotMapper()
    mapper.start()
