import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int32MultiArray
from multisim.srv import Reset
import numpy as np

class MultiAgentEnv(Node):
    def __init__(self):
        super().__init__('multi_agent_env')
        self.reset_service = self.create_client(Reset, '/multisim/reset')
        self.proposed_map_sub = self.create_subscription(
            OccupancyGrid, '/proposed_simplified_map', self.proposed_map_callback, 10)
        self.true_map_sub = self.create_subscription(
            OccupancyGrid, '/true_simplified_map', self.true_map_callback, 10)
        self.collision_sub = self.create_subscription(
            Int32MultiArray, '/collision_info', self.collision_callback, 10)

        self.proposed_map = None
        self.true_map = None
        self.collisions = []

    def reset(self, seed):
        req = Reset.Request()
        req.seed = seed
        while not self.reset_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        future = self.reset_service.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def proposed_map_callback(self, msg):
        self.proposed_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def true_map_callback(self, msg):
        self.true_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def collision_callback(self, msg):
        self.collisions = msg.data

    def get_state(self):
        return {
            'proposed_map': self.proposed_map,
            'true_map': self.true_map,
            'collisions': self.collisions
        }
    
    def calculate_reward(self):
        if self.proposed_map is None or self.true_map is None:
            return 0
        coverage = np.sum(self.proposed_map == self.true_map) / self.proposed_map.size
        penalty = np.sum(self.collisions)
        return coverage - penalty
