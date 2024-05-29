import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int32MultiArray
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
        self.reset_service.call_async(req)

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
