import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from training import train_policy

class TrialService(Node):
    def __init__(self):
        super().__init__('trial_service')
        self.srv = self.create_service(Empty, 'run_trials', self.run_trials_callback)

    def run_trials_callback(self, request, response):
        train_policy()
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TrialService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
