import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.action import ActionClient
from dobot_msgs.action import PointToPoint

class DobotController(Node):
    def __init__(self):
        super().__init__('dobot_controller')
        
        # Position variables
        self.x = 200.0  # Default values
        self.y = 0.0
        self.z = 100.0
        self.orientation = 0.0

        # Flags to track updates
        self.position_updated = {
            'x': False,
            'y': False,
            'z': False,
            'orientation': False
        }

        # Subscribe to position topics
        self.create_subscription(Float64, '/x_position', self.x_callback, 10)
        self.create_subscription(Float64, '/y_position', self.y_callback, 10)
        self.create_subscription(Float64, '/z_position', self.z_callback, 10)
        self.create_subscription(Float64, '/orientation', self.orientation_callback, 10)

        # Action client
        self.action_client = ActionClient(self, PointToPoint, '/PTP_action')
        
        # Timer to check and send goal
        self.create_timer(1.0, self.check_and_send_goal)

    def x_callback(self, msg):
        self.x = msg.data
        self.position_updated['x'] = True
        self.get_logger().info(f'X position updated: {self.x}')

    def y_callback(self, msg):
        self.y = msg.data
        self.position_updated['y'] = True
        self.get_logger().info(f'Y position updated: {self.y}')

    def z_callback(self, msg):
        self.z = msg.data
        self.position_updated['z'] = True
        self.get_logger().info(f'Z position updated: {self.z}')

    def orientation_callback(self, msg):
        self.orientation = msg.data
        self.position_updated['orientation'] = True
        self.get_logger().info(f'Orientation updated: {self.orientation}')

    def check_and_send_goal(self):
        # Check if all positions have been updated
        if all(self.position_updated.values()):
            self.get_logger().info('All positions received. Sending action goal.')
            self.send_goal()
            
            # Reset update flags
            for key in self.position_updated:
                self.position_updated[key] = False

    def send_goal(self):
        # Prepare and send action goal
        goal_msg = PointToPoint.Goal()
        goal_msg.motion_type = 1
        goal_msg.target_pose = [self.x, self.y, self.z, self.orientation]
        goal_msg.velocity_ratio = 0.5
        goal_msg.acceleration_ratio = 0.3

        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            return

        # Send goal
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        
        # Optional: Add result callback
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Action completed')

def main(args=None):
    rclpy.init(args=args)
    controller = DobotController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()