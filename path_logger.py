import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class PathLogger(Node):
    def __init__(self):
        super().__init__('path_logger')
        self.sub = self.create_subscription(Path, '/mpc/predicted_path', self.cb, 10)
        self.count = 0

    def cb(self, msg):
        self.count += 1
        if self.count % 20 == 0:
            if len(msg.poses) > 0:
                p0 = msg.poses[0].pose.position
                p_end = msg.poses[-1].pose.position
                self.get_logger().info(f'Path start: ({p0.x:.2f}, {p0.y:.2f}) -> end: ({p_end.x:.2f}, {p_end.y:.2f})')

def main():
    rclpy.init()
    node = PathLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
