import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration

class TrajPublisher(Node):
    def __init__(self):
        super().__init__('traj_publisher')
        self.pub = self.create_publisher(JointTrajectory, '/desired_joint_trajectory', 10)
        self.timer = self.create_timer(3.0, self.send_once)
        self.sent = False

    def send_once(self):
        if self.sent:
            return
        self.sent = True
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3']

        p0 = JointTrajectoryPoint()
        p0.positions = [0.0, 0.0, 0.0]
        p0.velocities = [0.0, 0.0, 0.0]
        p0.time_from_start = Duration(seconds=0.0).to_msg()

        p1 = JointTrajectoryPoint()
        p1.positions = [0.5, 0.5, -0.4]
        p1.velocities = [1.4, 1.4, -1.2]
        p1.time_from_start = Duration(seconds=0.3).to_msg()

        p2 = JointTrajectoryPoint()
        p2.positions = [0.9, 0.8, -0.7]
        p2.velocities = [0.0, 0.0, 0.0]
        p2.time_from_start = Duration(seconds=0.6).to_msg()

        msg.points = [p0, p1, p2]
        self.pub.publish(msg)
        self.get_logger().info('Sent trajectory.')

def main():
    rclpy.init()
    node = TrajPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()