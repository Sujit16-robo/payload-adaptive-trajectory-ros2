import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64

class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('trajectory_executor')
        self.pub1 = self.create_publisher(Float64, '/joint1/cmd_pos', 10)
        self.pub2 = self.create_publisher(Float64, '/joint2/cmd_pos', 10)
        self.pub3 = self.create_publisher(Float64, '/joint3/cmd_pos', 10)
        self.sub = self.create_subscription(JointTrajectory, '/scaled_trajectory', self.cb, 10)
        self.points = []
        self.times = []
        self.start_time = None
        self.timer = self.create_timer(0.02, self.tick)

    def cb(self, msg: JointTrajectory):
        self.points = [p.positions for p in msg.points]
        self.times = [p.time_from_start.sec + p.time_from_start.nanosec*1e-9 for p in msg.points]
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f'Executing trajectory: {len(self.points)} points over {self.times[-1]:.2f}s')

    def tick(self):
        if not self.points or self.start_time is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        t = now - self.start_time
        if t > self.times[-1]:
            t = self.times[-1]

        for i in range(len(self.times)-1):
            if self.times[i] <= t <= self.times[i+1]:
                ratio = (t - self.times[i]) / max(1e-6, self.times[i+1]-self.times[i])
                q = [self.points[i][j] + ratio*(self.points[i+1][j]-self.points[i][j]) for j in range(3)]
                self.pub1.publish(Float64(data=q[0]))
                self.pub2.publish(Float64(data=q[1]))
                self.pub3.publish(Float64(data=q[2]))
                return

def main():
    rclpy.init()
    rclpy.spin(TrajectoryExecutor())

if __name__ == '__main__':
    main()