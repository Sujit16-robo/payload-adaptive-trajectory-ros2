import rclpy, csv, sys
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Logger(Node):
    def __init__(self, filename):
        super().__init__('data_logger')
        self.rows = []
        self.sub = self.create_subscription(JointState, '/joint_states', self.cb, 50)
        self.filename = filename
        self.t0 = None

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if self.t0 is None:
            self.t0 = t
        row = [t - self.t0] + list(msg.position) + list(msg.velocity) + list(msg.effort)
        self.rows.append(row)

    def save(self):
        with open(self.filename, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t','q1','q2','q3','v1','v2','v3','e1','e2','e3'])
            w.writerows(self.rows)
        print(f'Saved {len(self.rows)} rows to {self.filename}')

def main():
    rclpy.init()
    filename = sys.argv[1] if len(sys.argv) > 1 else 'log.csv'
    node = Logger(filename)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save()

if __name__ == '__main__':
    main()