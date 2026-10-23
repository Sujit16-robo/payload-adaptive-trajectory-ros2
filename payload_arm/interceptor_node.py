import rclpy
import numpy as np
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration

G = 9.81
M1 = 74.13
M2, M3 = 38.64, 44.07
L2, L3 = 0.65, 0.508
LC2, LC3 = L2/2, L3/2
IZZ1 = 0.9823
I2 = 1.468
I3 = 1.004

T_MAX = np.array([240.0, 960.0, 400.0])

def joint_torques(q, qd, qdd, mp):
    q1, q2, q3 = q
    qd1, qd2, qd3 = qd
    qdd1, qdd2, qdd3 = qdd

    x2 = LC2*np.sin(q2)
    x3c = L2*np.sin(q2) + LC3*np.sin(q2+q3)
    x3t = L2*np.sin(q2) + L3*np.sin(q2+q3)
    I1_eff = IZZ1 + I2 + I3 + M2*x2**2 + M3*x3c**2 + mp*x3t**2
    tau1 = I1_eff*qdd1

    M11 = I2+I3+M2*LC2**2+M3*(L2**2+LC3**2+2*L2*LC3*np.cos(q3))+mp*(L2**2+L3**2+2*L2*L3*np.cos(q3))
    M12 = I3+M3*(LC3**2+L2*LC3*np.cos(q3))+mp*(L3**2+L2*L3*np.cos(q3))
    M22 = I3+M3*LC3**2+mp*L3**2
    h = -M3*L2*LC3*np.sin(q3) - mp*L2*L3*np.sin(q3)
    C11, C12, C21 = h*qd3, h*(qd2+qd3), -h*qd2
    Gc2 = (M2*LC2+M3*L2+mp*L2)*G*np.cos(q2) + (M3*LC3+mp*L3)*G*np.cos(q2+q3)
    Gc3 = (M3*LC3+mp*L3)*G*np.cos(q2+q3)

    tau2 = M11*qdd2 + M12*qdd3 + C11*qd2 + C12*qd3 + Gc2
    tau3 = M12*qdd2 + M22*qdd3 + C21*qd2 + Gc3
    return np.array([tau1, tau2, tau3])

class Interceptor(Node):
    def __init__(self):
        super().__init__('interceptor_node')
        self.declare_parameter('payload_mass', 0.001)
        self.declare_parameter('adaptive_enabled', True)
        self.sub = self.create_subscription(JointTrajectory, '/desired_joint_trajectory', self.cb, 10)
        self.pub = self.create_publisher(JointTrajectory, '/scaled_trajectory', 10)

    def cb(self, msg: JointTrajectory):
        mp = self.get_parameter('payload_mass').value
        adaptive = self.get_parameter('adaptive_enabled').value

        pts = msg.points
        times = [p.time_from_start.sec + p.time_from_start.nanosec*1e-9 for p in pts]
        max_ratio = 1.0

        N_SUB = 20  # samples per segment, matches offline_sim resolution

        for i in range(1, len(pts)):
            t0, t1 = times[i-1], times[i]
            T = t1 - t0
            if T <= 0:
                continue
            q0 = np.array(pts[i-1].positions)
            q1 = np.array(pts[i].positions)
            v0 = np.array(pts[i-1].velocities)
            v1 = np.array(pts[i].velocities)

            c2 = (3*(q1-q0) - T*(2*v0+v1)) / T**2
            c3 = (2*(q0-q1) + T*(v0+v1)) / T**3

            for s in range(N_SUB+1):
                tau_local = T * s / N_SUB
                q   = q0 + v0*tau_local + c2*tau_local**2 + c3*tau_local**3
                qd  = v0 + 2*c2*tau_local + 3*c3*tau_local**2
                qdd = 2*c2 + 6*c3*tau_local

                tau = joint_torques(q, qd, qdd, mp)
                ratio = np.max(np.abs(tau) / T_MAX)
                max_ratio = max(max_ratio, ratio)

            self.get_logger().info(f'segment{i}: peak_ratio_so_far={max_ratio:.2f}')

        if adaptive and max_ratio > 1.0:
            k = float(np.sqrt(max_ratio))
        else:
            k = 1.0

        self.get_logger().info(f'=== SCALE FACTOR k = {k:.3f} (adaptive={adaptive}, mass={mp}) ===')

        out = JointTrajectory()
        out.joint_names = msg.joint_names
        new_pts = []
        for i, p in enumerate(pts):
            np_ = JointTrajectoryPoint()
            np_.positions = p.positions
            np_.velocities = [v/k for v in p.velocities]
            new_t = times[i]*k
            np_.time_from_start = Duration(seconds=new_t).to_msg()
            new_pts.append(np_)
        out.points = new_pts
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(Interceptor())

if __name__ == '__main__':
    main()