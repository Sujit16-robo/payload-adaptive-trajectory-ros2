import numpy as np
import matplotlib.pyplot as plt

G = 9.81
M1 = 74.13
M2 = 38.64
L2, L3 = 0.65, 0.508
LC2, LC3 = L2/2, L3/2
IZZ1 = 0.9823
I2 = 1.468
I3 = 1.004

T_MAX = np.array([240.0, 960.0, 400.0])

def M3_eff(mp):
    return 44.07  # link3+gripper fixed structural mass (payload added separately in torque eq)

def joint_torques(q, qd, qdd, mp):
    q1, q2, q3 = q
    qd1, qd2, qd3 = qd
    qdd1, qdd2, qdd3 = qdd
    M3 = M3_eff(mp)

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

# ---- Trajectory definition (matches traj_publisher.py waypoints) ----
WP_POS = [
    np.array([0.0, 0.0, 0.0]),
    np.array([0.5, 0.5, -0.4]),
    np.array([0.9, 0.8, -0.7]),
]
WP_VEL = [
    np.array([0.0, 0.0, 0.0]),
    np.array([1.4, 1.4, -1.2]),
    np.array([0.0, 0.0, 0.0]),
]
WP_TIME = [0.0, 0.3, 0.6]

def cubic_coeffs(q0, v0, q1, v1, T):
    a0 = q0
    a1 = v0
    a2 = (3*(q1-q0) - T*(2*v0+v1)) / T**2
    a3 = (2*(q0-q1) + T*(v0+v1)) / T**3
    return a0, a1, a2, a3

def sample_trajectory(scale_k, dt=0.002):
    """scale_k: time dilation factor. positions unchanged, velocities/accel scaled."""
    times, Q, QD, QDD = [], [], [], []
    t_offset = 0.0
    for i in range(len(WP_TIME)-1):
        T = (WP_TIME[i+1]-WP_TIME[i]) * scale_k
        q0, q1 = WP_POS[i], WP_POS[i+1]
        v0, v1 = WP_VEL[i]/scale_k, WP_VEL[i+1]/scale_k
        n = max(2, int(T/dt))
        tt = np.linspace(0, T, n)
        seg_q = np.zeros((n,3)); seg_qd = np.zeros((n,3)); seg_qdd = np.zeros((n,3))
        for j in range(3):
            a0,a1,a2,a3 = cubic_coeffs(q0[j], v0[j], q1[j], v1[j], T)
            seg_q[:,j]  = a0 + a1*tt + a2*tt**2 + a3*tt**3
            seg_qd[:,j] = a1 + 2*a2*tt + 3*a3*tt**2
            seg_qdd[:,j]= 2*a2 + 6*a3*tt
        times.append(tt + t_offset)
        Q.append(seg_q); QD.append(seg_qd); QDD.append(seg_qdd)
        t_offset += T
    return np.concatenate(times), np.concatenate(Q), np.concatenate(QD), np.concatenate(QDD)

def compute_scale_factor(mp):
    t, Q, QD, QDD = sample_trajectory(1.0)
    max_ratio = 1.0
    for i in range(len(t)):
        tau = joint_torques(Q[i], QD[i], QDD[i], mp)
        ratio = np.max(np.abs(tau)/T_MAX)
        max_ratio = max(max_ratio, ratio)
    k = np.sqrt(max_ratio) if max_ratio > 1.0 else 1.0
    return k, max_ratio

def run_case(mp, k):
    t, Q, QD, QDD = sample_trajectory(k)
    TAU = np.array([joint_torques(Q[i], QD[i], QDD[i], mp) for i in range(len(t))])
    return t, TAU, QD

payload_classes = {'empty': 0.001, 'light': 1.5, 'heavy': 5.0}
results = {}

for name, mp in payload_classes.items():
    k, ratio = compute_scale_factor(mp)
    t_base, tau_base, qd_base = run_case(mp, 1.0)
    t_adapt, tau_adapt, qd_adapt = run_case(mp, k)
    results[name] = dict(k=k, ratio=ratio, t_base=t_base, tau_base=tau_base,
                          t_adapt=t_adapt, tau_adapt=tau_adapt,
                          qd_base=qd_base, qd_adapt=qd_adapt)
    print(f"{name:6s} mp={mp:5.2f}kg  peak_ratio={ratio:.2f}  scale_k={k:.3f}  "
          f"cycle_time_base={t_base[-1]:.3f}s  cycle_time_adapt={t_adapt[-1]:.3f}s")

# ---- Plot torque profiles ----
for name, r in results.items():
    fig, axs = plt.subplots(3, 1, figsize=(7, 9), sharex=False)
    for j in range(3):
        axs[j].plot(r['t_base'], r['tau_base'][:,j], 'r-', label='Baseline (unscaled)')
        axs[j].plot(r['t_adapt'], r['tau_adapt'][:,j], 'g-', label='Adaptive (scaled)')
        axs[j].axhline(T_MAX[j], color='k', ls='--', lw=1, label='T_max')
        axs[j].axhline(-T_MAX[j], color='k', ls='--', lw=1)
        axs[j].set_ylabel(f'Joint {j+1} Torque (N·m)')
        axs[j].legend(fontsize=8)
        axs[j].grid(alpha=0.3)
    axs[-1].set_xlabel('Time (s)')
    plt.suptitle(f'Torque Profile — Payload: {name} ({payload_classes[name]} kg), k={r["k"]:.2f}')
    plt.tight_layout()
    plt.savefig(f'torque_{name}.png', dpi=150)
    print(f'Saved torque_{name}.png')

# ---- Peak torque / power reduction summary table ----
print("\n=== SUMMARY TABLE ===")
print(f"{'Payload':8s} {'PeakTau_base':>14s} {'PeakTau_adapt':>15s} {'%Reduction':>12s} {'CycleTime_base':>16s} {'CycleTime_adapt':>17s} {'%TimePenalty':>14s}")
for name, r in results.items():
    pb = np.max(np.abs(r['tau_base']))
    pa = np.max(np.abs(r['tau_adapt']))
    reduction = 100*(1 - pa/pb) if pb>0 else 0
    tb, ta = r['t_base'][-1], r['t_adapt'][-1]
    penalty = 100*(ta/tb - 1)
    print(f"{name:8s} {pb:14.2f} {pa:15.2f} {reduction:11.1f}% {tb:16.3f} {ta:17.3f} {penalty:13.1f}%")

plt.show()
