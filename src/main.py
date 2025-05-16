import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from robot import RRRRobot
from sim   import Target

def animate(robot, history, target_history, dt, obstacle=None):
    """
    Draw the robot and (optionally) a red obstacle, pausing dt between frames.
    """
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_aspect('equal')
    ax.set_xlim(-3, 3)
    ax.set_ylim(-1, 4)

    if obstacle is not None:
        c, r = obstacle['center'], obstacle['r']
        ax.add_patch(Circle(c, r, color='red', alpha=0.3))

    line,    = ax.plot([], [], '-o', lw=2)
    targ_pt, = ax.plot([], [], 'go', ms=8)

    for q, xt in zip(history, target_history):
        # forward kinematics
        p0 = np.array([0.0, 0.0])
        p1 = p0 + np.array([robot.L1*np.cos(q[0]), robot.L1*np.sin(q[0])])
        p2 = p1 + np.array([robot.L2*np.cos(q[0]+q[1]), robot.L2*np.sin(q[0]+q[1])])
        pe = robot.forward_kinematics(q)

        line.set_data([0, p1[0], p2[0], pe[0]],
                      [0, p1[1], p2[1], pe[1]])
        targ_pt.set_data([xt[0]], [xt[1]])
        plt.pause(dt)

    plt.show()
    plt.close(fig)

def run(control_rate, measure_rate, duration, Kp, obstacle):
    # 1) Setup
    robot = RRRRobot(L=1.0)
    targ  = Target(L=1.0, f=0.5)
    q     = np.array([0.2, -0.4, 0.2])

    history, target_history = [], []
    dt = 1.0 / control_rate
    t, t_last = 0.0, 0.0
    x_des = np.zeros(2)
    xdot  = np.zeros(2)

    # 2) Control loop
    while t < duration:
        if t - t_last >= 1.0/measure_rate:
            x_des, xdot = targ.step(1.0/measure_rate)
            t_last = t

        x   = robot.forward_kinematics(q)
        e   = x_des - x
        J   = robot.jacobian(q)
        Jp  = np.linalg.pinv(J)
        qdot= Jp @ (xdot + Kp * e)

        # obstacle override
        if obstacle:
            pe = robot.forward_kinematics(q)
            c, r = obstacle['center'], obstacle['r']
            d = np.linalg.norm(pe - c)
            if d < r:
                dir = (pe - c)/d
                xblk = c + dir*r
                eblk = xblk - pe
                qdot = Jp @ (Kp * eblk)

        q = np.clip(q + qdot*dt, -np.pi, np.pi)
        history.append(q.copy())
        target_history.append(x_des.copy())
        t += dt

    # 3) Plot error
    hist = np.array(history)
    targ_hist = np.array(target_history)
    ee = np.vstack([robot.forward_kinematics(qi) for qi in hist])
    times = np.linspace(0, duration, len(hist))
    err   = np.linalg.norm(targ_hist - ee, axis=1)
    # compute and print RMS error
    rms = np.sqrt(np.mean(err**2))
    print(f"RMS error ({control_rate}Hz/{measure_rate}Hz): {rms:.3f} m")


    fig1, ax1 = plt.subplots()
    ax1.plot(times, err, lw=2)
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("||tracking error|| [m]")
    ax1.set_title("End-Effector Tracking Error")
    ax1.grid(True)
    plt.savefig(f"error_{control_rate}x{measure_rate}.png", dpi=200)
    plt.show()
    plt.close(fig1)

    # 4) Animate
    animate(robot, history, target_history, dt, obstacle)

if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("--mode", choices=["fast","slow","obstacle"], default="fast")
    p.add_argument("--duration", type=float, default=5.0,
                   help="Simulation duration in seconds")
    args = p.parse_args()

    if args.mode == "fast":
        print(f"=== Fast loop (200Hz,30Hz) for {args.duration}s ===")
        run(control_rate=200, measure_rate=30,
            duration=args.duration, Kp=3.0, obstacle=None)

    elif args.mode == "slow":
        print(f"=== Slow loop (50Hz,5Hz) for {args.duration}s ===")
        run(control_rate=50, measure_rate=5,
            duration=args.duration, Kp=3.0, obstacle=None)

    else:  # obstacle
        print(f"=== Obstacle avoidance (200Hz,30Hz) for {args.duration}s ===")
        obs = {"center": np.array([1.0,0.5]), "r": 0.125}
        run(control_rate=200, measure_rate=30,
            duration=args.duration, Kp=3.0, obstacle=obs)