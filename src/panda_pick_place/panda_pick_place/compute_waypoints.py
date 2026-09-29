"""Compute joint-space pick/place waypoints for the Panda + Franka Hand.

Solves numerical IK (damped least squares) for TCP targets expressed in the
robot base frame (panda_link0) with the hand pointing straight down, and prints
a ``pick_place_node`` parameter block for config/pick_place.yaml.

    ros2 run panda_pick_place compute_waypoints --y 0.30 --x 0.45 \
        --pick-z 0.25 --hover-z 0.40
"""

import argparse

import numpy as np

# Modified DH (Craig) parameters of the Panda arm.
A = [0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088]
D = [0.333, 0.0, 0.316, 0.0, 0.384, 0.0, 0.0]
ALPHA = [0.0, -np.pi / 2, np.pi / 2, np.pi / 2, -np.pi / 2, np.pi / 2, np.pi / 2]
FLANGE_D = 0.107
TCP_D = 0.1034  # Franka Hand TCP, rotated -45 deg about z w.r.t. the flange

Q_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_MAX = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
HOME = np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4])


def _dh(a, d, alpha, theta):
    ca, sa, ct, st = np.cos(alpha), np.sin(alpha), np.cos(theta), np.sin(theta)
    return np.array([
        [ct, -st, 0.0, a],
        [st * ca, ct * ca, -sa, -d * sa],
        [st * sa, ct * sa, ca, d * ca],
        [0.0, 0.0, 0.0, 1.0],
    ])


def _tz(d):
    t = np.eye(4)
    t[2, 3] = d
    return t


def _rz(angle):
    c, s = np.cos(angle), np.sin(angle)
    t = np.eye(4)
    t[:2, :2] = [[c, -s], [s, c]]
    return t


def forward(q):
    t = np.eye(4)
    for i in range(7):
        t = t @ _dh(A[i], D[i], ALPHA[i], q[i])
    return t @ _tz(FLANGE_D) @ _rz(-np.pi / 4) @ _tz(TCP_D)


def _pose_error(t_cur, t_des):
    pos = t_des[:3, 3] - t_cur[:3, 3]
    r_err = t_des[:3, :3] @ t_cur[:3, :3].T
    rot = 0.5 * np.array([r_err[2, 1] - r_err[1, 2],
                          r_err[0, 2] - r_err[2, 0],
                          r_err[1, 0] - r_err[0, 1]])
    return np.concatenate([pos, rot])


def solve_ik(target, seed, iterations=500, damping=0.05):
    q = np.array(seed, dtype=float)
    for _ in range(iterations):
        err = _pose_error(forward(q), target)
        if np.linalg.norm(err) < 1e-6:
            break
        jac = np.zeros((6, 7))
        eps = 1e-6
        for i in range(7):
            dq = q.copy()
            dq[i] += eps
            jac[:, i] = (err - _pose_error(forward(dq), target)) / eps
        step = jac.T @ np.linalg.solve(jac @ jac.T + damping**2 * np.eye(6), err)
        q = np.clip(q + step, Q_MIN, Q_MAX)
    err = _pose_error(forward(q), target)
    if np.linalg.norm(err) > 1e-4:
        raise RuntimeError(f'IK did not converge (residual {np.linalg.norm(err):.4f})')
    return q


def target_pose(x, y, z):
    """Hand pointing down, yawed along the radial direction of the arm."""
    t = _rz(np.arctan2(y, x))
    t[:3, :3] = t[:3, :3] @ np.diag([1.0, -1.0, -1.0])
    t[:3, 3] = [x, y, z]
    return t


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, default=0.45, help='TCP x in base frame [m]')
    parser.add_argument('--y', type=float, default=0.30, help='left/right offset [m]')
    parser.add_argument('--pick-z', type=float, default=0.25, help='TCP z at grasp [m]')
    parser.add_argument('--hover-z', type=float, default=0.40, help='TCP z while moving [m]')
    args = parser.parse_args()

    # Swing left/right with joint 1 (base); the wrist stays put. Seeding IK with
    # q1 = atan2(y, x) keeps it on that branch instead of twisting joints 3 and 7.
    points = {}
    for side, prefix in ((args.y, 'pick'), (-args.y, 'place')):
        q = HOME.copy()
        q[0] = np.arctan2(side, args.x)
        for name, z in ((prefix + '_above', args.hover_z), (prefix, args.pick_z)):
            q = solve_ik(target_pose(args.x, side, z), q)
            points[name] = q

    print('    home: [' + ', '.join(f'{v:.4f}' for v in HOME) + ']')
    for name, q in points.items():
        print(f'    {name}: [' + ', '.join(f'{v:.4f}' for v in q) + ']')


if __name__ == '__main__':
    main()
