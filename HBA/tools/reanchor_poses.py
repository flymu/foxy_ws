#!/usr/bin/env python3
"""Re-anchor HBA's optimized pose.json back into the original world frame.

HBA's write_pose() re-anchors its output so pose 0 is the identity —
q_out = q0^-1 * q, t_out = q0^-1 * (t - t0) — i.e. the optimized map is
expressed in scan 0's own (possibly tilted) LiDAR frame, which throws away
the gravity-aligned world frame FAST_LIO established. This tool restores it:
given the ORIGINAL first pose (Q0, T0) from pose_init.json and HBA's output
poses, it composes

    Q_i_world = Q0 * q_i,  T_i_world = Q0 * t_i + T0

which maps the HBA frame back to the world frame exactly (the inverse of the
re-anchoring HBA applied). Rotations stay unit quaternions.

usage: reanchor_poses.py <pose_init.json> <pose_hba_out.json> <pose_world.json>
"""

import sys

import numpy as np


def qmul(a, b):
    """Hamilton product of quaternions a, b, both (..., 4) in wxyz order."""
    aw, ax, ay, az = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bw, bx, by, bz = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ], axis=-1)


def qrot(q, v):
    """Rotate vectors v (..., 3) by unit quaternions q (..., 4) wxyz."""
    qv = q[..., 1:]
    uv = np.cross(qv, v)
    uuv = np.cross(qv, uv)
    return v + 2 * (q[..., :1] * uv + uuv)


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        sys.exit(1)
    init_path, hba_path, out_path = sys.argv[1:4]

    init = np.loadtxt(init_path)
    hba = np.loadtxt(hba_path)
    if init.shape[0] != hba.shape[0]:
        sys.exit(f"pose count mismatch: {init_path} has {init.shape[0]}, "
                 f"{hba_path} has {hba.shape[0]}")

    Q0 = init[0, 3:7]
    T0 = init[0, 0:3]

    q = hba[:, 3:7]
    t = hba[:, 0:3]

    q_world = qmul(Q0, q)
    t_world = qrot(Q0, t) + T0

    norms = np.linalg.norm(q_world, axis=1)
    print(f"poses: {len(q_world)}, q norms min {norms.min():.9f} max {norms.max():.9f}")

    out = np.hstack([t_world, q_world])
    np.savetxt(out_path, out, fmt="%.9f")

    # Sanity: pose 0 must reproduce pose_init.json's first line (HBA's output
    # pose 0 is the identity, so T_world_0 = (Q0, T0) exactly).
    err_t = np.linalg.norm(out[0, 0:3] - init[0, 0:3])
    err_q = np.linalg.norm(out[0, 3:7] - init[0, 3:7])
    print(f"pose 0 recovery error: t {err_t:.3e}, q {err_q:.3e}")
    if err_t > 1e-6 or err_q > 1e-6:
        sys.exit("pose 0 does not match pose_init.json — wrong anchor?")


if __name__ == "__main__":
    main()
