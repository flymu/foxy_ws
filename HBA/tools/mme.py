#!/usr/bin/env python3
"""MME (Modified Minimum Entropy) map-quality metric — Python replica of
HBA/source/calculate_MME.cpp, adapted to what this machine can run (no PCL
kdtree/flann here, so neighborhoods come from scipy's cKDTree).

For each sampled query point q of a map:
  - neighbors within radius r (query point included, matching PCL
    radiusSearch semantics); queries with <= min_n neighbors are skipped
  - C = sum((p-q)(p-q)^T) / N ; e = 0.5*log(det(2*pi*e*C))
  - MME = mean(e) over valid queries (lower = crisper map)
Also reports the mean distance to the 10th nearest neighbor (compactness).

Query sampling: each map is sampled from its OWN points with a fixed seed
(MME/10th-NN are local, frame-invariant metrics, so per-map sampling is
fair and robust to the re-anchoring HBA's write_pose applies).

usage: mme.py map_a.pcd [map_b.pcd ...] [--queries 30000] [--radius 0.3]
       [--min_n 15] [--k 10]
"""

import argparse
import math

import numpy as np
from scipy.spatial import cKDTree


def read_binary_pcd(path):
    """Read a binary PCD into an (N,3) float32 array of xyz.

    Only PCL point types with the leading x,y,z float32 fields are
    supported (PointXYZ, PointXYZRGB, PointXYZI, ...).
    """
    with open(path, "rb") as f:
        header = {}
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            if not line:
                continue
            key, _, rest = line.partition(" ")
            header[key] = rest
            if key == "DATA":
                break
        if header.get("DATA") != "binary":
            raise SystemExit(f"{path}: only binary PCD supported, got DATA={header.get('DATA')}")
        count = int(header["POINTS"])
        ftype = header["TYPE"].split()
        fsize = [int(s) for s in header["SIZE"].split()]
        fields = header["FIELDS"].split()
        if fields[:3] != ["x", "y", "z"]:
            raise SystemExit(f"{path}: expected x y z first fields, got {fields}")
        if ftype[:3] != ["F", "F", "F"] or fsize[:3] != [4, 4, 4]:
            raise SystemExit(f"{path}: unexpected xyz types {ftype[:3]} sizes {fsize[:3]}")
        point_step = sum(fsize)
        raw = f.read(count * point_step)
    pts = np.frombuffer(raw, dtype=np.uint8).reshape(count, point_step)
    return np.ascontiguousarray(pts[:, :12].copy()).view(np.float32).reshape(count, 3)


def mme(pts, queries, radius=0.3, min_n=15, k=10):
    """Entropy + k-NN compactness on neighborhoods of `queries` within `pts`."""
    tree = cKDTree(pts)
    nns = tree.query_ball_point(queries, radius)
    entropies, compacts = [], []
    for q, nbrs in zip(queries, nns):
        if len(nbrs) <= min_n:
            continue
        arr = pts[nbrs] - q
        n = len(nbrs)
        C = arr.T @ arr / n
        det = np.linalg.det(2 * math.pi * math.e * C)
        if det <= 0:
            continue
        entropies.append(0.5 * math.log(det))
        if n > k:
            d = np.sort(np.einsum("ij,ij->i", arr, arr))
            compacts.append(math.sqrt(d[k]))
    return np.array(entropies), np.array(compacts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("maps", nargs="+")
    ap.add_argument("--queries", type=int, default=30000)
    ap.add_argument("--radius", type=float, default=0.3)
    ap.add_argument("--min_n", type=int, default=15)
    ap.add_argument("--k", type=int, default=10)
    args = ap.parse_args()

    print(f"queries/map: {args.queries}, radius {args.radius}, min neighbors {args.min_n}\n")
    print(f"{'map':24s} {'MME':>8s} {'10thNN[m]':>10s} {'valid':>7s} {'voxels@0.05':>12s}")
    results = []
    for p in args.maps:
        pts = read_binary_pcd(p)
        rng = np.random.default_rng(12345)
        n = min(args.queries, len(pts))
        queries = pts[rng.choice(len(pts), n, replace=False)]
        e, c = mme(pts, queries, radius=args.radius, min_n=args.min_n, k=args.k)
        mme_val = e.mean() if len(e) else float("nan")
        comp = c.mean() if len(c) else float("nan")
        results.append((p, mme_val, comp, len(e), len(pts)))
        print(f"{p:24s} {mme_val:8.4f} {comp:10.4f} {len(e):7d} {len(pts):12,}")
    if len(results) == 2:
        m1 = results[0][1]
        m2 = results[1][1]
        print(f"\nMME delta = {m2 - m1:+.4f} ({'after lower = crisper' if m2 < m1 else 'before lower = crisper'})")


if __name__ == "__main__":
    main()
