#!/usr/bin/env python3
"""Score a SLAM-produced map against the reference tb3_sandbox map. Gates G2/G5.

The reference is nav2_bringup/maps/tb3_sandbox.{yaml,pgm}: 384x384 at 0.05 m,
origin (-10, -10). It is the same geometry nav2_minimal_tb3_sim renders, which
is why this comparison is possible at all.

Two things make a naive pixel diff wrong:

  1. Different origins. map_saver writes an origin derived from the SLAM graph,
     whose frame was fixed by wherever the robot happened to start. The two grids
     must be compared in world coordinates, not array indices.

  2. Different extents. slam_toolbox only maps what the lidar saw. Cells the
     robot never observed are unknown (-1) and must be excluded, or an
     unexplored corner counts as a mapping failure.

Three numbers, three gates. One number cannot do it: fault injection against a
real map (2026-08-17, DESIGN.md) showed each failure mode is caught by exactly
one of them, so a single combined score lets the other two hide.

  precision  of the cells SLAM called occupied, how many really are.
             Catches free space wrongly marked as wall -- 5% of free space
             flipped drops precision to 0.648 while recall stays 0.990.
  recall     of the walls that are there, how many SLAM found.
             Catches walls left unmapped in a contiguous region, and drift.
  coverage   of the reference's free space, how much SLAM observed at all.
             Catches a run that died early -- a flawless map of a quarter of the
             arena scores precision 1.000, recall 1.000, coverage 0.150.

Localisation drift shows up in precision and recall together: shifting the whole
map 0.10 m gives 0.559 / 0.690.

Known blind spot: recall does not catch wall cells erased at random, because the
one-cell tolerance bridges one-cell gaps by design -- erasing 50% of wall cells
at random still scores recall 0.914. That is accepted, because no real failure
erodes a map cell by cell. Dropped scans, a frozen map or a dead mapper all leave
contiguous holes, which the tolerance does not bridge, and show up in coverage as
well.

Matching is symmetric with a one-cell tolerance: a SLAM cell counts as correct
if a reference wall lies within one cell, and a reference wall counts as found if
a SLAM cell lies within one cell. Symmetry is the whole point --

  the previous version of this file dilated the REFERENCE only, and gated on
  occupied IoU >= 0.70. Measured 2026-08-17: a flawless map, restricted to the
  region a real run observed, scores IoU 0.533 under that metric, because a
  1-cell reference wall becomes a 5-cell plus shape that a 1-cell SLAM wall can
  never fill. The gate was unreachable by construction. Worse, the metric was
  not monotone in map quality: the real run scored 0.543, ABOVE the flawless
  map's 0.533, because its blurrier walls happened to overlap more of the
  inflated reference. Under the metric below a flawless map scores exactly
  1.000. The legacy IoU is still printed for continuity with RUNLOG.md entries
  that quote it, and is not a gate.

Honesty note that belongs in any report using these numbers: when the Gazebo
world and the reference map are both derived from tb3_sandbox, a high score
shows the pipeline is self-consistent. It is not independent verification of the
map.

Usage:
    python3 score_map.py --slam ~/maps/split_run1.yaml \\
        --ref /opt/ros/jazzy/share/nav2_bringup/maps/tb3_sandbox.yaml \\
        --offset=-2,-0.5 --min-precision 0.85 --min-recall 0.85 \\
        --min-coverage 0.90
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

# ROS map_server conventions.
OCC_THRESH_DEFAULT = 0.65
FREE_THRESH_DEFAULT = 0.196


def read_pgm(path: Path) -> np.ndarray:
    """Binary (P5) or ASCII (P2) PGM. map_saver writes P5; hand-edited files P2."""
    with open(path, 'rb') as fh:
        data = fh.read()

    # Strip comments, then pull the header tokens: magic, width, height, maxval.
    # The bound is 3 because magic is captured separately; reading a fourth
    # token would consume the first pixel byte.
    tokens = []
    i = 0
    magic = None
    while len(tokens) < 3:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < len(data) and data[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tok = data[i:j]
        i = j
        if magic is None:
            magic = tok.decode()
        else:
            tokens.append(int(tok))
    width, height, maxval = tokens[0], tokens[1], tokens[2]
    i += 1  # single whitespace byte after maxval

    if magic == 'P5':
        dtype = np.uint8 if maxval < 256 else '>u2'
        arr = np.frombuffer(data, dtype=dtype, count=width * height, offset=i)
    elif magic == 'P2':
        vals = np.array(data[i:].split(), dtype=int)[:width * height]
        arr = vals
    else:
        raise ValueError(f'{path}: unsupported PGM magic {magic!r}')
    return np.asarray(arr, dtype=float).reshape(height, width) / float(maxval)


def load_map(yaml_path: Path):
    meta = yaml.safe_load(yaml_path.read_text())
    image = yaml_path.parent / meta['image']
    p = read_pgm(image)

    # map_server: value = 1 - pixel/maxval for mode 'trinary'/'scale'.
    occupancy = 1.0 - p
    if meta.get('negate', 0):
        occupancy = p

    occ_t = float(meta.get('occupied_thresh', OCC_THRESH_DEFAULT))
    free_t = float(meta.get('free_thresh', FREE_THRESH_DEFAULT))

    state = np.full(occupancy.shape, -1, dtype=np.int8)   # unknown
    state[occupancy >= occ_t] = 1                         # occupied
    state[occupancy <= free_t] = 0                        # free

    # PGM row 0 is the top row, which is the HIGHEST y in map frame.
    state = np.flipud(state)

    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    return {'state': state, 'res': res, 'origin': (ox, oy), 'meta': meta,
            'path': yaml_path}


def describe(name, m):
    h, w = m['state'].shape
    s = m['state']
    print(f'{name}: {w}x{h} res={m["res"]:.3f} origin=({m["origin"][0]:.2f}, '
          f'{m["origin"][1]:.2f})  occ={int((s == 1).sum())} '
          f'free={int((s == 0).sum())} unknown={int((s == -1).sum())}')


# A world coordinate that lands exactly on a cell boundary is one ULP away from
# either neighbouring cell, and which side floor() picks then depends on how the
# coordinate was accumulated. That is not a rounding nicety: measured 2026-08-19,
# two SLAM maps of the same arena, three millimetres apart in origin, projected
# onto the reference as 8398 distinct cells and 5199 -- because one had its rows
# landing on exact boundaries and collapsed pairs of them together. Coverage read
# 0.983 and 0.598 for maps that are identical to the eye and within 0.7% on every
# cell count. The gate flipped PASS/FAIL on a defect in the scorer.
#
# Nudging by a millionth of a cell before flooring makes an exact boundary
# resolve upward every time. It cannot move a coordinate that is not already on
# one, and 1e-6 cells is 50 nm at this resolution.
CELL_EPS = 1e-6


def sample(m, xs, ys):
    """Nearest-cell lookup at world coordinates; out-of-bounds -> unknown."""
    res = m['res']
    ox, oy = m['origin']
    h, w = m['state'].shape
    cx = np.floor((xs - ox) / res + CELL_EPS).astype(int)
    cy = np.floor((ys - oy) / res + CELL_EPS).astype(int)
    inside = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
    out = np.full(xs.shape, -1, dtype=np.int8)
    out[inside] = m['state'][cy[inside], cx[inside]]
    return out


def dilate(mask, cells, diagonal=True):
    """Grow a boolean mask by `cells` in every direction, diagonals included.

    diagonal=False gives the old 4-connected (Manhattan) behaviour, and exists
    only so the legacy IoU line stays comparable with the RUNLOG.md entries that
    quote it. Nothing else should use it.

    Chebyshev, not Manhattan. "Within one cell" has to include the diagonal
    neighbour, which is also one cell away, and here it matters: tb3_sandbox is a
    hexagon whose walls run diagonally, so a half-cell registration error moves
    wall cells diagonally and a 4-connected tolerance scores them as misses.
    Measured 2026-08-17 over eight saved maps, 4- to 8-connected raised the worst
    run's precision 0.863 -> 0.910 and recall 0.905 -> 0.958, and cut the
    run-to-run spread of recall from 0.085 to 0.032.

    Separable: dilate along rows, then along columns of that result.
    """
    if cells <= 0:
        return mask.copy()
    rows = mask.copy()
    for k in range(1, cells + 1):
        rows[:-k, :] |= mask[k:, :]
        rows[k:, :] |= mask[:-k, :]
    # Column pass over the row-dilated result reaches the diagonals; over the
    # original mask it does not.
    src = rows if diagonal else mask
    grown = rows.copy()
    for k in range(1, cells + 1):
        grown[:, :-k] |= src[:, k:]
        grown[:, k:] |= src[:, :-k]
    return grown


def coverage(slam, ref, dx, dy):
    """Fraction of the reference's free cells that the SLAM map knows at all.

    Computed on the REFERENCE grid, not the SLAM grid: the question is how much
    of the arena was explored, and a SLAM grid only spans what it already saw,
    so measuring on it would always answer ~100%.
    """
    ref_free = (ref['state'] == 0)
    if not ref_free.any():
        return 0.0, 0, 0
    known = np.zeros(ref['state'].shape, dtype=bool)
    iy, ix = np.nonzero(slam['state'] != -1)
    h, w = ref['state'].shape

    if abs(slam['res'] - ref['res']) < 1e-9:
        # Equal resolutions, which is the case every run here: the projection is
        # a pure integer translation, so compute the shift once and add it. This
        # is 1:1 by construction -- no two SLAM cells can land on one reference
        # cell, and no reference cell inside the covered region can be skipped.
        # The float path below cannot promise that; see CELL_EPS.
        sx = int(round((slam['origin'][0] + dx - ref['origin'][0]) / ref['res']))
        sy = int(round((slam['origin'][1] + dy - ref['origin'][1]) / ref['res']))
        cx, cy = ix + sx, iy + sy
    else:
        xs = slam['origin'][0] + (ix + 0.5) * slam['res'] + dx
        ys = slam['origin'][1] + (iy + 0.5) * slam['res'] + dy
        cx = np.floor((xs - ref['origin'][0]) / ref['res'] + CELL_EPS).astype(int)
        cy = np.floor((ys - ref['origin'][1]) / ref['res'] + CELL_EPS).astype(int)

    ok = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
    known[cy[ok], cx[ok]] = True
    seen = int((ref_free & known).sum())
    total = int(ref_free.sum())
    # A projection that loses cells is a scorer defect, not a mapping result, and
    # it reads as low coverage. Say so rather than reporting the number.
    lost = int(ok.sum()) - int(known.sum())
    if lost:
        print(f'  WARNING: {lost} of {int(ok.sum())} projected cells collapsed onto '
              f'a reference cell another already claimed -- coverage is understated')
    return seen / total, seen, total


def verdict(value, floor, disabled_note=''):
    if floor <= 0.0:
        return f'{value:.3f}   gate off{disabled_note}', True
    ok = value >= floor
    return f'{value:.3f}   >= {floor:.2f}  {"ok" if ok else "LOW"}', ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--slam', required=True, help='map_saver .yaml from the run')
    ap.add_argument('--ref', required=True, help='reference tb3_sandbox.yaml')
    ap.add_argument('--offset', default='0,0',
                    help='x,y added to SLAM-frame coords to reach world coords. '
                         'The SLAM map origin is the robot start pose, so with '
                         'the default spawn at (-2.0, -0.5) this is "-2.0,-0.5". '
                         'Pass it as --offset=-2.0,-0.5: argparse reads a bare '
                         'leading-dash token as an option name.')
    ap.add_argument('--tol', type=int, default=1,
                    help='match tolerance in cells, applied to BOTH maps. '
                         'tb3_sandbox walls are 1-2 px, so a half-cell pose '
                         'error would otherwise read as a total miss')
    # Floors sit in the measured gap between healthy runs and injected faults,
    # roughly centred so neither side is close. Healthy: precision 0.910-0.998,
    # recall 0.958-0.990 over eight runs. Faulted: precision 0.559-0.648, recall
    # 0.690 at 0.10 m of drift.
    ap.add_argument('--min-precision', type=float, default=0.80)
    ap.add_argument('--min-recall', type=float, default=0.85)
    ap.add_argument('--min-coverage', type=float, default=0.90,
                    help='0 disables the coverage gate. Set it per robot: the '
                         'depot reference is 448.7 m2 of free space against '
                         'tb3_sandbox 19.8 m2, so a short tour there cannot '
                         'reach any high fraction and the number is not yet '
                         'measured for tb4')
    a = ap.parse_args()

    slam = load_map(Path(a.slam))
    ref = load_map(Path(a.ref))
    describe('slam', slam)
    describe('ref ', ref)

    dx, dy = (float(v) for v in a.offset.split(','))
    print(f'offset: SLAM + ({dx:+.2f}, {dy:+.2f}) -> world')

    # Evaluate accuracy on the SLAM map's own grid: it is the smaller, observed
    # region. Coverage is evaluated on the reference grid instead -- see above.
    h, w = slam['state'].shape
    res = slam['res']
    ox, oy = slam['origin']
    ix, iy = np.meshgrid(np.arange(w), np.arange(h))
    xs = ox + (ix + 0.5) * res + dx
    ys = oy + (iy + 0.5) * res + dy

    ref_state = sample(ref, xs, ys)
    slam_state = slam['state']

    # Only where both maps claim knowledge.
    known = (slam_state != -1) & (ref_state != -1)
    slam_occ = (slam_state == 1) & known
    ref_occ = (ref_state == 1) & known

    n_slam = int(slam_occ.sum())
    n_ref = int(ref_occ.sum())
    if n_slam == 0 or n_ref == 0:
        print(f'\ncompared cells   {int(known.sum())}')
        print(f'  slam occupied  {n_slam}')
        print(f'  ref  occupied  {n_ref}')
        print('RESULT: FAIL (nothing to match -- check --offset and the map)')
        return 1

    tol = max(0, a.tol)
    prec = int((slam_occ & dilate(ref_occ, tol)).sum()) / n_slam
    rec = int((ref_occ & dilate(slam_occ, tol)).sum()) / n_ref
    f1 = 2 * prec * rec / (prec + rec) if (prec + rec) else 0.0
    cov, cov_seen, cov_total = coverage(slam, ref, dx, dy)

    # Legacy: reference-only 4-connected dilation, occupied IoU. Reported, never
    # gated. Both halves of that -- one-sided and 4-connected -- have to stay as
    # they were, or the number no longer matches the RUNLOG.md entries it exists
    # to be compared against.
    legacy_ref = dilate((ref_state == 1), 1, diagonal=False) & known
    inter = int((slam_occ & legacy_ref).sum())
    union = int((slam_occ | legacy_ref).sum())
    legacy_iou = inter / union if union else 0.0

    p_txt, p_ok = verdict(prec, a.min_precision)
    r_txt, r_ok = verdict(rec, a.min_recall)
    c_txt, c_ok = verdict(cov, a.min_coverage, ' (not measured for this world)')

    print(f'\naccuracy, on the {int(known.sum())} cells both maps know, '
          f'{tol}-cell ({tol * res:.2f} m) tolerance')
    print(f'  slam occupied  {n_slam}')
    print(f'  ref  occupied  {n_ref}')
    print(f'  precision      {p_txt}')
    print(f'  recall         {r_txt}')
    print(f'  F1             {f1:.3f}   reported, not gated')
    print('coverage, against the whole reference')
    print(f'  ref free       {cov_total}, known by SLAM {cov_seen}')
    print(f'  coverage       {c_txt}')
    # No ceiling is quoted here on purpose. The 0.533 this used to print was
    # measured against tb3_sandbox and was printed for every reference, so on depot
    # it named a number belonging to another map -- and an argument was once built on
    # it. The docstring above carries the tb3 measurement with its provenance; the
    # ceiling for a given reference has to be measured against that reference or not
    # cited at all.
    print(f'legacy occupied IoU  {legacy_iou:.3f}   not a gate, and not comparable '
          f'across references')

    ok = p_ok and r_ok and c_ok
    if ok:
        print(f'RESULT: PASS (precision {prec:.3f}, recall {rec:.3f}, '
              f'coverage {cov:.3f})')
        return 0
    low = [n for n, good in (('precision', p_ok), ('recall', r_ok),
                             ('coverage', c_ok)) if not good]
    print(f'RESULT: FAIL (below floor: {", ".join(low)})')
    return 1


if __name__ == '__main__':
    sys.exit(main())
