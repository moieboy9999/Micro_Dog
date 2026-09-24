# -*- coding: utf-8 -*-
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 moieboy9999 - Micro Dog
"""
Bake the raw Inventor STL exports into per-link meshes.

The raw files (pass --raw <dir>) are one binary STL per Inventor
document, expressed in that document's own frame and in millimetres.  This
script applies the transform that takes each document into its URDF link
frame at the zero pose, converts to metres, merges the parts that share a
link (thigh + knee crank + pushrod), and writes ../meshes/visual/<link>.stl.

Run:  python bake_meshes.py [--raw <dir>]
"""
import argparse
import io
import os

import numpy as np
import yaml

from stl_io import read_stl, write_stl

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DEFAULT_RAW = None  # raw Inventor STL exports are not shipped; export them from cad/inventor
OUT = os.path.join(ROOT, 'meshes', 'visual')

MM = 0.001


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--raw', default=DEFAULT_RAW, required=True, help='directory with the raw Inventor STL exports')
    args = ap.parse_args()

    with io.open(os.path.join(ROOT, 'params', 'robot_params.yaml'), encoding='utf-8') as f:
        params = yaml.safe_load(f)

    if not os.path.isdir(OUT):
        os.makedirs(OUT)

    total = 0
    for link, entries in params['mesh_bake'].items():
        chunks = []
        for e in entries:
            path = os.path.join(args.raw, e['stl'])
            if not os.path.isfile(path):
                raise SystemExit('missing raw mesh: %s' % path)
            tris, _ = read_stl(path)
            R = np.array(e['R'], dtype=float)
            t = np.array(e['t'], dtype=float)
            pts = tris.reshape(-1, 3) * MM
            chunks.append((pts @ R.T + t).reshape(tris.shape))
        merged = np.concatenate(chunks, axis=0)
        dst = os.path.join(OUT, link + '.stl')
        write_stl(dst, merged)
        lo = merged.reshape(-1, 3).min(axis=0)
        hi = merged.reshape(-1, 3).max(axis=0)
        total += merged.shape[0]
        print('%-14s %7d tris   bbox %s .. %s' % (
            link, merged.shape[0],
            np.array2string(lo, precision=4, suppress_small=True),
            np.array2string(hi, precision=4, suppress_small=True)))
    print('\ntotal %d triangles -> %s' % (total, OUT))


if __name__ == '__main__':
    main()
