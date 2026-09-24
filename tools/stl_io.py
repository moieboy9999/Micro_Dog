# -*- coding: utf-8 -*-
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 moieboy9999 - Micro Dog
"""Minimal binary-STL reader/writer + rigid transform, no external deps."""
import struct
import numpy as np


def read_stl(path):
    with open(path, 'rb') as f:
        head = f.read(84)
        if head[:5] == b'solid' and b'facet' in head:
            raise ValueError('ascii stl not supported: %s' % path)
        n = struct.unpack('<I', head[80:84])[0]
        buf = f.read(n * 50)
    if len(buf) != n * 50:
        raise ValueError('truncated stl %s (%d tris, %d bytes)' % (path, n, len(buf)))
    a = np.frombuffer(buf, dtype=np.uint8).reshape(n, 50)
    floats = a[:, :48].copy().view('<f4').reshape(n, 4, 3)
    normals = floats[:, 0, :].astype(np.float64)
    tris = floats[:, 1:, :].astype(np.float64)
    return tris, normals


def write_stl(path, tris, normals=None):
    n = tris.shape[0]
    if normals is None:
        v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
        normals = np.cross(v1 - v0, v2 - v0)
        ln = np.linalg.norm(normals, axis=1)
        ln[ln == 0] = 1.0
        normals = normals / ln[:, None]
    out = np.zeros((n, 50), dtype=np.uint8)
    block = np.concatenate([normals[:, None, :], tris], axis=1).astype('<f4')
    out[:, :48] = block.reshape(n, 12).view(np.uint8).reshape(n, 48)
    with open(path, 'wb') as f:
        f.write(b'\0' * 80)
        f.write(struct.pack('<I', n))
        f.write(out.tobytes())


def transform(tris, R, t):
    """R: 3x3, t: 3 -- applied as R @ p + t"""
    flat = tris.reshape(-1, 3)
    out = flat @ np.asarray(R).T + np.asarray(t)
    return out.reshape(tris.shape)


def bbox(tris):
    flat = tris.reshape(-1, 3)
    return flat.min(axis=0), flat.max(axis=0)
