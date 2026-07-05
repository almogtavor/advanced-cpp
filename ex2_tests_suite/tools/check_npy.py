#!/usr/bin/env python3
"""Validate a .npy voxel map using only the Python standard library.

On success prints:  ok <d0> <d1> <d2> <voxels> <invalid> <mapped>
  d0 d1 d2  the 3-D shape
  voxels    total voxel count (product of the shape)
  invalid   number of voxels whose byte is NOT a legal occupancy code
  mapped    number of voxels the drone actually observed (empty/occupied/
            potentially-occupied), i.e. not unmapped and not out-of-bounds

Legal occupancy codes are the fixed VoxelOccupancy values -3,-2,-1,0,1. As raw
bytes those are 253,254,255,0,1 (identical for signed int8 or unsigned uint8),
so the check works regardless of how the map was saved.

Exits 0 if the file is a well-formed 3-D array whose data can be read; exits 1
otherwise. No numpy dependency.
"""
import ast
import struct
import sys

LEGAL = {0, 1, 253, 254, 255}        # 0,1 and -1,-2,-3 as raw bytes
OBSERVED = {0, 1, 253}               # empty, occupied, potentially-occupied


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: check_npy.py <file.npy>", file=sys.stderr)
        return 2
    path = sys.argv[1]
    try:
        with open(path, "rb") as fh:
            if fh.read(6) != b"\x93NUMPY":
                print(f"{path}: not a .npy file (bad magic)", file=sys.stderr)
                return 1
            major, _minor = fh.read(1)[0], fh.read(1)[0]
            hlen = struct.unpack("<H" if major == 1 else "<I",
                                 fh.read(2 if major == 1 else 4))[0]
            meta = ast.literal_eval(fh.read(hlen).decode("latin1"))
            shape = meta["shape"]
            if len(shape) != 3:
                print(f"{path}: expected a 3-D array, got shape {shape}", file=sys.stderr)
                return 1
            voxels = shape[0] * shape[1] * shape[2]
            # Voxel codes are one byte each; read exactly `voxels` of them.
            data = fh.read(voxels)
            if len(data) < voxels:
                print(f"{path}: truncated data ({len(data)} < {voxels} bytes)", file=sys.stderr)
                return 1
    except Exception as exc:  # noqa: BLE001 - report any parse failure
        print(f"{path}: {exc}", file=sys.stderr)
        return 1

    invalid = sum(1 for b in data if b not in LEGAL)
    mapped = sum(1 for b in data if b in OBSERVED)
    print(f"ok {shape[0]} {shape[1]} {shape[2]} {voxels} {invalid} {mapped}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
