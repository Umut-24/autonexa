#!/usr/bin/env python3
"""Pure-Python renderer for robot.urdf.xacro and the matching Nav2 footprint.

Used by both launch files (at startup) and ros2_mobile_bridge.py (live, when
the app POSTs to /api/robot_config). The template uses simple {placeholder}
substitution — we deliberately do NOT shell out to the `xacro` binary so the
bridge can re-render in the request hot path without process overhead.

Persistence path: ~/.autonexa/robot_dimensions.yaml (top-level mapping of
dimension keys to floats). Missing or unparseable file => defaults.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Dict, Tuple

try:
    import yaml  # type: ignore
except ImportError:  # pragma: no cover
    yaml = None


DEFAULT_DIMENSIONS: Dict[str, float] = {
    "chassis_length": 0.27,   # x extent (m) — measured 27 cm bumper-to-bumper
    "chassis_width":  0.20,   # y extent (m) — measured 20 cm side-to-side
    "chassis_height": 0.10,   # z extent (m)
    "wheelbase":      0.25,   # informational; not yet driving IK in this renderer
    "lidar_x":        0.015,  # LiDAR puck axis is ~1.5 cm forward of chassis center
    "lidar_y":        0.00,
    "lidar_z":        0.07,
    "camera_x":       0.10,
    "camera_z":       0.05,
    "footprint_padding": 0.03,  # 3 cm uniform berth vs walls + dynamic obstacles
}

# All dims are required keys in the output. wheelbase / footprint_padding are
# carried through so the bridge / costmap can read them without re-merging.
DIM_KEYS = tuple(DEFAULT_DIMENSIONS.keys())

OVERRIDE_PATH = Path.home() / ".autonexa" / "robot_dimensions.yaml"


def _package_share_urdf_dir() -> Path:
    """Find the urdf/ directory for the parking_system package.

    Works whether we're invoked from the source tree, the colcon install
    tree, or imported by an outside script (mobile_bridge).
    """
    here = Path(__file__).resolve()
    # source layout: src/parking_system/scripts/build_urdf.py -> ../urdf
    candidate = here.parent.parent / "urdf"
    if candidate.is_dir():
        return candidate
    # installed layout: ament_index resolves share/parking_system/urdf
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("parking_system")) / "urdf"
        if share.is_dir():
            return share
    except Exception:
        pass
    raise FileNotFoundError("Could not locate parking_system urdf directory")


def _read_template() -> str:
    return (_package_share_urdf_dir() / "robot.urdf.xacro").read_text()


def load_persisted_dimensions() -> Dict[str, float]:
    """Read ~/.autonexa/robot_dimensions.yaml. Missing/unparseable -> {}."""
    if not OVERRIDE_PATH.exists() or yaml is None:
        return {}
    try:
        with OVERRIDE_PATH.open("r") as f:
            raw = yaml.safe_load(f) or {}
        if not isinstance(raw, dict):
            return {}
        # Coerce to float, ignore unknown keys.
        out: Dict[str, float] = {}
        for k in DIM_KEYS:
            if k in raw:
                try:
                    out[k] = float(raw[k])
                except (TypeError, ValueError):
                    continue
        return out
    except Exception:
        return {}


def save_persisted_dimensions(dims: Dict[str, float]) -> None:
    """Atomic write of merged dimensions YAML."""
    if yaml is None:
        return
    OVERRIDE_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = OVERRIDE_PATH.with_suffix(".yaml.tmp")
    payload = {k: float(dims[k]) for k in DIM_KEYS if k in dims}
    with tmp.open("w") as f:
        yaml.safe_dump(payload, f, default_flow_style=False, sort_keys=True)
    os.replace(tmp, OVERRIDE_PATH)


def merge_dimensions(*sources: Dict[str, float]) -> Dict[str, float]:
    """Right-most source wins. Always returns a full dict over DIM_KEYS."""
    merged = dict(DEFAULT_DIMENSIONS)
    for s in sources:
        if not s:
            continue
        for k, v in s.items():
            if k in merged:
                try:
                    merged[k] = float(v)
                except (TypeError, ValueError):
                    continue
    return merged


def footprint_string(chassis_length: float, chassis_width: float) -> str:
    """Nav2 expects the footprint param as a JSON-ish string of [x, y] pairs."""
    L2 = chassis_length / 2.0
    W2 = chassis_width / 2.0
    return (
        f"[[{L2:.4f}, {W2:.4f}], "
        f"[{L2:.4f}, {-W2:.4f}], "
        f"[{-L2:.4f}, {-W2:.4f}], "
        f"[{-L2:.4f}, {W2:.4f}]]"
    )


def render(overrides: Dict[str, float] | None = None) -> Tuple[str, str, Dict[str, float]]:
    """Render URDF + footprint string.

    Returns (urdf_xml, footprint_str, effective_dims).
    """
    dims = merge_dimensions(load_persisted_dimensions(), overrides or {})
    template = _read_template()
    urdf = template.format(**{k: f"{dims[k]:.4f}" for k in DIM_KEYS})
    fp = footprint_string(dims["chassis_length"], dims["chassis_width"])
    return urdf, fp, dims


if __name__ == "__main__":
    urdf, fp, dims = render()
    print("# Effective dimensions:", dims)
    print("# Footprint:", fp)
    print(urdf)
