import os
from datetime import datetime

import omni.usd
import omni.timeline

from pxr import Usd, UsdPhysics


ROOT = os.path.expanduser(
    "~/jackson_dt_ws/revision_r1_1_sensitivity"
)

CONDITION_FILE = os.path.join(
    ROOT,
    "ACTIVE_CONDITION.txt",
)

CONTROLLER = "/Differential_drive/differential_controller"

LEFT_JOINT = (
    "/World/Jackson_mobile_robot/"
    "base_link/wheel_joints/left_wheel_joint"
)

RIGHT_JOINT = (
    "/World/Jackson_mobile_robot/"
    "base_link/wheel_joints/right_wheel_joint"
)


CONDITIONS = {
    "C00_BASE": {
        "wheel_radius": 0.03300,
        "wheel_distance": 0.19800,
        "damping": 10000.0,
    },

    "C01_R_MINUS5": {
        "wheel_radius": 0.03135,
        "wheel_distance": 0.19800,
        "damping": 10000.0,
    },

    "C02_R_PLUS5": {
        "wheel_radius": 0.03465,
        "wheel_distance": 0.19800,
        "damping": 10000.0,
    },

    "C03_B_MINUS5": {
        "wheel_radius": 0.03300,
        "wheel_distance": 0.18810,
        "damping": 10000.0,
    },

    "C04_B_PLUS5": {
        "wheel_radius": 0.03300,
        "wheel_distance": 0.20790,
        "damping": 10000.0,
    },

    "C05_D_MINUS10": {
        "wheel_radius": 0.03300,
        "wheel_distance": 0.19800,
        "damping": 9000.0,
    },

    "C06_D_PLUS10": {
        "wheel_radius": 0.03300,
        "wheel_distance": 0.19800,
        "damping": 11000.0,
    },
}


# ============================================================
# ACTIVE CONDITION
# ============================================================

if not os.path.isfile(CONDITION_FILE):
    raise RuntimeError(
        f"Missing ACTIVE_CONDITION.txt: {CONDITION_FILE}"
    )

with open(
    CONDITION_FILE,
    "r",
    encoding="utf-8",
) as f:
    condition = f.read().strip()

if condition not in CONDITIONS:
    raise RuntimeError(
        f"Unsupported condition: {condition}"
    )

cfg = CONDITIONS[condition]


# ============================================================
# ISAAC STATE
# ============================================================

timeline = omni.timeline.get_timeline_interface()

if timeline.is_playing():
    raise RuntimeError(
        "Isaac timeline is PLAYING. "
        "Press STOP before applying a condition."
    )

stage = omni.usd.get_context().get_stage()

if stage is None:
    raise RuntimeError(
        "No USD stage is currently open."
    )


# ============================================================
# USE SESSION LAYER ONLY
# ============================================================

original_edit_target = stage.GetEditTarget()

session_layer = stage.GetSessionLayer()

stage.SetEditTarget(
    Usd.EditTarget(session_layer)
)


try:

    # ========================================================
    # DIFFERENTIAL CONTROLLER
    # ========================================================

    controller = stage.GetPrimAtPath(
        CONTROLLER
    )

    if not controller.IsValid():
        raise RuntimeError(
            f"Missing Differential Controller prim: "
            f"{CONTROLLER}"
        )

    radius_attr = controller.GetAttribute(
        "inputs:wheelRadius"
    )

    distance_attr = controller.GetAttribute(
        "inputs:wheelDistance"
    )

    if not radius_attr.IsValid():
        attrs = [
            a.GetName()
            for a in controller.GetAttributes()
            if "wheel" in a.GetName().lower()
        ]

        raise RuntimeError(
            "inputs:wheelRadius not found. "
            f"Wheel-related attrs: {attrs}"
        )

    if not distance_attr.IsValid():
        attrs = [
            a.GetName()
            for a in controller.GetAttributes()
            if "wheel" in a.GetName().lower()
        ]

        raise RuntimeError(
            "inputs:wheelDistance not found. "
            f"Wheel-related attrs: {attrs}"
        )

    radius_attr.Set(
        float(cfg["wheel_radius"])
    )

    distance_attr.Set(
        float(cfg["wheel_distance"])
    )


    # ========================================================
    # PHYSX WHEEL-JOINT DAMPING
    # ========================================================

    damping_readbacks = {}

    for label, path in [
        ("left", LEFT_JOINT),
        ("right", RIGHT_JOINT),
    ]:

        prim = stage.GetPrimAtPath(path)

        if not prim.IsValid():
            raise RuntimeError(
                f"Missing wheel joint: {path}"
            )

        drive = UsdPhysics.DriveAPI.Get(
            prim,
            "angular",
        )

        damping_attr = drive.GetDampingAttr()

        if not damping_attr.IsValid():
            raise RuntimeError(
                f"No angular-drive damping attribute: "
                f"{path}"
            )

        damping_attr.Set(
            float(cfg["damping"])
        )

        damping_readbacks[label] = float(
            damping_attr.Get()
        )


    # ========================================================
    # READBACK
    # ========================================================

    radius_rb = float(
        radius_attr.Get()
    )

    distance_rb = float(
        distance_attr.Get()
    )

    left_damping_rb = damping_readbacks["left"]
    right_damping_rb = damping_readbacks["right"]


finally:

    # Restore the user's previous edit target.
    # Overrides remain active in the Session Layer.
    stage.SetEditTarget(
        original_edit_target
    )


# ============================================================
# STRICT VERIFICATION
# ============================================================

tol = 1e-9

checks = {
    "wheelRadius": (
        abs(
            radius_rb
            - cfg["wheel_radius"]
        ) <= tol
    ),

    "wheelDistance": (
        abs(
            distance_rb
            - cfg["wheel_distance"]
        ) <= tol
    ),

    "left damping": (
        abs(
            left_damping_rb
            - cfg["damping"]
        ) <= tol
    ),

    "right damping": (
        abs(
            right_damping_rb
            - cfg["damping"]
        ) <= tol
    ),
}

verification_pass = all(
    checks.values()
)

if not verification_pass:
    raise RuntimeError(
        "Condition readback verification FAILED: "
        f"{checks}"
    )


# ============================================================
# TRACEABILITY FILE
# ============================================================

condition_dir = os.path.join(
    ROOT,
    "runs",
    condition,
)

os.makedirs(
    condition_dir,
    exist_ok=True,
)

out = os.path.join(
    condition_dir,
    "PHYSX_CONDITION_SETUP.txt",
)

lines = [
    "JACKSON STAGE 3 — SENSITIVITY CONDITION SETUP",
    "=" * 72,

    f"Timestamp: {datetime.now().isoformat()}",
    f"Condition: {condition}",

    "",
    "USD stage:",
    f"  {stage.GetRootLayer().identifier}",

    "",
    "Authoring layer:",
    "  USD Session Layer (non-persistent)",

    "",
    "Isaac Sim Differential Controller:",
    f"  prim: {CONTROLLER}",
    (
        "  wheelRadius requested: "
        f"{cfg['wheel_radius']:.5f} m"
    ),
    (
        "  wheelRadius readback: "
        f"{radius_rb:.5f} m"
    ),
    (
        "  wheelDistance requested: "
        f"{cfg['wheel_distance']:.5f} m"
    ),
    (
        "  wheelDistance readback: "
        f"{distance_rb:.5f} m"
    ),

    "",
    "ROS wheel-odometry parameters expected:",
    (
        "  wheel_radius: "
        f"{cfg['wheel_radius']:.5f} m"
    ),
    (
        "  wheel_separation: "
        f"{cfg['wheel_distance']:.5f} m"
    ),

    "",
    "PhysX angular-drive damping:",
    (
        "  requested: "
        f"{cfg['damping']:.1f}"
    ),
    (
        "  left readback: "
        f"{left_damping_rb:.1f}"
    ),
    (
        "  right readback: "
        f"{right_damping_rb:.1f}"
    ),

    "",
    "Verification:",
    "  wheelRadius: PASS",
    "  wheelDistance: PASS",
    "  left damping: PASS",
    "  right damping: PASS",
    "Verification: PASS",

    "",
    "Timeline state during application:",
    "  NOT PLAYING",

    "",
    "IMPORTANT:",
    (
        "  wheelRadius and wheelDistance are applied "
        "to the Isaac Sim Differential Controller."
    ),
    (
        "  The experiment launcher must use identical "
        "values for ROS joint-state odometry."
    ),
]

with open(
    out,
    "w",
    encoding="utf-8",
) as f:

    f.write(
        "\n".join(lines)
        + "\n"
    )


# Keep Script Editor output intentionally short.
print("CONDITION APPLIED AND VERIFIED")
print(f"Condition: {condition}")
print(
    f"wheelRadius={radius_rb:.5f} m, "
    f"wheelDistance={distance_rb:.5f} m, "
    f"damping={left_damping_rb:.1f}"
)
print("Verification: PASS")
print(f"Audit saved: {out}")
