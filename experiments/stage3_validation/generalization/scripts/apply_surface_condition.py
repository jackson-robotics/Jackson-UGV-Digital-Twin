import os
from datetime import datetime

import omni.usd
import omni.timeline

from pxr import Usd, UsdPhysics, UsdShade


ROOT = os.path.expanduser(
    "~/jackson_dt_ws/revision_r1_5_generalization"
)

CONDITION_FILE = os.path.join(
    ROOT,
    "ACTIVE_CONDITION.txt",
)

COLLISION_PATH = "/World/GroundPlane/CollisionPlane"
MATERIAL_PATH = "/World/Stage3_G03_LowFrictionMaterial"

STATIC_FRICTION = 0.30
DYNAMIC_FRICTION = 0.25
RESTITUTION = 0.0


with open(CONDITION_FILE, "r", encoding="utf-8") as f:
    condition = f.read().strip()

if condition != "G03_LOW_FRICTION":
    raise RuntimeError(
        f"This script is only for G03_LOW_FRICTION; active={condition}"
    )


timeline = omni.timeline.get_timeline_interface()

if timeline.is_playing():
    raise RuntimeError(
        "Isaac timeline is PLAYING. Press STOP first."
    )


stage = omni.usd.get_context().get_stage()

if stage is None:
    raise RuntimeError("No USD stage is open.")


collision_prim = stage.GetPrimAtPath(COLLISION_PATH)

if not collision_prim.IsValid():
    raise RuntimeError(
        f"Collision plane not found: {COLLISION_PATH}"
    )


original_target = stage.GetEditTarget()
session = stage.GetSessionLayer()

stage.SetEditTarget(
    Usd.EditTarget(session)
)

try:
    material = UsdShade.Material.Define(
        stage,
        MATERIAL_PATH
    )

    mat_prim = material.GetPrim()

    physics_mat = UsdPhysics.MaterialAPI.Apply(
        mat_prim
    )

    physics_mat.CreateStaticFrictionAttr().Set(
        STATIC_FRICTION
    )

    physics_mat.CreateDynamicFrictionAttr().Set(
        DYNAMIC_FRICTION
    )

    physics_mat.CreateRestitutionAttr().Set(
        RESTITUTION
    )

    binding_api = UsdShade.MaterialBindingAPI.Apply(
        collision_prim
    )

    binding_api.Bind(
        material,
        materialPurpose="physics"
    )

    # ---------------- verification ----------------

    sf = float(
        physics_mat.GetStaticFrictionAttr().Get()
    )

    df = float(
        physics_mat.GetDynamicFrictionAttr().Get()
    )

    re = float(
        physics_mat.GetRestitutionAttr().Get()
    )

    rel = collision_prim.GetRelationship(
        "material:binding:physics"
    )

    targets = [
        str(x)
        for x in rel.GetTargets()
    ]

    ok = (
        abs(sf - STATIC_FRICTION) < 1e-6
        and abs(df - DYNAMIC_FRICTION) < 1e-6
        and abs(re - RESTITUTION) < 1e-6
        and MATERIAL_PATH in targets
    )

    if not ok:
        raise RuntimeError(
            "Surface material verification FAILED."
        )

finally:
    stage.SetEditTarget(
        original_target
    )


outdir = os.path.join(
    ROOT,
    "runs",
    condition
)

os.makedirs(
    outdir,
    exist_ok=True
)

audit = os.path.join(
    outdir,
    "SURFACE_CONDITION_SETUP.txt"
)

with open(
    audit,
    "w",
    encoding="utf-8"
) as f:
    f.write(
        "JACKSON STAGE 3 — GENERALIZATION SURFACE SETUP\n"
    )
    f.write("=" * 72 + "\n\n")
    f.write(f"Timestamp: {datetime.now().isoformat()}\n")
    f.write(f"Condition: {condition}\n")
    f.write(f"Collision prim: {COLLISION_PATH}\n")
    f.write(f"Physics material: {MATERIAL_PATH}\n")
    f.write(f"Static friction: {sf:.3f}\n")
    f.write(f"Dynamic friction: {df:.3f}\n")
    f.write(f"Restitution: {re:.3f}\n")
    f.write(f"Binding targets: {targets}\n")
    f.write("Authoring layer: USD Session Layer\n")
    f.write("Verification: PASS\n\n")
    f.write(
        "These coefficients define a controlled low-friction "
        "simulation condition and are not measurements of the "
        "physical floor friction.\n"
    )

print("LOW-FRICTION SURFACE APPLIED AND VERIFIED")
print(
    f"static={sf:.2f}, dynamic={df:.2f}, restitution={re:.2f}"
)
print("Verification: PASS")
print(f"Audit saved: {audit}")
