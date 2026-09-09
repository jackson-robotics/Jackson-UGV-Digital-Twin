import omni.usd
import omni.timeline

from pxr import Usd, UsdShade


COLLISION_PATH = "/World/GroundPlane/CollisionPlane"
G03_MATERIAL = "/World/Stage3_G03_LowFrictionMaterial"


timeline = omni.timeline.get_timeline_interface()

if timeline.is_playing():
    raise RuntimeError(
        "Isaac timeline is PLAYING. Press STOP first."
    )

stage = omni.usd.get_context().get_stage()

if stage is None:
    raise RuntimeError("No USD stage is open.")

plane = stage.GetPrimAtPath(COLLISION_PATH)

if not plane.IsValid():
    raise RuntimeError(
        f"Collision plane not found: {COLLISION_PATH}"
    )

original = stage.GetEditTarget()
session = stage.GetSessionLayer()

stage.SetEditTarget(
    Usd.EditTarget(session)
)

try:
    # G03 baseline audit showed no physics material binding.
    plane.RemoveProperty(
        "material:binding:physics"
    )

    # Remove the session-layer material created for G03.
    if stage.GetPrimAtPath(G03_MATERIAL).IsValid():
        stage.RemovePrim(G03_MATERIAL)

finally:
    stage.SetEditTarget(original)


rel = plane.GetRelationship(
    "material:binding:physics"
)

targets = (
    [str(x) for x in rel.GetTargets()]
    if rel.IsValid()
    else []
)

if targets:
    raise RuntimeError(
        f"Baseline surface reset FAILED: {targets}"
    )

print("BASELINE SURFACE RESTORED")
print("Physics material binding targets: []")
print("Verification: PASS")
