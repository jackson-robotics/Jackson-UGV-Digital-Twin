import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

stage = omni.usd.get_context().get_stage()

if stage is None:
    raise RuntimeError("No USD stage is open.")

out = (
    "/home/carlos/jackson_dt_ws/revision_r1_5_generalization/"
    "SURFACE_MATERIAL_AUDIT.txt"
)

bbox_cache = UsdGeom.BBoxCache(
    Usd.TimeCode.Default(),
    [UsdGeom.Tokens.default_]
)

lines = []
count = 0

for prim in stage.Traverse():
    if not prim.IsValid():
        continue

    path = str(prim.GetPath())
    name = prim.GetName().lower()

    has_collision = prim.HasAPI(UsdPhysics.CollisionAPI)

    try:
        box = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
        size = box.GetSize()
        sx, sy, sz = map(float, size)
    except Exception:
        sx = sy = sz = -1.0

    candidate = (
        any(k in name for k in ("ground", "floor", "plane"))
        or (has_collision and sx > 1.0 and sy > 1.0)
    )

    if not candidate:
        continue

    count += 1

    lines.append("=" * 80)
    lines.append(f"PRIM: {path}")
    lines.append(f"Type: {prim.GetTypeName()}")
    lines.append(f"CollisionAPI: {has_collision}")
    lines.append(
        f"World size approx: ({sx:.3f}, {sy:.3f}, {sz:.3f})"
    )
    lines.append(f"Applied schemas: {prim.GetAppliedSchemas()}")

    try:
        binding_api = UsdShade.MaterialBindingAPI(prim)
        rel = binding_api.GetDirectBindingRel()
        targets = rel.GetTargets()
        lines.append(f"Bound material targets: {targets}")

        for target in targets:
            mprim = stage.GetPrimAtPath(target)
            lines.append(f"  MATERIAL PRIM: {target}")
            lines.append(
                f"  Applied schemas: {mprim.GetAppliedSchemas()}"
            )

            for attr in mprim.GetAttributes():
                n = attr.GetName().lower()
                if any(
                    k in n
                    for k in (
                        "friction",
                        "restitution",
                        "density",
                        "material"
                    )
                ):
                    try:
                        lines.append(
                            f"  {attr.GetName()} = {attr.Get()}"
                        )
                    except Exception:
                        pass

    except Exception as e:
        lines.append(f"Material binding unavailable: {e}")

    for attr in prim.GetAttributes():
        n = attr.GetName().lower()
        if any(
            k in n
            for k in ("friction", "restitution", "material")
        ):
            try:
                lines.append(
                    f"{attr.GetName()} = {attr.Get()}"
                )
            except Exception:
                pass

with open(out, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")

print("SURFACE AUDIT SAVED")
print(out)
print(f"Candidate prims found: {count}")
