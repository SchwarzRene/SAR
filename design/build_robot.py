"""SAR v2 - parametric six-axis arm in Blender (run with the `bpy` pip module or `blender -b -P`).

Dimensions (mm) mirror simulation/index.html:
  base_h  J2 axis height above ground
  upper   J2 -> J3
  fore    J3 -> J5 (J4 roll splits the forearm at 55 %)
  wrist   J5 -> J6
  tool    J6 -> TCP
Motors: J1, J2, J3 = NEMA 23, J4 = NEMA 14, J5 = NEMA 17, J6 = NEMA 11, each followed by a cycloidal drive.

Usage:  python build_robot.py [outdir]      -> renders shell + cutaway views, saves .blend and .glb
"""
import math
import os
import sys

import bpy

DIM = dict(base_h=190, upper=150, fore=150, wrist=75, tool=75)
MOTOR = dict(  # square flange mm, body length mm
    J1=(57, 56), J2=(57, 56), J3=(57, 56), J4=(35, 36), J5=(42, 40), J6=(28, 45))
MM = 0.001


# ---------------------------------------------------------------- materials
def mat(name, color, rough=0.3, metal=0.0, coat=0.0, alpha=1.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    b = nt.nodes.new("ShaderNodeBsdfPrincipled")
    b.inputs["Base Color"].default_value = (*color, 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    b.inputs["Coat Weight"].default_value = coat
    if alpha < 1:
        t = nt.nodes.new("ShaderNodeBsdfTransparent")
        mix = nt.nodes.new("ShaderNodeMixShader")
        mix.inputs[0].default_value = alpha
        nt.links.new(t.outputs[0], mix.inputs[1])
        nt.links.new(b.outputs[0], mix.inputs[2])
        nt.links.new(mix.outputs[0], out.inputs[0])
    else:
        nt.links.new(b.outputs[0], out.inputs[0])
    return m


# ---------------------------------------------------------------- primitives
def _finish(o, parent, loc, material, bevel, smooth=True):
    o.parent = parent
    o.location = tuple(v * MM for v in loc)
    o.data.materials.append(material)
    if bevel:
        bm = o.modifiers.new("bevel", "BEVEL")
        bm.width = bevel * MM
        bm.segments = 4
        bm.limit_method = "ANGLE"
    if smooth:
        bpy.context.view_layer.objects.active = o
        o.select_set(True)
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(35))
        o.select_set(False)
    return o


ROT = {"x": (0, math.pi / 2, 0), "y": (-math.pi / 2, 0, 0), "z": (0, 0, 0)}


def cyl(parent, loc, axis, r, length, material, bevel=0.0):
    bpy.ops.mesh.primitive_cylinder_add(vertices=96, radius=r * MM, depth=length * MM)
    o = bpy.context.active_object
    o.rotation_euler = ROT[axis]
    return _finish(o, parent, loc, material, bevel)


def cone(parent, loc, axis, r1, r2, length, material, squash=1.0):
    """tapered link along `axis`; r1 at the start, r2 at the end; `squash` flattens it sideways (y)"""
    bpy.ops.mesh.primitive_cone_add(vertices=96, radius1=r1 * MM, radius2=r2 * MM, depth=length * MM)
    o = bpy.context.active_object
    o.rotation_euler = ROT[axis]
    o.scale = (1, squash, 1)
    return _finish(o, parent, loc, material, 0)


def box(parent, loc, size, material, bevel=1.5):
    bpy.ops.mesh.primitive_cube_add(size=1)
    o = bpy.context.active_object
    o.scale = tuple(s * MM for s in size)
    bpy.context.view_layer.objects.active = o
    bpy.ops.object.transform_apply(scale=True)
    return _finish(o, parent, loc, material, bevel)


def empty(name, parent=None, loc=(0, 0, 0)):
    e = bpy.data.objects.new(name, None)
    bpy.context.collection.objects.link(e)
    e.parent = parent
    e.location = tuple(v * MM for v in loc)
    return e


# ---------------------------------------------------------------- the robot
def build(pose, xray=False):
    a1, a2, a3, a4, a5, a6 = (math.radians(v) for v in pose)
    shell_alpha = 0.07 if xray else 1.0
    white = mat("shell", (0.93, 0.94, 0.96), 0.22, 0, 0.6, shell_alpha)
    dark = mat("seam", (0.025, 0.025, 0.03), 0.55)
    blue = mat("flange", (0.02, 0.16, 0.62), 0.25, 0, 0.5)
    silver = mat("silver", (0.72, 0.74, 0.78), 0.4, 0.9)
    motor_m = mat("motor", (0.10, 0.10, 0.11), 0.4, 0.8)
    drive_m = mat("cycloidal", (1.0, 0.42, 0.04), 0.35, 0.6)
    D = DIM
    H, U, F, W, T = D["base_h"], D["upper"], D["fore"], D["wrist"], D["tool"]

    # fixed base: mounting plate + pedestal that houses the J1 motor and drive
    box(None, (0, 0, 5), (170, 170, 10), silver, 2)
    cyl(None, (0, 0, 60), "z", 66, 100, white, 1)
    cyl(None, (0, 0, 108), "z", 66.5, 3, dark)

    J1 = empty("J1", None)
    J1.rotation_euler.z = a1
    # J1 inner parts (cutaway): vertical NEMA23 + cycloidal under the turret
    sq, ln = MOTOR["J1"]
    box(None, (0, 0, 14 + ln / 2), (sq, sq, ln), motor_m, 1)
    cyl(None, (0, 0, 14 + ln + 15), "z", 50, 30, drive_m)
    # turret column
    cyl(J1, (0, 0, (112 + H) / 2), "z", 58, H - 112, white, 1)

    def fork(parent, loc, rc, w_half, key, drive_dia):
        """U-joint about the y axis: motor+cycloidal housing on -y, bearing cap on +y; moving link sits between"""
        sq, ln = MOTOR[key]
        Ld = 30
        Lc = Ld + ln + 10
        x, _, z = loc
        cyl(parent, (x, -(w_half + Lc / 2), z), "y", rc, Lc, white, 1.5)
        cyl(parent, (x, -w_half + 1.5, z), "y", rc * 0.97, 3, dark)
        cyl(parent, (x, w_half + 7, z), "y", rc * 0.93, 14, white, 1.5)
        cyl(parent, (x, w_half + 0.8, z), "y", rc * 0.94, 2.5, dark)
        cyl(parent, (x, w_half + 15, z), "y", rc * 0.35, 3, silver)
        for i in range(6):  # cap screws
            t = i * math.pi / 3
            cyl(parent, (x + rc * 0.66 * math.cos(t), w_half + 14.6, z + rc * 0.66 * math.sin(t)), "y", 2.2, 1.5, dark)
        box(parent, (x, -(w_half + Ld + ln / 2), z), (sq, ln, sq), motor_m, 1)       # stepper
        cyl(parent, (x, -(w_half + Ld / 2), z), "y", drive_dia / 2, Ld, drive_m)     # cycloidal drive

    # ---- J2 shoulder (fork on turret) and upper arm
    fork(J1, (0, 0, H), 60, 38, "J2", 100)
    J2 = empty("J2", J1, (0, 0, H))
    J2.rotation_euler.y = -a2
    cyl(J2, (0, 0, 0), "y", 55, 76, white, 1)                       # shoulder hub
    cone(J2, (U / 2, 0, 0), "x", 54, 45, U, white, 0.66)            # upper arm
    # ---- J3 elbow (fork on upper arm) and forearm
    fork(J2, (U, 0, 0), 52, 31, "J3", 90)
    J3 = empty("J3", J2, (U, 0, 0))
    J3.rotation_euler.y = -a3
    cyl(J2, (U, 0, 0), "y", 47, 62, white, 1)                       # elbow hub
    fore_a = F * 0.55
    cone(J3, (fore_a / 2, 0, 0), "x", 44, 36, fore_a, white, 0.78)  # forearm, rear part
    # J4 roll: front part of the forearm rotates about the arm axis
    J4 = empty("J4", J3, (fore_a, 0, 0))
    J4.rotation_euler.x = a4
    cyl(J4, (-0.5, 0, 0), "x", 36.5, 3, dark).scale.y = 0.78
    fore_b = F - fore_a
    cone(J4, (fore_b / 2 - 4, 0, 0), "x", 36, 32, fore_b + 8, white, 0.78)
    sq, ln = MOTOR["J4"]
    box(J3, (fore_a * 0.5, 0, 0), (ln, sq, sq), motor_m, 1)
    cyl(J3, (fore_a * 0.85, 0, 0), "x", 26, 25, drive_m)
    # ---- J5 wrist pitch (fork on forearm front)
    fork(J4, (fore_b, 0, 0), 38, 25, "J5", 64)
    J5 = empty("J5", J4, (fore_b, 0, 0))
    J5.rotation_euler.y = -a5
    cyl(J5, (0, 0, 0), "y", 31, 50, white, 1)
    cone(J5, ((W - 8) / 2, 0, 0), "x", 29, 26, W - 8, white, 0.85)  # wrist link
    # ---- J6 tool roll
    cyl(J5, (W - 8, 0, 0), "x", 27.5, 3, dark)
    J6 = empty("J6", J5, (W, 0, 0))
    J6.rotation_euler.x = a6
    sq, ln = MOTOR["J6"]
    box(J5, (W * 0.45, 0, 0), (ln, sq, sq), motor_m, 1)
    cyl(J6, (10, 0, 0), "x", 27, 20, blue, 1.5)                     # blue flange like the Epson
    cyl(J6, (-1, 0, 0), "x", 26.5, 2, silver)
    # ---- gripper (tool, 75 mm J6 -> TCP)
    cyl(J6, (23, 0, 0), "x", 22, 6, dark)
    box(J6, (40, 0, 0), (28, 52, 22), dark, 2)
    for s in (-1, 1):
        box(J6, (T - 12, s * 20, 0), (24, 7, 16), silver, 1)
    return dict(J1=J1, J2=J2, J3=J3, J5=J5, J6=J6)


# ---------------------------------------------------------------- scene
def scene_setup(cam_loc, target, res=(1280, 800), samples=64):
    if os.environ.get("SAR_FAST"):
        res, samples = (720, 450), 12
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = samples
    sc.cycles.use_denoising = True
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.view_settings.view_transform = "Standard"
    # world: soft light gradient
    w = bpy.data.worlds.new("w")
    sc.world = w
    w.use_nodes = True
    bg = w.node_tree.nodes["Background"]
    bg.inputs[0].default_value = (0.82, 0.86, 0.92, 1)
    bg.inputs[1].default_value = 0.9
    # floor
    bpy.ops.mesh.primitive_plane_add(size=6)
    fl = bpy.context.active_object
    fl.data.materials.append(mat("floor", (0.78, 0.80, 0.84), 0.45, 0, 0.3))
    # lights
    for name, loc, power, size in (("key", (0.9, -1.2, 1.4), 650, 1.2), ("fill", (-1.2, -0.6, 0.8), 220, 1.5),
                                   ("rim", (-0.6, 1.2, 1.2), 300, 1.0)):
        ld = bpy.data.lights.new(name, "AREA")
        ld.energy, ld.size = power, size
        lo = bpy.data.objects.new(name, ld)
        bpy.context.collection.objects.link(lo)
        lo.location = loc
        c = lo.constraints.new("TRACK_TO")
        tgt = bpy.data.objects.new(name + "_t", None)
        bpy.context.collection.objects.link(tgt)
        tgt.location = (0.15, 0, 0.25)
        c.target, c.track_axis, c.up_axis = tgt, "TRACK_NEGATIVE_Z", "UP_Y"
    # camera
    cd = bpy.data.cameras.new("cam")
    cd.lens = 62
    co = bpy.data.objects.new("cam", cd)
    bpy.context.collection.objects.link(co)
    co.location = tuple(v * MM for v in cam_loc)
    ct = bpy.data.objects.new("cam_t", None)
    bpy.context.collection.objects.link(ct)
    ct.location = tuple(v * MM for v in target)
    c = co.constraints.new("TRACK_TO")
    c.target, c.track_axis, c.up_axis = ct, "TRACK_NEGATIVE_Z", "UP_Y"
    sc.camera = co


SHOTS = {  # name: (pose a1..a6 deg, xray, camera, target)
    "hero":      ((-20, 70, -85, 0, -10, 0), False, (700, -1050, 560), (110, 0, 260)),
    "stretched": ((0, 0, 0, 0, 0, 0), False, (420, -1250, 430), (210, 0, 190)),
    "cutaway":   ((0, 0, 0, 0, 0, 0), True, (420, -1250, 430), (210, 0, 190)),
    "folded":    ((0, 90, -150, 0, 60, 0), False, (700, -1050, 560), (60, 0, 230)),
}


def render(name, out, save=False):
    pose, xray, cam, tgt = SHOTS[name]
    bpy.ops.wm.read_factory_settings(use_empty=True)
    build(pose, xray)
    scene_setup(cam, tgt)
    bpy.context.scene.render.filepath = os.path.join(out, f"sar_{name}.png")
    bpy.ops.render.render(write_still=True)
    if save:
        bpy.ops.wm.save_as_mainfile(filepath=os.path.join(out, "sar_v2.blend"))
        try:
            bpy.ops.export_scene.gltf(filepath=os.path.join(out, "sar_v2.glb"), export_format="GLB")
        except Exception as e:  # exporter add-on may be missing
            print("glb export skipped:", e)


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "renders")
    which = sys.argv[2].split(",") if len(sys.argv) > 2 else ["hero", "stretched", "cutaway"]
    os.makedirs(out, exist_ok=True)
    for i, n in enumerate(which):
        render(n, out, save=(n == which[0]))
