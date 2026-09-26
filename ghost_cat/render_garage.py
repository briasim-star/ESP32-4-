"""Photoreal Blender (Cycles) mock-up: enlarged ghost cat on the garage torch light.

Run:  python3 ghost_cat/render_garage.py [scale] [samples] [res_percent]
Needs: pip install bpy==4.2.0   and ghost_cat/cat_orig.stl (mm, from the 3MF)
"""
import math
import sys
from pathlib import Path

import bpy

HERE = Path(__file__).resolve().parent
SCALE = float(sys.argv[1]) if len(sys.argv) > 1 else 2.5
SAMPLES = int(sys.argv[2]) if len(sys.argv) > 2 else 128
RES = int(sys.argv[3]) if len(sys.argv) > 3 else 100
ROT = float(sys.argv[4]) if len(sys.argv) > 4 else 0   # degrees; turns the cat to face the camera

WALL_Y = 0.19          # wall face, metres behind the post centre
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene


def mat(name, color, rough=0.5, metal=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    p = m.node_tree.nodes["Principled BSDF"]
    p.inputs["Base Color"].default_value = (*color, 1)
    p.inputs["Roughness"].default_value = rough
    p.inputs["Metallic"].default_value = metal
    return m


def assign(ob, m):
    ob.data.materials.append(m)
    return ob


# --- materials -------------------------------------------------------------
siding = mat("siding", (0.52, 0.50, 0.46), 0.75)
paint = mat("white_paint", (0.80, 0.80, 0.78), 0.35)
trim = mat("trim", (0.70, 0.70, 0.68), 0.5)

# --- wall: lap siding boards, each tilted so its bottom edge kicks out -------
BOARD = 0.17
for i in range(-8, 6):
    z = i * BOARD
    bpy.ops.mesh.primitive_cube_add(size=1, location=(0, WALL_Y + 0.012, z + BOARD / 2))
    b = bpy.context.object
    b.scale = (2.4, 0.02, BOARD + 0.004)
    b.rotation_euler.x = math.radians(-4)
    bpy.ops.object.shade_flat()
    assign(b, siding)
# soffit
bpy.ops.mesh.primitive_plane_add(size=1, location=(0, -0.3, 0.62))
s = bpy.context.object; s.scale = (2.4, 1.1, 1); assign(s, trim)
# corner trim board at the right
bpy.ops.mesh.primitive_cube_add(size=1, location=(0.55, WALL_Y - 0.01, -0.3))
t = bpy.context.object; t.scale = (0.11, 0.04, 2.0); assign(t, trim)

# --- torch-style fixture (white): cup, post, collar, arm, wall plate --------
def cyl(r1, r2, depth, loc, rot=(0, 0, 0), m=paint, verts=64):
    bpy.ops.mesh.primitive_cone_add(vertices=verts, radius1=r1, radius2=r2, depth=depth,
                                    location=loc, rotation=rot)
    o = bpy.context.object; bpy.ops.object.shade_smooth(); assign(o, m); return o

def ball(r, loc, sz=(1, 1, 1)):
    bpy.ops.mesh.primitive_uv_sphere_add(radius=r, location=loc, segments=48, ring_count=24)
    o = bpy.context.object; o.scale = sz; bpy.ops.object.shade_smooth(); assign(o, paint); return o

cyl(0.030, 0.080, 0.05, (0, 0, -0.025))            # cup the cat sits in
ball(0.055, (0, 0, -0.075), (1, 1, 0.45))           # bulge under cup
cyl(0.022, 0.034, 0.30, (0, 0, -0.25))              # tapered post
ball(0.04, (0, 0, -0.41), (1, 1, 0.55))             # bottom collar
cyl(0.016, 0.016, WALL_Y, (0, WALL_Y / 2, -0.42), rot=(math.radians(90), 0, 0))  # arm to wall
bpy.ops.mesh.primitive_cube_add(size=1, location=(0, WALL_Y - 0.012, -0.30))
wp = bpy.context.object; wp.scale = (0.13, 0.025, 0.30)
bev = wp.modifiers.new("bevel", "BEVEL"); bev.width = 0.02; bev.segments = 6
bpy.ops.object.shade_smooth(); assign(wp, paint)

# --- LED bulb inside ---------------------------------------------------------
bpy.ops.mesh.primitive_uv_sphere_add(radius=0.03, location=(0, 0, 0.075))
bulb = bpy.context.object
bm = bpy.data.materials.new("bulb"); bm.use_nodes = True
nt = bm.node_tree; nt.nodes.remove(nt.nodes["Principled BSDF"])
em = nt.nodes.new("ShaderNodeEmission"); em.inputs["Strength"].default_value = 3
em.inputs["Color"].default_value = (1.0, 0.93, 0.82, 1)
nt.links.new(em.outputs[0], nt.nodes["Material Output"].inputs[0]); assign(bulb, bm)
bulb.visible_shadow = False

bpy.ops.object.light_add(type="POINT", location=(0, 0, 0.08))
lamp = bpy.context.object
lamp.data.energy = 5; lamp.data.shadow_soft_size = 0.03
lamp.data.color = (1.0, 0.92, 0.80)

# --- the ghost cat -----------------------------------------------------------
bpy.ops.wm.stl_import(filepath=str(HERE / "cat_orig.stl"), global_scale=0.001 * SCALE)
cat = bpy.context.selected_objects[0]
bpy.ops.object.origin_set(type="ORIGIN_GEOMETRY", center="BOUNDS")
cat.location = (0, 0, cat.dimensions.z / 2)
cat.rotation_euler.z = math.radians(ROT)
bpy.ops.object.shade_smooth()
cat.visible_shadow = False   # let the inner lamp light the wall through the shell

gm = bpy.data.materials.new("clear_petg"); gm.use_nodes = True
nt = gm.node_tree; N = nt.nodes; L = nt.links
p = N["Principled BSDF"]
p.inputs["Base Color"].default_value = (0.80, 0.95, 0.72, 1)   # glow-in-the-dark PLA: pale green
p.inputs["Roughness"].default_value = 0.3
p.inputs["IOR"].default_value = 1.57
p.inputs["Transmission Weight"].default_value = 0.55   # milky, not clear
# glow falloff: brightest near the bulb, fading toward paws/ears/hem
tc = N.new("ShaderNodeTexCoord"); tc.object = bulb
ln = N.new("ShaderNodeVectorMath"); ln.operation = "LENGTH"
mr = N.new("ShaderNodeMapRange")
mr.inputs["From Min"].default_value = 0.04; mr.inputs["From Max"].default_value = 0.09 * SCALE
mr.inputs["To Min"].default_value = 1.0; mr.inputs["To Max"].default_value = 0.05
pw = N.new("ShaderNodeMath"); pw.operation = "POWER"; pw.inputs[1].default_value = 1.6
gl = N.new("ShaderNodeEmission"); gl.inputs["Color"].default_value = (0.55, 1.0, 0.45, 1)   # phosphor green
k = N.new("ShaderNodeMath"); k.operation = "MULTIPLY"; k.inputs[1].default_value = 0.45
add = N.new("ShaderNodeAddShader")
L.new(tc.outputs["Object"], ln.inputs[0]); L.new(ln.outputs["Value"], mr.inputs["Value"])
L.new(mr.outputs["Result"], pw.inputs[0]); L.new(pw.outputs[0], k.inputs[0])
L.new(k.outputs[0], gl.inputs["Strength"])
L.new(p.outputs[0], add.inputs[0]); L.new(gl.outputs[0], add.inputs[1])
L.new(add.outputs[0], N["Material Output"].inputs["Surface"])
assign(cat, gm)

# --- night world + faint moonlight ------------------------------------------
w = bpy.data.worlds.new("night"); scene.world = w; w.use_nodes = True
w.node_tree.nodes["Background"].inputs["Color"].default_value = (0.004, 0.006, 0.012, 1)
bpy.ops.object.light_add(type="SUN", rotation=(math.radians(60), 0, math.radians(-30)))
sun = bpy.context.object; sun.data.energy = 0.04; sun.data.color = (0.6, 0.7, 1.0)

# --- camera: 3/4 view from below-left, like standing in the driveway --------
bpy.ops.object.camera_add(location=(-0.62, -1.05, -0.12))
cam = bpy.context.object; scene.camera = cam; cam.data.lens = 50
tgt = bpy.data.objects.new("tgt", None); tgt.location = (0, 0, 0.02)
scene.collection.objects.link(tgt)
c = cam.constraints.new("TRACK_TO"); c.target = tgt
cam.data.dof.use_dof = True; cam.data.dof.focus_object = tgt; cam.data.dof.aperture_fstop = 2.8

# --- render settings ----------------------------------------------------------
scene.render.engine = "CYCLES"
scene.cycles.device = "CPU"
scene.cycles.samples = SAMPLES
scene.cycles.use_denoising = True
scene.render.resolution_x, scene.render.resolution_y = 1080, 1350
scene.render.resolution_percentage = RES
scene.view_settings.view_transform = "AgX"
scene.view_settings.look = "AgX - Punchy"

scene.use_nodes = True
ct = scene.node_tree
rl = ct.nodes["Render Layers"]; comp = ct.nodes["Composite"]
glare = ct.nodes.new("CompositorNodeGlare"); glare.glare_type = "FOG_GLOW"
glare.quality = "HIGH"; glare.size = 8; glare.threshold = 1.0
ct.links.new(rl.outputs["Image"], glare.inputs["Image"])
ct.links.new(glare.outputs["Image"], comp.inputs["Image"])

out = HERE / (f"garage_ghost_cat_{SCALE:g}x.png" if len(sys.argv) < 6 else sys.argv[5])
scene.render.filepath = str(out)
bpy.ops.render.render(write_still=True)
print("wrote", out, "cat size mm:", [round(d * 1000) for d in cat.dimensions])
