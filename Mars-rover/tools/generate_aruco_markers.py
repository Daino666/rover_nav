#!/usr/bin/env python3
"""
Generate the ArUco landmark markers used for vision-based EKF localization
corrections, and drop them into the Gazebo sim world.

The real 2026 Mars Yard already has 15 surveyed points reserved for exactly
this purpose -- L1..L15 in Coordinates_MarsYard2026.txt (group "L" =
"Landmarks (ArUco)", see tools/generate_occupancy_grid.py's GROUP_COLOR).
Rather than inventing new marker positions, this script builds the sim
markers straight from those surveyed coordinates so the sim landmark layout
matches the real yard.

Marker size, dictionary, ID range, and mounting height all come from the
official ERC Mars Yard landmark spec ("Figure 9: Dimensions of landmarks" +
the "MY - Landmarks - ArUco Markers" slide): each landmark is a signboard
with a printed number (1-15, human-readable only) above a 150mm ArUco code
from the 5x5 dictionary -- but the ENCODED marker ID is 50 + the printed
number (51-65), not the printed number itself. OpenCV's 5x5 dictionaries are
nested (ID 51 is identical in DICT_5X5_100/250/1000), so DICT_5X5_100 -- the
smallest one that reaches 65 -- is used. Per the spec's isometric sketch, the
real sign is a 4-sided post-mounted box with the same number+marker graphic
printed on all 4 vertical faces (so it's readable from any approach
direction) -- not a single flat panel.

For each Lk point it:
  1. renders the full sign face (number plate + ArUco code, see
     render_sign_face()) as one texture, saved under a name unique to that
     marker (sign_<id>.png) -- Gazebo/Ogre2 caches textures by filename, so
     giving every model the same "sign.png" name silently collapses them
     all onto whichever one loaded first (this was a real bug: every sign
     rendered as marker 1 in-sim even though the per-model files on disk
     were each correct),
  2. samples the actual terrain mesh Gazebo renders (not the raw survey
     "H (local)" column, which disagrees with the real mesh by several cm
     to over a decimeter at some points) to find solid ground at that
     (x, y) -- specifically the highest mesh vertex within the sign's own
     footprint radius, since this terrain is rocky enough at small scale
     that just the nearest single point can land in a dip next to a taller
     rock and make the sign clip into the hill,
  3. writes a Gazebo model (model.config + model.sdf: a post + a 4-sided
     sign box, textured identically on all 4 vertical faces) under
     rover_description/models/aruco_marker_<id>/,
  4. (re)writes the <include> block for all 15 markers into
     rover_description/worlds/marsyard.sdf, replacing only the
     auto-generated region between the AUTO-GENERATED ARUCO LANDMARKS
     markers -- everything else in the world file is left untouched.

Since the sign is omnidirectional (same graphic on all 4 sides), yaw doesn't
matter for detectability and is just left at 0.

HOW TO USE: edit the PARAMETERS block below, then run with no arguments:
  python3 generate_aruco_markers.py
Re-run any time to change the dictionary/size/mount height/facing -- it
always regenerates all 15 markers from scratch.
"""

import os

import cv2.aruco as aruco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial import cKDTree

# ============================================================================
# PARAMETERS -- edit these, then just run the script with no arguments.
# ============================================================================

ARUCO_DICT = aruco.DICT_5X5_100
# Smallest predefined 5x5 dictionary that covers IDs up to 65. OpenCV's 5x5
# dictionaries are nested (a given ID renders identically in 5x5_100/250/1000)
# so this is equivalent to any larger 5x5_* choice.

ID_OFFSET = 50
# Encoded ArUco ID = ID_OFFSET + printed sign number. Per the ERC spec:
# "Markers from the 5x5 ArUco Library shall be considered (numbers 51-65)."
# The big printed digit (1-15) on the sign is just a human-readable label --
# it is NOT the bit pattern encoded in the marker.

MARKER_SIZE_M = 0.150
# ArUco code square size, incl. its own built-in border module. "150+-2" in
# the spec drawing.

BOARD_WIDTH_M = 0.250
BOARD_HEIGHT_M = 0.297
# Signboard outer size. Width "250+-5" is given directly; height isn't
# explicitly dimensioned in the drawing but matches the printed sheet's A4
# proportions ("210" wide x "297" tall) implied alongside it.

NUMBER_LABEL_HEIGHT_M = 0.030
# Height of the white number-plate band at the top of the sign ("30" in the
# spec drawing).

BOARD_TOP_HEIGHT_M = 0.417
MARKER_TOP_HEIGHT_M = 0.320
# Ground-referenced mounting heights straight from the spec drawing
# ("417+-5" to the top of the board, "320+-5" to the top of the ArUco code).
# Everything else (post height, internal gaps) is derived from these two so
# the marker ends up at the exact real-world height above ground.

POST_RADIUS_M = 0.025
PX_PER_MM = 4  # texture resolution

# Sign box footprint is BOARD_WIDTH_M x BOARD_WIDTH_M (square, cube-like) x
# BOARD_HEIGHT_M -- 4 identical vertical faces, per the spec's isometric
# sketch, each carrying the same rendered sign texture.

# ============================================================================
# End of parameters. Nothing below this line needs editing for normal use.
# ============================================================================

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COORDS_FILE = os.path.join(
    os.path.expanduser("~/jazzy_ws/src/marsyard"),
    "2026_MarsYard_3D_Model-20260812T165935Z-1-001",
    "2026_MarsYard_3D_Model", "Coordinates_MarsYard2026.txt",
)
MODELS_DIR = os.path.join(REPO_ROOT, "rover_description", "models")
WORLD_FILE = os.path.join(REPO_ROOT, "rover_description", "worlds", "marsyard.sdf")
FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

COLLISION_MESH_FILE = os.path.join(
    REPO_ROOT, "rover_description", "models", "marsyard_mesh", "marsyard2026_collision.obj",
)
# The exact geometry Gazebo actually renders/collides with -- more
# authoritative for "where's the ground here" than either the survey's own
# "H (local)" column or the separately-rasterized heightmap.tif, both of
# which disagree with it by several cm at some points (see module docstring).

GROUND_SAMPLE_RADIUS_M = 0.20
# This terrain is rocky/rough at small scale -- e.g. near L1, mesh vertices
# within 20cm of each other span a good 0.1-0.2m in Z. Sampling only the
# single nearest vertex can land in a local dip next to a taller rock or
# ridge, so the post's base ends up below that neighbor and the sign visibly
# clips into the terrain. Taking the MAX height within this radius (roughly
# the sign's own footprint) instead means the post always plants at or above
# everything immediately around it.

GROUND_SAMPLE_MARGIN_M = 0.02
# Small extra clearance above that local max, so the post's base doesn't
# just barely graze the highest nearby vertex.

AUTO_BEGIN = "    <!-- AUTO-GENERATED ARUCO LANDMARKS BEGIN (tools/generate_aruco_markers.py) -->"
AUTO_END = "    <!-- AUTO-GENERATED ARUCO LANDMARKS END -->"

# Derived, ground-referenced layout (see BOARD_TOP_HEIGHT_M / MARKER_TOP_HEIGHT_M above)
POST_HEIGHT_M = BOARD_TOP_HEIGHT_M - BOARD_HEIGHT_M  # ground to bottom of board
_MARKER_TOP_OFFSET_M = BOARD_TOP_HEIGHT_M - MARKER_TOP_HEIGHT_M  # board top down to marker top


def load_landmark_points(path):
    """Parse Coordinates_MarsYard2026.txt, keep only the 'L' (Landmark) group.
    File columns are (Name, Y, X, H)."""
    points = []
    with open(path, encoding="utf-8", errors="ignore") as f:
        for line in f:
            parts = [p.strip() for p in line.strip().split("\t") if p.strip() != ""]
            if len(parts) < 4:
                continue
            name = parts[0]
            if not name.startswith("L") or name.lower().startswith("point"):
                continue
            try:
                y, x, h = (float(v.replace(",", ".")) for v in parts[1:4])
            except ValueError:
                continue
            points.append((name, x, y, h))
    points.sort(key=lambda p: int(p[0][1:]))
    return points


def render_sign_face(sign_number, marker_id, out_path):
    """Full sign face: tan board, white inset, number plate, ArUco code --
    laid out per the spec's ground-referenced heights, not eyeballed."""
    mm = PX_PER_MM
    W, H = int(BOARD_WIDTH_M * 1000 * mm), int(BOARD_HEIGHT_M * 1000 * mm)
    img = Image.new("RGB", (W, H), (194, 178, 149))  # tan board
    draw = ImageDraw.Draw(img)

    inset_margin = int(0.020 * 1000 * mm)
    draw.rectangle([inset_margin, inset_margin, W - inset_margin, H - inset_margin], fill=(255, 255, 255))

    # Number plate: top of inset, NUMBER_LABEL_HEIGHT_M tall, outlined.
    label_top = inset_margin
    label_bottom = int(label_top + NUMBER_LABEL_HEIGHT_M * 1000 * mm)
    label_w = int(0.150 * 1000 * mm)
    label_left = (W - label_w) // 2
    label_right = label_left + label_w
    draw.rectangle([label_left, label_top, label_right, label_bottom], outline=(0, 0, 0), width=max(1, mm))
    font_size = int((label_bottom - label_top) * 0.75)
    font = ImageFont.truetype(FONT_PATH, font_size)
    text = str(sign_number)
    tb = draw.textbbox((0, 0), text, font=font)
    tw, th = tb[2] - tb[0], tb[3] - tb[1]
    draw.text(
        (label_left + (label_w - tw) / 2 - tb[0], label_top + ((label_bottom - label_top) - th) / 2 - tb[1]),
        text, fill=(0, 0, 0), font=font,
    )

    # ArUco code: top edge at MARKER_TOP_HEIGHT_M above ground -> _MARKER_TOP_OFFSET_M below board top.
    dictionary = aruco.Dictionary_get(ARUCO_DICT)
    marker_px = int(MARKER_SIZE_M * 1000 * mm)
    marker_img = aruco.drawMarker(dictionary, marker_id, marker_px)
    marker_top = int(_MARKER_TOP_OFFSET_M * 1000 * mm)
    marker_left = (W - marker_px) // 2
    img.paste(Image.fromarray(marker_img).convert("RGB"), (marker_left, marker_top))

    img.save(out_path)


_mesh_verts = None
_mesh_tree = None


def _load_collision_mesh():
    global _mesh_verts, _mesh_tree
    if _mesh_tree is not None:
        return
    verts = []
    with open(COLLISION_MESH_FILE) as f:
        for line in f:
            if line.startswith("v "):
                _, x, y, z = line.split()[:4]
                verts.append((float(x), float(y), float(z)))
    _mesh_verts = np.array(verts)
    _mesh_tree = cKDTree(_mesh_verts[:, :2])


def sample_ground_height(x, y):
    """Highest actual terrain-mesh vertex within GROUND_SAMPLE_RADIUS_M of
    (x, y), plus a small margin -- see GROUND_SAMPLE_RADIUS_M comment above
    for why max-in-radius instead of nearest-point."""
    _load_collision_mesh()
    idxs = _mesh_tree.query_ball_point([x, y], r=GROUND_SAMPLE_RADIUS_M)
    if not idxs:
        _, idx = _mesh_tree.query([x, y])
        idxs = [idx]
    return float(_mesh_verts[idxs, 2].max()) + GROUND_SAMPLE_MARGIN_M


def write_model(sign_number, marker_id, landmark_name):
    model_name = f"aruco_marker_{marker_id}"
    model_dir = os.path.join(MODELS_DIR, model_name)
    texture_dir = os.path.join(model_dir, "materials", "textures")
    os.makedirs(texture_dir, exist_ok=True)

    # Unique filename per marker -- Gazebo/Ogre2 caches textures by filename,
    # so reusing "sign.png" across every model directory collapses them all
    # onto whichever one loaded first (this was the "every sign shows 1" bug).
    texture_name = f"sign_{marker_id}.png"
    render_sign_face(sign_number, marker_id, os.path.join(texture_dir, texture_name))

    with open(os.path.join(model_dir, "model.config"), "w") as f:
        f.write(f"""<?xml version="1.0"?>
<model>
  <name>ArUco Marker {marker_id} ({landmark_name})</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <description>
    Auto-generated ArUco landmark sign, printed number {sign_number}, encoded
    ID {marker_id} (DICT_5X5_100, ERC "5x5 library, numbers 51-65" spec), at
    surveyed landmark point {landmark_name} from Coordinates_MarsYard2026.txt.
    4-sided box, same sign texture on all 4 vertical faces so it's readable
    from any approach direction. Used for vision-based EKF localization
    corrections. Regenerate via tools/generate_aruco_markers.py.
  </description>
</model>
""")

    with open(os.path.join(model_dir, "model.sdf"), "w") as f:
        f.write(f"""<sdf version="1.10">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <visual name="post_visual">
        <pose>0 0 {POST_HEIGHT_M / 2} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{POST_RADIUS_M}</radius>
            <length>{POST_HEIGHT_M}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
        </material>
      </visual>
      <collision name="post_collision">
        <pose>0 0 {POST_HEIGHT_M / 2} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{POST_RADIUS_M}</radius>
            <length>{POST_HEIGHT_M}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="sign_visual">
        <pose>0 0 {POST_HEIGHT_M + BOARD_HEIGHT_M / 2} 0 0 0</pose>
        <geometry>
          <box>
            <size>{BOARD_WIDTH_M} {BOARD_WIDTH_M} {BOARD_HEIGHT_M}</size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <pbr>
            <metal>
              <albedo_map>materials/textures/{texture_name}</albedo_map>
              <roughness>0.8</roughness>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
        </material>
      </visual>
      <collision name="sign_collision">
        <pose>0 0 {POST_HEIGHT_M + BOARD_HEIGHT_M / 2} 0 0 0</pose>
        <geometry>
          <box>
            <size>{BOARD_WIDTH_M} {BOARD_WIDTH_M} {BOARD_HEIGHT_M}</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
""")

    return model_name


def build_include(model_name, landmark_name, x, y, ground_z):
    # Sign is omnidirectional (same graphic on all 4 sides) so yaw is irrelevant -- left at 0.
    return (
        f"    <include>\n"
        f"      <name>{model_name}_{landmark_name}</name>\n"
        f"      <uri>model://{model_name}</uri>\n"
        f"      <pose>{x:.3f} {y:.3f} {ground_z:.3f} 0 0 0</pose>\n"
        f"    </include>"
    )


def patch_world_file(includes_xml):
    with open(WORLD_FILE) as f:
        world = f.read()

    block = f"{AUTO_BEGIN}\n{includes_xml}\n{AUTO_END}"

    if AUTO_BEGIN in world and AUTO_END in world:
        start = world.index(AUTO_BEGIN)
        end = world.index(AUTO_END) + len(AUTO_END)
        world = world[:start] + block + world[end:]
    else:
        marker = "  </world>"
        idx = world.rindex(marker)
        world = world[:idx] + block + "\n\n" + world[idx:]

    with open(WORLD_FILE, "w") as f:
        f.write(world)


def main():
    points = load_landmark_points(COORDS_FILE)
    print(f"{len(points)} landmark points loaded from {COORDS_FILE}")
    print(f"post height {POST_HEIGHT_M * 1000:.0f}mm, marker top {MARKER_TOP_HEIGHT_M * 1000:.0f}mm, "
          f"board top {BOARD_TOP_HEIGHT_M * 1000:.0f}mm above ground\n")

    includes = []
    for name, x, y, survey_h in points:
        sign_number = int(name[1:])
        marker_id = ID_OFFSET + sign_number
        model_name = write_model(sign_number, marker_id, name)
        ground_z = sample_ground_height(x, y)
        includes.append(build_include(model_name, name, x, y, ground_z))
        print(f"  {name:<4} sign={sign_number:<3} id={marker_id:<3} -> {model_name}  "
              f"world=({x:6.2f},{y:6.2f})  ground_z={ground_z:6.3f} (survey_h={survey_h:6.3f})")

    patch_world_file("\n".join(includes))
    print(f"\nwrote {len(points)} marker models under {MODELS_DIR}")
    print(f"patched {WORLD_FILE}")


if __name__ == "__main__":
    main()
