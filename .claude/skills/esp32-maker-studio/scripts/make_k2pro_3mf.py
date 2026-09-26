"""Scale a 3MF from Creality Print and re-save it as a K2 Pro project.

Usage:
  python3 make_k2pro_3mf.py ORIGINAL.3mf --scale 2.5 --out Name_K2Pro.3mf [--walls 4]

ORIGINAL must be a Creality Print .3mf (it carries the user's K2 Pro printer,
plate, filament and support settings). Every vertex is scaled about the
object origin, the build/assembly Z offsets are scaled so the part still sits
on the bed, and any old slice (G-code) is dropped because it was made for the
old size. The user then opens it in Creality Print -> Slice -> Print.
"""
import argparse
import json
import re
import zipfile

STALE = {"Metadata/slice_info.config"}


def scale_z_offset(text, s):
    # 12-number 3MF transforms: "m00 ... m22 tx ty tz"; only tz moves with scale
    def fix(m):
        nums = m.group(2).split()
        if len(nums) == 12:
            nums[11] = f"{float(nums[11]) * s:.7g}"
        return f'{m.group(1)}="{" ".join(nums)}"'
    return re.sub(r'\b(transform)="([^"]+)"', fix, text)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("original")
    ap.add_argument("--scale", type=float, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--walls", type=int, help="wall loops to set (default: keep)")
    a = ap.parse_args()
    s = a.scale
    num = lambda m: f'{m.group(1)}="{float(m.group(2)) * s:.6g}"'

    src = zipfile.ZipFile(a.original)
    out = zipfile.ZipFile(a.out, "w", zipfile.ZIP_DEFLATED)
    for info in src.infolist():
        n = info.filename
        if n in STALE or ".gcode" in n:
            continue
        data = src.read(n)
        if n.startswith("3D/") and n.endswith(".model"):
            t = data.decode()
            t = re.sub(r"<vertex [^>]*/>",
                       lambda v: re.sub(r'\b([xyz])="([-0-9.eE+]+)"', num, v.group(0)), t)
            data = scale_z_offset(t, s).encode()
        elif n == "Metadata/model_settings.config":
            data = scale_z_offset(data.decode(), s).encode()
        elif n == "Metadata/project_settings.config" and a.walls:
            d = json.loads(data)
            d["wall_loops"] = str(a.walls)
            data = json.dumps(d, indent=4).encode()
        out.writestr(info, data)
    out.close()
    print("wrote", a.out)


if __name__ == "__main__":
    main()
