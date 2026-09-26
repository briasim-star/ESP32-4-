---
name: esp32-maker-studio
description: ESP32 firmware and 3D printing/rendering for a hands-on maker (MADD PEMF devices, LCDWiki ESP32 display boards, Creality K2 Pro printer). Use this skill whenever the user wants ESP32/Arduino code written, fixed or compiled; sends or asks for any 3D model (3MF, STL, OBJ) to scale, remix, fit or print; wants a realistic render or mock-up of a printed part or device; or mentions PEMF hardware, enclosures, lamp shades, glow/clear filament, Creality Print, or Blender — even if they don't name a tool.
---

# ESP32 + 3D Maker Studio

The user builds real hardware — ESP32 touch-screen controllers (the MADD PEMF line)
and 3D prints on a **Creality K2 Pro** (300 × 300 × 300 mm, 0.4 mm nozzle, textured
PEI). They often dictate by voice from an iPhone, so words get garbled ("my reality
printer" = Creality printer, "new neck" = Nanuk case). Read generously, keep
replies short, and don't make them wait on avoidable round trips — they have said
plainly that wasted time is the thing they mind most.

## Rule 1 — every 3D file is a K2 Pro Creality Print project, delivered as a link

Whenever the output is a 3D model, deliver a **`.3mf` Creality Print project
with the K2 Pro settings inside**, ready to open → Slice → Print. A bare STL
"didn't work" for them. Build it from the user's own Creality Print `.3mf`
(it carries their printer, plate, filament and support settings):

```
python3 scripts/make_k2pro_3mf.py ORIGINAL.3mf --scale 2.5 --walls 4 --out Name_K2Pro.3mf
```

The script drops the old slice (G-code) — a stale slice for a different size is
a real hazard. You can't slice here (slicer downloads are blocked), so the one
remaining step for them is: open in Creality Print → pick filament → Slice → Print.

**Delivery: they can't open chat file cards.** Commit the `.3mf` to `main` in
`briasim-star/ESP32-4-` and give the full clickable link:
`https://github.com/briasim-star/ESP32-4-/raw/main/<path>/<file>.3mf`
Check it with `curl -sSL -o /dev/null -w "%{http_code} %{size_download}"`
before sending. The repo is public, so mention that once for third-party models.

## Rule 2 — you're in a cloud container, not on their laptop

You can't open `C:\Users\...` paths or reach their phone. When they paste a
local path, reply in one line: drag the file into this chat, or upload it to
Google Drive (you can search/download Drive; `download_file_content` returns
base64 JSON — decode it). Don't re-explain if they resend the path.

## Rule 3 — best free tools, previewed before sending

- Use **Blender** (`pip install bpy==4.2.0`; blender.org is blocked, PyPI works)
  for modeling, measuring and Cycles renders. They banned OpenSCAD. `trimesh`
  is fine as a 3MF loader and for quick checks.
- Render cheap previews first (16–32 samples, 30–50 % res, ~10–20 s), look at
  them yourself, fix orientation/exposure, then do the final.

## Workflow: 3D model → print

1. Inspect with Blender (`bpy`) or `trimesh`: size, the slicer settings in
   `Metadata/project_settings.config`, wall thickness (ray-cast inward from
   faces), hollow or solid, and the max scale that fits 300 × 300 × 300 mm.
2. If the part must fit something real (a light fixture, a case, a bracket),
   ask once for the exact 2–3 measurements that matter, named concretely. Still
   deliver a best-guess `.3mf` now so they're never blocked.
3. Make the `.3mf` (Rule 1). Wall thickness scales with the model — set
   `--walls` ≈ new wall thickness ÷ 0.4 mm (min 2).
4. One short checklist: rough weight/time (scale² × original for shells),
   filament notes, fit caveats.

Material and heat notes: LED bulbs ≤ 8 W inside printed shades (PLA softens
~55–60 °C); PETG for clear/outdoor sun; glow-in-the-dark filament is abrasive
(hardened nozzle); thicker walls glow longer but pass less light.

## Workflow: photoreal render / mock-up

`ghost_cat/render_garage.py` in the repo is the worked example: Blender Cycles,
the part placed on a torch-post fixture against lap siding, glow-in-the-dark or
clear material with an inner LED, glow-falloff shader, fog-glow compositing,
AgX Punchy. Copy and adapt it, building the surroundings from the user's
photos with simple primitives. Args: `scale samples res% rotation out.png`.

## Workflow: ESP32 firmware

Board: LCDWiki 4.0" ESP32-32E (E32R40T) — ST7796 320×480 TFT (MISO 12, MOSI 13,
SCLK 14, CS 15, DC 2, BL 27), touch CS 33 / IRQ 36, SD CS 5 (18/19/23), battery
sense 34, MD10C driver PWM 25 / DIR 32 via the 4-pin "I2C" header. Onboard pins
are hard-wired — never reassign. Display config: `extras/TFT_eSPI_User_Setup.h`.

Build: arduino-cli, core `esp32:esp32@2.0.17`, FQBN
`esp32:esp32:esp32:PartitionScheme=huge_app` with the custom
`PEMF_Controller/partitions.csv` (two 0x1F0000 OTA slots). The binary must stay
≤ 2,031,616 bytes. Libraries: TFT_eSPI, WiFiManager, ESP32-A2DP (git),
ESP32CertBundle (git). The exact steps are in
`.github/workflows/build_and_publish.yml` — mirror them to compile locally
before pushing.

Release: commit to `main` with a version bump in the message (e.g. `v2.0.6: …`);
GitHub Actions compiles and auto-commits binaries to `docs/firmware/` and
`docs/manifest.json` for the web installer — never edit those by hand.

## Housekeeping

Save everything on `main` of `briasim-star/ESP32-4-` — that's where all their
work lives; don't leave it on side branches or in temp folders. Keep the raw
source meshes of third-party models (Creality Cloud, Printables…) out of git;
commit the print-ready `.3mf` the user asked for, scripts and renders.
