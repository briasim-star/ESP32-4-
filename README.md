# ESP32-4- (map of this repository)

One repository, several small ESP32 projects. This page says what is where.

| Folder | What it is | Built by |
|---|---|---|
| `PEMF_Controller/` | Firmware for the 4-inch touch-screen wellness controller (ESP32, E32R40T board). Current version 2.0.x. | GitHub Actions (`.github/workflows/build_and_publish.yml`) |
| `docs/` | The published web installer (GitHub Pages): `install.html`, `manifest.json`, and the built firmware files in `firmware/`. Updated automatically by the build. Do not edit by hand. | automatic |
| `extras/` | Helper files, e.g. the TFT_eSPI display setup. | by hand |
| `.claude/skills/` | Notes the AI helper reads (ESP32 firmware + 3D printing workflow). | by hand |
| `.github/workflows/` | The recipe that builds the firmware and publishes it. | by hand |

## How a firmware change reaches the device
1. Edit/upload the source in `PEMF_Controller/`.
2. The Actions workflow builds it and commits the result into `docs/`.
3. The web installer in `docs/install.html` flashes the device over USB.

## Branches
- `main` = the real, current code.
- `claude/pmf-g-force-theory-b2afm0` and `copilot/fix-github-actions-build-job` = old helper branches. Kept, not deleted.

## License
Free software under the GNU General Public License v3.0 (see `LICENSE` and `NOTICE.md`). No warranty. Hobby and general-wellness use only; not a medical device.
