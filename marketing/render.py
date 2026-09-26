"""Render the MADD PEMF teaser carousel and Reel.

Usage:  python3 marketing/render.py
Needs:  pip install playwright imageio-ffmpeg
Drop device screenshots (png/jpg) into marketing/screenshots/ and re-run;
up to 3 are shown blurred/cropped as "classified" teasers.
"""
import base64
import mimetypes
import os
import subprocess
from pathlib import Path

import imageio_ffmpeg
from playwright.sync_api import sync_playwright

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"
FPS = 30


def data_url(p):
    mime = mimetypes.guess_type(p.name)[0] or "image/png"
    return f"data:{mime};base64," + base64.b64encode(p.read_bytes()).decode()


def main():
    OUT.mkdir(exist_ok=True)
    shots = sorted(p for p in (HERE / "screenshots").glob("*")
                   if p.suffix.lower() in (".png", ".jpg", ".jpeg", ".webp"))
    exe = "/opt/pw-browsers/chromium-1194/chrome-linux/chrome"
    with sync_playwright() as pw:
        browser = pw.chromium.launch(executable_path=exe if os.path.exists(exe) else None)
        page = browser.new_page(viewport={"width": 1080, "height": 1920})
        page.goto((HERE / "stage.html").as_uri())
        page.evaluate("ready")
        page.evaluate("urls => loadShots(urls)", [data_url(p) for p in shots])
        canvas = page.locator("#c")

        posts = ["post1()", "post2()"] + [f"postShot({i})" for i in range(len(shots))]
        for n, call in enumerate(posts, 1):
            page.evaluate(call)
            canvas.screenshot(path=str(OUT / f"post_{n}.png"))
            print("wrote", OUT / f"post_{n}.png")

        length = page.evaluate("reelLength()")
        frames = int(length * FPS)
        video = OUT / "teaser_reel.mp4"
        ff = subprocess.Popen(
            [imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error",
             "-f", "image2pipe", "-framerate", str(FPS), "-i", "-",
             # silent track so Instagram accepts it; add trending audio in-app
             "-f", "lavfi", "-i", "anullsrc=r=44100:cl=stereo", "-shortest",
             "-c:v", "libx264", "-pix_fmt", "yuv420p", "-profile:v", "high",
             "-crf", "18", "-c:a", "aac", "-movflags", "+faststart", str(video)],
            stdin=subprocess.PIPE)
        for f in range(frames):
            page.evaluate(f"reel({f / FPS})")
            ff.stdin.write(canvas.screenshot(type="png"))
        ff.stdin.close()
        ff.wait()
        browser.close()
        print(f"wrote {video} ({length:.1f}s, {frames} frames)")


if __name__ == "__main__":
    main()
