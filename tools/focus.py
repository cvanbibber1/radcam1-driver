#!/usr/bin/env python3
"""Live focus aid for the AR1335, usable over a plain SSH session.

Focusing by taking a picture, looking at it, nudging the lens and repeating is
slow and has no notion of "better" - it is easy to walk past the peak. This
gives a continuously updating number, remembers the best value seen, and tells
you whether the last move helped, so focusing becomes "turn until the number
stops rising".

Three ways to see what the camera sees, all over SSH:

    python3 tools/focus.py                    # meter only
    python3 tools/focus.py --preview          # + live ANSI image in the terminal
    python3 tools/focus.py --http 8080        # + MJPEG at http://<pi>:8080/

The metric is a normalised Tenengrad: mean squared Sobel gradient divided by
the region's variance. Dividing out the variance matters - a raw gradient score
rises when the scene simply gets brighter or more contrasty, which makes it
useless while AE is still settling. Normalised, it responds to sharpness alone.

Focus on detail, not a blank wall: the metric needs edges to work with. The
chart, printed text or a ruler all work well.
"""

import argparse
import io
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np
from PIL import Image

SOI = b"\xff\xd8"
EOI = b"\xff\xd9"

_latest_jpeg: bytes | None = None
_latest_lock = threading.Lock()


# ----------------------------------------------------------------- metric

def focus_score(gray: np.ndarray) -> float:
    """Normalised Tenengrad. Higher is sharper; exposure-independent."""
    g = gray.astype(np.float32)
    # Sobel, done by hand to avoid a scipy dependency.
    gx = (g[:-2, 2:] + 2 * g[1:-1, 2:] + g[2:, 2:]
          - g[:-2, :-2] - 2 * g[1:-1, :-2] - g[2:, :-2])
    gy = (g[2:, :-2] + 2 * g[2:, 1:-1] + g[2:, 2:]
          - g[:-2, :-2] - 2 * g[:-2, 1:-1] - g[:-2, 2:])
    energy = float((gx * gx + gy * gy).mean())
    var = float(g.var())
    if var < 1e-6:
        return 0.0
    return energy / var


# ---------------------------------------------------------------- preview

def draw_grid(im: Image.Image, rows: int = 3, cols: int = 6,
              inset: float = 0.1) -> Image.Image:
    """Overlay the 6x3 sampling grid the calibration tool will use.

    Numbers alone do not tell you *where* to put the chart. Drawing the grid
    the sampler expects turns framing into "line the patches up with the
    boxes", which is a much easier target to hit.
    """
    from PIL import ImageDraw

    im = im.convert("RGB").copy()
    d = ImageDraw.Draw(im)
    W, H = im.size
    x0, y0 = int(W * inset), int(H * inset)
    x1, y1 = int(W * (1 - inset)), int(H * (1 - inset))

    d.rectangle([x0, y0, x1, y1], outline=(255, 255, 0), width=3)
    for c in range(1, cols):
        x = x0 + (x1 - x0) * c // cols
        d.line([x, y0, x, y1], fill=(255, 255, 0), width=2)
    for r in range(1, rows):
        y = y0 + (y1 - y0) * r // rows
        d.line([x0, y, x1, y], fill=(255, 255, 0), width=2)
    return im


def ansi_preview(im: Image.Image, cols: int) -> str:
    """Render an image with half-block characters and 24-bit colour.

    Each character cell shows two vertically stacked pixels - foreground for
    the top, background for the bottom - so the vertical resolution is twice
    the number of text rows.
    """
    w, h = im.size
    rows = max(2, int(cols * h / w * 0.5)) * 2      # even number of pixel rows
    small = im.convert("RGB").resize((cols, rows))
    a = np.asarray(small)

    out = []
    for y in range(0, rows - 1, 2):
        line = []
        for x in range(cols):
            tr, tg, tb = a[y, x]
            br, bg, bb = a[y + 1, x]
            line.append(f"\x1b[38;2;{tr};{tg};{tb}m\x1b[48;2;{br};{bg};{bb}m▀")
        out.append("".join(line) + "\x1b[0m")
    return "\n".join(out)


# ------------------------------------------------------------- http mjpeg

class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path not in ("/", "/stream.mjpg"):
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=FRAME")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        try:
            while True:
                with _latest_lock:
                    frame = _latest_jpeg
                if frame:
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError):
            pass


def serve_http(port: int):
    srv = HTTPServer(("0.0.0.0", port), MJPEGHandler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


# ------------------------------------------------------------------ main

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--framerate", type=int, default=15)
    ap.add_argument("--roi", default="0.3,0.3,0.4,0.4",
                    help="measurement region x,y,w,h as fractions")
    ap.add_argument("--preview", action="store_true",
                    help="draw the image in the terminal")
    ap.add_argument("--preview-cols", type=int, default=0,
                    help="preview width in characters (0 = fit terminal)")
    ap.add_argument("--http", type=int, default=0, metavar="PORT")
    ap.add_argument("--crop", default=None,
                    help="sensor ROI to capture x,y,w,h (digital zoom)")
    ap.add_argument("--grid", action="store_true",
                    help="overlay the 6x3 chart sampling grid on the preview")
    args = ap.parse_args()

    rx, ry, rw, rh = (float(v) for v in args.roi.split(","))

    cmd = ["rpicam-vid", "-n", "-t", "0", "--codec", "mjpeg",
           "--width", str(args.width), "--height", str(args.height),
           "--framerate", str(args.framerate), "-o", "-"]
    if args.crop:
        cmd += ["--roi", args.crop]

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, bufsize=0)
    if args.http:
        serve_http(args.http)
        ip = subprocess.run(["hostname", "-I"], capture_output=True,
                            text=True).stdout.split()
        where = ip[0] if ip else "<pi>"
        print(f"MJPEG stream: http://{where}:{args.http}/\n")

    peak = 0.0
    hist: deque[float] = deque(maxlen=8)
    buf = bytearray()
    frames = 0
    t0 = time.monotonic()

    def cleanup(*_):
        proc.terminate()
        print("\x1b[?25h")          # show the cursor again
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    print("\x1b[?25l", end="")      # hide cursor

    try:
        while True:
            chunk = proc.stdout.read(65536)
            if not chunk:
                break
            buf += chunk

            # Pull out whole JPEGs; keep only the newest if we fall behind.
            while True:
                s = buf.find(SOI)
                e = buf.find(EOI, s + 2) if s >= 0 else -1
                if s < 0 or e < 0:
                    break
                jpeg = bytes(buf[s:e + 2])
                del buf[:e + 2]

                if args.grid:
                    buf2 = io.BytesIO()
                    draw_grid(Image.open(io.BytesIO(jpeg))).save(
                        buf2, format="JPEG", quality=80)
                    with _latest_lock:
                        globals()["_latest_jpeg"] = buf2.getvalue()
                else:
                    with _latest_lock:
                        globals()["_latest_jpeg"] = jpeg

                im = Image.open(io.BytesIO(jpeg))
                W, H = im.size
                box = (int(W * rx), int(H * ry),
                       int(W * (rx + rw)), int(H * (ry + rh)))
                gray = np.asarray(im.convert("L").crop(box))
                score = focus_score(gray)
                frames += 1

                hist.append(score)
                peak = max(peak, score)
                trend = ""
                if len(hist) >= 4:
                    older = sum(list(hist)[:len(hist) // 2]) / (len(hist) // 2)
                    newer = sum(list(hist)[len(hist) // 2:]) / (len(hist) - len(hist) // 2)
                    if newer > older * 1.03:
                        trend = "\x1b[32mimproving\x1b[0m"
                    elif newer < older * 0.97:
                        trend = "\x1b[31mgetting worse - turn back\x1b[0m"
                    else:
                        trend = "steady"

                pct = score / peak if peak else 0
                bar_w = 30
                filled = int(bar_w * pct)
                colour = "\x1b[32m" if pct > 0.95 else ("\x1b[33m" if pct > 0.7
                                                        else "\x1b[31m")
                bar = colour + "#" * filled + "\x1b[0m" + "-" * (bar_w - filled)
                fps = frames / max(time.monotonic() - t0, 1e-6)

                if args.preview:
                    cols = args.preview_cols or min(
                        shutil.get_terminal_size((80, 24)).columns - 2, 96)
                    shown = draw_grid(im) if args.grid else im
                    print("\x1b[H\x1b[2J", end="")
                    print(ansi_preview(shown, cols))
                    print()
                    print(f"focus {score:9.1f}  [{bar}] {pct*100:5.1f}% of peak "
                          f"{peak:9.1f}   {trend}   {fps:4.1f} fps")
                    print("turn the lens until the number stops rising, "
                          "then back off to the peak.  Ctrl-C to stop")
                else:
                    print(f"\rfocus {score:9.1f}  [{bar}] {pct*100:5.1f}% of "
                          f"peak {peak:9.1f}   {trend}   {fps:4.1f} fps   ",
                          end="", flush=True)
    finally:
        proc.terminate()
        print("\x1b[?25h")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
