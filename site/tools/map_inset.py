#!/usr/bin/env python3
"""Puts the explored map into the corner of a frontier exploration's video.

    python3 site/tools/map_inset.py RUN.mp4 RUN.npz OUT.mp4

RUN.mp4 is the simulator's recording of the run (``--record``) and RUN.npz
what ``record_map.py`` kept of it. Both are on the simulator's clock: frame n
of the recording is at n / 30 s, and the inset shows the latest map, the path
walked or flown and the goal of that moment:

    white        seen, free          blue line    the path so far
    black        obstacle            red arrow    the robot
    grey         not seen yet        ring         the frontier it is going to
    orange       frontier cells

The inset's header gives the mapped area and the time since the trigger.
The output starts LEAD seconds before the exploration is triggered and ends
TAIL seconds after it finishes; ``--speed 2`` plays it twice as fast. Needs
numpy, Pillow and ffmpeg.
"""

import argparse
import math
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFont

FPS = 30
LEAD, TAIL = 2.0, 2.0  # s
WIDTH = 344  # px, of the map in the inset
HEADER = 34  # px
PAD = 8  # px around the map
MARGIN = 16  # px from the video's corner
SS = 2  # the inset is drawn SS times larger and scaled down (smooth lines)
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf"

UNKNOWN, FREE, OBSTACLE, FRONTIER = (74, 78, 86), (238, 240, 242), (22, 24, 28), (255, 150, 30)
PANEL, TEXT, DIM = (30, 33, 38, 235), (255, 255, 255), (176, 182, 192)
PATH, ROBOT, GOAL = (40, 110, 235), (225, 45, 45), (210, 60, 200)


def crop_of(grid, origin, res, area, margin=0.3):
    """Cells (i0, j0, i1, j1) of ``area`` (xmin, ymin, xmax, ymax, m); without
    it, of what the last map has seen, with a margin."""
    if area is None:
        jj, ii = np.where(grid != -1)
        area = (origin[0] + ii.min() * res - margin, origin[1] + jj.min() * res - margin,
                origin[0] + (ii.max() + 1) * res + margin, origin[1] + (jj.max() + 1) * res + margin)
    i0, j0 = (int(math.floor((area[k] - origin[k]) / res + 1e-6)) for k in (0, 1))
    i1, j1 = (int(math.ceil((area[k + 2] - origin[k]) / res - 1e-6)) for k in (0, 1))
    return max(i0, 0), max(j0, 0), min(i1, grid.shape[1]), min(j1, grid.shape[0])


class Inset:
    def __init__(self, run, title, area):
        self.run, self.title = run, title
        self.res = float(run["resolution"])
        self.origin = run["origin"]
        self.crop = crop_of(run["map"][-1], self.origin, self.res, area)
        i0, j0, i1, j1 = self.crop
        self.scale = SS * WIDTH / (i1 - i0)  # px per cell, drawn
        self.map_size = (SS * WIDTH, int(round((j1 - j0) * self.scale)))
        self.size = (WIDTH + 2 * PAD, HEADER + self.map_size[1] // SS + PAD)
        self.bold = ImageFont.truetype(FONT % "-Bold", 17 * SS)
        self.plain = ImageFont.truetype(FONT % "", 14 * SS)
        self.start, self.finish = float(run["start"]), float(run["finish"])
        self.cache = (None, None)

    def px(self, x, y):
        """World (m) to the drawn map's pixels: x to the right, y up."""
        i0, _, _, j1 = self.crop
        return ((x - self.origin[0]) / self.res - i0) * self.scale, \
               (j1 - (y - self.origin[1]) / self.res) * self.scale

    def map_image(self, k):
        if self.cache[0] != k:
            i0, j0, i1, j1 = self.crop
            rgb = np.empty((j1 - j0, i1 - i0, 3), np.uint8)
            rgb[:] = UNKNOWN
            if k >= 0:
                grid = self.run["map"][k][j0:j1, i0:i1]
                rgb[grid == 0] = FREE
                rgb[grid > 0] = OBSTACLE
                rgb[self.run["frontier"][k][j0:j1, i0:i1] & (grid == 0)] = FRONTIER
            image = Image.fromarray(rgb[::-1]).resize(self.map_size, Image.NEAREST)
            known = 0.0 if k < 0 else float((self.run["map"][k] != -1).sum()) * self.res ** 2
            self.cache = (k, (image, known))
        return self.cache[1]

    def frame(self, t):
        """The inset at simulator time ``t``, RGBA."""
        run = self.run
        k = int(np.searchsorted(run["map_t"], t, "right")) - 1
        base, known = self.map_image(k)
        image = base.copy()
        draw = ImageDraw.Draw(image)
        track = run["track"]
        n = int(np.searchsorted(track[:, 0], t, "right"))
        if n > 1:
            draw.line([self.px(x, y) for x, y in track[:n:2, 1:3]] + [self.px(*track[n - 1, 1:3])],
                      fill=PATH, width=2 * SS, joint="curve")
        g = int(np.searchsorted(run["goal"][:, 0], t, "right")) - 1
        if g >= 0 and t < self.finish:
            x, y = self.px(*run["goal"][g, 1:3])
            r = 5 * SS
            draw.ellipse([x - r, y - r, x + r, y + r], outline=GOAL, width=2 * SS)
        if n > 0:
            x, y = self.px(*track[n - 1, 1:3])
            yaw, r = track[n - 1, 3], 10 * SS
            # An arrow head pointing along the heading (the image's y is down)
            draw.polygon([(x + r * math.cos(yaw + a) * s, y - r * math.sin(yaw + a) * s)
                          for a, s in ((0.0, 1.0), (2.5, 0.85), (math.pi, 0.3), (-2.5, 0.85))],
                         fill=ROBOT, outline=(255, 255, 255))

        panel = Image.new("RGBA", (self.size[0] * SS, self.size[1] * SS), PANEL)
        panel.paste(image, (PAD * SS, HEADER * SS))
        draw = ImageDraw.Draw(panel)
        draw.text((PAD * SS, 7 * SS), self.title, font=self.bold, fill=TEXT)
        seconds = min(max(t, self.start), self.finish) - self.start
        text = "%.0f m²   %3.0f s" % (known, seconds)
        font = self.plain  # getlength: Pillow 8 and later
        width = font.getlength(text) if hasattr(font, "getlength") else font.getsize(text)[0]
        draw.text((panel.size[0] - PAD * SS - width, 10 * SS), text, font=self.plain, fill=DIM)
        return panel.resize(self.size, Image.LANCZOS)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("video")
    parser.add_argument("run", help="the .npz of record_map.py")
    parser.add_argument("out")
    parser.add_argument("--title", default="Explored map")
    parser.add_argument("--area", type=float, nargs=4, metavar=("XMIN", "YMIN", "XMAX", "YMAX"),
                        help="the part of the map to show (m); default: what was seen")
    parser.add_argument("--speed", type=float, default=1.0, help="play the output this many times faster")
    parser.add_argument("--crf", type=int, default=27, help="x264 quality of the output")
    parser.add_argument("--frame", type=float, metavar="T",
                        help="only write the inset at simulator time T (s) to OUT, a PNG")
    args = parser.parse_args()

    run = dict(np.load(args.run))
    inset = Inset(run, args.title, args.area)
    if args.frame is not None:
        inset.frame(args.frame).save(args.out)
        return

    first = max(0, int(round((inset.start - LEAD) * FPS)))
    last = int(round((inset.finish + TAIL) * FPS))
    width, height = inset.size
    ffmpeg = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error",
         "-ss", "%.4f" % (first / FPS), "-i", args.video,
         "-f", "rawvideo", "-pix_fmt", "rgba", "-s", "%dx%d" % (width, height),
         "-r", str(FPS), "-i", "-",
         "-filter_complex", "[0:v][1:v]overlay=W-w-%d:%d:shortest=1,setpts=PTS/%g,fps=%d"
         % (MARGIN, MARGIN, args.speed, FPS),
         "-c:v", "libx264", "-crf", str(args.crf), "-preset", "slow",
         "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-an", args.out],
        stdin=subprocess.PIPE)
    try:
        for n in range(first, last + 1):
            ffmpeg.stdin.write(inset.frame(n / FPS).tobytes())
    except BrokenPipeError:  # the recording ended before TAIL was over
        pass
    else:
        ffmpeg.stdin.close()
    if ffmpeg.wait():
        raise SystemExit("ffmpeg failed")
    print("wrote %s: %.1f s, exploration %.1f s" % (args.out, (last - first + 1) / FPS / args.speed,
                                                    inset.finish - inset.start))


if __name__ == "__main__":
    main()
