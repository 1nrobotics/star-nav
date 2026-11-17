# Project page

`index.html` is a static page with no build step. The only external scripts
are those of the live MuJoCo scene (below), loaded from jsDelivr when a visitor
scrolls to it. `.github/workflows/pages.yml` publishes this folder to GitHub Pages
on every push to `main` that touches it; enable it once under
Settings → Pages → Source: GitHub Actions.

Preview locally:

```bash
python3 -m http.server -d site 8000   # then open http://localhost:8000
```

## Live MuJoCo scene

The "Live MuJoCo in your browser" section runs MuJoCo 4.0.1 compiled to
WebAssembly (`@mujoco/mujoco`, Apache-2.0, ~11 MB) and draws it with three.js
0.170 (MIT), both from jsDelivr. Files in `assets/mujoco/`:

| File | What |
| --- | --- |
| `forest.xml` | MJCF of the default world (seed 1) around the exploration area, with the quadrotor |
| `setpoints.json` | The `pos_cmd` setpoints of one recorded MuJoCo mission (25 Hz) |
| `flight.js` | JavaScript port of `quadrotor.py` (same parameters and SE(3) controller) and the replay loop; no rendering |
| `live.js` | three.js rendering of the MuJoCo geoms, mouse orbit, buttons |

The planner does not run in the browser: the drone replays the setpoints the
planner produced, through the real controller and physics. To regenerate the
two data files after changing the world or the planner:

```bash
mujoco_sim/.venv/bin/python site/tools/export_web_scene.py site/assets/mujoco/forest.xml
roslaunch star_nav_bringup exploration_mujoco.launch viewer:=false &   # wait for it to start
python3 site/tools/record_setpoints.py site/assets/mujoco/setpoints.json
```

`flight.js` must stay equivalent to `quadrotor.py`; its controller was checked
against the Python one on random states (thrusts equal to 1e-14 N).

## Demo videos

The videos (`exploration-demo.mp4`, `lidar-exploration-demo.mp4`,
`microduck-exploration-demo.mp4`) are not tracked in git, to keep the
repository small (`.gitignore`). They are attached to the `site-assets` GitHub
release, and the Pages workflow downloads them at deploy time. To preview
locally, download them into `assets/`; without them the page shows the poster
images (`assets/*.jpg`).

```bash
gh release download site-assets --pattern '*.mp4' --dir site/assets
```

To publish a new video, replace the release asset:

```bash
gh release create site-assets --prerelease --title "Project page assets" \
  --notes "Media for the project page; not a software release."   # first time only
gh release upload site-assets site/assets/exploration-demo.mp4 --clobber
```

The Microduck video was made the same way with
`exploration_microduck.launch use_rviz:=true` (RViz window `"microduck.rviz`),
played at 4x (`setpts=PTS/4`) because the duck walks a mission in ~6.5 minutes.

It puts the simulator's follow camera next to the RViz window for one mission
(seed 1, default exploration area). To regenerate it:

1. Record the simulator and the RViz window during one mission:

   ```bash
   roslaunch star_nav_bringup exploration_mujoco.launch use_rviz:=true \
     viewer:=false sim_args:="--record /tmp/mujoco.mp4"
   # in another terminal, once RViz is up:
   W=$(xwininfo -root -tree | grep '"star_nav.rviz' | awk '{print $1}')
   ffmpeg -f x11grab -framerate 30 -window_id "$W" -i "$DISPLAY" \
     -vf "crop=trunc(iw/2)*2:trunc(ih/2)*2" -c:v libx264 -pix_fmt yuv420p /tmp/rviz.mp4
   # trigger exploration, wait for FINISH, then stop ffmpeg and roslaunch (Ctrl+C)
   ```

2. Align the two clips on the trigger time (the simulator video starts at
   simulation time 0, the RViz one when ffmpeg starts), crop the RViz 3D view
   and stack them; keep the result under ~5 MB:

   ```bash
   ffmpeg -ss <sim_offset> -t 60 -i /tmp/mujoco.mp4 -ss <rviz_offset> -t 60 -i /tmp/rviz.mp4 \
     -filter_complex "[0]scale=960:540,setsar=1[a];[1]crop=<w>:<h>:<x>:<y>,scale=-2:540,setsar=1[b];[a][b]hstack,fps=30,format=yuv420p" \
     -c:v libx264 -preset slow -crf 27 -movflags +faststart -an site/assets/exploration-demo.mp4
   ffmpeg -ss 30 -i site/assets/exploration-demo.mp4 -frames:v 1 -q:v 4 site/assets/exploration-demo.jpg
   ```
