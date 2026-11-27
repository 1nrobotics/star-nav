# Project page

Four static pages with no build step, sharing `style.css`. The walking
robots come first, then the quadrotor:

| Page | What |
| --- | --- |
| `index.html` | Home: the three robots, the live MuJoCo scene with a robot switcher, a results summary, the architecture, quick start |
| `quadruped.html` | The Lite3 quadruped: videos, live scene, results and key facts |
| `humanoid.html` | The Booster T1 humanoid: videos, live scene, results and key facts |
| `quadrotor.html` | The quadrotor: videos, live scene, results and key facts |

The only external scripts are those of the live MuJoCo scene (below), loaded
from jsDelivr when a visitor scrolls to it. `.github/workflows/pages.yml`
publishes this folder to GitHub Pages on every push to `main` that touches it;
enable it once under Settings → Pages → Source: GitHub Actions.

Preview locally:

```bash
python3 -m http.server -d site 8000   # then open http://localhost:8000
```

## Live MuJoCo scene

Every page has a "Live in your browser" section. It runs MuJoCo 4.0.1
compiled to WebAssembly (`@mujoco/mujoco`, Apache-2.0, ~11 MB) and draws it
with three.js 0.170 (MIT), both from jsDelivr. No planner runs in the browser:
each robot follows what its planner produced in one recorded mission, through
the simulator's controller and physics.

| Robot | Follows | Controller in the browser |
| --- | --- | --- |
| Lite3, T1 | The path walked in one mission with the CMU planner (`<robot>-path.json`), as a moving position setpoint | `walk.js`: the setpoint tracker (`tracking.py`), the manufacturer's walking policy and the joint PD of `robots/lite3.py`, `robots/t1.py` |
| Quadrotor | The `pos_cmd` setpoints EGO-Planner sent in one mission (`setpoints.json`) | `flight.js`: the SE(3) controller of `quadrotor.py` |

With **Drive**, a visitor's keys (or the buttons on the scene) give a walking
robot's policy its velocity command in place of the path. Nothing avoids
obstacles then. A robot that falls starts its mission again.

Files in `assets/mujoco/`:

| File | What |
| --- | --- |
| `embed.js` | Starts the scene of a page's `<div class="live" data-live="lite3 \| t1 \| quadrotor">` when it is near; buttons with `data-robot` switch the robot |
| `live.js` | three.js rendering of the MuJoCo geoms (meshes too), mouse orbit, buttons, Drive |
| `walk.js` | The walking robots: policy (ONNX or TorchScript weights, read in JavaScript), PD, path following; no rendering |
| `flight.js` | The quadrotor: JavaScript port of `quadrotor.py` (same parameters and SE(3) controller) and the replay loop; no rendering |
| `lite3.xml`, `t1.xml` | MJCF of the house (each robot's wall height), with the robot's upstream MJCF attached unchanged |
| `lite3.json`, `t1.json` | The robot's constants from `robots/*.py` (gains, standing pose, tracker), and where its upstream files and policy are |
| `lite3-path.json`, `t1-path.json` | The path of one recorded mission (20 Hz, without the body's sway) |
| `forest.xml` | MJCF of the default world (seed 1) around the exploration area, with the quadrotor |
| `setpoints.json` | The `pos_cmd` setpoints of one recorded MuJoCo mission (25 Hz) |

**Upstream files are not in this repository.** The Lite3's and the T1's MJCF,
meshes and walking policy are DeepRobotics' (BSD-3-Clause) and Booster
Robotics' (Apache-2.0). The page downloads them when the scene starts, from
jsDelivr's mirror of their GitHub repositories at the commits the simulator
pins (`robots/lite3.py`, `robots/t1.py`), and checks the policy's SHA-256
against the simulator's.

To regenerate the scenes and the recorded missions after changing a world, a
robot's constants or a planner:

```bash
mujoco_sim/.venv/bin/python site/tools/export_web_scene.py site/assets/mujoco

roslaunch star_nav_bringup exploration_lite3.launch viewer:=false &   # wait for it to start
python3 site/tools/record_path.py site/assets/mujoco/lite3-path.json  # then stop roslaunch
roslaunch star_nav_bringup exploration_t1.launch viewer:=false &
python3 site/tools/record_path.py site/assets/mujoco/t1-path.json
roslaunch star_nav_bringup exploration_mujoco.launch viewer:=false &
python3 site/tools/record_setpoints.py site/assets/mujoco/setpoints.json
```

Record a mission that finished without a timeout or a fall (the launch's log
says so); the T1 falls on the steps in some.

`walk.js` and `flight.js` must stay equivalent to the simulator's Python.
What was checked:

- `flight.js`: its controller against the Python one on random states
  (thrusts equal to 1e-14 N).
- `walk.js`: its policy against the simulator's on random observations
  (largest difference 1e-5), and each robot's whole mission in MuJoCo's
  WebAssembly build under Node, which must end without a fall and within
  0.3 m of the path's end:

  ```bash
  npm install --prefix /tmp/mujoco-web @mujoco/mujoco@4.0.1
  node site/tools/check_walk.mjs /tmp/mujoco-web             # downloads the robots from jsDelivr
  STAR_NAV_ASSETS=~/.cache/star_nav node site/tools/check_walk.mjs /tmp/mujoco-web   # or uses the simulator's copies
  ```

  Measured 2026-11-27: Lite3 no fall, at most 0.18 m from the path; T1 no
  fall, at most 0.11 m. One run each: the T1's fall on the steps
  ([docs/TESTING.md](../docs/TESTING.md#failures-that-do-not-happen-every-time))
  can happen in the browser too.

## Demo videos

The videos (on the quadruped page `lite3-house-rviz.mp4`, `lite3-house.mp4`,
`lite3-forest-lidar.mp4`, `lite3-house-frontier.mp4`; on the humanoid page
`t1-house.mp4`; on the quadrotor page `exploration-demo.mp4`,
`lidar-exploration-demo.mp4`) are not tracked in git, to keep the
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
gh release upload site-assets site/assets/exploration-demo.mp4 site/assets/lidar-exploration-demo.mp4 --clobber
```

The walking robots' videos show the CMU planner (recorded November 2026).
`lite3-house-rviz.mp4` is made like the quadrotor video below, with
`exploration_lite3.launch use_rviz:=true` (its RViz window is 1600 × 900; the
3D view is crop 1000:810:585:60) and sped up 4× (`setpts=PTS/4` in the
filter; crf 29). The other two quadruped videos are the simulator's follow
camera alone (`--record` in `sim_args`), sped up 4× (crf 27 to 28). Posters:
`assets/lite3-*.jpg`.
`t1-house.mp4` is the follow camera of one `exploration_t1.launch` mission
(`sim_args:="--record ..."`), sped up 4× and scaled to 960 × 540 (crf 30);
poster `assets/t1-house.jpg`. Each starts 2 s before the trigger and ends
4 s after the mission's `FINISH`.
Pass `-draw_mouse 0` to ffmpeg's x11grab to keep the pointer out.

The frontier exploration video (`lite3-house-frontier.mp4`) is the
simulator's follow camera with the explorer's map in the corner: what has
been seen, the frontiers, the path so far, the robot and the frontier it is
going to. `tools/record_map.py` keeps these from the ROS topics during the
run, and `tools/map_inset.py` draws them over the simulator's recording.
Both are on the simulator's clock, so nothing has to be aligned. They need
numpy, Pillow and ffmpeg:

```bash
roslaunch star_nav_bringup exploration_lite3.launch explore:=frontier \
  viewer:=false sim_args:="--record /tmp/lite3-frontier.mp4" &    # wait for it to start
python3 site/tools/record_map.py /tmp/lite3-frontier.npz          # triggers the run, returns at FINISH; then stop roslaunch
python3 site/tools/map_inset.py /tmp/lite3-frontier.mp4 /tmp/lite3-frontier.npz \
  site/assets/lite3-house-frontier.mp4 --speed 2
ffmpeg -ss 25 -i site/assets/lite3-house-frontier.mp4 -frames:v 1 -q:v 4 site/assets/lite3-house-frontier.jpg
```

The tools take a run of the T1 (`exploration_t1.launch`) or of the
quadrotor (`exploration_mujoco.launch`) in the same way; the site shows the
Lite3's only. The video starts 2 s before the trigger and ends 2 s after `FINISH`.
`map_inset.py --frame T` writes the inset alone at simulator time `T`, to
look at it without encoding a video.

The quadrotor video puts the simulator's follow camera next to the RViz window for one mission
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
