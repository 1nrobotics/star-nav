# Project page

`index.html` is a self-contained static page (no build step, no external
scripts). `.github/workflows/pages.yml` publishes this folder to GitHub Pages
on every push to `main` that touches it; enable it once under
Settings → Pages → Source: GitHub Actions.

Preview locally:

```bash
python3 -m http.server -d site 8000   # then open http://localhost:8000
```

## Demo video

`assets/exploration-demo.mp4` is not tracked in git, to keep the repository
small (`.gitignore`). It is attached to the `site-assets` GitHub release, and
the Pages workflow downloads it at deploy time. To preview locally, download it
into `assets/`; without it the page shows the poster image
`assets/exploration-demo.jpg`.

```bash
gh release download site-assets --pattern '*.mp4' --dir site/assets
```

To publish a new video, replace the release asset:

```bash
gh release create site-assets --prerelease --title "Project page assets" \
  --notes "Media for the project page; not a software release."   # first time only
gh release upload site-assets site/assets/exploration-demo.mp4 --clobber
```

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
