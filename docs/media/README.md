# Media

This folder holds images and GIFs used in the documentation.

## Current files

| File | Shows | Used in |
|------|-------|---------|
| `graph_pose_slam.gif` | Graph-pose SLAM mapping the maze with loop-closure correction (RViz + Gazebo side by side) | Demo 1 in the main `README.md` |
| `localization.gif` | Particle filter localizing, then recovering after wheel odometry is corrupted (robot driven against a wall) | Demo 2 in the main `README.md` |
| `navigation.gif` | Goal sent via 2D Goal Pose; robot plans a path and drives it through the maze | Demo 3 in the main `README.md` |

The `.mp4` source recordings are kept locally only (gitignored) — GitHub does
not play repo-committed videos inline in a README, and they would bloat the
repository.

## Recording tips

- Capture both the Gazebo window and RViz (path + map) so viewers see cause and effect.
- Keep it short: 5–15 seconds of content, looping cleanly — and stop the
  recorder when the demo ends (no blank tail).
- Convert to GIF before committing, target well under 10 MB so GitHub renders
  it. The conversion used for the current GIFs (OpenCV + imageio): trim, crop
  the desktop bars, resize to ~900–1080 px wide (the largest that stays under
  10 MB — GitHub's README column is narrower anyway, so this looks identical
  to full resolution), posterize colors (`(f // 16) * 16`) so static regions
  compress, then `imageio.mimsave(..., palettesize=128, subrectangles=True)`.
- To highlight key moments (e.g. "set initial pose", "send goal"), insert a
  2–3 s freeze: repeat the frame `hold_sec * fps` times with a dark caption
  bar drawn at the bottom (`cv2.rectangle` + `cv2.putText`). Repeated frames
  cost almost nothing thanks to `subrectangles=True` — `navigation.gif` has
  three pauses plus an end hold and is still ~2.3 MB.
