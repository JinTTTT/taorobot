# Media

This folder holds images and GIFs used in the documentation.

## Current files

| File | Shows | Used in |
|------|-------|---------|
| `graph_pose_slam.gif` | Graph-pose SLAM mapping the maze with loop-closure correction (RViz + Gazebo side by side) | Demo 1 in the main `README.md` |
| `localization.gif` | Particle filter localizing, then recovering after the robot is kidnapped | Demo 2 in the main `README.md` |
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
  the desktop bars, resize to 640 px wide, posterize colors (`(f // 16) * 16`)
  so static regions compress, then `imageio.mimsave(..., palettesize=128,
  subrectangles=True)`.
