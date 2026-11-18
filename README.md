# KISS-ICP Visualizer — 2D LiDAR Odometry Contrast

Interactive 2D demo of LiDAR odometry. Three methods run on the **same** robot, the **same** 12×12 m map, and the **same** laser scans, so only the method differs:

- **Scan-to-scan ICP** (orange): point-to-point ICP between consecutive scans. No map, no robust kernel, no motion prediction. Drift accumulates step after step.
- **Scan-to-map ICP** (purple): point-to-point ICP against a growing global map, voxel-downsampled at a fixed 0.25 m. No robust kernel, no motion prediction.
- **KISS-ICP** (blue): point-to-point ICP against a bounded *local* map (points within 8 m of the estimate), with a Geman–McClure robust kernel, adaptive voxel size, and a constant-velocity motion prediction as the ICP initial guess.

## What KISS-ICP is

KISS-ICP is a LiDAR odometry system by Vizzo et al. (University of Bonn): *"KISS-ICP: In Defense of Point-to-Point ICP — Simple, Accurate, and Robust Registration If Done the Right Way"*, IEEE Robotics and Automation Letters, 2023 (DOI: 10.1109/LRA.2023.3236571; arXiv:2209.15397; code: https://github.com/PRBonn/kiss-icp). Its point is that point-to-point ICP — the simplest registration formulation, no features, no IMU, no loop closure — beats far more complex pipelines when done right: adaptive voxelization, a robust kernel, and motion compensation. This page is a 2D teaching simplification of those ideas, not the real 3D system.

## What to watch

- **Trajectories**: the orange scan-to-scan path drifts away from the true (green) path within one loop; the purple and blue paths stay close, and the blue one stays closest.
- **Outliers**: raise the outlier slider to 15–20%. The purple estimate degrades visibly while the blue one holds — that is the robust kernel at work.
- **Scoreboard**: live ATE (mean position error), compute time per frame, and map size. KISS-ICP's map stays bounded (local map + adaptive voxel) while the global scan-to-map map keeps growing.
- **Voxel size**: watch the KISS-ICP line — the voxel size adapts up and down to keep the local map near its target size.

## Verified behavior

Seeded simulation harness (8 seeds, 150 steps each):

- Default settings (noise σ = 0.05 m, 5% outliers): KISS-ICP ATE 0.23–0.52 m, vs scan-to-scan 2.85–3.44 m and scan-to-map 0.38–2.53 m. KISS-ICP had the lowest ATE on 8/8 seeds.
- 15% outliers: KISS-ICP ATE 0.26–0.55 m vs scan-to-map 2.75–3.44 m. KISS-ICP won 8/8 seeds.

## Notes and simplifications

- 2D simulation: a 72-ray 360° laser (adjustable), closed-form 2D Procrustes solve per ICP iteration, brute-force nearest neighbors, max correspondence distance 1.0 m, up to 15 iterations.
- All three methods start from the true initial pose: odometry measures relative motion, so trajectories are aligned at the start, as in standard evaluations.
- The robust kernel scale follows the noise slider (κ = max(0.3, 3σ)): the kernel must stay above the sensor noise floor or it downweights good correspondences too.
- The motion prediction is guarded: if an ICP step jumps more than 0.5 m or 0.5 rad, the prediction resets to identity instead of propagating a bad delta.
- The voxel adaptation band (350–700 points), the 8 m local-map radius, and the kernel scale are demo tunings for this 2D world, not the paper's 3D values.

## Run it

Open `index.html` in a browser (or enable GitHub Pages on this repo). **Auto** runs the simulation, **Step** advances one scan, **Reset** restarts. Sliders adjust LiDAR noise, outlier fraction, and ray count live.
