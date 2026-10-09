# Existing-file analysis

## Reused or adapted from `~/Desktop/fallrobotics`

- `mocap_test.py`, `mocap_coordinates.py`: NatNet acquisition, Y-up Motive to
  experiment `(X, -Z)` convention, raw quaternion heading, and tracking flags.
- `velocity_sequence.py`: ROS `Twist` publishing, fixed-rate repeats, segment
  start timestamps, StopMove retries, and publication audit log.
- `main.py`: start mocap first, require recent valid tracking, retain leading and
  trailing mocap samples, orderly shutdown, and never overwrite a run folder.
- `data_process.py` and `reprocess_holonomic.py`: exact constant-command
  holonomic dynamics, measured per-run initial pose, timestamp interpolation,
  marker-to-body correction, tracking-gap rejection, and position-only `E`.
- `NatNetClient.py`, `DataDescriptions.py`, `MoCapData.py`: copied unchanged into
  `vendor/` because the mocap logger depends on this SDK.

The old `velocity_sequence.py` was not copied unchanged because its input path
is fixed to `velocity_commands.csv`; its `--commands-path` argument is an output
log path. Phase 1 requires a different generated CSV for every candidate.

## Reused or adapted from the uploaded HW1 planner

- `rrt.py`: seeded random generators, nearest-node expansion pattern,
  parent-chain recovery, goal-biased sampling, and conservative collision
  sampling.
- `occupancy_grid.py`: boundary checking and the idea of conservative obstacle
  inflation. The unknown/free/occupied sensing state is not needed here.
- `kinodynamic_rrt.py`: command/state data structures and command-sequence
  generation concept.

## Not directly reusable

- HW1 `config.py` conflicts by module name and assumes a fixed 10 m square map,
  fixed start/goal, lidar, and unicycle limits. Its relevant choices now live in
  the new project's single `config.py`.
- HW1 `rrt.py` performs simulated sensing and online replanning around physical
  obstacles. The protocol uses known virtual obstacles and plans once before the
  robot run.
- HW1 `kinodynamic_rrt.py` has no lateral velocity and uses durations shorter
  than the protocol's settled command segments. Its dynamics also omit the
  duration factor from translational distance.

## Still requires physical validation

- actual combined Go2 command-interface limits;
- robot-side receipt/acknowledgement, if available;
- measured yaw and marker translation offsets;
- positive yaw and lateral signs;
- Motive/host clock offset and capture latency;
- usable workspace boundaries and mocap dropout behavior;
- whether the chosen RRT settings yield enough candidates in the real workspace.
- safe autonomous-reset speeds/tolerances and an unobstructed physical return path.
