# Holonomic hardware protocol

This folder implements the two-phase protocol in
`hardware_protocol_full_holonomic.md`. It separates:

- Phase 0 deviation pilots (`phase0.py`) and their model fit (`phase0analyze.py`)
- Phase 1 trial collection (`main.py`)
- offline Phase 1 deviation analysis (`analyze_phase1.py`)
- hardware-free model/CSV code (`core.py`)
- robot and mocap processes (`hardware/`)
- the full-holonomic kinodynamic RRT (`planner.py`)
- a mocap reader for measuring the map boundary (`robot_coordinate_getter.py`)

No experiment has been prepared or executed.

## Before using it

1. Edit every site-specific or experiment-specific value in `config.py`.
2. Keep `ALLOW_HARDWARE_EXECUTION = False` while inspecting and testing.
3. Start the existing Go2 driver separately. The new runner publishes to the
   configured `CMD_VEL_TOPIC`; it does not start or configure the driver.
4. Validate the Motive ground-plane convention, rigid-body ID, network addresses,
   clock synchronization, and ROS topic names in the lab.
5. Remember that `commands.csv` proves what this computer published. Without a
   robot-side acknowledgement it cannot prove what the robot received.

## Safe offline checks

```bash
cd ~/Desktop/holonomic_hardware_protocol
python3 -m unittest discover -s tests -v
python3 phase0.py
```

`phase0.py` without `--execute` only prints an example plan. It does not
connect to hardware and saves nothing.

## Map boundary

```bash
python3 robot_coordinate_getter.py
```

Stand the robot on each corner of the usable area and press Enter. After four
corners it prints the `WORKSPACE_*` lines for `config.py` (the largest
axis-aligned rectangle inside them). It only reads mocap.

## Yaw offset

```bash
python3 yaw_offset_calibration.py --execute
```

Drives one vx-only command (`YAW_CALIBRATION_VX_MPS` for
`YAW_CALIBRATION_DURATION_S`) and prints the measured `YAW_OFFSET_RAD`: the
direction of the travelled line minus the mean raw marker heading. Enter the
value in `config.py` yourself. Without `--execute` it only previews.

## Phase 0 workflow

Every call collects one pilot. Put the robot anywhere inside the boundary, at
least `GOAL_RADIUS_M` from the goal, and run:

```bash
python3 phase0.py --execute
```

It reads the robot's pose from mocap, draws random virtual obstacles, and
plans an RRT path from that pose to within `GOAL_RADIUS_M` of
(`GOAL_X_M`, `GOAL_Y_M`). The path only has to keep the body
center inside the workspace; no margin is kept at its edge. The resulting
velocity commands are run open-loop and saved as the next `pilot_NNN`. There is
no fixed number of pilots. A call that fails for any reason (no path, tracking
lost, Ctrl+C) leaves nothing behind, so the next call uses the same number.

When there are enough pilots:

```bash
python3 phase0analyze.py
```

For each of +vx, +vy, -vy, +omega and -omega the nominal model predicts
`y = gain * x + offset`, where `x` is the motion a command asks for in one
segment (command x dt, in the robot's frame). Two more terms come from the
Go2's turn-then-walk behaviour: an in-place turn that starts a turn begins
`MODEL_TURN_LAG_S` late from standstill and `MODEL_TURN_LAG_AFTER_WALK_S` late
right after walking, and walking forward yaws the robot left by
`MODEL_WALK_YAW_RAD_PER_M` per commanded metre. A seeded random search picks
the five gains, five offsets and these three terms that minimize `q` (the 90 %
quantile of E over the pilots) and then reports every pilot's E, `q` and `rho`
with them. The report
is saved as `data/phase0/phase0_analysis.txt` and includes the lines to enter
in `config.py`; `config.py` itself is not changed. Phase 1 accepts
`data/phase0/phase0_results.csv` only when its status is `ready` (every saved
pilot is valid and `q < rho < GOAL_RADIUS_M`) and `config.py` holds the fitted
numbers.

## Phase 1 workflow

Phase 1 works like Phase 0: every call collects one trial, saved as the next
`trial_NNN`. There is no fixed number of trials and no reset; walk the robot
back yourself between trials. First check that `config.py` still matches the
Phase 0 result:

```bash
python3 main.py --validate
```

Then, for every trial, put the robot anywhere inside the boundary, more than
`GOAL_RADIUS_M` from the goal, and run:

```bash
python3 main.py --execute
```

It reads the robot's pose from mocap, draws random virtual obstacles, and
plans an RRT path from that pose to within `GOAL_RADIUS_M - rho` of
(`GOAL_X_M`, `GOAL_Y_M`), with `rho` frozen from
`data/phase0/phase0_results.csv`. Obstacles are inflated by
`PHASE1_ROBOT_RADIUS_M + rho` (`PHASE1_ROBOT_RADIUS_M` is separate from
`ROBOT_RADIUS_M`, which stays frozen at its Phase 0 value). As in Phase 0, no
margin is kept at the boundary: the body center may reach it. Without
`--execute` it only previews a plan.

During every run, the collector stops the robot if its body center comes within
`EDGE_STOP_MARGIN_M` of the workspace edge, or if mocap loses it for
`LIVE_TRACKING_TIMEOUT_S`. A call that fails for any reason (no path, tracking
lost, Ctrl+C) leaves nothing behind, so the next call uses the same number.

After each trial it prints E and saves `trajectory.png` in the trial folder.
The summary over all trials can be rebuilt at any time:

```bash
python3 analyze_phase1.py
python3 plot_phase1.py all
```

`analyze_phase1.py` writes E, `rho` and whether E <= rho for every trial to
`data/phase1/analysis/execution_metrics.csv`.

## Training data for a learned model

`collect_training.py` collects random-command runs for training a learned
dynamics model. Repeat the same command for every trial; each run is saved as
the next `trial_NNN`:

```bash
python3 collect_training.py             # preview the next trial
python3 collect_training.py --execute   # run it on the robot
```

Each trial reads the robot's pose from mocap and draws `TRAINING_SEGMENTS`
commands whose nominal path stays `TRAINING_PLAN_MARGIN_M` inside the
workspace. About 80 % of trials are mixed; the rest are single-axis. Failed
trials are kept as `trial_NNN_discarded` and their numbers are not reused.
Step-by-step lab instructions are in `TRAINING_INSTRUCTIONS.txt`.

## Files produced

```text
data/
  phase0/
    pilot_001/plan.csv           planned velocity commands
    pilot_001/problem.json       start, goal, obstacles, seeds
    pilot_001/raw/{commands.csv,mocap.csv,publish_log.csv}
    pilot_001/trajectory.png     planned and real trajectory
    phase0_analysis.txt
    execution_metrics.csv
    phase0_results.csv
  phase1/
    trial_001/plan.csv           planned velocity commands
    trial_001/problem.json       start, goal, obstacles, seeds, rho, robot radius
    trial_001/raw/{commands.csv,mocap.csv,publish_log.csv}
    trial_001/trajectory.png     planned and real trajectory
    analysis/execution_metrics.csv
```

Raw mocap stores uncorrected `theta_raw`; calibration is applied only during
processing. Deviation uses position only, mocap interpolation at nominal segment
boundaries, and ends at the nominal sum of command durations.

## RRT adaptation

The uploaded homework planner was not copied verbatim. `planner.py` preserves
its seeded random sampling, tree expansion, parent-chain recovery, and sampled
collision checking. The following were changed to match the protocol:

- unicycle `(v, omega)` became full-holonomic `(vx, vy, omega)`;
- `vx` is never negative: the robot only walks forward and turns to go back;
- every segment either turns in place (`omega` only) or translates without
  turning (`vx`, `vy` only);
- with `PLANNER_TURN_THEN_STRAIGHT = True` (the default) plans are polylines:
  an RRT over positions finds waypoints, the path is shortcut to few corners,
  and the robot turns in place to face each corner and walks straight to it
  (`vx` and `omega` only), at a random speed per leg;
- variable 0.2–0.5 s commands became exactly `DT_S`;
- online lidar discovery/replanning was removed because obstacles are virtual;
- obstacles are inflated by the frozen Phase 0 `rho`;
- physical obstacle geometry is first inflated by `PHASE1_ROBOT_RADIUS_M`, then
  by the separate learned `rho`, so the robot center stays clear of both;
- the goal acceptance radius is shrunk to `GOAL_RADIUS_M - rho`;
- every plan has exactly `T_SEGMENTS` commands;
- planner failures are recorded rather than resampled away.

The original `kinodynamic_rrt.py` also advances distance without multiplying
linear speed by duration, so its `apply_dynamics` function must not be used for
hardware candidates without correction.
