# Holonomic hardware protocol

This folder implements the two-phase protocol in
`hardware_protocol_full_holonomic.md`. It separates:

- Phase 0 pipeline verification and deviation pilot (`phase0.py`)
- Phase 1 problem/candidate generation and collection (`main.py`)
- offline Phase 1 deviation analysis (`analyze_phase1.py`)
- hardware-free model/CSV code (`core.py`)
- robot and mocap processes (`hardware/`)
- the full-holonomic kinodynamic RRT (`planner.py`)

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
python3 phase0.py prepare
```

`phase0.py prepare` only creates candidate CSV files and a review checklist. It
does not connect to hardware. Every plan is checked against the per-axis and
combined command limits before any file is written; single-axis pilots draw
from the smaller of the axis limit and `TRANSLATIONAL_SPEED_MAX_MPS`. Do not run
it twice in the same data folder.

## Phase 0 workflow

`phase0.py prepare` creates three pipeline sequences and the configured 15-run
pilot split. For each planned run, collection is deliberately one command at a
time:

```bash
python3 phase0.py collect pipeline_forward --execute
```

Collection still refuses unless `ALLOW_HARDWARE_EXECUTION = True`, and by
default requires the operator to type the run ID. After inspecting forward,
arc, and lateral results and independently verifying clock/yaw/lateral signs,
change each status in `data/phase0/pipeline_review.csv` from `pending` to `pass`.

After all planned raw folders exist:

```bash
python3 phase0.py summarize
```

This produces `data/phase0/phase0_results.csv`. Phase 1 will only accept it when
its status is `ready`, all pilot runs are valid, all pipeline reviews pass, and
the estimated values satisfy `q < rho < goal radius`.

## Phase 1 workflow

```bash
python3 main.py validate
python3 main.py prepare
python3 main.py status
```

`prepare` is planning-only. It samples all problems without conditioning on
success, generates `m` seeded candidates, shuffles their order, records empty
instances, and writes the randomized collection order.

Collect exactly one instance at a time:

```bash
python3 main.py collect I001 --execute
```

Between candidates, collection now uses a separate mocap-guided autonomous
reset. Candidate 1's measured initial pose becomes the instance home pose. Each
completed candidate is processed first; then bounded reset commands are issued
and remeasured until both position and heading tolerances are met. The next
candidate cannot start unless reset succeeds. Reset recordings live under
`data/phase1/resets/` and are excluded from experimental deviations.

`AUTONOMOUS_RESET_ENABLED` is deliberately `False` in the delivered config.
Review `RESET_*` speeds, tolerances, maximum attempts, the open physical return
path, and emergency-stop procedure before enabling it. A failed reset discards
the complete instance.

If any candidate fails, every raw run belonging to that instance is renamed
with `_discarded` and the instance status becomes `discarded`. It is not rerun.

After collection, offline analysis is:

```bash
python3 analyze_phase1.py
```

It writes per-execution deviations and the maximum deviation across all `m`
candidates for each complete instance.

## Files produced

```text
data/
  phase0/
    plans/
    raw/<run_id>/{commands.csv,mocap.csv,publish_log.csv}
    pipeline_review.csv
    execution_metrics.csv
    phase0_results.csv
  phase1/
    notes.txt
    problems.csv
    collection_order.csv
    plans/I001/C01.csv
    raw/I001_C01/{commands.csv,mocap.csv,publish_log.csv}
    resets/I001_after_C01/attempt_01/{reset_plan.csv,raw/...}
    analysis/{execution_metrics.csv,instance_metrics.csv}
```

Raw mocap stores uncorrected `theta_raw`; calibration is applied only during
processing. Deviation uses position only, mocap interpolation at nominal segment
boundaries, and ends at the nominal sum of command durations.

## RRT adaptation

The uploaded homework planner was not copied verbatim. `planner.py` preserves
its seeded random sampling, tree expansion, parent-chain recovery, and sampled
collision checking. The following were changed to match the protocol:

- unicycle `(v, omega)` became full-holonomic `(vx, vy, omega)`;
- variable 0.2–0.5 s commands became exactly `DT_S`;
- online lidar discovery/replanning was removed because obstacles are virtual;
- obstacles are inflated by the frozen Phase 0 `rho`;
- physical obstacle geometry is first inflated by `ROBOT_RADIUS_M`, then by
  the separate learned `rho`, so the robot center stays clear of both;
- the goal acceptance radius is shrunk to `GOAL_RADIUS_M - rho`;
- every candidate has exactly `T_SEGMENTS` commands;
- planner failures are recorded rather than resampled away.

The original `kinodynamic_rrt.py` also advances distance without multiplying
linear speed by duration, so its `apply_dynamics` function must not be used for
hardware candidates without correction.
