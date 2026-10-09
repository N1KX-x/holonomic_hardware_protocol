import glob, math
from pathlib import Path
import config
from core import _load_mocap, _interp, load_sent_commands
SKIP = 0.5  # ignore the first 0.5 s of each segment (speeding up)
fits = {"vx": [0.0, 0.0], "vy": [0.0, 0.0], "omega": [0.0, 0.0]}
for run in sorted(glob.glob("data/phase0/raw/pilot_*_only_*")):
    run = Path(run)
    try:
        cmds = load_sent_commands(run / "commands.csv")
        t, x, y, th = _load_mocap(run / "mocap.csv", config)
    except (OSError, ValueError) as error:
        print(f"skipped {run.name}: {error}")
        continue
    for c in cmds:
        a, b = c.t_send + SKIP, c.t_send + c.dt
        try:
            x0, y0, h0 = _interp(a, t, x), _interp(a, t, y), _interp(a, t, th)
            x1, y1, h1 = _interp(b, t, x), _interp(b, t, y), _interp(b, t, th)
        except ValueError:
            continue
        T, h = b - a, (h0 + h1) / 2
        fwd = (math.cos(h)*(x1-x0) + math.sin(h)*(y1-y0)) / T
        left = (-math.sin(h)*(x1-x0) + math.cos(h)*(y1-y0)) / T
        for axis, cmd, got in (("vx", c.vx, fwd), ("vy", c.vy, left), ("omega", c.omega, (h1-h0)/T)):
            if abs(cmd) > 0.05:
                fits[axis][0] += cmd*got
                fits[axis][1] += cmd*cmd
for axis, (num, den) in fits.items():
    print(f"{axis:6s}: robot does {num/den:.2f} x the command" if den else f"{axis}: no data")