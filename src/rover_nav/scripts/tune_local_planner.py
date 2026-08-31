#!/usr/bin/env python3
"""Sweep local_planner parameters over a fixed scenario and compare the outcomes.

The detector has tune_obstacle_detection.py; the planner had nothing, so its
parameters were being picked by argument rather than by measurement. This runs
the same course, with the same obstacles, once per parameter value, and prints
what each one actually did.

    # does the hard margin change how close it really gets?
    ros2 run rover_nav tune_local_planner.py --sweep collision_margin=0.10,0.15,0.25

    # how hard should the preferred keepout pull?
    ros2 run rover_nav tune_local_planner.py --sweep inflation_weight=0,1,2,4

    # on the real marsyard route, with rocks 5 m and 14 m along it
    ros2 run rover_nav tune_local_planner.py \\
        --path ~/jazzy_ws/src/marsyard/global_path_hybrid.csv \\
        --start-yaw 1.8326 --rocks 5.0,14.0 \\
        --sweep cost_weight=0,3,8 --timeout 400

Everything runs in the desk sim -- no rover, no camera, no ODrives. That is the
point: a parameter sweep needs the scenario held still, and a real rover on real
terrain repeats nothing exactly.

WHAT IT REPORTS, and why each one matters
    done      did the run finish, or hit the timeout / give up
    t         seconds to finish -- the cost of being cautious
    replans   how often it stopped to think. Climbing numbers mean thrash
    detours   how many separate ways round it committed to
    blocked   times it had no plan at all. Non-zero is a tuning failure
    minclr    CLOSEST the rover's body actually came to a rock, metres. This is
              the number the margins exist to control, and the one that decides
              whether a setting is safe
    maxdev    how far it strayed from the global route -- the cost of the detour
    goal      how close it finished to the final waypoint

A setting is better if minclr goes UP without done going false or t running
away. Nothing here is a pass/fail: it is a table to read.
"""

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time

HOME = os.path.expanduser("~")
DEFAULT_PATH = f"{HOME}/jazzy_ws/src/marsyard/test_6m.csv"


def declared_types(src):
    """Map each local_planner parameter to the type its default declares.

    Read from the source rather than hard-coded, so the sweep cannot drift out
    of step with the node. ROS infers a type from the literal you pass, so
    `inflation_weight=0` arrives as INTEGER, is rejected against a DOUBLE
    declaration, and the node dies at startup -- which shows up here as a run
    that silently did nothing at all."""
    types = {}
    pat = re.compile(r'declare_parameter\(\s*"([a-z_]+)"\s*,\s*([^,\)]+)')
    try:
        text = open(src).read()
    except OSError:
        return types
    for name, default in pat.findall(text):
        d = default.strip()
        if d in ("True", "False"):
            types[name] = bool
        elif re.fullmatch(r"-?\d+", d):
            types[name] = int
        elif re.fullmatch(r"-?\d*\.\d+", d):
            types[name] = float
        else:
            types[name] = str
    return types


def typed(name, value, types):
    """Format a sweep value so ROS infers the declared type."""
    t = types.get(name)
    if t is bool:
        return "true" if str(value).strip().lower() in ("1", "true", "yes") else "false"
    if t is int:
        return str(int(float(value)))
    if t is float:
        return repr(float(value))
    return str(value)


def sh(cmd, **kw):
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, preexec_fn=os.setsid, **kw)


def kill(p):
    if p and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


class Run:
    """One scenario at one parameter setting."""

    def __init__(self, args, overrides, workdir):
        self.args = args
        self.overrides = overrides
        self.workdir = workdir
        self.procs = []
        self.metrics = {}

    def _log(self, name):
        return os.path.join(self.workdir, name)

    def start(self):
        a = self.args
        wp = a.path.replace(".csv", "_waypoints.csv")
        arb = ["ros2", "run", "rover_nav", "cmd_vel_arbiter.py", "--ros-args",
               "-p", f"path_csv:={a.path}", "-p", "path_frame:=odom",
               "-p", "local_plan_enabled:=true"]
        if os.path.exists(wp):
            arb += ["-p", f"waypoints_csv:={wp}"]
        sim = ["ros2", "run", "rover_nav", "sim_local_planner.py", "--ros-args",
               "-p", f"rocks:=[{a.rocks}]", "-p", f"rock_radius:={a.rock_radius}",
               "-p", f"start_yaw:={a.start_yaw}", "-p", f"drift:={a.drift}"]
        lp = ["ros2", "run", "rover_nav", "local_planner.py", "--ros-args"]
        for k, v in self.overrides.items():
            lp += ["-p", f"{k}:={typed(k, v, self.args._types)}"]

        self.f_arb = open(self._log("arbiter.log"), "w")
        self.f_lp = open(self._log("planner.log"), "w")
        self.f_st = open(self._log("state.log"), "w")

        self.procs = [
            subprocess.Popen(arb, stdout=self.f_arb, stderr=subprocess.STDOUT,
                             preexec_fn=os.setsid),
            subprocess.Popen(sim, stdout=subprocess.DEVNULL,
                             stderr=subprocess.STDOUT, preexec_fn=os.setsid),
            subprocess.Popen(lp, stdout=self.f_lp, stderr=subprocess.STDOUT,
                             preexec_fn=os.setsid),
        ]
        time.sleep(a.settle)
        # Wait for the publisher to exist before attaching. Started too early,
        # ros2 topic echo prints "does not appear to be published yet" and then
        # captures nothing for the whole run -- which reads as a clean run with
        # no clearance measurement rather than as a broken sample.
        for _ in range(int(a.settle * 4)):
            listing = subprocess.run(["ros2", "topic", "list"], capture_output=True,
                                     text=True, timeout=20).stdout
            if "/local_plan/state" in listing:
                break
            time.sleep(0.5)
        else:
            print("      warning: /local_plan/state never appeared", file=sys.stderr)
        # Sampled continuously rather than read from the logs: the planner only
        # LOGS on a state change, so the closest approach -- which is what the
        # margins are being tuned for -- happens between two log lines.
        # stdbuf: piped to a file, ros2 topic echo block-buffers, and a run
        # short enough not to fill a 4 KB block loses every sample -- which
        # showed up as minclr=inf on exactly the fast runs.
        self.procs.append(subprocess.Popen(
            ["stdbuf", "-oL", "ros2", "topic", "echo", "/local_plan/state",
             "--field", "data"],
            stdout=self.f_st, stderr=subprocess.DEVNULL, preexec_fn=os.setsid))
        time.sleep(1.0)
        subprocess.run(["ros2", "service", "call", "/planner/start",
                        "std_srvs/srv/Trigger"], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, timeout=30)

    def wait(self):
        t0 = time.time()
        done = False
        while time.time() - t0 < self.args.timeout:
            time.sleep(2.0)
            self.f_arb.flush()
            try:
                txt = open(self._log("arbiter.log")).read()
            except OSError:
                continue
            if "COMPLETE" in txt:
                done = True
                break
        self.metrics["t"] = time.time() - t0
        self.metrics["done"] = done

    def stop(self):
        for p in self.procs:
            kill(p)
        for f in (self.f_arb, self.f_lp, self.f_st):
            try:
                f.close()
            except Exception:
                pass
        time.sleep(self.args.gap)

    def harvest(self):
        m = self.metrics
        try:
            lp = open(self._log("planner.log")).read()
        except OSError:
            lp = ""
        try:
            arb = open(self._log("arbiter.log")).read()
        except OSError:
            arb = ""

        if "Exception" in lp or "exited with failure" in lp:
            m["crashed"] = True
        m["detours"] = len(re.findall(r"detour planned", lp))
        # Startup BLOCKED lines ("no odometry", "waiting for global route") are
        # the node coming up, not the planner failing to find a way round.
        # Counting them made every run look like it had given up twice.
        m["blocked"] = len([
            b for b in re.findall(r"BLOCKED -- ([^\n(]+)", lp)
            if not b.startswith(("no odometry", "odometry stale",
                                 "waiting for", "no obstacle detector",
                                 "obstacle detector stale", "no costmap"))])
        replans = re.findall(r"replan #(\d+)", lp)
        m["replans"] = int(replans[-1]) if replans else 0

        minclr = float("inf")
        samples = 0
        try:
            for line in open(self._log("state.log")):
                line = line.strip().strip('"')
                if not line.startswith("{"):
                    continue
                try:
                    d = json.loads(line)
                except ValueError:
                    continue
                c = d.get("clearance_m")
                # null is "nothing known yet", not a real measurement
                samples += 1
                if c is not None and c < 1e6:
                    minclr = min(minclr, c)
        except OSError:
            pass
        m["minclr"] = minclr
        m["samples"] = samples

        dev = re.findall(r"max ([0-9.]+) m", arb)
        m["maxdev"] = float(dev[-1]) if dev else float("nan")
        goal = re.findall(r"worst: ([0-9.]+) cm", arb)
        m["goal"] = float(goal[-1]) / 100.0 if goal else float("nan")
        return m


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sweep", action="append", required=True,
                    help="param=v1,v2,v3 -- repeat for a grid over several params")
    ap.add_argument("--path", default=DEFAULT_PATH)
    ap.add_argument("--rocks", default="3.0", help="arclengths (m) along the route")
    ap.add_argument("--rock-radius", type=float, default=0.25)
    ap.add_argument("--drift", type=float, default=0.0)
    ap.add_argument("--start-yaw", type=float, default=0.0)
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument("--settle", type=float, default=6.0)
    ap.add_argument("--gap", type=float, default=3.0)
    ap.add_argument("--out-dir", default="/tmp/tune_local_planner")
    args = ap.parse_args()

    grid = []
    for spec in args.sweep:
        if "=" not in spec:
            raise SystemExit(f"--sweep wants param=v1,v2 -- got '{spec}'")
        k, vs = spec.split("=", 1)
        grid.append([(k.strip(), v.strip()) for v in vs.split(",")])

    combos = [{}]
    for axis in grid:
        combos = [dict(c, **{k: v}) for c in combos for k, v in axis]

    args._types = declared_types(os.path.join(os.path.dirname(
        os.path.abspath(__file__)), "local_planner.py"))
    unknown = [k for axis in grid for k, _ in axis if k not in args._types]
    if unknown:
        print(f"warning: {', '.join(sorted(set(unknown)))} not declared by "
              f"local_planner.py -- passed through untyped", file=sys.stderr)

    os.makedirs(args.out_dir, exist_ok=True)
    print(f"{len(combos)} run(s), timeout {args.timeout:.0f}s each "
          f"-- worst case {len(combos) * args.timeout / 60:.0f} min")
    print(f"path {os.path.basename(args.path)}, rocks [{args.rocks}] "
          f"r={args.rock_radius}, drift {args.drift}\n")

    rows = []
    for i, combo in enumerate(combos, 1):
        label = " ".join(f"{k}={v}" for k, v in combo.items())
        print(f"[{i}/{len(combos)}] {label} ...", flush=True)
        wd = os.path.join(args.out_dir, f"run{i:02d}")
        os.makedirs(wd, exist_ok=True)
        r = Run(args, combo, wd)
        try:
            r.start()
            r.wait()
        finally:
            r.stop()
        m = r.harvest()
        rows.append((label, m))
        if m.get("crashed"):
            print(f"      local_planner FAILED TO START -- see {wd}/planner.log",
                  file=sys.stderr)
        print(f"      done={m['done']} t={m['t']:.0f}s replans={m['replans']} "
              f"detours={m['detours']} blocked={m['blocked']} "
              f"minclr={m['minclr']:.2f} goal={m['goal']:.2f}", flush=True)

    w = max(len(r[0]) for r in rows)
    print(f"\n{'setting':<{w}} {'done':>5} {'t':>6} {'replans':>8} {'detours':>8} "
          f"{'blocked':>8} {'minclr':>7} {'maxdev':>7} {'goal':>6}")
    print("-" * (w + 60))
    for label, m in rows:
        print(f"{label:<{w}} {str(m['done']):>5} {m['t']:>5.0f}s {m['replans']:>8} "
              f"{m['detours']:>8} {m['blocked']:>8} {m['minclr']:>7.2f} "
              f"{m['maxdev']:>7.2f} {m['goal']:>6.2f}")
    print(f"\nlogs: {args.out_dir}/run*/")
    print("read minclr first -- it is the clearance the margins are meant to buy.")


if __name__ == "__main__":
    sys.exit(main())
