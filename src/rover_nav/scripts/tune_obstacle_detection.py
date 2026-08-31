#!/usr/bin/env python3
"""Interactive tuner for the obstacle-detection pipeline.

Sets ROS parameters on the live nodes (camera -> passthrough -> SOR -> detector)
while showing what each stage is actually doing, so a change can be judged
without alt-tabbing to RViz and guessing.

    ros2 run rover_nav tune_obstacle_detection.py          # interactive
    ros2 run rover_nav tune_obstacle_detection.py --watch  # monitor only
    ros2 run rover_nav tune_obstacle_detection.py --set eps=0.2 min_pts=12
    ros2 run rover_nav tune_obstacle_detection.py --dump tune.yaml
    ros2 run rover_nav tune_obstacle_detection.py --load tune.yaml

Parameters are typed from each node's own descriptors rather than parsed as
YAML, so `set field y` sets the string "y" instead of tripping the YAML 1.1
boolean trap that makes `ros2 param set ... y` fail.
"""

import argparse
import json
import os
import sys
import threading
import time
from collections import deque

import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import (
    DescribeParameters,
    GetParameters,
    ListParameters,
    SetParameters,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray

try:
    import readline  # noqa: F401  -- line editing + history when available
except ImportError:
    readline = None


# Pipeline stages, in data-flow order. Alias -> default node name. Any of them
# may be absent (e.g. running the detector standalone); missing ones are
# reported once at startup and then ignored.
TARGETS = [
    ('pass', '/passthrough_filter'),
    ('sor',  '/sor_filter'),
    ('det',  '/obstacle_detector'),
]
DEFAULT_TARGET = 'det'

# Short names accepted on the command line, so the common knobs are two
# keystrokes instead of twenty. Prefix matching covers everything else.
ALIASES = {
    'eps':      'dbscan_eps',
    'min_pts':  'dbscan_min_pts',
    'minpts':   'dbscan_min_pts',
    'voxel':    'voxel_size',
    'range':    'max_range',
    'height':   'min_obstacle_height',
    'minh':     'min_obstacle_height',
    'margin':   'ground_margin',
    'cam':      'camera_height',
    'band':     'ground_band',
    'fit':      'fit_ground_plane',
    'npts':     'min_cluster_pts',
}

# Starting points worth having one keystroke away. Values come from measured
# behaviour on this rover -- see session_handoff_2026-08-27.md, findings 7-8.
PRESETS = {
    'tight': {
        # eps 0.80 at voxel 0.05 tolerates a 16-voxel gap, so residual floor
        # points chain into the real obstacle and everything merges into one
        # giant box. 0.20 is a 4-voxel gap; min_pts 5 lets five noise points
        # seed a cluster, hence 12.
        '_doc': 'indoor / cluttered: stop clusters merging into one box',
        'dbscan_eps': 0.20, 'dbscan_min_pts': 12, 'voxel_size': 0.05,
        'min_cluster_pts': 20,
    },
    'lowrock': {
        '_doc': 'catch short rocks (~10 cm) at the cost of more false positives',
        'voxel_size': 0.03, 'ground_margin': 0.04, 'min_obstacle_height': 0.10,
        'min_cluster_pts': 10, 'dbscan_eps': 0.25, 'dbscan_min_pts': 8,
    },
    'fast': {
        '_doc': 'cheapest per frame; misses small/short objects',
        'voxel_size': 0.08, 'min_cluster_pts': 20, 'dbscan_eps': 0.30,
        'dbscan_min_pts': 8, 'min_obstacle_height': 0.20,
    },
    'open': {
        '_doc': 'outdoor / marsyard: full range, sloped ground',
        'dbscan_eps': 0.30, 'dbscan_min_pts': 8, 'voxel_size': 0.05,
        'max_range': 2.0, 'min_obstacle_height': 0.15, 'fit_ground_plane': True,
    },
}

TYPE_NAMES = {
    ParameterType.PARAMETER_BOOL: 'bool',
    ParameterType.PARAMETER_INTEGER: 'int',
    ParameterType.PARAMETER_DOUBLE: 'double',
    ParameterType.PARAMETER_STRING: 'string',
    ParameterType.PARAMETER_BOOL_ARRAY: 'bool[]',
    ParameterType.PARAMETER_INTEGER_ARRAY: 'int[]',
    ParameterType.PARAMETER_DOUBLE_ARRAY: 'double[]',
    ParameterType.PARAMETER_STRING_ARRAY: 'string[]',
}

# Parameters every node has and nobody tunes.
HIDDEN = {'use_sim_time', 'qos_overrides', 'start_type_description_service'}


class ParamLookupError(Exception):
    """Raised when a parameter name can't be turned into exactly one match.

    `ambiguous` distinguishes 'this node has no such parameter' -- which just
    means keep looking in the next node of the pipeline -- from 'you typed a
    prefix that matches several', which is an answer and must not be swallowed.
    """

    def __init__(self, message, ambiguous=False):
        super().__init__(message)
        self.ambiguous = ambiguous


def _c(code, s, enable=True):
    return f'\033[{code}m{s}\033[0m' if enable else s


class Colors:
    def __init__(self, enable):
        self.on = enable

    def bold(self, s):   return _c('1', s, self.on)
    def dim(self, s):    return _c('2', s, self.on)
    def red(self, s):    return _c('31', s, self.on)
    def green(self, s):  return _c('32', s, self.on)
    def yellow(self, s): return _c('33', s, self.on)
    def cyan(self, s):   return _c('36', s, self.on)


def value_to_python(pv: ParameterValue):
    t = pv.type
    if t == ParameterType.PARAMETER_BOOL:            return pv.bool_value
    if t == ParameterType.PARAMETER_INTEGER:         return pv.integer_value
    if t == ParameterType.PARAMETER_DOUBLE:          return pv.double_value
    if t == ParameterType.PARAMETER_STRING:          return pv.string_value
    if t == ParameterType.PARAMETER_BOOL_ARRAY:      return list(pv.bool_array_value)
    if t == ParameterType.PARAMETER_INTEGER_ARRAY:   return list(pv.integer_array_value)
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:    return list(pv.double_array_value)
    if t == ParameterType.PARAMETER_STRING_ARRAY:    return list(pv.string_array_value)
    if t == ParameterType.PARAMETER_BYTE_ARRAY:      return list(pv.byte_array_value)
    return None


def python_to_value(py, ptype: int) -> ParameterValue:
    """Coerce a Python value into the type the node declared.

    Typing off the descriptor, not off the literal, is what keeps `set
    filter_field_name y` from becoming a bool: the node says the parameter is a
    string, so "y" stays "y".
    """
    pv = ParameterValue(type=ptype)
    if ptype == ParameterType.PARAMETER_BOOL:
        if isinstance(py, str):
            s = py.strip().lower()
            if s in ('1', 'true', 't', 'yes', 'y', 'on'):
                py = True
            elif s in ('0', 'false', 'f', 'no', 'n', 'off'):
                py = False
            else:
                raise ValueError(f'{py!r} is not a bool')
        pv.bool_value = bool(py)
    elif ptype == ParameterType.PARAMETER_INTEGER:
        pv.integer_value = int(round(float(py)))
    elif ptype == ParameterType.PARAMETER_DOUBLE:
        pv.double_value = float(py)
    elif ptype == ParameterType.PARAMETER_STRING:
        pv.string_value = str(py)
    elif ptype == ParameterType.PARAMETER_DOUBLE_ARRAY:
        pv.double_array_value = [float(v) for v in py]
    elif ptype == ParameterType.PARAMETER_INTEGER_ARRAY:
        pv.integer_array_value = [int(v) for v in py]
    elif ptype == ParameterType.PARAMETER_STRING_ARRAY:
        pv.string_array_value = [str(v) for v in py]
    elif ptype == ParameterType.PARAMETER_BOOL_ARRAY:
        pv.bool_array_value = [bool(v) for v in py]
    else:
        raise ValueError(f'unsupported parameter type {ptype}')
    return pv


def split_assignment(pair):
    """'eps=0.2' -> ('eps', '0.2');  'eps+=0.05' -> ('eps', '+=0.05').

    The nudge marker sits on the key side when written as one shell word, so
    strip it off there and hand do_set the '+=N' form it understands.
    """
    if '=' not in pair:
        return None, None
    k, _, v = pair.partition('=')
    k = k.strip()
    if k[-1:] in ('+', '-'):
        return k[:-1].strip(), k[-1] + '=' + v.strip()
    return k, v.strip()


def fmt(v):
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, float):
        # Keep the decimal point: a double written as `2` reloads from yaml as
        # an int, and the node then refuses it at startup with a type error.
        s = f'{v:g}'
        return s if any(ch in s for ch in '.en') else s + '.0'
    if isinstance(v, str):
        return f"'{v}'"
    return str(v)


class ParamTarget:
    """Parameter client for one node, driven through its raw services."""

    def __init__(self, node: Node, alias: str, node_name: str):
        self.alias = alias
        self.node_name = node_name
        self.online = False
        self.names = []
        self.types = {}
        self.baseline = {}
        cbg = ReentrantCallbackGroup()
        base = node_name.rstrip('/')
        self._list = node.create_client(ListParameters, f'{base}/list_parameters', callback_group=cbg)
        self._get = node.create_client(GetParameters, f'{base}/get_parameters', callback_group=cbg)
        self._set = node.create_client(SetParameters, f'{base}/set_parameters', callback_group=cbg)
        self._desc = node.create_client(DescribeParameters, f'{base}/describe_parameters', callback_group=cbg)

    def _call(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f'{self.node_name}: service unavailable')
        future = client.call_async(request)
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                raise TimeoutError(f'{self.node_name}: call timed out')
            time.sleep(0.01)
        return future.result()

    def connect(self, timeout=3.0) -> bool:
        try:
            resp = self._call(self._list, ListParameters.Request(), timeout)
        except TimeoutError:
            self.online = False
            return False
        self.names = sorted(n for n in resp.result.names
                            if n not in HIDDEN and not n.startswith('qos_overrides'))
        if self.names:
            desc = self._call(self._desc, DescribeParameters.Request(names=self.names))
            self.types = {d.name: d.type for d in desc.descriptors}
        self.online = True
        self.baseline = self.get_all()
        return True

    def get_all(self) -> dict:
        if not self.names:
            return {}
        resp = self._call(self._get, GetParameters.Request(names=self.names))
        return {n: value_to_python(v) for n, v in zip(self.names, resp.values)}

    def get(self, name):
        resp = self._call(self._get, GetParameters.Request(names=[name]))
        return value_to_python(resp.values[0])

    def set(self, name, value):
        """Set one parameter. Returns (ok, message)."""
        ptype = self.types.get(name)
        if ptype is None:
            return False, f'{self.node_name} has no parameter {name}'
        try:
            pv = python_to_value(value, ptype)
        except (TypeError, ValueError) as exc:
            return False, f'{value!r} is not a {TYPE_NAMES.get(ptype, ptype)}: {exc}'
        req = SetParameters.Request(parameters=[ParameterMsg(name=name, value=pv)])
        try:
            resp = self._call(self._set, req)
        except TimeoutError as exc:
            return False, str(exc)
        r = resp.results[0]
        return r.successful, r.reason


class TunerNode(Node):
    """Subscribes to the detector's output so a parameter change can be judged."""

    def __init__(self):
        super().__init__('obstacle_tuner')
        cbg = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.stats = None
        self.stats_time = 0.0
        self.flag = None
        self.marker_count = None
        self._stamps = deque(maxlen=30)
        self.create_subscription(String, '/obstacles/stats', self._on_stats, 10, callback_group=cbg)
        self.create_subscription(Bool, '/obstacle_detected', self._on_flag, 10, callback_group=cbg)
        self.create_subscription(MarkerArray, '/obstacles/markers', self._on_markers, 10, callback_group=cbg)

    def _on_stats(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self.lock:
            self.stats = data
            self.stats_time = time.time()
            self._stamps.append(self.stats_time)

    def _on_flag(self, msg):
        with self.lock:
            self.flag = msg.data

    def _on_markers(self, msg):
        # Only ADD markers are real boxes; DELETEs clean up the previous frame.
        n = sum(1 for m in msg.markers if m.action == 0)
        with self.lock:
            self.marker_count = n
            if self.stats is None:
                # Old detector without /obstacles/stats -- still give a rate.
                self._stamps.append(time.time())

    def rate(self):
        with self.lock:
            stamps = list(self._stamps)
        if len(stamps) < 2:
            return 0.0
        span = stamps[-1] - stamps[0]
        return (len(stamps) - 1) / span if span > 0 else 0.0

    def snapshot(self):
        with self.lock:
            return (dict(self.stats) if self.stats else None,
                    self.stats_time, self.flag, self.marker_count)


class Tuner:
    def __init__(self, node, targets, colors):
        self.node = node
        self.targets = targets           # alias -> ParamTarget
        self.c = colors
        self.current = DEFAULT_TARGET if DEFAULT_TARGET in targets else next(iter(targets))

    # ── name resolution ──────────────────────────────────────────────────────
    def resolve(self, token):
        """'eps' / 'dbscan_eps' / 'det:dbscan_eps' -> (ParamTarget, name).

        Raises KeyError with a human message when nothing or too much matches.
        """
        if ':' in token:
            alias, _, name = token.partition(':')
            if alias not in self.targets:
                raise ParamLookupError(
                    f'unknown target {alias!r}; have {", ".join(self.targets)}')
            tgt = self.targets[alias]
            return tgt, self._match(tgt, name)

        # Search the selected target first, then the rest of the pipeline.
        order = [self.current] + [a for a in self.targets if a != self.current]
        hits = []
        for alias in order:
            tgt = self.targets[alias]
            try:
                hits.append((tgt, self._match(tgt, token)))
            except ParamLookupError as exc:
                if exc.ambiguous:
                    raise   # naming several params is an answer, not a miss
                continue
        if not hits:
            raise ParamLookupError(f'no parameter matching {token!r}')
        if len(hits) > 1 and hits[0][0].alias != self.current:
            where = ', '.join(f'{t.alias}:{n}' for t, n in hits)
            raise ParamLookupError(f'{token!r} is ambiguous: {where}')
        return hits[0]

    def _match(self, tgt, name):
        name = ALIASES.get(name, name)
        if name in tgt.types:
            return name
        pref = [n for n in tgt.names if n.startswith(name)]
        if len(pref) == 1:
            return pref[0]
        if len(pref) > 1:
            raise ParamLookupError(
                f'{name!r} is ambiguous in {tgt.alias}: {", ".join(pref)}', ambiguous=True)
        sub = [n for n in tgt.names if name in n]
        if len(sub) == 1:
            return sub[0]
        if len(sub) > 1:
            raise ParamLookupError(
                f'{name!r} is ambiguous in {tgt.alias}: {", ".join(sub)}', ambiguous=True)
        raise ParamLookupError(f'{tgt.alias} has no parameter matching {name!r}')

    def completions(self):
        out = []
        for alias, tgt in self.targets.items():
            out += tgt.names + [f'{alias}:{n}' for n in tgt.names]
        return sorted(set(out + list(ALIASES) + list(PRESETS)))

    # ── actions ──────────────────────────────────────────────────────────────
    def do_set(self, token, raw):
        try:
            tgt, name = self.resolve(token)
        except ParamLookupError as exc:
            print(self.c.red(f'  {exc}'))
            return False

        value = raw
        if isinstance(raw, str) and raw[:2] in ('+=', '-='):
            # Relative nudge: `set eps +=0.05`. Spelled with '=' so that a bare
            # `-0.02` stays an absolute value -- ground_margin can legitimately
            # go negative, and guessing wrong there is silent.
            cur = tgt.get(name)
            if isinstance(cur, bool) or not isinstance(cur, (int, float)):
                print(self.c.red(f'  {name} is not numeric; cannot apply {raw}'))
                return False
            try:
                delta = float(raw[2:])
            except ValueError:
                print(self.c.red(f'  {raw[2:]!r} is not a number'))
                return False
            value = cur + delta if raw[0] == '+' else cur - delta

        before = tgt.get(name)
        ok, reason = tgt.set(name, value)
        if not ok:
            print(self.c.red(f'  {tgt.alias}:{name} rejected: {reason or "no reason given"}'))
            return False
        after = tgt.get(name)
        print(f'  {self.c.cyan(tgt.alias + ":" + name)} {fmt(before)} '
              f'{self.c.dim("->")} {self.c.green(fmt(after))}')
        return True

    def do_list(self, alias=None):
        aliases = [alias] if alias else list(self.targets)
        for a in aliases:
            tgt = self.targets.get(a)
            if tgt is None:
                print(self.c.red(f'  unknown target {a!r}'))
                continue
            marker = ' *' if a == self.current else '  '
            print(f'{marker}{self.c.bold(tgt.alias)} {self.c.dim(tgt.node_name)}')
            values = tgt.get_all()
            for n in tgt.names:
                v = values[n]
                base = tgt.baseline.get(n)
                changed = base is not None and v != base
                line = f'    {n:<22} {fmt(v):>10}'
                if changed:
                    line += self.c.yellow(f'   (was {fmt(base)})')
                print(self.c.green(line) if changed else line)

    def do_diff(self):
        any_change = False
        for tgt in self.targets.values():
            values = tgt.get_all()
            rows = [(n, tgt.baseline[n], values[n]) for n in tgt.names
                    if n in tgt.baseline and values[n] != tgt.baseline[n]]
            if rows:
                any_change = True
                print(f'  {self.c.bold(tgt.alias)}')
                for n, was, now in rows:
                    print(f'    {n:<22} {fmt(was)} {self.c.dim("->")} {self.c.green(fmt(now))}')
        if not any_change:
            print(self.c.dim('  nothing changed since startup'))

    def do_reset(self, alias=None):
        aliases = [alias] if alias else list(self.targets)
        for a in aliases:
            tgt = self.targets.get(a)
            if tgt is None:
                continue
            values = tgt.get_all()
            for n, base in tgt.baseline.items():
                if values.get(n) != base:
                    ok, reason = tgt.set(n, base)
                    status = self.c.green('ok') if ok else self.c.red(reason)
                    print(f'    {tgt.alias}:{n} -> {fmt(base)}  {status}')

    def do_preset(self, name):
        preset = PRESETS.get(name)
        if preset is None:
            print(self.c.red(f'  unknown preset {name!r}; have: {", ".join(PRESETS)}'))
            return
        print(self.c.dim(f'  {name}: {preset["_doc"]}'))
        for k, v in preset.items():
            if k != '_doc':
                self.do_set(f'det:{k}' if 'det' in self.targets else k, v)

    def as_yaml(self):
        out = []
        for tgt in self.targets.values():
            values = tgt.get_all()
            if not values:
                continue
            out.append(f'{tgt.node_name.lstrip("/")}:')
            out.append('  ros__parameters:')
            for n in tgt.names:
                out.append(f'    {n}: {fmt(values[n])}')
            out.append('')
        return '\n'.join(out)

    def load_yaml(self, path):
        try:
            import yaml
        except ImportError:
            print(self.c.red('  PyYAML not available'))
            return
        with open(path) as fh:
            doc = yaml.safe_load(fh) or {}
        for node_name, body in doc.items():
            params = (body or {}).get('ros__parameters', {})
            alias = next((a for a, t in self.targets.items()
                          if t.node_name.lstrip('/') == node_name.lstrip('/')), None)
            if alias is None:
                print(self.c.yellow(f'  skipping {node_name}: not running'))
                continue
            for n, v in params.items():
                self.do_set(f'{alias}:{n}', v)


def render_status(node, colors, width=None):
    """One block describing what the detector did with the last frame."""
    c = colors
    stats, stamp, flag, markers = node.snapshot()
    rate = node.rate()
    lines = []

    age = time.time() - stamp if stamp else None
    if stats is None:
        state = c.yellow('no /obstacles/stats — detector not running, or an '
                         'older build without the stats topic')
        if markers is not None:
            state += f'  |  markers: {markers}'
        lines.append('  ' + state)
        lines.append(f'  rate {rate:5.1f} Hz   flag {fmt(flag)}')
        return lines

    stale = age is not None and age > 2.0
    head = c.red(f'STALE ({age:.0f}s)') if stale else c.green(f'{rate:5.1f} Hz')
    lines.append(
        f'  {head}   in {stats.get("in_pts", 0):>7,} pts'
        f' {c.dim("->")} voxel {stats.get("voxel_pts", 0):>6,}'
        f' {c.dim("->")} above-ground {stats.get("kept_pts", 0):>6,}'
        f'   ground={c.cyan(str(stats.get("ground", "?")))}'
    )
    shown = stats.get('published', 0)
    drops = (f'small {stats.get("drop_small", 0)}  '
             f'far {stats.get("drop_far", 0)}  short {stats.get("drop_short", 0)}')
    lines.append(
        f'  clusters {stats.get("raw_clusters", 0)} raw '
        f'{c.dim("->")} {c.bold(str(shown))} published'
        f'   dropped: {c.yellow(drops)}   flag {fmt(flag)}'
    )
    for i, ob in enumerate(stats.get('obstacles', [])[:6]):
        w, h, d = (ob.get('size') or [0, 0, 0])
        lines.append(f'    #{i}  {ob.get("range", 0):5.2f} m   '
                     f'{w:4.2f} x {h:4.2f} x {d:4.2f} m   {ob.get("pts", 0):>5} pts')
    if len(stats.get('obstacles', [])) > 6:
        lines.append(c.dim(f'    ... {len(stats["obstacles"]) - 6} more'))
    return lines


def watch(node, colors, interval=0.4, duration=None):
    """Redraw the status block in place until Ctrl-C."""
    tty = sys.stdout.isatty()
    prev = 0
    start = time.time()
    try:
        while rclpy.ok():
            lines = render_status(node, colors)
            if tty and prev:
                sys.stdout.write(f'\033[{prev}A')
            for ln in lines:
                sys.stdout.write('\033[2K' + ln + '\n' if tty else ln + '\n')
            # Keep the block a constant height so the cursor maths stays right.
            for _ in range(max(0, prev - len(lines))):
                sys.stdout.write('\033[2K\n')
            prev = max(prev, len(lines)) if tty else 0
            sys.stdout.flush()
            if duration and time.time() - start > duration:
                return
            time.sleep(interval)
    except KeyboardInterrupt:
        print()


HELP = """
commands
  ls [target]              show parameters (changed ones highlighted)
  set <param> <value>      set one parameter; `+=N` / `-=N` nudges instead
  <param>=<value> ...      same thing, several at once (`eps+=0.05` nudges)
  get <param>              read one parameter
  w [seconds]              watch live detection stats (Ctrl-C to stop)
  use <target>             default target for bare parameter names
  preset <name>            apply a starting point: {presets}
  diff                     what changed since this tuner started
  reset [target]           put everything back
  yaml [file]              print (or write) a ros__parameters block
  load <file>              apply a yaml written by `yaml`
  help / q

parameter names may be given as `det:dbscan_eps`, `dbscan_eps`, or any unique
prefix (`eps`, `voxel`, `minh`). targets: {targets}
"""


def build_targets(node, overrides, colors):
    targets = {}
    for alias, default_name in TARGETS:
        name = overrides.get(alias, default_name)
        tgt = ParamTarget(node, alias, name)
        if tgt.connect(timeout=2.0):
            targets[alias] = tgt
            print(f'  {colors.green("connected")} {alias:<5} {name} '
                  f'({len(tgt.names)} params)')
        else:
            print(f'  {colors.dim("offline  ")} {alias:<5} {name}')
    return targets


def main():
    ap = argparse.ArgumentParser(
        description='Tune the obstacle-detection pipeline live.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=HELP.format(presets=', '.join(PRESETS), targets=', '.join(a for a, _ in TARGETS)),
    )
    ap.add_argument('--set', nargs='+', metavar='K=V',
                    help='apply parameters and exit')
    ap.add_argument('--preset', help='apply a preset and exit')
    ap.add_argument('--dump', nargs='?', const='-', metavar='FILE',
                    help='print current parameters as yaml and exit')
    ap.add_argument('--load', metavar='FILE', help='apply a yaml file and exit')
    ap.add_argument('--watch', action='store_true', help='monitor only, no prompt')
    ap.add_argument('--detector', default=None, help='detector node name')
    ap.add_argument('--passthrough', default=None, help='passthrough node name')
    ap.add_argument('--sor', default=None, help='SOR node name')
    ap.add_argument('--no-color', action='store_true')
    args, ros_args = ap.parse_known_args()

    colors = Colors(sys.stdout.isatty() and not args.no_color
                    and os.environ.get('TERM') not in (None, 'dumb'))

    rclpy.init(args=ros_args)
    node = TunerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    overrides = {k: v for k, v in
                 (('det', args.detector), ('pass', args.passthrough), ('sor', args.sor))
                 if v}

    print(colors.bold('obstacle-detection tuner'))
    targets = build_targets(node, overrides, colors)

    try:
        if not targets:
            print(colors.red('\nNo pipeline nodes are running. Start one, e.g.:\n'
                             '  ros2 launch rover_nav obstacle_detection.launch.py'))
            return 1

        tuner = Tuner(node, targets, colors)

        # ── non-interactive modes ────────────────────────────────────────────
        if args.dump is not None:
            text = tuner.as_yaml()
            if args.dump == '-':
                print(text)
            else:
                with open(args.dump, 'w') as fh:
                    fh.write(text)
                print(f'  wrote {args.dump}')
            return 0
        if args.load:
            tuner.load_yaml(args.load)
            return 0
        if args.preset:
            tuner.do_preset(args.preset)
            return 0
        if args.set:
            ok = True
            for pair in args.set:
                k, v = split_assignment(pair)
                if k is None:
                    print(colors.red(f'  expected K=V, got {pair!r}'))
                    ok = False
                    continue
                ok &= tuner.do_set(k, v)
            return 0 if ok else 1
        if args.watch:
            watch(node, colors)
            return 0

        # ── interactive ──────────────────────────────────────────────────────
        if readline:
            def complete(text, state):
                opts = [w for w in tuner.completions() + [
                    'ls', 'set', 'get', 'use', 'preset', 'diff', 'reset',
                    'yaml', 'load', 'help', 'quit'] if w.startswith(text)]
                return opts[state] if state < len(opts) else None
            readline.set_completer(complete)
            readline.parse_and_bind('tab: complete')
            readline.set_completer_delims(' \t\n=')

        print(colors.dim("\ntype 'help' for commands, 'w' to watch, 'q' to quit"))
        for ln in render_status(node, colors):
            print(ln)

        while rclpy.ok():
            try:
                raw = input(colors.cyan(f'\n{tuner.current}> ')).strip()
            except EOFError:
                print()
                break
            except KeyboardInterrupt:
                # Ctrl-C abandons the line being typed, as in any shell; 'q'
                # or Ctrl-D quits. Leaving a tuning session by accident means
                # losing every unsaved change.
                print(colors.dim('  (interrupted -- q or Ctrl-D to quit)'))
                continue
            if not raw:
                for ln in render_status(node, colors):
                    print(ln)
                continue

            parts = raw.split()
            cmd, rest = parts[0], parts[1:]

            if cmd in ('q', 'quit', 'exit'):
                break
            elif cmd in ('h', 'help', '?'):
                print(HELP.format(presets=', '.join(PRESETS),
                                  targets=', '.join(targets)))
            elif cmd in ('ls', 'list', 'l'):
                tuner.do_list(rest[0] if rest else None)
            elif cmd in ('w', 'watch'):
                watch(node, colors, duration=float(rest[0]) if rest else None)
            elif cmd == 'set':
                if len(rest) < 2:
                    print(colors.red('  usage: set <param> <value>'))
                else:
                    tuner.do_set(rest[0], ' '.join(rest[1:]))
            elif cmd == 'get':
                if not rest:
                    print(colors.red('  usage: get <param>'))
                else:
                    try:
                        tgt, name = tuner.resolve(rest[0])
                        print(f'  {tgt.alias}:{name} = {fmt(tgt.get(name))} '
                              f'{colors.dim(TYPE_NAMES.get(tgt.types[name], "?"))}')
                    except ParamLookupError as exc:
                        print(colors.red(f'  {exc}'))
            elif cmd == 'use':
                if rest and rest[0] in targets:
                    tuner.current = rest[0]
                else:
                    print(colors.red(f'  targets: {", ".join(targets)}'))
            elif cmd == 'preset':
                if rest:
                    tuner.do_preset(rest[0])
                else:
                    for n, p in PRESETS.items():
                        print(f'  {colors.bold(n):<14} {colors.dim(p["_doc"])}')
            elif cmd == 'diff':
                tuner.do_diff()
            elif cmd == 'reset':
                tuner.do_reset(rest[0] if rest else None)
            elif cmd == 'yaml':
                text = tuner.as_yaml()
                if rest:
                    with open(rest[0], 'w') as fh:
                        fh.write(text)
                    print(f'  wrote {rest[0]}')
                else:
                    print(text)
            elif cmd == 'load':
                if rest:
                    tuner.load_yaml(rest[0])
                else:
                    print(colors.red('  usage: load <file>'))
            elif '=' in raw:
                for pair in parts:
                    k, v = split_assignment(pair)
                    if k and v:
                        tuner.do_set(k, v)
            else:
                print(colors.red(f'  unknown command {cmd!r} — try help'))
        return 0
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
