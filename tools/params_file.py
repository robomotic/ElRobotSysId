"""servo_params.toml and sysid_safety.toml for the Python tools (issue #5): a small TOML reader (sections, key = value,
numbers, strings, booleans, arrays over several lines) and an updater that changes values in place and keeps comments.
No dependency: Python 3.8 has no tomllib.

servo_params.toml: `[default]` applies to every joint, `[servo.Mn]` overrides what was measured on that joint.
sysid_safety.toml: the calibration (zero, sign, ranges) stays there, the one place for it.
"""
import ast
import os
import re

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
PARAMS = os.path.join(REPO, "servo_params.toml")
SAFETY = os.path.join(REPO, "sysid_safety.toml")


def _strip_comment(line):
    out, q = [], False
    for ch in line:
        if ch == '"':
            q = not q
        if ch == "#" and not q:
            break
        out.append(ch)
    return "".join(out).strip()


def _value(text):
    t = re.sub(r"\btrue\b", "True", re.sub(r"\bfalse\b", "False", text))
    return ast.literal_eval(t)


def parse(text):
    """{section: {key: value}}; keys before any section go under ''."""
    data, sec = {"": {}}, ""
    pending, key = None, None
    for raw in text.splitlines():
        line = _strip_comment(raw)
        if pending is not None:
            pending += " " + line
            if pending.count("[") <= pending.count("]"):
                data[sec][key] = _value(pending)
                pending = None
            continue
        if not line:
            continue
        m = re.fullmatch(r"\[([A-Za-z0-9_.]+)\]", line)
        if m:
            sec = m.group(1)
            data.setdefault(sec, {})
            continue
        m = re.fullmatch(r"([A-Za-z0-9_]+)\s*=\s*(.*)", line)
        if not m:
            raise ValueError("cannot parse TOML line: %r" % raw)
        key, val = m.group(1), m.group(2)
        if val.startswith("[") and val.count("[") > val.count("]"):
            pending = val
        else:
            data[sec][key] = _value(val)
    return data


def load(path):
    with open(path) as f:
        return parse(f.read())


def joint_params(joint, path=PARAMS):
    """`[default]` merged with `[servo.M<joint>]`; {} when the file does not exist."""
    if not os.path.exists(path):
        return {}
    d = load(path)
    out = dict(d.get("default", {}))
    out.update(d.get("servo.M%d" % joint, {}))
    return out


def calibration(path=SAFETY):
    """zero, sign (M1..M7) and range_min/max (M1..M8) from sysid_safety.toml, or None for missing keys."""
    if not os.path.exists(path):
        return {}
    top = load(path).get("", {})
    return {k: top.get(k) for k in ("zero", "sign", "range_min", "range_max")}


def _fmt(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, str):
        return '"%s"' % v
    if isinstance(v, int):
        return str(v)
    t = ("%.4g" % v) if abs(v) >= 1e-3 or v == 0 else ("%.3e" % v)
    return t if any(c in t for c in ".en") else t + ".0"  # a whole float stays a float


def update(joint, values, source=None, path=PARAMS):
    """Set `values` (key -> number) in [servo.M<joint>], keeping every other line and comment. Creates the section
    (and the file's header keys `fitted`/`source` are updated when `source` is given)."""
    with open(path) as f:
        lines = f.read().splitlines()
    sec_name = "[servo.M%d]" % joint
    start = next((i for i, l in enumerate(lines) if l.strip() == sec_name), None)
    if start is None:
        if lines and lines[-1].strip():
            lines.append("")
        lines += [sec_name]
        start = len(lines) - 1
    end = next((i for i in range(start + 1, len(lines)) if lines[i].strip().startswith("[")), len(lines))
    while end > start + 1 and not lines[end - 1].strip():
        end -= 1
    for key, val in values.items():
        pat = re.compile(r"^(\s*%s\s*=\s*)([^#]*?)(\s*(#.*)?)$" % re.escape(key))
        for i in range(start + 1, end):
            m = pat.match(lines[i])
            if m:
                lines[i] = m.group(1) + _fmt(val) + (m.group(3) or "")
                break
        else:
            lines.insert(end, "%s = %s" % (key, _fmt(val)))
            end += 1
    if source:
        import datetime
        for key, val in (("fitted", datetime.date.today().isoformat()), ("source", source)):
            pat = re.compile(r"^(%s\s*=\s*)(\"[^\"]*\")(.*)$" % key)
            for i, l in enumerate(lines):
                if l.strip().startswith("["):
                    break
                m = pat.match(l)
                if m:
                    lines[i] = m.group(1) + _fmt(val) + m.group(3)
                    break
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
