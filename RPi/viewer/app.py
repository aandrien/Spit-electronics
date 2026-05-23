"""Flask viewer for SpitTronics log files.

Reads CSVs written by RPi/spit_logger.py and renders:
  /                  — sessions table, newest first
  /session/<name>    — Plotly charts for one session
  /events            — event log table
  /download/<name>   — raw CSV download for one session

Designed to run on the Pi as a systemd service (spit-viewer.service)
alongside the control script. Uses only stdlib + Flask; charts use
Plotly loaded from a CDN.
"""

import csv
import os
from collections import OrderedDict

from flask import Flask, render_template, abort, send_from_directory

LOG_DIR = os.path.expanduser("~/.spit_logs")
TS_DIR = os.path.join(LOG_DIR, "ts")
SESSIONS_CSV = os.path.join(LOG_DIR, "sessions.csv")
EVENTS_CSV = os.path.join(LOG_DIR, "events.csv")

# Above this row count we sample-down before sending to Plotly so the
# browser stays responsive. A 4-hour session at 20 Hz is ~288k rows;
# 4000 points is plenty for visual inspection.
MAX_PLOT_POINTS = 4000

app = Flask(__name__)


def _read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _read_ts(path):
    """Read a time-series file, downsampling if it's huge."""
    rows = _read_csv(path)
    if len(rows) <= MAX_PLOT_POINTS:
        return rows
    stride = max(1, len(rows) // MAX_PLOT_POINTS)
    return rows[::stride]


def _column(rows, name, cast=float):
    out = []
    for r in rows:
        v = r.get(name, "")
        if v == "" or v is None:
            out.append(None)
            continue
        try:
            out.append(cast(v))
        except (ValueError, TypeError):
            out.append(None)
    return out


@app.route("/")
def index():
    sessions = _read_csv(SESSIONS_CSV)
    sessions.sort(key=lambda r: r.get("start_iso", ""), reverse=True)
    return render_template("sessions.html", sessions=sessions)


@app.route("/session/<name>")
def session(name):
    # Path-traversal guard: only accept simple session_*.csv names.
    if "/" in name or "\\" in name or ".." in name or not name.endswith(".csv"):
        abort(404)
    path = os.path.join(TS_DIR, name)
    if not os.path.isfile(path):
        abort(404)
    rows = _read_ts(path)

    t = _column(rows, "t_s")
    series = OrderedDict([
        ("vel_ref",        _column(rows, "vel_ref")),
        ("vel_measured",   _column(rows, "vel_measured")),
        ("vel_error",      _column(rows, "vel_error")),
        ("u_duty",         _column(rows, "u_duty")),
        ("error_integral", _column(rows, "error_integral")),
        ("spit_angle_deg", _column(rows, "spit_angle_deg")),
        ("pos_ref_deg",    _column(rows, "pos_ref_deg")),
        ("pos_error_deg",  _column(rows, "pos_error_deg")),
    ])
    return render_template(
        "session.html",
        name=name,
        t=t,
        series=series,
        n_rows_displayed=len(rows),
    )


@app.route("/download/<name>")
def download(name):
    if "/" in name or "\\" in name or ".." in name or not name.endswith(".csv"):
        abort(404)
    return send_from_directory(TS_DIR, name, as_attachment=True)


@app.route("/events")
def events():
    rows = _read_csv(EVENTS_CSV)
    rows.reverse()  # newest first
    return render_template("events.html", events=rows[:500])


if __name__ == "__main__":
    # Bind to all interfaces so the dashboard is reachable from the LAN.
    app.run(host="0.0.0.0", port=5000)
