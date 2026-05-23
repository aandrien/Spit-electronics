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
import time
from collections import OrderedDict

from flask import Flask, render_template, abort, send_from_directory, jsonify

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


def _filename_to_iso(filename):
    # session_2026-05-23_18-32-15.csv -> "2026-05-23 18:32:15"
    stamp = filename[len("session_"):-len(".csv")] if filename.startswith("session_") else filename
    if "_" in stamp:
        date_part, time_part = stamp.split("_", 1)
        return f"{date_part} {time_part.replace('-', ':')}"
    return stamp


@app.route("/")
def index():
    # Build the session list from the ts/ directory directly (not just from
    # sessions.csv), so the running session and any previously-crashed sessions
    # without a summary row still show up. Files whose summary row is missing
    # get a "running" badge if mtime is fresh, otherwise "incomplete".
    by_filename = {r.get("filename"): r for r in _read_csv(SESSIONS_CSV)}

    if os.path.isdir(TS_DIR):
        files = sorted(
            [f for f in os.listdir(TS_DIR) if f.endswith(".csv")],
            reverse=True,
        )
    else:
        files = []

    now = time.time()
    sessions = []
    for f in files:
        summary = by_filename.get(f)
        if summary:
            row = dict(summary)
            row["status"] = "done"
        else:
            try:
                mtime = os.path.getmtime(os.path.join(TS_DIR, f))
            except OSError:
                mtime = 0
            # 10 s of staleness covers the 1 s flush + a bit of slack. Anything
            # older than that without a summary row is a session that died
            # without a clean shutdown (power-cut etc).
            status = "running" if (now - mtime) < 10 else "incomplete"
            row = {
                "filename": f,
                "start_iso": _filename_to_iso(f),
                "status": status,
                "duration_s": "",
                "total_abs_rounds": "",
                "direction_flips": "",
                "avg_u": "",
                "max_u": "",
                "avg_abs_vel_error": "",
                "max_abs_vel_error": "",
                "crc_errors": "",
                "dropped_rows": "",
            }
        sessions.append(row)
    return render_template("sessions.html", sessions=sessions)


def _session_data(name):
    """Shared by the HTML page and the JSON polling endpoint."""
    if "/" in name or "\\" in name or ".." in name or not name.endswith(".csv"):
        return None
    path = os.path.join(TS_DIR, name)
    if not os.path.isfile(path):
        return None
    rows = _read_ts(path)
    return {
        "t": _column(rows, "t_s"),
        "series": {
            "vel_ref":        _column(rows, "vel_ref"),
            "vel_measured":   _column(rows, "vel_measured"),
            "vel_error":      _column(rows, "vel_error"),
            "u_duty":         _column(rows, "u_duty"),
            "error_integral": _column(rows, "error_integral"),
            "spit_angle_deg": _column(rows, "spit_angle_deg"),
            "pos_ref_deg":    _column(rows, "pos_ref_deg"),
            "pos_error_deg":  _column(rows, "pos_error_deg"),
        },
        "n_rows_displayed": len(rows),
    }


@app.route("/session/<name>")
def session(name):
    data = _session_data(name)
    if data is None:
        abort(404)
    return render_template("session.html", name=name, **data)


@app.route("/session/<name>/data")
def session_data(name):
    data = _session_data(name)
    if data is None:
        abort(404)
    return jsonify(data)


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
