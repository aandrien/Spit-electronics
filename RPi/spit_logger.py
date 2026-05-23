"""Background data logger for the SpitTronics control script.

The control loop pushes telemetry rows into a bounded queue; a dedicated
writer thread drains the queue into per-session CSV files. Pushes are
non-blocking: if the writer ever falls behind, rows are dropped and
counted rather than back-pressuring the control loop.

Files under ~/.spit_logs/:
  sessions.csv   one row per Pi-commanded motor-on window (summary stats)
  events.csv     append-only event log (gain changes, mode flips, CRC errors)
  ts/session_YYYY-MM-DD_HH-MM-SS.csv   20 Hz time-series, one per session

The ts/ directory is capped at MAX_TS_BYTES; oldest files are deleted on
startup and after every session close.

Session boundaries are decided by the caller (the main script's start/stop
edge handler), not here. This module just takes start/end signals.
"""

import csv
import os
import threading
import time
from datetime import datetime
from queue import Queue, Empty, Full

LOG_DIR = os.path.expanduser("~/.spit_logs")
TS_DIR = os.path.join(LOG_DIR, "ts")
SESSIONS_CSV = os.path.join(LOG_DIR, "sessions.csv")
EVENTS_CSV = os.path.join(LOG_DIR, "events.csv")

MAX_TS_BYTES = 500 * 1024 * 1024   # 500 MB cap on ts/ directory total
QUEUE_MAX = 2000
FLUSH_INTERVAL_S = 1.0
FLUSH_ROWS = 100
COUNTS_PER_ROUND = 305             # matches the existing num_rounds divisor

TS_COLUMNS = [
    "t_s", "vel_ref", "vel_measured", "vel_error", "u_duty",
    "error_integral", "encoder_count", "spit_angle_deg",
    "direction", "control_mode", "pos_ref_deg", "pos_error_deg",
]

SESSION_COLUMNS = [
    "start_iso", "end_iso", "duration_s", "filename",
    "num_rows", "total_abs_rounds", "direction_flips",
    "avg_u", "max_u", "avg_abs_vel_error", "max_abs_vel_error",
    "crc_errors", "dropped_rows",
]

EVENT_COLUMNS = ["timestamp_iso", "event_type", "details"]


class SpitLogger:
    def __init__(self):
        os.makedirs(TS_DIR, exist_ok=True)
        self._ensure_header(SESSIONS_CSV, SESSION_COLUMNS)
        self._ensure_header(EVENTS_CSV, EVENT_COLUMNS)
        self._run_retention()

        self._q = Queue(maxsize=QUEUE_MAX)
        self._thread = None
        self._dropped = 0
        self._dropped_lock = threading.Lock()

    def start(self):
        self._thread = threading.Thread(target=self._run, name="SpitLogger", daemon=True)
        self._thread.start()

    def stop(self, timeout=2.0):
        try:
            self._q.put(("stop", None), timeout=0.5)
        except Full:
            pass
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    # ---- thread-safe public API ----

    def push_telemetry(self, row):
        # Non-blocking. Stamping the wall clock here (on the caller's thread)
        # rather than at dequeue keeps per-sample timestamps honest even if
        # the writer batches several rows in one wake-up.
        row = dict(row)
        row["_t"] = time.time()
        try:
            self._q.put_nowait(("row", row))
        except Full:
            with self._dropped_lock:
                self._dropped += 1

    def start_session(self):
        # Blocking put is fine — session edges are rare and not on the hot path.
        self._q.put(("start", None))

    def end_session(self):
        self._q.put(("end", None))

    def note_crc_error(self):
        self._q.put(("crc", None))

    def log_event(self, event_type, details=""):
        """Append a low-volume event synchronously (no queue)."""
        try:
            with open(EVENTS_CSV, "a", newline="") as f:
                csv.writer(f).writerow([
                    datetime.now().isoformat(timespec="seconds"),
                    event_type,
                    details,
                ])
        except OSError as e:
            print(f"SpitLogger: failed to write event: {e}")

    # ---- writer thread ----

    def _run(self):
        session = None
        last_flush = time.time()
        rows_since_flush = 0

        while True:
            try:
                msg = self._q.get(timeout=FLUSH_INTERVAL_S)
            except Empty:
                msg = None

            now = time.time()

            if msg is not None:
                kind, payload = msg
                if kind == "stop":
                    if session is not None:
                        self._close_session(session)
                    return
                elif kind == "start":
                    if session is not None:
                        # start without preceding end — close gracefully and reopen
                        self._close_session(session)
                    session = self._open_session(now)
                    last_flush = now
                    rows_since_flush = 0
                elif kind == "end":
                    if session is not None:
                        self._close_session(session)
                        session = None
                elif kind == "crc":
                    if session is not None:
                        session["crc_errors"] += 1
                elif kind == "row":
                    if session is not None:
                        self._append_row(session, payload)
                        rows_since_flush += 1

            if session is not None:
                if rows_since_flush >= FLUSH_ROWS or (now - last_flush) >= FLUSH_INTERVAL_S:
                    try:
                        session["file"].flush()
                    except OSError:
                        pass
                    last_flush = now
                    rows_since_flush = 0

    def _open_session(self, t_now):
        stamp = datetime.fromtimestamp(t_now).strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"session_{stamp}.csv"
        path = os.path.join(TS_DIR, filename)
        f = open(path, "w", newline="")
        writer = csv.DictWriter(f, fieldnames=TS_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        return {
            "start": t_now,
            "start_iso": datetime.fromtimestamp(t_now).isoformat(timespec="seconds"),
            "filename": filename,
            "path": path,
            "file": f,
            "writer": writer,
            "first_row_t": None,
            "num_rows": 0,
            "abs_pulses": 0,
            "prev_encoder": None,
            "prev_direction": None,
            "direction_flips": 0,
            "sum_u": 0,
            "max_u": 0,
            "sum_abs_vel_err": 0.0,
            "max_abs_vel_err": 0.0,
            "crc_errors": 0,
        }

    def _append_row(self, session, row):
        t = row.get("_t", time.time())
        if session["first_row_t"] is None:
            session["first_row_t"] = t
        row["t_s"] = round(t - session["first_row_t"], 3)
        session["writer"].writerow(row)
        session["num_rows"] += 1

        u = row.get("u_duty") or 0
        session["sum_u"] += u
        if u > session["max_u"]:
            session["max_u"] = u

        verr = row.get("vel_error") or 0.0
        abs_verr = abs(verr)
        session["sum_abs_vel_err"] += abs_verr
        if abs_verr > session["max_abs_vel_err"]:
            session["max_abs_vel_err"] = abs_verr

        enc = row.get("encoder_count")
        if enc is not None:
            if session["prev_encoder"] is not None:
                session["abs_pulses"] += abs(enc - session["prev_encoder"])
            session["prev_encoder"] = enc

        d = row.get("direction")
        if d is not None:
            if session["prev_direction"] is not None and d != session["prev_direction"]:
                session["direction_flips"] += 1
            session["prev_direction"] = d

    def _close_session(self, session):
        try:
            session["file"].close()
        except OSError:
            pass

        end_t = time.time()
        end_iso = datetime.fromtimestamp(end_t).isoformat(timespec="seconds")
        duration = max(0.0, end_t - session["start"])
        n = session["num_rows"]
        avg_u = (session["sum_u"] / n) if n else 0
        avg_err = (session["sum_abs_vel_err"] / n) if n else 0.0

        with self._dropped_lock:
            dropped = self._dropped
            self._dropped = 0

        row = {
            "start_iso": session["start_iso"],
            "end_iso": end_iso,
            "duration_s": round(duration, 2),
            "filename": session["filename"],
            "num_rows": n,
            "total_abs_rounds": round(session["abs_pulses"] / COUNTS_PER_ROUND, 2),
            "direction_flips": session["direction_flips"],
            "avg_u": round(avg_u, 2),
            "max_u": session["max_u"],
            "avg_abs_vel_error": round(avg_err, 3),
            "max_abs_vel_error": round(session["max_abs_vel_err"], 3),
            "crc_errors": session["crc_errors"],
            "dropped_rows": dropped,
        }
        try:
            with open(SESSIONS_CSV, "a", newline="") as f:
                csv.DictWriter(f, fieldnames=SESSION_COLUMNS).writerow(row)
        except OSError as e:
            print(f"SpitLogger: failed to append session summary: {e}")

        self._run_retention()

    @staticmethod
    def _ensure_header(path, columns):
        if not os.path.exists(path) or os.path.getsize(path) == 0:
            with open(path, "w", newline="") as f:
                csv.writer(f).writerow(columns)

    @staticmethod
    def _run_retention():
        try:
            entries = []
            total = 0
            for name in os.listdir(TS_DIR):
                full = os.path.join(TS_DIR, name)
                if not os.path.isfile(full):
                    continue
                st = os.stat(full)
                entries.append((st.st_mtime, st.st_size, full))
                total += st.st_size
            entries.sort()  # oldest first
            for _mtime, size, full in entries:
                if total <= MAX_TS_BYTES:
                    break
                try:
                    os.remove(full)
                    total -= size
                except OSError:
                    pass
        except FileNotFoundError:
            pass
