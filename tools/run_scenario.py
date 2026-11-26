#!/usr/bin/env python3
"""Fly a SITL scenario headless, record it, tear down, and say whether it did what it asked
(roadmap R2.6a). The SITL equivalent of tools/bench.sh. Run it in the dev image with the
GPU (tools/docker_dev.sh run --gpu -- ...).

    tools/run_scenario.py config/scenarios/x500_emmental_smoke.yaml
    tools/run_scenario.py config/scenarios/*.yaml                  # one after another
    tools/run_scenario.py config/scenarios/*.yaml --runs 3         # each three times, then the set scored
    tools/run_scenario.py SCENARIO --out log/scenarios --gui       # with Gazebo and RViz windows

For each scenario (tools/scenario.py: the file format):
  1. a run directory, OUT/<name>/<UTC time>/, with a copy of the scenario;
  2. tools/sim_run.sh with the scenario's settings, headless, in its own process group,
     under the scenario's time limit (past it: Ctrl+C, so sim_run.sh tears down; then kill);
  3. teardown: nothing of the simulation may be left running (anything found is stopped,
     and the run fails);
  4. the recording: the bag (sitl_bag) and GDN's run log (log/gdn_runs/...) written, and
     the run log replays bit for bit (gdn_replay);
  5. tools/check_scenario.py: the run did what the scenario asked;
  6. tools/score_run.py (R2.7a): GDN scored against the requirements, metrics.json and a
     short report (report.md, report.png). While the run flies, eskf_node's CPU and
     memory are sampled once a second into cpu.csv. A score is a measurement, not a
     verdict on the scenario: it doesn't change PASSED or FAILED.
  7. with more than one run: tools/score_set.py (R2.7b) pools them per scenario and compares
     them with the accepted baseline (config/baselines/sitl.json), into OUT/set_<UTC time>/.
     --runs N flies the list N times over (all scenarios, then all again), so a set's runs
     are spread over the session. The comparison is reported, not part of the exit code
     (tools/score_set.py's own exit code says WORSE).

result.json in the run directory holds every step's outcome. The exit code is 0 when
every scenario passed, 1 if any failed, 2 if a scenario file is wrong.
"""
import argparse
import datetime
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import scenario as scn  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
# Everything tools/sim_run.sh starts (its SIM_PROCS), to make sure none is left.
SIM_PROCS = ("px4_sitl_default/bin/px4", "gz sim", "MicroXRCEAgent", "ros_gz_bridge/parameter_bridge",
             "gdn_fusion/sim_sensors", "tools/sim_adapter.py", "tools/sim_spoof.py", "tools/replay_scorer.py", "tools/sim_fly.py",
             "gdn_fusion/eskf_node", "gdn_fusion/map_viz_node", "ros2 bag record -s mcap -o")


def left_running():
    out = []
    for pat in SIM_PROCS:
        r = subprocess.run(["pgrep", "-af", pat], capture_output=True, text=True)
        for line in r.stdout.splitlines():
            pid, _, cmd = line.partition(" ")
            if int(pid) == os.getpid() or "pgrep" in cmd or "run_scenario.py" in cmd:
                continue
            out.append((int(pid), cmd[:120]))
    return out


def stop(pids):
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for pid, _ in pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        time.sleep(3)


def sample_cpu(path, stop_evt, pattern="lib/gdn_fusion/eskf_node"):
    """eskf_node's CPU (share of one core) and resident memory, once a second, to cpu.csv."""
    tick = os.sysconf("SC_CLK_TCK")
    page_mb = os.sysconf("SC_PAGE_SIZE") / 1e6
    last = None
    with open(path, "w") as f:
        f.write("t,cpu_pct,rss_mb\n")
        while not stop_evt.wait(1.0):
            r = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True)
            pids = [int(x) for x in r.stdout.split()]
            if not pids:
                last = None
                continue
            try:
                stat = Path(f"/proc/{pids[0]}/stat").read_text().rsplit(")", 1)[1].split()
                cpu_s = (int(stat[11]) + int(stat[12])) / tick          # utime + stime
                rss = int(Path(f"/proc/{pids[0]}/statm").read_text().split()[1]) * page_mb
            except (OSError, IndexError, ValueError):
                continue
            now = time.time()
            if last is not None and last[0] == pids[0]:
                f.write(f"{now:.3f},{100.0 * (cpu_s - last[1]) / (now - last[2]):.2f},{rss:.1f}\n")
                f.flush()
            last = (pids[0], cpu_s, now)


def git_describe():
    r = subprocess.run(["git", "-C", str(ROOT), "describe", "--always", "--dirty"], capture_output=True, text=True)
    return r.stdout.strip() or "unknown"


def run_one(path, out_root, gui):
    s = scn.load(path)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%d_%H%M%S")
    run = (Path(out_root) / s["name"] / stamp).resolve()
    run.mkdir(parents=True)
    shutil.copy(scn.ROOT / path if not Path(path).exists() else path, run / "scenario.yaml")
    env = dict(os.environ)
    env.update(scn.sim_env(s))
    env["GDN_LOG_DIR"] = str(run)
    if gui:
        env.pop("HEADLESS", None)
    res = {"scenario": s["name"], "file": str(path), "run_dir": str(run), "gdn_version": git_describe(),
           "started_utc": stamp, "settings": scn.sim_env(s), "timeout_s": s["limits"]["timeout_s"]}
    print(f"run_scenario: {s['name']} -> {run}")
    before = left_running()
    if before:
        print("run_scenario: simulation processes already running (stopping them first): "
              + ", ".join(c for _, c in before))
        stop(before)
    t0 = time.time()
    stop_cpu = threading.Event()
    cpu_thread = threading.Thread(target=sample_cpu, args=(run / "cpu.csv", stop_cpu), daemon=True)
    cpu_thread.start()
    with open(run / "sim_run.log", "w") as log:
        p = subprocess.Popen([str(ROOT / "tools/sim_run.sh")], cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                             start_new_session=True)
        try:
            res["sim_run_exit"] = p.wait(timeout=s["limits"]["timeout_s"])
            res["timed_out"] = False
        except subprocess.TimeoutExpired:
            res["timed_out"] = True
            print(f"run_scenario: past the {s['limits']['timeout_s']:g} s limit: stopping (Ctrl+C, as sim_run.sh expects)")
            os.killpg(p.pid, signal.SIGINT)
            try:
                res["sim_run_exit"] = p.wait(timeout=90)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                res["sim_run_exit"] = p.wait()
    stop_cpu.set()
    cpu_thread.join(timeout=5)
    res["duration_s"] = round(time.time() - t0, 1)
    time.sleep(2)
    left = left_running()
    res["teardown_clean"] = not left
    res["left_running"] = [c for _, c in left]
    if left:
        print("run_scenario: left running after the run (stopping them): " + ", ".join(c for _, c in left))
        stop(left)
    # The recording.
    bag = run / "sitl_bag"
    res["bag_ok"] = (bag / "metadata.yaml").exists()
    res["bag_mb"] = round(sum(f.stat().st_size for f in bag.glob("*")) / 1e6, 1) if bag.is_dir() else 0.0
    m = re.search(r"run log: (\S+)", (run / "gdn.log").read_text()) if (run / "gdn.log").exists() else None
    res["run_log"] = m.group(1) if m else None
    res["run_log_ok"] = bool(m) and (Path(m.group(1)) / "metadata.yaml").exists()
    if res["run_log_ok"]:
        rp = subprocess.run([str(ROOT / "install/gdn_fusion/lib/gdn_fusion/gdn_replay"), res["run_log"]],
                            capture_output=True, text=True)
        (run / "replay.txt").write_text(rp.stdout + rp.stderr)
        if rp.returncode == 0 and "IDENTICAL" in rp.stdout:
            res["replay"] = "IDENTICAL"
        else:
            bad = re.search(r"^(DIFFERENT.*|ERROR.*)$", rp.stdout, re.M)
            res["replay"] = bad.group(1) if bad else f"exit {rp.returncode}"
    else:
        res["replay"] = "no run log"
    (run / "result.json").write_text(json.dumps(res, indent=1) + "\n")
    # Did it do what it asked?
    ck = subprocess.run([sys.executable, str(ROOT / "tools/check_scenario.py"), "--run", str(run)],
                        capture_output=True, text=True, cwd=ROOT)
    (run / "check.txt").write_text(ck.stdout + ck.stderr)
    res["check_passed"] = ck.returncode == 0
    res["checks_failed"] = [l[6:] for l in ck.stdout.splitlines() if l.startswith("FAIL")]
    res["passed"] = res["check_passed"]
    (run / "result.json").write_text(json.dumps(res, indent=1) + "\n")
    print(ck.stdout.rstrip())
    # GDN's score (R2.7a): a measurement, not part of the verdict.
    if res["run_log_ok"]:
        sr = subprocess.run([sys.executable, str(ROOT / "tools/score_run.py"), str(run)], capture_output=True, text=True,
                            cwd=ROOT)
        (run / "score.txt").write_text(sr.stdout + sr.stderr)
        if sr.returncode == 0 and (run / "metrics.json").exists():
            m = json.loads((run / "metrics.json").read_text())
            w5 = m["windows"]["300"]
            res["score"] = {"requirement": m["requirement"], "cep95_5min_m": w5["cep95_m"], "max_5min_m": w5["max_m"],
                            "cep95_ok": w5["cep95_ok"], "max_ok": w5["max_ok"],
                            "over3sigma_pct": m["consistency"]["over3sigma_pct"], "report": str(run / "report.md")}
            print(sr.stdout.rstrip())
        else:
            res["score"] = {"error": (sr.stderr or sr.stdout).strip().splitlines()[-1:]}
            print(f"run_scenario: scoring failed (see {run / 'score.txt'})")
        (run / "result.json").write_text(json.dumps(res, indent=1) + "\n")
    print(f"run_scenario: {s['name']}: {'PASSED' if res['passed'] else 'FAILED'} in {res['duration_s']:.0f} s "
          f"({run})")
    return res["passed"], run


def main():
    sys.stdout.reconfigure(line_buffering=True)   # progress shows as it happens, even into a file
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("scenarios", nargs="+", help="scenario files (config/scenarios/*.yaml)")
    ap.add_argument("--out", default=str(ROOT / "log/scenarios"), help="where run directories go")
    ap.add_argument("--gui", action="store_true", help="Gazebo and RViz windows (default headless)")
    ap.add_argument("--runs", type=int, default=1, help="fly the list this many times over (default 1)")
    a = ap.parse_args()
    try:
        for f in a.scenarios:
            scn.load(f)
    except (scn.ScenarioError, OSError) as e:
        print(f"run_scenario: {e}")
        return 2
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%d_%H%M%S")
    results = []
    for k in range(a.runs):
        for f in a.scenarios:
            if a.runs > 1:
                print(f"run_scenario: run {k + 1} of {a.runs}")
            results.append((f,) + run_one(f, a.out, a.gui))
    if len(results) > 1:
        print("\nrun_scenario: " + ", ".join(f"{Path(f).stem} {'passed' if ok else 'FAILED'}" for f, ok, _ in results))
        scored = [str(r) for _, _, r in results if (r / "series.npz").exists()]
        if len(scored) > 1:
            out = Path(a.out) / f"set_{stamp}"
            ss = subprocess.run([sys.executable, str(ROOT / "tools/score_set.py"), *scored, "--out", str(out)],
                                capture_output=True, text=True, cwd=ROOT)
            print(ss.stdout.rstrip() or ss.stderr.rstrip())
    return 0 if all(ok for _, ok, _ in results) else 1


if __name__ == "__main__":
    sys.exit(main())
