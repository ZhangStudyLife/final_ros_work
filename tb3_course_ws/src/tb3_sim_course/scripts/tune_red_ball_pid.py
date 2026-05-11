#!/usr/bin/env python3

import argparse
import re
import signal
import subprocess
import time


STATUS_RE = re.compile(
    r"radius_px=(?P<radius>[-0-9.]+).*cmd=\((?P<linear>[-0-9.]+),(?P<angular>[-0-9.]+)\)"
)


PROFILES = [
    {
        "name": "stable",
        "linear_kp": 0.010,
        "linear_ki": 0.0005,
        "linear_kd": 0.0020,
        "angular_kp": 0.0065,
        "angular_ki": 0.0001,
        "angular_kd": 0.0020,
        "max_linear": 0.34,
        "max_angular": 1.25,
    },
    {
        "name": "balanced",
        "linear_kp": 0.012,
        "linear_ki": 0.0008,
        "linear_kd": 0.0030,
        "angular_kp": 0.0080,
        "angular_ki": 0.0002,
        "angular_kd": 0.0025,
        "max_linear": 0.42,
        "max_angular": 1.65,
    },
    {
        "name": "fast",
        "linear_kp": 0.015,
        "linear_ki": 0.0010,
        "linear_kd": 0.0035,
        "angular_kp": 0.0100,
        "angular_ki": 0.0002,
        "angular_kd": 0.0035,
        "max_linear": 0.48,
        "max_angular": 1.90,
    },
]


def launch_profile(profile):
    args = [
        "roslaunch",
        "tb3_sim_course",
        "red_ball_tracking.launch",
        "open_viewers:=false",
    ]
    for key, value in profile.items():
        if key != "name":
            args.append("{}:={}".format(key, value))
    return subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, preexec_fn=None)


def sample_status(duration):
    proc = subprocess.Popen(
        ["rostopic", "echo", "/red_ball/status"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    samples = []
    start = time.time()
    try:
        while time.time() - start < duration:
            line = proc.stdout.readline()
            if not line:
                continue
            match = STATUS_RE.search(line)
            if match:
                samples.append({
                    "radius": float(match.group("radius")),
                    "linear": float(match.group("linear")),
                    "angular": float(match.group("angular")),
                })
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
    return samples


def score(samples, desired_radius):
    if not samples:
        return 9999.0, {}
    radius_errors = [abs(s["radius"] - desired_radius) for s in samples]
    angular_effort = [abs(s["angular"]) for s in samples]
    linear_effort = [abs(s["linear"]) for s in samples]
    avg_radius_error = sum(radius_errors) / len(radius_errors)
    avg_angular = sum(angular_effort) / len(angular_effort)
    avg_linear = sum(linear_effort) / len(linear_effort)
    metric = avg_radius_error + 4.0 * avg_angular - 2.0 * avg_linear
    return metric, {
        "samples": len(samples),
        "avg_radius_error": avg_radius_error,
        "avg_abs_angular": avg_angular,
        "avg_abs_linear": avg_linear,
    }


def stop_process(proc):
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            proc.wait(timeout=4)
        except subprocess.TimeoutExpired:
            proc.kill()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--warmup", type=float, default=8.0)
    parser.add_argument("--desired-radius", type=float, default=46.0)
    args = parser.parse_args()

    results = []
    for profile in PROFILES:
        print("Testing", profile["name"])
        proc = launch_profile(profile)
        time.sleep(args.warmup)
        samples = sample_status(args.duration)
        metric, detail = score(samples, args.desired_radius)
        results.append((metric, profile, detail))
        stop_process(proc)
        time.sleep(3)

    results.sort(key=lambda row: row[0])
    print("\nPID tuning result:")
    for metric, profile, detail in results:
        print(profile["name"], "score={:.3f}".format(metric), detail, profile)
    print("\nRecommended:", results[0][1]["name"])


if __name__ == "__main__":
    main()
