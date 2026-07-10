#!/usr/bin/env python3
"""
桥接状态 CSV 记录器。

本脚本只通过 HTTP 读取 bridge 的 /status，不访问串口、不发送电机命令。
它用于记录慢速 micro / push_preview 测试中的：

- bridge 目标角与实际角；
- common 机械目标角与实际角；
- 跟踪误差；
- bridge 报错与四连杆映射错误。

使用示例：

    /home/claww/miniforge3/envs/go2-convex-mpc/bin/python \
      scripts/40_record_leg_bridge_status.py \
      --duration-sec 20 \
      --dt 0.05 \
      --csv ~/leg_logs/micro_001.csv
"""

import argparse
import csv
import json
import time
import urllib.request
from datetime import datetime
from pathlib import Path


BRIDGE_FIELDS = ("hip_motor", "thigh_motor", "calf_motor")
COMMON_FIELDS = ("hip_abduction", "thigh_pitch", "knee_pitch")


def get_status(base_url, timeout_sec):
    """读取 bridge 状态；不控制电机。"""
    url = f"{base_url.rstrip('/')}/status"
    with urllib.request.urlopen(url, timeout=timeout_sec) as response:
        return json.loads(response.read().decode("utf-8"))


def value_or_blank(mapping, key):
    """映射缺失时写空白，避免日志器因单次几何映射错误中断。"""
    if not isinstance(mapping, dict):
        return ""
    value = mapping.get(key)
    return "" if value is None else value


def build_row(status, elapsed_sec):
    """将 bridge JSON 展开为一行 CSV。"""
    row = {
        "wall_time": datetime.now().isoformat(timespec="milliseconds"),
        "elapsed_sec": f"{elapsed_sec:.3f}",
        "motors_ready": status.get("motors_ready", False),
        "last_error": status.get("last_error", ""),
        "common_mapping_error": status.get("common_mapping_error", ""),
    }

    for prefix, status_key in (
        ("target_bridge", "target_deg"),
        ("current_bridge", "current_deg"),
        ("error_bridge", "tracking_error_bridge_deg"),
    ):
        mapping = status.get(status_key, {})
        for name in BRIDGE_FIELDS:
            row[f"{prefix}_{name}_deg"] = value_or_blank(mapping, name)

    for prefix, status_key in (
        ("target_common", "target_common_deg"),
        ("current_common", "current_common_deg"),
        ("error_common", "tracking_error_common_deg"),
    ):
        mapping = status.get(status_key, {})
        for name in COMMON_FIELDS:
            row[f"{prefix}_{name}_deg"] = value_or_blank(mapping, name)

    return row


def main():
    parser = argparse.ArgumentParser(
        description="Record bridge /status to CSV without commanding motors."
    )
    parser.add_argument("--bridge-url", default="http://127.0.0.1:8765")
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--timeout-sec", type=float, default=1.0)
    parser.add_argument("--csv", required=True)
    args = parser.parse_args()

    if args.duration_sec <= 0.0:
        raise ValueError("--duration-sec must be positive")
    if args.dt <= 0.0:
        raise ValueError("--dt must be positive")

    csv_path = Path(args.csv).expanduser()
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    first_status = get_status(args.bridge_url, args.timeout_sec)
    first_row = build_row(first_status, elapsed_sec=0.0)
    fieldnames = list(first_row.keys())

    print(f"bridge: {args.bridge_url}")
    print(f"csv:    {csv_path}")
    print(f"duration={args.duration_sec:.2f}s, dt={args.dt:.3f}s")
    print("This script only reads /status; it does not command motors.")

    t0 = time.monotonic()
    next_time = t0
    samples = 0

    with csv_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()

        while True:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed > args.duration_sec:
                break

            try:
                status = get_status(args.bridge_url, args.timeout_sec)
                row = build_row(status, elapsed)
                writer.writerow(row)
                file.flush()
                samples += 1

                if samples % max(1, int(1.0 / args.dt)) == 0:
                    common = status.get("current_common_deg", {})
                    error = status.get("tracking_error_common_deg", {})
                    print(
                        f"t={elapsed:6.2f}s "
                        f"common_now={common} "
                        f"common_error={error}"
                    )
            except Exception as exc:
                print(f"WARNING: cannot read /status: {exc}")

            next_time += args.dt
            time.sleep(max(0.0, next_time - time.monotonic()))

    print(f"Done. samples={samples}, csv={csv_path}")


if __name__ == "__main__":
    main()