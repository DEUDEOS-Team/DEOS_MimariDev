#!/usr/bin/env python3
import json
import os
import subprocess
import time
from dataclasses import dataclass


def sh(cmd: list[str], timeout_s: int = 8) -> tuple[int, str]:
    p = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout_s,
    )
    return p.returncode, p.stdout.strip()


@dataclass(frozen=True)
class HealthSpec:
    critical_nodes: list[str]
    require_stm32_subscriber: bool = True


SPEC = HealthSpec(
    critical_nodes=[
        "/realsense_d415_node",
        "/gps_node",
        "/imu_node",
        "/sick_multiscan165",
        "/stereo_detector_node",
        "/lidar_obstacle_node",
        "/perception_fusion_node",
        "/pcl_localization_node",
        "/mission_planning_node",
        "/vehicle_controller_node",
    ],
    require_stm32_subscriber=False,  # yarışma günü STM32 offline olabilir → WARN olarak raporla
)

INTERVAL_S = int(os.environ.get("DEOS_HEALTH_INTERVAL_S", "10"))
FAIL_THRESHOLD = int(os.environ.get("DEOS_HEALTH_FAIL_THRESHOLD", "3"))
REPO_ROOT = os.environ.get("REPO_ROOT", "/opt/deos")


def docker_ps() -> dict[str, str]:
    rc, out = sh(["docker", "ps", "--format", "{{.Names}} {{.Status}}"], timeout_s=5)
    if rc != 0:
        return {}
    result: dict[str, str] = {}
    for line in out.splitlines():
        parts = line.split(" ", 1)
        if len(parts) == 2:
            result[parts[0].strip()] = parts[1].strip()
    return result


def ros_exec(cmd: str, timeout_s: int = 10) -> tuple[int, str]:
    # ros2 komutlarını deos container içinde çalıştır
    return sh(
        [
            "docker",
            "compose",
            "-f",
            os.path.join(REPO_ROOT, "docker-compose.yml"),
            "exec",
            "-T",
            "deos",
            "bash",
            "-lc",
            f"source /opt/ros/jazzy/setup.bash && "
            f"if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi && "
            f"{cmd}",
        ],
        timeout_s=timeout_s,
    )


def check_nodes() -> tuple[bool, list[str], str]:
    rc, out = ros_exec("ros2 node list", timeout_s=10)
    if rc != 0:
        return False, SPEC.critical_nodes, out
    nodes = set([ln.strip() for ln in out.splitlines() if ln.strip()])
    missing = [n for n in SPEC.critical_nodes if n not in nodes]
    return len(missing) == 0, missing, out


def check_stm32_subscriber() -> tuple[bool, str]:
    rc, out = ros_exec("ros2 topic info -v /cmd_vel", timeout_s=10)
    if rc != 0:
        return False, out
    ok = "/stm32_node" in out
    return ok, out


def log(event: dict) -> None:
    line = json.dumps(event, ensure_ascii=False)
    print(line, flush=True)


def main() -> None:
    fail_count = 0

    while True:
        ts = time.time()

        containers = docker_ps()
        bringup_ok = "epic_torvalds" in containers and "Up" in containers.get("epic_torvalds", "")
        agent_ok = "deos_micro_ros_agent" in containers and "Up" in containers.get("deos_micro_ros_agent", "")

        nodes_ok, missing_nodes, nodes_dump = check_nodes() if bringup_ok else (False, SPEC.critical_nodes, "")
        stm_ok, stm_dump = check_stm32_subscriber() if bringup_ok else (False, "")

        critical_ok = bringup_ok and agent_ok and nodes_ok
        if not critical_ok:
            fail_count += 1
        else:
            fail_count = 0

        log(
            {
                "ts": ts,
                "bringup_container_ok": bringup_ok,
                "agent_container_ok": agent_ok,
                "nodes_ok": nodes_ok,
                "missing_nodes": missing_nodes,
                "stm32_cmd_vel_subscriber_ok": stm_ok,
                "fail_count": fail_count,
            }
        )

        # STM32 subscriber: require_stm32_subscriber=False ise sadece raporla
        if SPEC.require_stm32_subscriber and bringup_ok and not stm_ok:
            fail_count += 1

        if fail_count >= FAIL_THRESHOLD:
            log({"ts": time.time(), "action": "restart_deos_bringup", "reason": "consecutive_failures"})
            sh(["systemctl", "restart", "deos-bringup.service"], timeout_s=15)
            fail_count = 0

        time.sleep(INTERVAL_S)


if __name__ == "__main__":
    main()

