import subprocess
nodes = [
    "/amcl", "/map_server", "/controller_server", "/planner_server",
    "/behavior_server", "/bt_navigator", "/velocity_smoother",
    "/smoother_server", "/waypoint_follower", "/collision_monitor",
]
for node in nodes:
    result = subprocess.run(
        ["ros2", "lifecycle", "get", node],
        capture_output=True, text=True, timeout=5
    )
    state = result.stdout.strip() or result.stderr.strip()
    print(f"{node}: {state}")

    # Activate if not active
    if "unconfigured" in state:
        subprocess.run(["ros2", "lifecycle", "set", node, "configure"], timeout=5)
        subprocess.run(["ros2", "lifecycle", "set", node, "activate"], timeout=5)
        print(f"  -> activated")
    elif "inactive" in state:
        subprocess.run(["ros2", "lifecycle", "set", node, "activate"], timeout=5)
        print(f"  -> activated")
