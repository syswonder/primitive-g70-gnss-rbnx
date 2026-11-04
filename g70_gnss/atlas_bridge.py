# SPDX-License-Identifier: MulanPSL-2.0
import json
import logging
import math
import os
import time
from pathlib import Path

from robonix_api import Err, Ok, Primitive
from .driver import Driver, validate_config

log = logging.getLogger("g70_gnss")
gnss = Primitive(
    id=os.environ.get("RBNX_INSTANCE_NAME", "g70_gnss"),
    namespace="robonix/primitive/gnss",
)
timeout_s = 10.0
declared = False
config = {}
driver = Driver(Path(__file__).resolve().parents[1])


@gnss.on_init
def init(cfg: dict):
    global timeout_s, config
    if driver.process is not None:
        return Err("Deactivate before changing GNSS configuration")
    try:
        validate_config(cfg)
    except ValueError as exc:
        return Err(str(exc))
    value = cfg.get("sentinel_timeout_s", 10.0)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return Err("sentinel_timeout_s must be a number")
    if not math.isfinite(value) or not 0 < value <= 120:
        return Err("sentinel_timeout_s must be in (0, 120] seconds")
    timeout_s = float(value)
    config = dict(cfg)
    return Ok()


@gnss.on_activate
def activate():
    global declared
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String

    # A private context avoids interfering with the provider's ROS runtime.
    context = Context()
    node = None
    executor = None
    seen = set()
    ready = False

    def on_status(msg):
        try:
            status = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(status, dict) and all(key in status for key in (
            "state", "receiver_quality", "rtk_fixed", "fix_age_s",
            "measurement_age_s", "serial_state", "ntrip",
        )):
            if config.get("manage_driver", True) and status["serial_state"] != "OPEN":
                return
            seen.add("/gps/rtk_status")

    try:
        rclpy.init(context=context)
        executor = SingleThreadedExecutor(context=context)
        node = rclpy.create_node("g70_gnss_sentinel", context=context)
        executor.add_node(node)
        node.create_subscription(NavSatFix, "/gps/fix",
                                 lambda msg: seen.add("/gps/fix"), qos_profile_sensor_data)
        node.create_subscription(String, "/gps/rtk_status", on_status, qos_profile_sensor_data)
        if config.get("manage_driver", True) and driver.process is None:
            discovery_deadline = time.monotonic() + 1.0
            while time.monotonic() < discovery_deadline:
                executor.spin_once(timeout_sec=0.1)
            if node.count_publishers("/gps/fix") or node.count_publishers("/gps/rtk_status"):
                return Err("An external GNSS publisher already exists; stop it before using manage_driver=true")
            driver.start(config)
        deadline = time.monotonic() + timeout_s
        while len(seen) < 2 and time.monotonic() < deadline:
            if driver.process is not None and driver.process.poll() is not None:
                return Err("Managed G70 driver exited; see rbnx-build/data/driver.log")
            executor.spin_once(timeout_sec=min(0.1, max(0, deadline - time.monotonic())))
        if len(seen) < 2:
            missing = sorted({"/gps/fix", "/gps/rtk_status"} - seen)
            return Err(f"No correctly typed GNSS sample / RTK JSON on {missing}; "
                       "start the external G70 driver and check ROS_DOMAIN_ID / RMW")
        if not declared:
            gnss.declare_ros2_topic("robonix/primitive/gnss/fix", "/gps/fix", qos="reliable")
            gnss.declare_ros2_topic("robonix/primitive/gnss/rtk_status", "/gps/rtk_status", qos="reliable")
            declared = True
        ready = True
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
        if not ready:
            driver.stop()

    log.info("G70 topics available; activation does not imply a valid GNSS or RTK fix")
    return Ok()


@gnss.on_deactivate
def deactivate():
    driver.stop()
    return Ok()


@gnss.on_shutdown
def shutdown():
    driver.stop()
    return Ok()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    gnss.run()
