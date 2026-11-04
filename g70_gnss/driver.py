"""Optional ownership of the existing G70 ROS driver and its NTRIP config."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile

import yaml


def validate_config(cfg):
    unknown = set(cfg) - {"sentinel_timeout_s", "manage_driver", "driver_config", "serial", "frame_id", "fix_timeout", "ntrip"}
    if unknown:
        raise ValueError(f"Unknown config keys: {sorted(unknown)}")
    if not isinstance(cfg.get("manage_driver", True), bool):
        raise ValueError("manage_driver must be a boolean")
    ntrip = cfg.get("ntrip", {})
    if not isinstance(ntrip, dict):
        raise ValueError("ntrip must be an object")
    if ntrip and not cfg.get("manage_driver", True):
        raise ValueError("ntrip overrides require manage_driver=true; external driver config is not changed")
    if set(ntrip) - {"server_ip", "port", "username", "password", "password_env", "mountpoint"}:
        raise ValueError("Unknown ntrip configuration key")
    for key in ("server_ip", "username", "password", "password_env", "mountpoint"):
        if key in ntrip and (not isinstance(ntrip[key], str) or not ntrip[key] or
                             any(c in ntrip[key] for c in "\r\n\0")):
            raise ValueError(f"ntrip.{key} must be a nonempty single-line string")
    if "password" in ntrip and "password_env" in ntrip:
        raise ValueError("Choose ntrip.password or ntrip.password_env")
    if "port" in ntrip and (type(ntrip["port"]) is not int or not 1 <= ntrip["port"] <= 65535):
        raise ValueError("ntrip.port must be an integer in [1, 65535]")
    serial = cfg.get("serial", {})
    if not isinstance(serial, dict) or set(serial) - {"port", "baud"}:
        raise ValueError("serial accepts port and baud")
    if "port" in serial and (not isinstance(serial["port"], str) or not serial["port"]):
        raise ValueError("serial.port must be a device path or auto")
    if "baud" in serial and (type(serial["baud"]) is not int or serial["baud"] <= 0):
        raise ValueError("serial.baud must be a positive integer")
    if "frame_id" in cfg and (not isinstance(cfg["frame_id"], str) or not cfg["frame_id"]):
        raise ValueError("frame_id must be a nonempty string")
    if "fix_timeout" in cfg and (type(cfg["fix_timeout"]) not in (int, float) or not 0 < cfg["fix_timeout"] <= 120):
        raise ValueError("fix_timeout must be in (0, 120] seconds")
    for key in ("driver_config",):
        if key in cfg and (not isinstance(cfg[key], str) or not Path(cfg[key]).is_absolute()):
            raise ValueError(f"{key} must be an absolute path")


def serial_port(value):
    if value != "auto":
        return value
    candidates = [Path(p) for p in ("/dev/gnss", "/dev/gps", "/dev/g70") if Path(p).exists()]
    candidates += [p for p in Path("/dev/serial/by-id").glob("*")
                   if any(word in p.name.lower() for word in ("gnss", "gps", "g70", "rtk"))]
    unique = {p.resolve() for p in candidates}
    if len(unique) != 1:
        raise ValueError("Cannot uniquely identify GNSS hardware; configure serial.port with a stable /dev/serial/by-id path")
    return str(unique.pop())


def prepare_config(cfg):
    """Return driver settings plus a child environment; never serialize a password."""
    path = Path(cfg["driver_config"]) if cfg.get("driver_config") else None
    data = yaml.safe_load(path.read_text()) if path else {
        "serial": {"port": "auto", "baud": 115200},
        "frame_id": "gnss_link", "fix_timeout": 3.0,
        "ntrip": {"host": "", "port": 2101, "user": "", "mount": "",
                  "allow_empty_password": False, "gga_interval": 5, "stream_timeout": 15},
    }
    if not isinstance(data, dict) or not isinstance(data.get("ntrip"), dict) or "serial" not in data:
        raise ValueError("driver_config must contain serial and ntrip settings")
    ntrip = data["ntrip"]
    data["serial"].update(cfg.get("serial", {}))
    data["serial"]["port"] = serial_port(data["serial"].get("port", "auto"))
    for key in ("frame_id", "fix_timeout"):
        if key in cfg:
            data[key] = cfg[key]
    overrides = cfg.get("ntrip", {})
    for public, native in (("server_ip", "host"), ("username", "user"), ("mountpoint", "mount"), ("port", "port")):
        if public in overrides:
            ntrip[native] = overrides[public]
    if ntrip.get("password_file"):
        ntrip["password_file"] = str((path.parent / ntrip["password_file"]).resolve())
    env = dict(os.environ)
    secret = overrides.get("password")
    if "password_env" in overrides:
        secret = os.environ.get(overrides["password_env"])
        if not secret:
            raise ValueError("The configured NTRIP password environment variable is empty or missing")
    if secret is not None:
        env["NTRIP_PASSWORD"] = secret
        ntrip.pop("password_file", None)
    ntrip.pop("password", None)
    if (env.get("NTRIP_PASSWORD") or ntrip.get("password_file")) and not all(ntrip.get(key) for key in ("host", "user", "mount")):
        raise ValueError("NTRIP credentials require server_ip, username and mountpoint")
    source = Path(__file__).resolve().parents[1] / "vendor"
    env["PYTHONPATH"] = str(source) + os.pathsep + env.get("PYTHONPATH", "")
    return data, env


class Driver:
    """Own only the child and temporary configuration created by this primitive."""

    def __init__(self, root):
        self.root = root
        self.process = None
        self.config_path = None
        self.log = None

    def start(self, cfg):
        data, env = prepare_config(cfg)
        directory = self.root / "rbnx-build/data"
        directory.mkdir(parents=True, exist_ok=True)
        fd, name = tempfile.mkstemp(prefix="g70-", suffix=".yaml", dir=directory)
        self.config_path = Path(name)
        try:
            with os.fdopen(fd, "w") as stream:
                yaml.safe_dump(data, stream)
            log_fd = os.open(directory / "driver.log", os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
            self.log = os.fdopen(log_fd, "a")
            self.process = subprocess.Popen([
                sys.executable, "-c", "from g70_bridge.node import main; main()",
                "--ros-args", "-p", f"config_file:={self.config_path}",
            ], env=env, stdout=self.log, stderr=subprocess.STDOUT)
        except Exception:
            self.stop()
            raise

    def stop(self):
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
            self.process = None
        if self.log is not None:
            self.log.close()
            self.log = None
        if self.config_path is not None:
            self.config_path.unlink(missing_ok=True)
            self.config_path = None
