import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml

from g70_gnss.driver import Driver, prepare_config, serial_port, validate_config


class NtripConfigTests(unittest.TestCase):
    def test_standalone_defaults_need_no_external_project(self):
        with patch.dict(os.environ, {}, clear=True):
            data, env = prepare_config({"serial": {"port": "/dev/example"}})
        self.assertEqual(data["serial"]["baud"], 115200)
        self.assertEqual(data["frame_id"], "gnss_link")
        self.assertNotIn("NTRIP_PASSWORD", env)
        self.assertTrue(Path(env["PYTHONPATH"].split(os.pathsep)[0], "g70_bridge/node.py").is_file())

    def test_ambiguous_or_unidentified_serial_is_rejected(self):
        with patch("g70_gnss.driver.Path.exists", return_value=False):
            with patch("g70_gnss.driver.Path.glob", return_value=[]), self.assertRaises(ValueError):
                serial_port("auto")
            candidates = [Path("/dev/serial/by-id/g70-one"), Path("/dev/serial/by-id/g70-two")]
            with patch("g70_gnss.driver.Path.glob", return_value=candidates), self.assertRaises(ValueError):
                serial_port("auto")

    def test_overrides_reach_existing_driver_without_persisting_password(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            base = root / "base.yaml"
            base.write_text(yaml.safe_dump({"serial": {"port": "/dev/example", "baud": 115200},
                "ntrip": {"host": "old", "port": 2101, "user": "old", "mount": "old",
                          "password_file": "old.secret"}}))
            cfg = {"manage_driver": True, "driver_config": str(base), "ntrip": {
                "server_ip": "192.0.2.1", "port": 2102, "username": "test-user",
                "password": "test-secret", "mountpoint": "TEST"}}
            validate_config(cfg)
            data, env = prepare_config(cfg)
            self.assertEqual(data["ntrip"], {"host": "192.0.2.1", "port": 2102,
                                            "user": "test-user", "mount": "TEST"})
            self.assertEqual(env["NTRIP_PASSWORD"], "test-secret")
            owner = Driver(root)
            with patch("g70_gnss.driver.subprocess.Popen") as launch:
                launch.return_value.poll.return_value = None
                owner.start(cfg)
                text = owner.config_path.read_text()
                self.assertNotIn("test-secret", text)
                self.assertNotIn("test-secret", repr(launch.call_args.args))
                self.assertEqual(owner.config_path.stat().st_mode & 0o777, 0o600)
                runtime_path = owner.config_path
                owner.stop()
                owner.stop()
                self.assertFalse(runtime_path.exists())
                launch.return_value.terminate.assert_called_once()

    def test_overrides_cannot_be_silently_ignored_in_external_mode(self):
        with self.assertRaises(ValueError):
            validate_config({"manage_driver": False, "ntrip": {"server_ip": "192.0.2.1"}})

    def test_env_password_and_relative_secret_path(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp) / "base.yaml"
            base.write_text(yaml.safe_dump({"serial": {"port": "/dev/example"}, "ntrip": {
                "host": "192.0.2.1", "user": "user", "mount": "TEST", "password_file": "ntrip.secret"}}))
            with patch.dict(os.environ, {"GNSS_TEST_PASSWORD": "env-secret"}):
                data, env = prepare_config({"driver_config": str(base), "ntrip": {"password_env": "GNSS_TEST_PASSWORD"}})
                self.assertEqual(env["NTRIP_PASSWORD"], "env-secret")
                self.assertNotIn("password_file", data["ntrip"])
            data, _ = prepare_config({"driver_config": str(base)})
            self.assertEqual(data["ntrip"]["password_file"], str(Path(tmp) / "ntrip.secret"))
            with self.assertRaises(ValueError):
                prepare_config({"driver_config": str(base), "ntrip": {"password_env": "GNSS_MISSING_TEST_SECRET"}})

    def test_invalid_parameters(self):
        for ntrip in ({"port": 0}, {"port": True}, {"username": "a\nb"},
                      {"password": "p", "password_env": "KEY"}, {"mountpoint": ""}):
            with self.subTest(keys=list(ntrip)), self.assertRaises(ValueError):
                validate_config({"manage_driver": True, "ntrip": ntrip})


if __name__ == "__main__":
    unittest.main()
