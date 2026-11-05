# Driver CMD_INIT configuration; independent of robot body.
# manage_driver: boolean, default true. false attaches to existing GPS topics.
# serial.port: device path or "auto" (default); prefer /dev/serial/by-id.
# auto accepts /dev/{gnss,gps,g70}, or by-id names containing GNSS/GPS/G70/RTK.
# Exactly one resolved device is required; arbitrary ttyUSB devices are never guessed.
# serial.baud: positive integer, default 115200.
# frame_id: nonempty string, default gnss_link. Deployment supplies TF as needed.
# fix_timeout: number (0,120], seconds, default 3.0.
# sentinel_timeout_s: number (0,120], seconds, default 10.0.
# driver_config: optional absolute path to existing G70 YAML as base settings.
# Omitted by default: built-in settings and bundled vendor/g70_bridge are used.
# ntrip: optional overrides, require manage_driver=true.
#   server_ip: NTRIP caster IP or hostname.
#   port: integer [1,65535], default 2101.
#   username: NTRIP account.
#   mountpoint: NTRIP mount point.
#   password: optional secret string, never serialized to runtime YAML or argv.
#   password_env: environment variable name; mutually exclusive with password.
# Without credentials NTRIP is disabled and ordinary GNSS remains available.
# Credentials require server_ip/username/mountpoint.
# Topics fixed to /gps/fix and /gps/rtk_status.
# require_velocity: bool, default false. When true ACTIVATE also waits for finite ENU /gps/velocity.
# RMC and GST are consumed only if already emitted by the receiver; no device setup writes.
