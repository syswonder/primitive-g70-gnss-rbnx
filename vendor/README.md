# Bundled G70 driver

g70_bridge/{node,protocol,ntrip}.py and __init__.py are copied unchanged from
the existing G70 project on ranger, gnss/ws/src/g70_bridge/g70_bridge/ (2026-11-04).
Its setup.py and package.xml identify the driver license as Apache-2.0.
Only source is included; no device configuration, accounts or password files are copied.
The original source remains in the GNSS workspace. This copy provides independent
serial ownership, NMEA validation and NTRIP/RTCM transport inside this package.
