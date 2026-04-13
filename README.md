# udi-irobot-poly

Polyglot V3 / PG3x nodeserver for iRobot Roomba and Braava robots. Local control only — no cloud account required.

## Features

- Multiple robots in one nodeserver (prefix-based params: `robot1_*`, `robot2_*`, ...)
- On-device pairing: give it an IP, click **Fetch Credentials**, press Home on the robot — nodeserver retrieves blid, password, and robot name automatically
- Per-robot node with battery, phase, bin full/present, docked, charging state, error code, signal
- Commands: Start, Stop, Pause, Resume, Return to Dock, Locate

## Setup

1. Install the nodeserver from the PG3 store (or sideload from this repo).
2. In Custom Parameters, set `robot1_ip` to the robot's local IP. Use a static DHCP reservation.
3. On the iRobot Controller node in the admin console, click **Fetch Credentials**.
4. Within ~30s, press and hold the **Home** button on the robot for about 2 seconds, until it beeps and the Wi-Fi LED pulses.
5. Wait — `robot1_blid`, `robot1_password`, and `robot1_name` will populate. The robot node will appear under the controller.

Add more robots by setting `robot2_ip`, `robot3_ip`, etc., and repeating the pairing step.

## Credits

- [roombapy](https://github.com/pschmitt/roombapy) — local MQTT library, password fetcher, discovery
- Inspired by the Home Assistant `roomba` integration and the original `udi-roomba-poly` by BME-nodeservers
