# udi-irobot-poly

Polyglot V3 / PG3x nodeserver for iRobot Roomba and Braava robots. Local MQTT control — once paired, no cloud account is needed at runtime.

## Features

- Multiple robots per nodeserver (prefix-based params: `robot1_*`, `robot2_*`, ...)
- On-device pairing for older models (6xx/8xx/9xx/older S-series): give it an IP, click **Fetch Credentials**, hold the Home/Clean button — nodeserver retrieves blid, password, and robot name automatically
- Manual pairing path for j/i-series and newer firmware (see below)
- Per-robot node exposing battery, phase, bin (full / present), docked, charging state, error code, signal, suction mode, passes, child lock, bin-full pause, area cleaned and runtime this mission, Clean Base bag full
- Commands: Start, Stop, Pause, Resume, Return to Dock, Locate, Empty Bin (Clean Base), Reboot, Set Suction, Set Passes, Set Child Lock, Set Bin-Full Pause

## Firmware caveats

**Current iRobot firmware progressively locks down local MQTT.** You may need to permanently block the robot's outbound internet (`*.irobotapi.com`, `*.iot.*.amazonaws.com`) to keep port 8883 open on the robot. The robot remains fully functional on your LAN; you lose the phone app, firmware updates, and Smart Maps edits.

Once paired and credentials are saved, the plugin communicates entirely over local MQTT and never talks to the iRobot cloud.

## Setup — older models (Fetch Credentials)

1. Install the nodeserver from the PG3 store (or sideload from this repo).
2. Use a static DHCP reservation for the robot. Set `robot1_ip` in Custom Parameters.
3. If local control is firmware-gated on your model, block the robot from the internet at your router.
4. Click **Fetch Credentials** on the iRobot Controller node.
5. Within ~30s, press and hold **Home** (or the single **Clean** button on newer robots) for about 2 seconds — until a beep and pulsing Wi-Fi LED. Release. Do NOT hold longer, it'll trigger bag-empty or power-off.
6. `robot1_blid`, `robot1_password`, `robot1_name` populate automatically and the robot node appears.

## Setup — j / i-series (manual, via dorita980)

Newer j and i-series firmware may not respond to the legacy local password probe. Retrieve credentials via iRobot's cloud API instead (one time):

1. Make sure the robot is registered in your iRobot account (add it in the iRobot app if it isn't — you can remove/re-block it after).
2. On any machine with Node.js:
   ```
   npm install -g dorita980
   get-roomba-password-cloud 'your-irobot-email' 'your-irobot-password'
   ```
   This prints BLID and password for every robot on your account.
3. In PG3 Custom Parameters, set for the new robot:
   ```
   robotN_ip = <robot LAN IP>
   robotN_blid = <BLID from dorita980>
   robotN_password = <:1:... string from dorita980>
   robotN_name = <friendly label>
   ```
4. Save. The plugin picks up the complete triplet and adds the robot node. No Fetch Credentials needed.
5. Re-block the robot from the internet if desired — local MQTT continues working.

## Adding more robots

Repeat either setup path with `robot2_*`, `robot3_*`, etc. Each robot becomes its own node under the single iRobot Controller.

## Credits

- [roombapy](https://github.com/pschmitt/roombapy) — local MQTT library, password fetcher
- [dorita980](https://github.com/koalazak/dorita980) — cloud credential retrieval for newer firmware
- Inspired by the Home Assistant `roomba` integration and the original `udi-roomba-poly` by BME-nodeservers
