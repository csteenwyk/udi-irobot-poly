#!/usr/bin/env python3
# GPL-3.0 License — Copyright (c) 2026 csteenwyk
"""
iRobot Polyglot v3 NodeServer

Local control of Roomba and Braava robots via roombapy (no cloud).
Multiple robots via prefixed custom params: robot1_ip, robot1_blid, robot1_password, robot1_name.
Pairing command fetches blid/password/name after the user presses Home on the robot.
"""

import os
import re
import sys
import threading
import time

import udi_interface
from udi_interface import Custom

_IMPORT_ERR = None
try:
    from roombapy import RoombaFactory, RoombaPassword, RoombaDiscovery
except ImportError as _e:
    _IMPORT_ERR = str(_e)
    RoombaFactory = RoombaPassword = RoombaDiscovery = None

LOGGER = udi_interface.LOGGER

_ROBOT_KEY = re.compile(r'^robot(\d+)_(ip|blid|password|name)$')

# Roomba "phase" strings → compact ISY index
_PHASE_MAP = {
    'charge':      1,
    'run':         2,
    'stop':        3,
    'stuck':       4,
    'hmUsrDock':   5,
    'hmMidMsn':    5,
    'hmPostMsn':   5,
    'dockend':     6,
    'dock':        6,
    'evac':        7,
    'pause':       8,
    'chargingerror': 0,
    'new':         2,
    'resume':      2,
    'cancelled':   3,
}

# chargingState reported strings → editor index
_CHARGE_MAP = {
    'none':            0,
    'charging':        1,
    'reconditioning':  1,
    'full':            2,
    'trickle':         3,
    'waiting':         4,
    'fault':           5,
}

# Suction modes, passes — index ↔ (carpetBoost, vacHigh) / (noAutoPasses, twoPass)
_SUCTION_TO_PREFS = {
    0: {'carpetBoost': True,  'vacHigh': False},   # Auto
    1: {'carpetBoost': False, 'vacHigh': True},    # Performance
    2: {'carpetBoost': False, 'vacHigh': False},   # Eco
}
_PASSES_TO_PREFS = {
    0: {'noAutoPasses': False, 'twoPass': False},  # Auto
    1: {'noAutoPasses': True,  'twoPass': False},  # 1 pass
    2: {'noAutoPasses': True,  'twoPass': True},   # 2 passes
}

def _mission_state(mission):
    """Combine cleanMissionStatus.cycle + .phase into one ISY index.

    Returns:
      0 Idle               — no mission active, just sitting
      1 Cleaning           — whole-floor mission running
      2 Spot Cleaning      — spot cycle running
      3 Mapping            — training / learning run
      4 Mid-Mission Charge — paused on dock, will resume
      5 Paused             — user paused, not on dock
      6 Returning to Dock
      7 Stuck / Error
      8 Evacuating         — dumping into Clean Base
    """
    cycle = (mission.get('cycle') or 'none').lower()
    phase = (mission.get('phase') or '').lower()

    if phase == 'stuck':
        return 7
    if phase == 'evac' or cycle == 'evac':
        return 8
    if cycle == 'none':
        return 0
    if cycle == 'train':
        return 3
    if cycle == 'spot':
        return 2
    # cycle == 'clean' (whole-floor) from here down
    if phase in ('hmmidmsn', 'hmpostmsn', 'hmusrdock', 'dock', 'dockend'):
        return 6
    if phase == 'charge':
        return 4
    if phase == 'pause' or phase == 'stop':
        return 5
    if phase in ('run', 'resume', 'new'):
        return 1
    return 1  # cycle is active but phase unknown — assume cleaning


def _suction_index(reported):
    cb = bool(reported.get('carpetBoost'))
    vh = bool(reported.get('vacHigh'))
    if cb and not vh: return 0
    if vh and not cb: return 1
    return 2  # both False → Eco; both True is invalid, treat as Eco

def _passes_index(reported):
    if not reported.get('noAutoPasses'):
        return 0
    return 2 if reported.get('twoPass') else 1

# Common error codes → human-readable string (for log and notice).
_ERROR_TEXT = {
    0: 'OK', 1: 'Left wheel off floor', 2: 'Right wheel off floor',
    5: 'Left wheel stuck', 6: 'Brush stuck', 9: 'Bumper stuck',
    10: 'Right wheel stuck', 14: 'Bin missing', 15: 'Reboot required',
    16: 'Bumped — picked up?', 17: 'Navigation problem', 18: 'Docking problem',
    20: 'Low battery', 31: 'Clean left wheel', 32: 'Clean right wheel',
    38: 'Vacuum motor problem', 43: 'Clean Base bag full',
    46: 'Battery disconnect', 52: 'Mission cannot continue',
    65: 'Hardware problem', 68: 'Hardware problem',
    73: 'Pad type changed', 74: 'Mop pad missing',
    101: 'Battery not detected', 105: 'Charging fault',
    216: 'Clean Base bag full',  # S9+/i/j-series reported via MQTT error field
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _parse_robots(params):
    """Return {index: {ip,blid,password,name}} from flat customParams dict."""
    robots = {}
    for key, val in (params or {}).items():
        m = _ROBOT_KEY.match(key)
        if not m:
            continue
        idx, field = int(m.group(1)), m.group(2)
        robots.setdefault(idx, {})[field] = (val or '').strip()
    return robots


def _address_for(idx, blid):
    """Stable ISY node address — prefer blid, fall back to index."""
    if blid:
        return re.sub(r'[^a-z0-9]', '', blid.lower())[:14]
    return f'robot{idx}'


# ---------------------------------------------------------------------------
# Robot Node
# ---------------------------------------------------------------------------

class RobotNode(udi_interface.Node):

    id = 'irobot_robot'

    drivers = [
        {'driver': 'ST',    'value': 0, 'uom': 25},  # Phase
        {'driver': 'BATLVL','value': 0, 'uom': 51},  # Battery %
        {'driver': 'GV1',   'value': 0, 'uom': 2},   # Bin full
        {'driver': 'GV2',   'value': 0, 'uom': 2},   # Bin present
        {'driver': 'GV3',   'value': 0, 'uom': 2},   # Docked
        {'driver': 'GV4',   'value': 0, 'uom': 25},  # Charging state
        {'driver': 'GV5',   'value': 0, 'uom': 56},  # Error code
        {'driver': 'GV6',   'value': 0, 'uom': 51},  # Signal %
        {'driver': 'GV7',   'value': 0, 'uom': 25},  # Suction mode
        {'driver': 'GV8',   'value': 0, 'uom': 25},  # Passes mode
        {'driver': 'GV9',   'value': 0, 'uom': 2},   # Child lock
        {'driver': 'GV10',  'value': 0, 'uom': 2},   # Bin-Full pause
        {'driver': 'GV11',  'value': 0, 'uom': 56},  # Area this run (m²)
        {'driver': 'GV12',  'value': 0, 'uom': 56},  # Runtime this run (min)
        {'driver': 'GV13',  'value': 0, 'uom': 2},   # Clean Base bag full
        {'driver': 'GV14',  'value': 0, 'uom': 25},  # Mission state
    ]

    def __init__(self, polyglot, primary, address, name, ip, blid, password, ctrl):
        super().__init__(polyglot, primary, address, name)
        self._ip = ip
        self._blid = blid
        self._password = password
        self._ctrl = ctrl
        self._roomba = None
        self._cache = {}
        self._dumped = False
        self._connect()

    def _connect(self):
        if RoombaFactory is None:
            LOGGER.error('roombapy not installed')
            return
        # Run connect in a background thread with retries. The robot's MQTT
        # listener can take several seconds to accept clients right after
        # pairing, and it only allows one client at a time — so the first
        # try often gets Connection refused.
        threading.Thread(target=self._connect_loop, daemon=True,
                         name=f'connect-{self.address}').start()

    def _connect_loop(self):
        """Retry forever with capped backoff. Robots with cloud blocked
        cycle Wi-Fi and only briefly accept connections — we need to keep
        trying indefinitely to catch an open window."""
        time.sleep(3)  # initial delay after pairing
        attempt = 0
        while True:
            attempt += 1
            try:
                self._roomba = RoombaFactory.create_roomba(
                    address=self._ip, blid=self._blid, password=self._password,
                    continuous=True, delay=10)
                self._roomba.register_on_message_callback(self._on_message)
                self._roomba.register_on_disconnect_callback(self._on_disconnect)
                self._roomba.connect()
                LOGGER.info(f'{self.name}: connected to {self._ip} '
                            f'(attempt {attempt})')
                return
            except Exception as e:
                LOGGER.warning(
                    f'{self.name}: connect attempt {attempt} failed: {e}')
                self._roomba = None
                # Backoff: 5, 10, 15, ... capped at 60 s.
                time.sleep(min(60, 5 * attempt))

    def _on_message(self, json_data):
        self._apply_state()

    def _on_disconnect(self, error=None):
        """Re-launch the connect loop after an unexpected disconnect so we
        pick the robot back up when its Wi-Fi/port becomes reachable again."""
        LOGGER.warning(
            f'{self.name}: disconnected ({error}); restarting connect loop')
        self._roomba = None
        self._dumped = False  # re-dump on next successful connect
        threading.Thread(target=self._connect_loop, daemon=True,
                         name=f'connect-{self.address}').start()

    def _set(self, driver, value):
        if self._cache.get(driver) != value:
            self._cache[driver] = value
            self.setDriver(driver, value)

    def _apply_state(self):
        if not self._roomba:
            return
        try:
            state = self._roomba.master_state or {}
            reported = state.get('state', {}).get('reported', {})
            # One-shot dump of every reported key to help map firmware-specific
            # fields (charging state, bag full, etc.).
            if not self._dumped and reported:
                self._dumped = True
                LOGGER.info(
                    f'{self.name}: reported keys = {sorted(reported.keys())}')
                for key in ('dock', 'bin', 'cleanMissionStatus', 'signal',
                            'bbchg3', 'bbrun', 'bbpause', 'bbswitch',
                            'bbmssn', 'missionTelemetry', 'runtimeStats'):
                    if key in reported:
                        LOGGER.info(f'{self.name}: reported.{key} = {reported[key]}')
            mission  = reported.get('cleanMissionStatus', {}) or {}
            bin_     = reported.get('bin', {}) or {}
            dock     = reported.get('dock', {}) or {}

            phase   = mission.get('phase')
            err     = mission.get('error', 0) or 0
            sqft    = mission.get('sqft', 0) or 0
            mssnM   = mission.get('mssnM', 0) or 0
            notReady = mission.get('notReady', 0) or 0
            batt    = reported.get('batPct')
            rssi    = reported.get('signal', {}).get('rssi', 0)
            chgState = (reported.get('bbchg3') or {}).get('avgMin')  # placeholder
            chargingState = reported.get('chargingState') or reported.get('cleanMissionStatus', {}).get('chargingState')
            if isinstance(chargingState, dict):
                chargingState = chargingState.get('state')

            # --- battery / phase / signal ---
            if batt is not None:
                self._set('BATLVL', int(batt))
            if phase is not None:
                self._set('ST', _PHASE_MAP.get(phase, 0))
            if rssi:
                pct = max(0, min(100, int(2 * (rssi + 100))))  # -100..-30 → 0..100
                # Throttle: RSSI jitters a few percent every second; only
                # publish when the signal moves by ≥10 points (or we've never
                # reported it).
                prev = self._cache.get('GV6')
                if prev is None or abs(pct - prev) >= 10:
                    self._set('GV6', pct)

            # --- bin ---
            if 'full' in bin_:
                self._set('GV1', 1 if bin_.get('full') else 0)
            if 'present' in bin_:
                self._set('GV2', 1 if bin_.get('present') else 0)

            # --- docked: prefer authoritative dock.known, fall back to phase ---
            if 'known' in dock:
                self._set('GV3', 1 if dock.get('known') else 0)
            else:
                self._set('GV3', 1 if phase in ('charge', 'dockend', 'dock') else 0)

            # --- charging state (multi-enum). Prefer explicit chargingState
            #     string; else derive from phase. ---
            if isinstance(chargingState, str):
                self._set('GV4', _CHARGE_MAP.get(chargingState.lower(), 0))
            elif phase == 'charge':
                self._set('GV4', 1)
            elif batt is not None and batt >= 100 and phase in ('stop', 'dockend', 'charge'):
                self._set('GV4', 2)  # Full
            else:
                self._set('GV4', 0)

            # --- error (numeric + readable notice) ---
            self._set('GV5', int(err))
            notice_key = f'err_{self.address}'
            if err:
                text = _ERROR_TEXT.get(err, f'Error {err}')
                self._ctrl.poly.Notices[notice_key] = f'{self.name}: {text} (code {err})'
            else:
                try:
                    del self._ctrl.poly.Notices[notice_key]
                except Exception:
                    pass

            # --- suction / passes ---
            self._set('GV7', _suction_index(reported))
            self._set('GV8', _passes_index(reported))

            # --- child lock / bin pause ---
            if 'childLock' in reported:
                self._set('GV9', 1 if reported.get('childLock') else 0)
            if 'binPause' in reported:
                self._set('GV10', 1 if reported.get('binPause') else 0)

            # --- area & runtime this mission ---
            # Firmware doesn't always populate cleanMissionStatus.sqft on
            # current S9+/j9+ releases — the field is often absent. Runtime
            # comes from wall-clock diff against mssnStrtTm since mssnM only
            # updates intermittently.
            strt_tm = mission.get('mssnStrtTm') or 0
            cycle_now = (mission.get('cycle') or 'none').lower()
            if cycle_now != 'none' and strt_tm:
                import time as _t
                runtime_min = max(0, int((_t.time() - strt_tm) / 60))
            else:
                runtime_min = int(mssnM)  # last reported value (0 when idle)
            self._set('GV12', runtime_min)
            # Area: try several fields; if none populated, leave 0. Real data
            # from a run will tell us where it lives on this firmware.
            area_sqft = (
                sqft
                or (reported.get('bbmssn', {}) or {}).get('sqft')
                or (reported.get('missionTelemetry', {}) or {}).get('sqft')
                or 0
            )
            self._set('GV11', round(area_sqft * 0.0929, 1))

            # --- Clean Base bag full: multiple fields vary by firmware.
            #     Report if ANY of the candidates indicates full. Log the raw
            #     candidates at DEBUG so we can tighten this later. ---
            bag_candidates = {
                'dock.bagFull': dock.get('bagFull'),
                'dock.state': dock.get('state'),
                'bin.bagFull': bin_.get('bagFull'),
                'notReady': notReady,
                'error43': err == 43,
            }
            bag_full = (
                bool(dock.get('bagFull')) or
                bool(bin_.get('bagFull')) or
                err in (43, 216) or            # 216 = S9+/i/j bag full
                notReady in (43, 216)
            )
            self._set('GV13', 1 if bag_full else 0)
            LOGGER.debug(f'{self.name} bag signals: {bag_candidates}')

            # --- mission state (derived from cycle + phase) ---
            self._set('GV14', _mission_state(mission))
        except Exception as e:
            LOGGER.debug(f'{self.name}: state parse error: {e}')

    def disconnect(self):
        if self._roomba:
            try:
                self._roomba.disconnect()
            except Exception:
                pass

    def _send(self, cmd):
        if not self._roomba:
            LOGGER.warning(f'{self.name}: not connected')
            return
        try:
            self._roomba.send_command(cmd)
        except Exception as e:
            LOGGER.error(f'{self.name}: send {cmd!r} failed: {e}')

    def _set_prefs(self, prefs):
        """Apply a dict of preferences to the robot via set_preference."""
        if not self._roomba:
            LOGGER.warning(f'{self.name}: not connected')
            return
        for k, v in prefs.items():
            try:
                self._roomba.set_preference(k, v)
            except Exception as e:
                LOGGER.error(f'{self.name}: set_preference({k}={v}) failed: {e}')

    def cmd_start(self, command):  self._send('start')
    def cmd_stop(self, command):   self._send('stop')
    def cmd_pause(self, command):  self._send('pause')
    def cmd_resume(self, command): self._send('resume')
    def cmd_dock(self, command):   self._send('dock')
    def cmd_locate(self, command): self._send('find')
    def cmd_evac(self, command):   self._send('evac')
    def cmd_reboot(self, command): self._send('reset')

    def cmd_set_suction(self, command):
        idx = int(command.get('value', 0))
        prefs = _SUCTION_TO_PREFS.get(idx)
        if prefs:
            self._set_prefs(prefs)

    def cmd_set_passes(self, command):
        idx = int(command.get('value', 0))
        prefs = _PASSES_TO_PREFS.get(idx)
        if prefs:
            self._set_prefs(prefs)

    def cmd_set_child_lock(self, command):
        self._set_prefs({'childLock': bool(int(command.get('value', 0)))})

    def cmd_set_bin_pause(self, command):
        self._set_prefs({'binPause': bool(int(command.get('value', 0)))})

    def query(self, command=None):
        self._apply_state()
        self.reportDrivers()

    commands = {
        'START':          cmd_start,
        'STOP':           cmd_stop,
        'PAUSE':          cmd_pause,
        'RESUME':         cmd_resume,
        'DOCK':           cmd_dock,
        'LOCATE':         cmd_locate,
        'EVAC':           cmd_evac,
        'REBOOT':         cmd_reboot,
        'SET_SUCTION':    cmd_set_suction,
        'SET_PASSES':     cmd_set_passes,
        'SET_CHILD_LOCK': cmd_set_child_lock,
        'SET_BIN_PAUSE':  cmd_set_bin_pause,
        'QUERY':          query,
    }


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

class Controller(udi_interface.Node):

    id = 'irobot_controller'

    drivers = [{'driver': 'ST', 'value': 0, 'uom': 2}]

    def __init__(self, polyglot, primary, address, name):
        super().__init__(polyglot, primary, address, name)
        self.poly = polyglot
        self._params = Custom(polyglot, 'customparams')
        self._robots = {}          # idx → RobotNode
        self._node_added = threading.Event()
        self._controller_added = False
        self._last_params = {}
        self._reconcile_lock = threading.Lock()
        self._pair_in_progress = False

        polyglot.subscribe(polyglot.CONFIGDONE,   self._on_config_done)
        polyglot.subscribe(polyglot.START,        self.start)
        polyglot.subscribe(polyglot.CUSTOMPARAMS, self._on_params)
        polyglot.subscribe(polyglot.POLL,         self.poll)
        polyglot.subscribe(polyglot.STOP,         self.stop)
        polyglot.subscribe(polyglot.ADDNODEDONE,  lambda d: self._node_added.set())
        polyglot.ready()

    def _add_node_wait(self, node, timeout=15):
        self._node_added.clear()
        self.poly.addNode(node)
        if not self._node_added.wait(timeout=timeout):
            LOGGER.warning(f'Timeout adding node {getattr(node, "address", "?")}')

    def _on_config_done(self):
        if self._controller_added:
            return
        self._add_node_wait(self, timeout=3)
        self._controller_added = True
        self.setDriver('ST', 1)
        self._reconcile_robots()

    def start(self):
        self._controller_added = True
        self.setDriver('ST', 1)

    def stop(self):
        self.setDriver('ST', 0)
        for node in self._robots.values():
            node.disconnect()

    def _on_params(self, params):
        self._params.load(params)
        self._last_params = dict(params or {})
        self.poly.Notices.clear()
        if self._controller_added:
            self._reconcile_robots()

    def _reconcile_robots(self):
        """Add nodes for any fully-configured robotN_* entries. Serialized
        under a lock because rapid successive CUSTOMPARAMS events (e.g. during
        pairing, when we write blid/password/name) trigger concurrent calls."""
        with self._reconcile_lock:
            configured = _parse_robots(self._last_params)
            if not configured:
                self.poly.Notices['config'] = (
                    'Add `robot1_ip` in Custom Parameters, then click Fetch Credentials.')
                return

            for idx, cfg in configured.items():
                if idx in self._robots:
                    continue
                ip, blid, password = cfg.get('ip'), cfg.get('blid'), cfg.get('password')
                if not (ip and blid and password):
                    self.poly.Notices[f'pair{idx}'] = (
                        f'Robot {idx}: IP set but not paired — click Fetch Credentials.')
                    continue
                name = cfg.get('name') or f'Roomba {idx}'
                address = _address_for(idx, blid)
                LOGGER.info(f'Adding robot node {address} ({name})')
                # Reserve the slot before creating the node so a concurrent
                # reconcile can't race us past the `idx in self._robots` check.
                self._robots[idx] = None
                try:
                    node = RobotNode(self.poly, self.address, address, name,
                                     ip, blid, password, self)
                    self._add_node_wait(node)
                    self._robots[idx] = node
                except Exception as e:
                    LOGGER.error(f'Failed to add robot {idx}: {e}')
                    del self._robots[idx]

    # --- Pairing ---

    def cmd_fetch_creds(self, command=None):
        """Fetch blid/password/name for every robotN with ip set but creds missing."""
        if RoombaPassword is None:
            self.poly.Notices['pair'] = (
                f'roombapy import failed ({_IMPORT_ERR}) — check install.log and reinstall.')
            LOGGER.error(f'roombapy import failed: {_IMPORT_ERR}')
            return
        configured = _parse_robots(self._last_params)
        targets = [(i, c) for i, c in configured.items()
                   if c.get('ip') and not (c.get('blid') and c.get('password'))]
        if not targets:
            self.poly.Notices['pair'] = 'No robots need pairing.'
            return
        if self._pair_in_progress:
            self.poly.Notices['pair'] = (
                'Pairing already running — wait for the current attempt to '
                'finish before clicking again.')
            return

        self._pair_in_progress = True
        thread = threading.Thread(
            target=self._pair_wrapper, args=(targets,), daemon=True)
        thread.start()

    def _pair_wrapper(self, targets):
        try:
            self._pair_loop(targets)
        finally:
            self._pair_in_progress = False

    def _pair_loop(self, targets):
        for idx, cfg in targets:
            ip = cfg['ip']
            self.poly.Notices[f'pair{idx}'] = (
                f'Pairing Robot {idx} ({ip}) — DO THIS NOW: '
                '(1) put robot on its dock, powered on. '
                '(2) Press and HOLD the HOME button (or Clean button on '
                'single-button j/i-series robots) for about 2 seconds, '
                'until the robot beeps and the Wi-Fi LED starts pulsing. '
                'Do NOT hold longer — longer holds trigger bag-empty or '
                'power-off. (3) Release and wait — this window closes '
                'automatically on success or after 2 minutes.')
            blid, password, name = self._fetch_one(ip, timeout=120)
            if blid and password:
                self._params[f'robot{idx}_blid'] = blid
                self._params[f'robot{idx}_password'] = password
                if name:
                    self._params[f'robot{idx}_name'] = name
                self.poly.Notices[f'pair{idx}'] = (
                    f'Robot {idx}: paired as {name or blid[:8]}.')
                LOGGER.info(f'Robot {idx} paired: blid={blid}')
            else:
                self.poly.Notices[f'pair{idx}'] = (
                    f'Robot {idx} ({ip}): pairing failed. '
                    'Verify IP, power, and that you pressed Home on the robot.')

    def _fetch_one(self, ip, timeout=120):
        """Look up blid/name via discovery (no user action), then retry
        password getter until the user has put the robot into pairing mode
        (Home held ~2s, Wi-Fi LED pulsing)."""
        blid = name = None
        try:
            info = RoombaDiscovery().get(ip)
            if info:
                blid = getattr(info, 'blid', None)
                name = getattr(info, 'robot_name', None)
        except Exception as e:
            LOGGER.warning(f'Discovery for {ip}: {e}')

        deadline = time.time() + timeout
        last_err = None
        while time.time() < deadline:
            try:
                password = RoombaPassword(ip).get_password()
                if password:
                    return blid, password, name
            except Exception as e:
                last_err = e
            time.sleep(1)
        if last_err:
            LOGGER.warning(f'Password fetch for {ip}: {last_err}')
        return blid, None, name

    # --- Commands / poll ---

    def cmd_discover(self, command=None):
        self._reconcile_robots()
        for node in self._robots.values():
            node.query()

    def query(self, command=None):
        self.reportDrivers()
        for node in self._robots.values():
            node.query()

    def poll(self, flag):
        if flag == 'shortPoll':
            for node in self._robots.values():
                node.query()

    commands = {
        'DISCOVER':    cmd_discover,
        'FETCH_CREDS': cmd_fetch_creds,
        'QUERY':       query,
    }


# ---------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        poly = udi_interface.Interface([])
        poly.start()
        Controller(poly, 'controller', 'controller', 'iRobot')
        poly.runForever()
    except (KeyboardInterrupt, SystemExit):
        sys.exit(0)
    except Exception as e:
        LOGGER.exception(f'Fatal error: {e}')
        sys.exit(1)
