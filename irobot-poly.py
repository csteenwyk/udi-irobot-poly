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

try:
    from roombapy import RoombaFactory
    from roombapy.getpassword import RoombaPassword
    from roombapy.discovery import RoombaDiscovery
except ImportError:
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

# chargingState reported ints (roombapy master_state)
_CHARGE_MAP = {
    'none':            0,
    'charging':        1,
    'reconditioning':  1,
    'full':            2,
    'trickle':         3,
    'waiting':         4,
    'fault':           5,
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
        {'driver': 'ST',    'value': 0, 'uom': 25},
        {'driver': 'BATLVL','value': 0, 'uom': 51},
        {'driver': 'GV1',   'value': 0, 'uom': 2},
        {'driver': 'GV2',   'value': 0, 'uom': 2},
        {'driver': 'GV3',   'value': 0, 'uom': 2},
        {'driver': 'GV4',   'value': 0, 'uom': 25},
        {'driver': 'GV5',   'value': 0, 'uom': 56},
        {'driver': 'GV6',   'value': 0, 'uom': 51},
    ]

    def __init__(self, polyglot, primary, address, name, ip, blid, password, ctrl):
        super().__init__(polyglot, primary, address, name)
        self._ip = ip
        self._blid = blid
        self._password = password
        self._ctrl = ctrl
        self._roomba = None
        self._cache = {}
        self._connect()

    def _connect(self):
        if RoombaFactory is None:
            LOGGER.error('roombapy not installed')
            return
        try:
            self._roomba = RoombaFactory.create_roomba(
                address=self._ip, blid=self._blid, password=self._password,
                continuous=True, delay=10)
            self._roomba.register_on_message_callback(self._on_message)
            self._roomba.connect()
            LOGGER.info(f'{self.name}: connected to {self._ip}')
        except Exception as e:
            LOGGER.error(f'{self.name}: connect failed: {e}')

    def _on_message(self, json_data):
        self._apply_state()

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

            batt   = reported.get('batPct')
            phase  = reported.get('cleanMissionStatus', {}).get('phase')
            bin_   = reported.get('bin', {})
            docked = reported.get('cleanMissionStatus', {}).get('notReady', 1) == 39 \
                     or phase in ('charge', 'dockend')
            charge = reported.get('batteryType') and reported.get('cleanMissionStatus', {}).get('notReady')
            charging = reported.get('cleanMissionStatus', {}).get('phase') == 'charge'
            err    = reported.get('cleanMissionStatus', {}).get('error', 0)
            rssi   = reported.get('signal', {}).get('rssi', 0)

            if batt is not None:
                self._set('BATLVL', int(batt))
            if phase is not None:
                self._set('ST', _PHASE_MAP.get(phase, 0))
            if 'full' in bin_:
                self._set('GV1', 1 if bin_.get('full') else 0)
            if 'present' in bin_:
                self._set('GV2', 1 if bin_.get('present') else 0)
            self._set('GV3', 1 if docked else 0)
            self._set('GV4', 1 if charging else 0)
            self._set('GV5', int(err or 0))
            # Map rssi (-100..-30 dBm) → 0..100 %
            if rssi:
                pct = max(0, min(100, int(2 * (rssi + 100))))
                self._set('GV6', pct)
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

    def cmd_start(self, command):  self._send('start')
    def cmd_stop(self, command):   self._send('stop')
    def cmd_pause(self, command):  self._send('pause')
    def cmd_resume(self, command): self._send('resume')
    def cmd_dock(self, command):   self._send('dock')
    def cmd_locate(self, command): self._send('find')

    def query(self, command=None):
        self._apply_state()
        self.reportDrivers()

    commands = {
        'START':  cmd_start,
        'STOP':   cmd_stop,
        'PAUSE':  cmd_pause,
        'RESUME': cmd_resume,
        'DOCK':   cmd_dock,
        'LOCATE': cmd_locate,
        'QUERY':  query,
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
        """Add nodes for any fully-configured robotN_* entries."""
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
            node = RobotNode(self.poly, self.address, address, name,
                             ip, blid, password, self)
            self._add_node_wait(node)
            self._robots[idx] = node

    # --- Pairing ---

    def cmd_fetch_creds(self, command=None):
        """Fetch blid/password/name for every robotN with ip set but creds missing."""
        if RoombaPassword is None:
            self.poly.Notices['pair'] = 'roombapy not installed — reinstall the nodeserver.'
            return
        configured = _parse_robots(self._last_params)
        targets = [(i, c) for i, c in configured.items()
                   if c.get('ip') and not (c.get('blid') and c.get('password'))]
        if not targets:
            self.poly.Notices['pair'] = 'No robots need pairing.'
            return

        thread = threading.Thread(
            target=self._pair_loop, args=(targets,), daemon=True)
        thread.start()

    def _pair_loop(self, targets):
        for idx, cfg in targets:
            ip = cfg['ip']
            self.poly.Notices[f'pair{idx}'] = (
                f'Robot {idx} ({ip}): hold HOME for ~2s until you hear a beep. '
                'Retrieving credentials...')
            blid, password, name = self._fetch_one(ip, timeout=45)
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

    def _fetch_one(self, ip, timeout=45):
        """Retry roombapy password getter until the robot is in pairing mode."""
        deadline = time.time() + timeout
        last_err = None
        while time.time() < deadline:
            try:
                getter = RoombaPassword(ip)
                result = getter.get_password()
                if result:
                    blid = getattr(result, 'blid', None) or result.get('blid')
                    password = getattr(result, 'password', None) or result.get('password')
                    name = getattr(result, 'robotName', None) or result.get('robotName')
                    if blid and password:
                        return blid, password, name
            except Exception as e:
                last_err = e
            time.sleep(3)
        if last_err:
            LOGGER.warning(f'Password fetch for {ip}: {last_err}')
        return None, None, None

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
