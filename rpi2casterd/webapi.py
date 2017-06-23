# -*- coding: utf-8 -*-
"""Flask web API for rpi2casterd"""

from collections import OrderedDict
from functools import partial, wraps
from flask import Flask, abort, jsonify, url_for
from flask.globals import request

from rpi2casterd import exceptions as exc
from rpi2casterd.converters import parse_signals

APP = Flask('rpi2caster')
INTERFACES = {}


def success(**kwargs):
    """Return a success JSON dict"""
    return jsonify(OrderedDict(success=True, **kwargs))


def failure(**kwargs):
    """Return an error JSON dict"""
    return jsonify(OrderedDict(success=False, **kwargs))


def handle_request(routine):
    """Boilerplate code for the flask API functions,
    used for handling requests to interfaces."""
    @wraps(routine)
    def wrapper(prefix, *args, **kwargs):
        """wraps the routine"""
        try:
            interface = INTERFACES[prefix]
            # does the function return any json-ready parameters?
            outcome = routine(interface, *args, **kwargs) or dict()
            # if caught no exceptions, all went well => return success
            return success(**outcome)
        except KeyError:
            abort(404)
        except (exc.InterfaceNotStarted, exc.InterfaceBusy,
                exc.MachineStopped) as err:
            return failure(error=err.name, error_code=err.code)
        except (exc.UnsupportedMode, exc.UnsupportedRow16Mode) as err:
            return failure(error=err.name, error_code=err.code,
                           offending_value=str(err))
    return wrapper


@APP.route('/')
def index():
    """Main page for rpi2caster interface"""
    return 'It works!'


@APP.route('/interfaces')
def list_interfaces():
    """Lists available interfaces"""
    return jsonify({i: str(interface) for i, interface in INTERFACES.items()})


@APP.route('/interfaces/<prefix>')
def interface_page(prefix):
    """Interface's browsable API"""
    url = partial(url_for, prefix=prefix)
    return '\n'.join(['config: {}'.format(url('get_config')),
                      'status: {}'.format(url('get_status')),
                      'wedges: {}'.format(url('get_wedge_positions')),
                      'valves off: {}'.format(url('valves_off')),
                      'water: {}'.format(url('water_control')),
                      'air: {}'.format(url('air_control')),
                      'motor: {}'.format(url('motor_control'))])


@APP.route('/interfaces/<prefix>/config')
@handle_request
def get_config(interface):
    """Get the interface configuration"""
    return interface.config


@APP.route('/interfaces/<prefix>/status')
@handle_request
def get_status(interface):
    """Gets the current interface status"""
    return interface.get_status()


@APP.route('/interfaces/<prefix>/wedges')
@handle_request
def get_wedge_positions(interface):
    """Get the current 0005 and 0075 justifying wedge positions."""
    return interface.check_wedge_positions()


@APP.route('/interfaces/<prefix>/start', methods=('POST',))
@handle_request
def start_machine(interface):
    """Starts the machine"""
    mode = request.json.get('mode')
    return interface.start(mode=mode)


@APP.route('/interfaces/<prefix>/stop', methods=('POST',))
@handle_request
def stop_machine(interface):
    """Stops the machine"""
    mode = request.json.get('mode')
    return interface.stop(mode=mode)


@APP.route('/interfaces/<prefix>/send', methods=('POST',))
@handle_request
def send_signals(interface):
    """Sends the signals to the machine"""
    signals = request.json.get('signals')
    mode = request.json.get('mode')
    row16_mode = request.json.get('row16_mode')
    codes = parse_signals(signals)
    return interface.send_signals(codes, mode=mode, row16_mode=row16_mode)


@APP.route('/interfaces/<prefix>/valves_on', methods=('POST',))
@handle_request
def valves_on(interface):
    """Turns specified valves on. Low-level control method."""
    signals = request.json.get('signals')
    codes = parse_signals(signals)
    return interface.valves_on(codes)


@APP.route('/interfaces/<prefix>/valves_off', methods=('POST',))
@handle_request
def valves_off(interface):
    """Turns all valves off on the interface."""
    return interface.valves_off()


@APP.route('/interfaces/<prefix>/water', methods=('GET', 'POST'))
@handle_request
def water_control(interface):
    """Cooling water control:
        GET or POST with no data: get status,
        POST with any value: turn on or off.
    """
    if request.method == 'POST':
        state = request.json
        return interface.water_control(state)
    else:
        return interface.water_control()


@APP.route('/interfaces/<prefix>/motor', methods=('GET', 'POST'))
@handle_request
def motor_control(interface):
    """Motor control:
        GET or POST with no data: get status,
        POST with any value: turn on or off.
    """
    if request.method == 'POST':
        state = request.json
        return interface.motor_control(state)
    else:
        return interface.motor_control()


@APP.route('/interfaces/<prefix>/air', methods=('GET', 'POST'))
@handle_request
def air_control(interface):
    """Air supply control:
        GET or POST with no data: get status,
        POST with any value: turn on or off.
    """
    if request.method == 'POST':
        state = request.json
        return interface.air_control(state)
    else:
        return interface.air_control()
