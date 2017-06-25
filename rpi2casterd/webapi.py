# -*- coding: utf-8 -*-
"""Flask web API for rpi2casterd"""

from collections import OrderedDict
from functools import partial, wraps
from flask import Flask, abort, jsonify, url_for
from flask.globals import request

from rpi2casterd import exceptions as exc
from rpi2casterd.converters import parse_signals

# method names for convenience
GET, PUT, POST = 'GET', 'PUT', 'POST'

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


def pass_state(routine):
    """Get the state from request and pass it to the decorated function"""
    @wraps(routine)
    def wrapper(interface, *args, **kwargs):
        """wraps the routine"""
        if request.method in (POST, PUT):
            state = request.get_json().get('state')
            return routine(interface, state, *args, **kwargs)
        else:
            return routine(interface, None, *args, **kwargs)
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
    retval = dict()
    retval.update(interface.state)
    retval.update(speed='{}rpm'.format(interface.rpm()))
    return retval


@APP.route('/interfaces/<prefix>/rpm')
@handle_request
def get_speed(interface):
    """Measure the current RPM"""
    return dict(speed='{}rpm'.format(interface.rpm()))


@APP.route('/interfaces/<prefix>/wedges')
@handle_request
def get_wedge_positions(interface):
    """Get the current 0005 and 0075 justifying wedge positions."""
    return interface.check_wedge_positions()


@APP.route('/interfaces/<prefix>/start')
@handle_request
def start_machine(interface):
    """Starts the machine"""
    return interface.start()


@APP.route('/interfaces/<prefix>/stop')
@handle_request
def stop_machine(interface):
    """Stops the machine"""
    return interface.stop()


@APP.route('/interfaces/<prefix>/modes', methods=(GET, POST, PUT))
@handle_request
def mode_control(interface):
    """Get or set the interface's operation and row 16 addressing modes.
    GET: gets the modes,
    POST: sets one or both modes."""
    if request.method in (POST, PUT):
        request_data = request.get_json()
        row16_mode = request_data.get('row16_mode')
        operation_mode = request_data.get('operation_mode')
        return interface.mode_control(operation_mode, row16_mode)
    else:
        return interface.mode_control()


@APP.route('/interfaces/<prefix>/signals', methods=(GET, POST, PUT))
@handle_request
def send_signals(interface):
    """Sends the signals to the machine"""
    if request.method in (POST, PUT):
        request_data = request.get_json() or {}
        signals = request_data.get('signals') or []
        codes = parse_signals(signals)
        interface.send_signals(codes)
        return interface.state
    else:
        signals = interface.state['signals']
        return dict(signals=signals)


@APP.route('/interface/<prefix>/valves', methods=(GET, POST, PUT))
@handle_request
@pass_state
def valve_control(interface, state):
    """Control the solenoid valves.
    state: None = get status,
           False = turn all valves off,
           list of signals = turn specified valves on.
    """
    outcome = interface.valve_control(state)
    return dict(signals=outcome)


@APP.route('/interfaces/<prefix>/water', methods=(GET, POST, PUT))
@handle_request
@pass_state
def water_control(interface, state):
    """Cooling water control:
        GET or POST with no data: get status,
        POST with any value: turn on or off.
    """
    outcome = interface.water_control(state)
    return dict(state=outcome)


@APP.route('/interfaces/<prefix>/motor', methods=(GET, POST, PUT))
@handle_request
@pass_state
def motor_control(interface, state):
    """Motor control:
        GET or POST with no data: get status,
        POST with any value: turn on or off.
    """
    outcome = interface.motor_control(state)
    return dict(state=outcome)


@APP.route('/interfaces/<prefix>/air', methods=(GET, POST, PUT))
@handle_request
@pass_state
def air_control(interface, state):
    """Air supply control:
        GET or POST with no data: get status,
        POST with any value: turn on or off.
    """
    outcome = interface.air_control(state)
    return dict(state=outcome)


@APP.route('/interfaces/<prefix>/pump', methods=(GET, POST, PUT))
@handle_request
@pass_state
def pump_control(interface, state):
    """Get a current pump working state."""
    outcome = interface.pump_control()
    return dict(state=outcome)
