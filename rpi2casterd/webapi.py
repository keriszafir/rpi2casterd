# -*- coding: utf-8 -*-
"""Flask web API for rpi2casterd"""

from collections import OrderedDict
from functools import partial, wraps
from flask import Flask, abort, jsonify, url_for
from flask.globals import request

from rpi2casterd import exceptions as exc
from rpi2casterd.converters import parse_signals

# method names for convenience
ALL_METHODS = GET, PUT, POST, DELETE = 'GET', 'PUT', 'POST', 'DELETE'

APP = Flask('rpi2caster')
INTERFACES = {}


def success(**kwargs):
    """Return a success JSON dict"""
    return jsonify(OrderedDict(success=True, **kwargs))


def failure(exception, **kwargs):
    """Return an error JSON dict"""
    return jsonify(OrderedDict(error_name=exception.name,
                               error_code=exception.code,
                               success=False, **kwargs))


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
                exc.MachineStopped) as error:
            return failure(error)
        except (exc.UnsupportedMode, exc.UnsupportedRow16Mode) as error:
            return failure(error, offending_value=str(error))
    return wrapper


def pass_content(routine):
    """Get the content from request and pass it to the decorated function"""
    @wraps(routine)
    def wrapper(interface, *args, **kwargs):
        """wraps the routine"""
        if request.method in (POST, PUT):
            content = request.get_json().get('content')
            return routine(interface, content, *args, **kwargs)
        elif request.method == DELETE:
            # turn off
            return routine(interface, False, *args, **kwargs)
        else:
            # get state
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
                      'signals: {}'.format(url('signals')),
                      'mode control: {}'.format(url('mode_control')),
                      'machine control: {}'.format(url('machine_control')),
                      'pump control: {}'.format(url('pump_control')),
                      'valve control: {}'.format(url('valve_control')),
                      'water control: {}'.format(url('water_control')),
                      'air control: {}'.format(url('air_control')),
                      'motor control: {}'.format(url('motor_control'))])


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


@APP.route('/interfaces/<prefix>/modes', methods=ALL_METHODS)
@handle_request
def mode_control(interface):
    """Get or set the interface's operation and row 16 addressing modes.
    GET: gets the modes.
    PUT/POST: sets one or both modes.
    DELETE: resets the modes to the default values."""
    if request.method in (POST, PUT):
        request_data = request.get_json()
        row16_mode = request_data.get('row16_mode')
        operation_mode = request_data.get('operation_mode')
        return interface.mode_control(operation_mode, row16_mode)
    elif request.method == DELETE:
        return interface.mode_control('reset', 'reset')
    else:
        return interface.mode_control()


@APP.route('/interfaces/<prefix>/signals', methods=ALL_METHODS)
@handle_request
def signals(interface):
    """Sends the signals to the machine.
    GET: gets the current signals,
    PUT/POST: sends the signals to the machine;
        the interface will parse and process them according to the current
        operation and row 16 addressing mode."""
    if request.method in (POST, PUT):
        request_data = request.get_json() or {}
        raw_codes = request_data.get('signals') or []
        timeout = request_data.get('timeout')
        codes = parse_signals(raw_codes)
        interface.send_signals(codes, timeout)
    return dict(signals=interface.signals)


@APP.route('/interfaces/<prefix>/machine', methods=ALL_METHODS)
@handle_request
@pass_content
def machine_control(interface, content):
    """Machine on/off control.
    GET: checks the machine running status.
    PUT/POST: starts or stops the machine.
    DELETE: turns the machine off.
    """
    outcome = interface.machine_control(content)
    return dict(running=outcome)


@APP.route('/interface/<prefix>/valves', methods=ALL_METHODS)
@handle_request
@pass_content
def valve_control(interface, content):
    """Control the solenoid valves.
    GET: get the signals,
    PUT/POST:
    content: None = get status,
           False = turn all valves off,
           list of signals = turn specified valves on.
    DELETE: turn all valves off.
    """
    outcome = interface.valve_control(content)
    return dict(signals=outcome)


@APP.route('/interfaces/<prefix>/water', methods=ALL_METHODS)
@handle_request
@pass_content
def water_control(interface, content):
    """Cooling water control:
        GET, PUT or POST with no data: get status.
        PUT/POST with any value: turn on or off.
        DELETE: turn off.
    """
    outcome = interface.water_control(content)
    return dict(state=outcome)


@APP.route('/interfaces/<prefix>/motor', methods=ALL_METHODS)
@handle_request
@pass_content
def motor_control(interface, content):
    """Motor control:
        GET, PUT or POST with no data: get status.
        PUT/POST with any value: turn on or off.
        DELETE: turn off.
    """
    outcome = interface.motor_control(content)
    return dict(state=outcome)


@APP.route('/interfaces/<prefix>/air', methods=ALL_METHODS)
@handle_request
@pass_content
def air_control(interface, content):
    """Air supply control:
        GET or PUT/POST with no data: get status,
        PUT/POST with any value: turn on or off.
        DELETE: turn off.
    """
    outcome = interface.air_control(content)
    return dict(state=outcome)


@APP.route('/interfaces/<prefix>/pump', methods=ALL_METHODS)
@handle_request
@pass_content
def pump_control(interface, content):
    """Pump control:
        GET or POST with no data: get a current pump working state.
        PUT/POST with any value: turn on or off.
        DELETE: turn off."""
    outcome = interface.pump_control(content)
    return dict(state=outcome)
