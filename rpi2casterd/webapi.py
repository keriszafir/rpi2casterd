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
    def wrapper(interface_id, *args, **kwargs):
        """wraps the routine"""
        try:
            interface = INTERFACES[interface_id]
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


@APP.route('/')
def index():
    """Main page for rpi2caster interface"""
    return 'It works!'


@APP.route('/interfaces')
def list_interfaces():
    """Lists available interfaces"""
    return jsonify({i: str(interface) for i, interface in INTERFACES.items()})


@APP.route('/interfaces/<interface_id>')
def interface_page(interface_id):
    """Interface's browsable API"""
    url = partial(url_for, interface_id=interface_id)
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


@APP.route('/interfaces/<interface_id>/config')
@handle_request
def get_config(interface):
    """Get the interface configuration"""
    return interface.config


@APP.route('/interfaces/<interface_id>/status')
@handle_request
def get_status(interface):
    """Gets the current interface status"""
    retval = dict()
    retval.update(interface.state)
    retval.update(speed='{}rpm'.format(interface.rpm()))
    return retval


@APP.route('/interfaces/<interface_id>/rpm')
@handle_request
def get_speed(interface):
    """Measure the current RPM"""
    return dict(speed='{}rpm'.format(interface.rpm()))


@APP.route('/interfaces/<interface_id>/wedges')
@handle_request
def get_wedge_positions(interface):
    """Get the current 0005 and 0075 justifying wedge positions."""
    return interface.check_wedge_positions()


@APP.route('/interfaces/<interface_id>/modes', methods=ALL_METHODS)
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


@APP.route('/interfaces/<interface_id>/signals', methods=ALL_METHODS)
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


@APP.route('/interfaces/<interface_id>/<device_name>', methods=ALL_METHODS)
@handle_request
def control(interface, device_name):
    """Control or check the status of one of the machine/interface's devices:
        -caster's pump,
        -caster's motor (using two relays),
        -compressed air supply,
        -cooling water supply,
        -solenoid valves.

    GET checks the device's state.
    DELETE turns the device off (sends False).
    POST or PUT requests turn the device on (state=True), off (state=False)
    or check the device's state (state=None or not specified).
    """
    method_name = '{}_control'.format(device_name)
    # look up the method - if it fails, handle_request will raise 404
    print(method_name)
    print(dir(interface))
    print(interface.__dict__)
    method = interface.__dict__[method_name]
    if request.method in (POST, PUT):
        device_state = request.get_json().get(device_name)
        result = method(interface, device_state)
    elif request.method == DELETE:
        result = method(interface, False)
    elif request.method == GET:
        result = method(interface)
    return dict(device_name=result)
