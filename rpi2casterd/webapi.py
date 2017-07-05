# -*- coding: utf-8 -*-
"""Flask web API for rpi2casterd"""

from collections import OrderedDict
from functools import wraps
from flask import Flask, abort, jsonify
from flask.globals import request

from rpi2casterd import exceptions as exc

# method names for convenience
ALL_METHODS = GET, PUT, POST, DELETE = 'GET', 'PUT', 'POST', 'DELETE'

APP = Flask('rpi2caster')
INTERFACES = {}


def success(**kwargs):
    """Return a success JSON dict"""
    return jsonify(OrderedDict(success=True, **kwargs))


def failure(exception, **kwargs):
    """Return an error JSON dict"""
    offending_value = str(exception) or None
    return jsonify(OrderedDict(error_name=exception.name,
                               error_code=exception.code,
                               offending_value=offending_value,
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
        except NotImplementedError:
            abort(501)
        except (exc.InterfaceNotStarted, exc.InterfaceBusy, exc.MachineStopped,
                exc.UnsupportedMode, exc.UnsupportedRow16Mode) as error:
            return failure(error)
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
@handle_request
def interface_page(interface):
    """Get the read-only information about the interface.
    Return the JSON-encoded dictionary with:
        name: interface name
        status: current interface state,
        settings: static configuration (in /etc/rpi2casterd.conf)
    """
    status = dict()
    status.update(interface.state)
    status.update(speed='{}rpm'.format(interface.rpm()))
    status.update(signals=interface.signals)
    config = interface.config
    config.update(name=str(interface))
    return dict(status=status, settings=config)


@APP.route('/interfaces/<interface_id>/operation_mode', methods=ALL_METHODS)
@handle_request
def operation_mode(interface):
    """Get or set the operation mode (None = testing; casting or punching).
    GET: gets the current setting,
    POST or PUT: updates the setting,
    DELETE: resets the setting to its default value.
    """
    if request.method in (POST, PUT):
        request_data = request.get_json()
        interface.operation_mode = request_data.get('mode')
    elif request.method == DELETE:
        interface.operation_mode = 'reset'
    return dict(mode=interface.operation_mode)


@APP.route('/interfaces/<interface_id>/cast', methods=(POST, PUT))
@handle_request
def cast(interface, signals):
    """Cast the signals with a composition caster.
    Raise UnsupportedMode if the casting mode is not supported."""
    request_data = request.get_json()
    signals = request_data.get('signals')
    timeout = request_data.get('timeout')
    interface.cast(signals, timeout)
    return dict(signals=interface.signals)


@APP.route('/interfaces/<interface_id>/punch', methods=(POST, PUT))
@handle_request
def punch(interface, signals):
    """Send the signals to a ribbon perforator.
    Raise UnsupportedMode if the punching mode is not supported."""
    request_data = request.get_json()
    signals = request_data.get('signals')
    interface.punch(signals)
    return dict(signals=interface.signals)


@APP.route('/interfaces/<interface_id>/test', methods=(POST, PUT))
@handle_request
def test(interface, signals):
    """Test the interface/valves/machine by sending arbitrary signals.
    This will keep the valves open until called off."""
    request_data = request.get_json()
    signals = request_data.get('signals')
    interface.test(signals)
    return dict(signals=interface.signals)


@APP.route('/interfaces/<interface_id>/row16_mode', methods=ALL_METHODS)
@handle_request
def row_16_addressing_mode(interface):
    """Get or set the row16 addressing mode (None = off; HMN, KMN, unit shift).
    GET: gets the current setting,
    POST or PUT: updates the setting,
    DELETE: resets the setting to its default value.
    """
    if request.method in (POST, PUT):
        request_data = request.get_json()
        interface.row16_mode = request_data.get('mode')
    elif request.method == DELETE:
        interface.row16_mode = 'reset'
    return dict(mode=interface.row16_mode)


@APP.route('/interfaces/<interface_id>/justification', methods=ALL_METHODS)
@handle_request
def justification(interface):
    """GET: get the current 0005 and 0075 justifying wedge positions,
    PUT/POST: set new wedge positions (if position is None, keep current),
    DELETE: reset wedges to 15/15."""
    if request.method in (PUT, POST):
        request_data = request.get_json()
        wedge_0075 = request_data.get('wedge_0075')
        wedge_0005 = request_data.get('wedge_0005')
        galley_trip = request_data.get('galley_trip')
        interface.justification(wedge_0005, wedge_0075, galley_trip)
    elif request.method == DELETE:
        interface.justification(wedge_0005=15, wedge_0075=15,
                                galley_trip=False)

    # get the current wedge positions
    current_0075 = interface.status['wedge_0075']
    current_0005 = interface.status['wedge_0005']
    return dict(wedge_0005=current_0005, wedge_0075=current_0075)


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
        signals = request_data.get('signals') or []
        timeout = request_data.get('timeout')
        interface.send_signals(signals, timeout)
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
    # find a suitable interface method, otherwise it's not implemented
    # handle_request will reply 501
    method_name = '{}_control'.format(device_name)
    try:
        routine = getattr(interface, method_name)
    except AttributeError:
        raise NotImplementedError
    # we're sure that we have a method
    if request.method in (POST, PUT):
        device_state = request.get_json().get(device_name)
        result = routine(device_state)
    elif request.method == DELETE:
        result = routine(False)
    elif request.method == GET:
        result = routine(None)
    return dict(active=result)
