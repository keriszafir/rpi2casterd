# -*- coding: utf-8 -*-
"""Flask web API for rpi2caster-driver"""

from collections import OrderedDict
from functools import wraps
from flask import Flask, abort, jsonify
from flask.globals import request

from rpi2caster_driver.converters import parse_signals

APP = Flask('rpi2caster')
INTERFACES = {}


def handle_request(routine):
    """Boilerplate code for the flask API functions,
    used for handling requests to interfaces."""
    @wraps(routine)
    def wrapper(prefix, *args, **kwargs):
        """wraps the routine"""
        try:
            interface = INTERFACES[prefix]
            retval = routine(interface, *args, **kwargs) or dict()
            if 'error' in retval:
                outcome = OrderedDict(success=False)
            else:
                outcome = OrderedDict(success=True)
            outcome.update(retval)
            return jsonify(outcome)
        except KeyError:
            abort(404)
    return wrapper


@APP.route('/interfaces', methods=('GET',))
def list_interfaces():
    """Lists available interfaces"""
    return {i: interface.name for (i, interface) in INTERFACES.items()}


@APP.route('/interfaces/<prefix>/config', methods=('GET', 'POST'))
@handle_request
def configuration(interface):
    """GET: reads the interface configuration,
    POST: changes the configuration"""
    if request.method == 'GET':
        return interface.config
    elif request.method == 'POST':
        return interface.set_config(request.json)


@APP.route('/interfaces/<prefix>/status')
@handle_request
def get_status(interface):
    """Gets the current interface status"""
    return interface.status


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


@APP.route('/interfaces/<prefix>/send', methods=('POST',))
@handle_request
def send_signals(interface):
    """Sends the signals to the machine"""
    signals = request.json.get('signals')
    codes = parse_signals(signals)
    return interface.send_signals(codes)


@APP.route('/interfaces/<prefix>/valves_on', methods=('POST',))
@handle_request
def valves_on(interface):
    """Turns specified valves on. Low-level control method."""
    signals = request.json.get('signals')
    return interface.valves_on(signals)


@APP.route('/interfaces/<prefix>/valves_off')
@handle_request
def valves_off(interface):
    """Turns all valves off on the interface."""
    return interface.valves_off()
