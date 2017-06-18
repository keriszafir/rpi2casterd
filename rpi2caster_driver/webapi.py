# -*- coding: utf-8 -*-
"""Flask web API for rpi2caster-driver"""

from collections import OrderedDict
from functools import wraps
from flask import Flask, abort, jsonify
from flask.globals import request

from .converters import parse_signals

APP = Flask('rpi2caster')
INTERFACES = {}


def handle_request(routine):
    """Boilerplate code for the flask API functions"""
    @wraps(routine)
    def wrapper(*args, **kwargs):
        """wraps the routine"""
        try:
            retval = routine(*args, **kwargs) or dict()
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
@handle_request
def list_interfaces():
    """Lists available interfaces"""
    return {i: interface.name for i, interface in INTERFACES.values()}


@APP.route('/interfaces/<prefix>/config', methods=('GET', 'POST'))
@handle_request
def configuration(prefix):
    """GET: reads the interface configuration,
    POST: changes the configuration"""
    interface = INTERFACES[prefix]
    if request.method == 'GET':
        return interface.config
    elif request.method == 'POST':
        return interface.set_config(request.json)


@APP.route('/interfaces/<prefix>/status')
@handle_request
def get_status(prefix):
    """Gets the current interface status"""
    interface = INTERFACES[prefix]
    return interface.status


@APP.route('/interfaces/<prefix>/start')
@handle_request
def start_machine(prefix):
    """Starts the machine"""
    interface = INTERFACES[prefix]
    return interface.start()


@APP.route('/interfaces/<prefix>/stop')
@handle_request
def stop_machine(prefix):
    """Stops the machine"""
    interface = INTERFACES[prefix]
    return interface.stop()


@APP.route('/interfaces/<prefix>/send', methods=('POST',))
@handle_request
def send_signals(prefix):
    """Sends the signals to the machine"""
    interface = INTERFACES[prefix]
    signals = request.json.get('signals')
    codes = parse_signals(signals)
    return interface.send_signals(codes)


@APP.route('/interfaces/<prefix>/valves_on', methods=('POST',))
@handle_request
def valves_on(prefix):
    """Turns specified valves on. Low-level control method."""
    interface = INTERFACES[prefix]
    signals = request.json.get('signals')
    return interface.valves_on(signals)


@APP.route('/interfaces/<prefix>/valves_off')
@handle_request
def valves_off(prefix):
    """Turns all valves off on the interface."""
    interface = INTERFACES[prefix]
    return interface.valves_off()