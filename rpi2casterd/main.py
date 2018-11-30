# -*- coding: utf-8 -*-
"""rpi2casterd: hardware control daemon for the rpi2caster software.

This program runs on a Raspberry Pi or a similar single-board computer
and listens on its address(es) on a specified port using the HTTP protocol.
It communicates with client(s) via a JSON API and controls the machine
using selectable backend libraries for greater configurability.
"""
from collections import deque, OrderedDict
from contextlib import suppress
from functools import wraps
import configparser
import signal
import subprocess
import time

import librpi2caster
from flask import Flask, abort, jsonify
from flask.globals import request
import RPi.GPIO as GPIO

ALL_METHODS = GET, PUT, POST, DELETE = 'GET', 'PUT', 'POST', 'DELETE'
ON, OFF = True, False
OUTPUT_SIGNALS = tuple(['0075', 'S', '0005', *'ABCDEFGHIJKLMN',
                        *(str(x) for x in range(1, 15)), 'O15'])

# Where to look for config?
CONFIGURATION_PATH = '/etc/rpi2casterd.conf'
DEFAULTS = dict(name='Monotype composition caster',
                listen_address='0.0.0.0:23017', output_driver='smbus',
                shutdown_gpio='24', shutdown_command='shutdown -h now',
                reboot_gpio='23', reboot_command='shutdown -r now',
                startup_timeout='30', sensor_timeout='5',
                pump_stop_timeout='120',
                punching_on_time='0.2', punching_off_time='0.3',
                debounce_milliseconds='25',
                ready_led_gpio='18', sensor_gpio='17',
                working_led_gpio='', error_led_gpio='',
                air_gpio='', water_gpio='', emergency_stop_gpio='',
                motor_start_gpio='', motor_stop_gpio='',
                mode_detect_gpio='27',
                i2c_bus='1', mcp0_address='0x20', mcp1_address='0x21',
                valve1='N,M,L,K,J,I,H,G',
                valve2='F,S,E,D,0075,C,B,A',
                valve3='1,2,3,4,5,6,7,8',
                valve4='9,10,11,12,13,14,0005,O15')
CFG = configparser.ConfigParser(defaults=DEFAULTS)
CFG.read(CONFIGURATION_PATH)

# Initialize the application
GPIO.setmode(GPIO.BCM)
LEDS = dict()


def setup_gpio(name, direction, pull=None, callbk=None,
               edge=GPIO.FALLING, bouncetime=50):
    """Set up a GPIO input/output"""
    gpio = get(name, CFG.defaults(), 'inone')
    if gpio:
        # set up an input or output
        if pull:
            GPIO.setup(gpio, direction, pull_up_down=pull)
        else:
            GPIO.setup(gpio, direction)

        # try registering a callback function on interrupt
        if callbk:
            try:
                GPIO.add_event_detect(gpio, edge, callbk,
                                      bouncetime=bouncetime)
            except RuntimeError:
                GPIO.add_event_callback(gpio, callbk)
            except TypeError:
                pass
    return gpio


def turn_on(gpio, raise_exception=False):
    """Turn on a specified GPIO output"""
    if not gpio:
        if raise_exception:
            raise NotImplementedError
        else:
            return
    GPIO.output(gpio, ON)


def turn_off(gpio, raise_exception=False):
    """Turn off a specified GPIO output"""
    if not gpio:
        if raise_exception:
            raise NotImplementedError
        else:
            return
    GPIO.output(gpio, OFF)


def get_state(gpio):
    """Get the state of a GPIO input or output"""
    return GPIO.input(gpio)


def toggle(gpio):
    """Change the state of a GPIO output"""
    current_state = GPIO.input(gpio)
    GPIO.output(gpio, not current_state)


def blink(gpio=None, seconds=0.5, times=3):
    """Blinks the LED"""
    led_gpio = LEDS.get(gpio)
    if not led_gpio:
        return
    for _ in range(times * 2):
        toggle(led_gpio)
        time.sleep(seconds)


def get(parameter, source, convert):
    """Gets a value from a specified source for a given parameter,
    converts it to a desired data type"""
    def address_and_port(input_string):
        """Get an IP or DNS address and a port"""
        try:
            address, _port = input_string.split(':')
            port = int(_port)
        except ValueError:
            address = input_string
            port = 23017
        return address, port

    def list_of_signals(input_string):
        """Convert 'a,b,c,d,e' -> ['A', 'B', 'C', 'D', 'E'].
        Allow only known defined signals."""
        raw = [x.strip().upper() for x in input_string.split(',')]
        return [x for x in raw if x in OUTPUT_SIGNALS]

    def list_of_strings(input_string):
        """Convert 'abc , def, 012' -> ['abc', 'def', '012']
        (no case change; strip whitespace)."""
        return [x.strip() for x in input_string.split(',')]

    def lowercase_string(input_string):
        """Return a lowercase string stripped of all whitespace"""
        return input_string.strip().lower()

    def any_integer(input_string):
        """Convert a decimal, octal, binary or hexadecimal string to integer"""
        return int(lowercase_string(input_string), 0)

    def int_or_none(input_string):
        """Return integer or None"""
        stripped = input_string.strip()
        try:
            return int(stripped)
        except ValueError:
            return None

    def command(input_string):
        """Operating system command: string -> accepted by subprocess.run"""
        chunks = input_string.split(' ')
        return [x.strip() for x in chunks]

    converters = dict(anyint=any_integer, address=address_and_port,
                      inone=int_or_none, signals=list_of_signals,
                      lcstring=lowercase_string, strings=list_of_strings,
                      command=command)
    routine = converters.get(convert, convert)
    # get the string from the source configuration
    source_value = source[parameter]
    # convert and return
    return routine(source_value)


def parse_signals(input_signals):
    """Parses the source sequence (str, list, tuple etc.)
    and returns an arranged list of Monotype signals."""
    def is_present(value):
        """Detect and dispatch known signals in source string"""
        nonlocal sequence
        string = str(value)
        if string in sequence:
            # required for correct parsing of numbers
            sequence = sequence.replace(string, '')
            return True
        return False

    try:
        sequence = input_signals.upper()
    except AttributeError:
        sequence = ''.join(str(x) for x in input_signals).upper()

    useful = ['0005', '0075', 'O15', *(str(x) for x in range(15, 0, -1)),
              *'ABCDEFGHIJKLMNOS']
    parsed_signals = {s for s in useful if is_present(s)}
    arranged = deque(s for s in OUTPUT_SIGNALS if s in parsed_signals)
    # put NI, NL, NK, NJ, NKJ etc. at the front
    if 'N' in arranged:
        for other in 'JKLI':
            if other in parsed_signals:
                arranged.remove('N')
                arranged.remove(other)
                arranged.appendleft(other)
                arranged.appendleft('N')
    return list(arranged)


def parse_configuration(source):
    """Get the interface parameters from a config parser section"""
    config = OrderedDict()
    # determine the output driver
    config['output_driver'] = get('output_driver', source, 'lcstring')

    # get timings
    config['startup_timeout'] = get('startup_timeout', source, float)
    config['sensor_timeout'] = get('sensor_timeout', source, float)
    config['pump_stop_timeout'] = get('pump_stop_timeout', source, float)
    config['punching_on_time'] = get('punching_on_time', source, float)
    config['punching_off_time'] = get('punching_off_time', source, float)

    # time (in milliseconds) for software debouncing
    debounce_milliseconds = get('debounce_milliseconds', source, int)
    config['debounce_milliseconds'] = debounce_milliseconds

    # interface settings: output
    config['i2c_bus'] = get('i2c_bus', source, 'anyint')
    config['mcp0_address'] = get('mcp0_address', source, 'anyint')
    config['mcp1_address'] = get('mcp1_address', source, 'anyint')
    config['signal_mappings'] = dict(valve1=get('valve1', source, 'signals'),
                                     valve2=get('valve2', source, 'signals'),
                                     valve3=get('valve3', source, 'signals'),
                                     valve4=get('valve4', source, 'signals'))

    # configuration ready to ship
    return config


def handle_machine_stop(routine):
    """Ensure that when MachineStopped occurs, the interface will run
    its stop() method."""
    @wraps(routine)
    def wrapper(interface, *args, **kwargs):
        """wraps the routine"""
        def check_emergency_stop():
            """check if the emergency stop button registered any events"""
            if interface.emergency_stop_state:
                interface.emergency_stop_state = OFF
                raise librpi2caster.MachineStopped

        try:
            # unfortunately we cannot abort the routine
            check_emergency_stop()
            retval = routine(interface, *args, **kwargs)
            check_emergency_stop()
            return retval
        except (librpi2caster.MachineStopped, KeyboardInterrupt):
            interface.stop()
            raise librpi2caster.MachineStopped
    return wrapper


def daemon_setup():
    """Configure the "ready" LED and shutdown/reboot buttons"""
    def shutdown(*_):
        """Shut the system down"""
        print('Shutdown button pressed. Hold down for 2s to shut down...')
        time.sleep(2)
        # the button is between GPIO and GND i.e. pulled up - negative logic
        if not get_state(shdn):
            print('Shutting down...')
            blink('ready')
            cmd = get('shutdown_command', CFG.defaults(), 'command')
            subprocess.run(cmd)

    def reboot(*_):
        """Restart the system"""
        print('Reboot button pressed. Hold down for 2s to reboot...')
        time.sleep(2)
        # the button is between GPIO and GND i.e. pulled up - negative logic
        if not get_state(reset):
            print('Rebooting...')
            blink('ready')
            cmd = get('reboot_command', CFG.defaults(), 'command')
            subprocess.run(cmd)

    def signal_handler(*_):
        """Exit gracefully if SIGINT or SIGTERM received"""
        raise KeyboardInterrupt

    # set up the ready LED and shutdown/reboot buttons, if possible
    LEDS['ready'] = setup_gpio('ready_led_gpio', GPIO.OUT)
    shdn = setup_gpio('shutdown_gpio', GPIO.IN, GPIO.PUD_UP, shutdown)
    reset = setup_gpio('reboot_gpio', GPIO.IN, GPIO.PUD_UP, reboot)
    # register callbacks for signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)


def handle_request(routine):
    """Boilerplate code for the flask API functions,
    used for handling requests to interfaces."""
    @wraps(routine)
    def wrapper(*args, **kwargs):
        """wraps the routine"""
        try:
            # does the function return any json-ready parameters?
            outcome = routine(*args, **kwargs) or dict()
            # if caught no exceptions, all went well => return success
            return jsonify(OrderedDict(success=True, **outcome))
        except KeyError:
            abort(404)
        except NotImplementedError:
            abort(501)
        except (librpi2caster.InterfaceNotStarted, librpi2caster.InterfaceBusy,
                librpi2caster.MachineStopped) as exc:
            # HTTP response with an error code
            return jsonify(OrderedDict(success=False, error_name=exc.message,
                                       error_code=exc.code,
                                       offending_value=str(exc) or None))
    return wrapper


def webapi(interface, address, port):
    """JSON web API for communicating with the casting software."""
    app = Flask('rpi2casterd')

    @app.route('/', methods=('GET', 'PUT', 'POST', 'DELETE'))
    @handle_request
    def index():
        """Get the read-only information about the interface.
        Return the JSON-encoded dictionary with
            status: current interface state,
            settings: static configuration (in /etc/rpi2casterd.conf)
        """
        if request.method in (POST, PUT):
            request_data = request.get_json()
            print(request_data)
        return dict(status=interface.current_status,
                    config=interface.config)

    @app.route('/justification', methods=('GET', 'PUT', 'POST', 'DELETE'))
    @handle_request
    def justification():
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

    @app.route('/signals', methods=('GET', 'PUT', 'POST', 'DELETE'))
    @handle_request
    def signals():
        """Sends the signals to the machine.
        GET: gets the current signals,
        PUT/POST: sends the signals to the machine;
            the interface will parse and process them according to the current
            operation and row 16 addressing mode."""
        if request.method == GET:
            return dict(signals=interface.signals)
        if request.method in (POST, PUT):
            request_data = request.get_json() or {}
            codes = request_data.get('signals') or []
            timeout = request_data.get('timeout')
            testing_mode = request_data.get('testing_mode')
            interface.send_signals(codes, timeout, testing_mode)
            return interface.current_status
        if request.method == DELETE:
            interface.valves_control(OFF)
            return interface.current_status
        return {}

    @app.route('/<device_name>', methods=('GET', 'PUT', 'POST', 'DELETE'))
    @handle_request
    def control(device_name):
        """Change or check the status of one of the
        machine/interface's controls:
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
            result = routine(OFF)
        elif request.method == GET:
            result = routine()
        return dict(active=result)

    if not interface:
        msg = 'Interface initialization failed. Not starting web API!'
        raise librpi2caster.ConfigurationError(msg)
    app.run(address, port)


def main():
    """Starts the application. Contains web API subroutines."""
    interface = None
    try:
        # get the listen address and port
        config = CFG.defaults()
        address, port = get('listen_address', config, 'address')
        # initialize hardware
        daemon_setup()
        # interface configuration
        settings = parse_configuration(CFG.defaults())
        interface = Interface(settings)
        # all configured - it's ready to work
        ready_led_gpio = LEDS.get('ready')
        turn_on(ready_led_gpio)
        # start the web application
        webapi(interface, address, port)

    except KeyError as exception:
        raise librpi2caster.ConfigurationError(exception)

    except (OSError, PermissionError, RuntimeError) as exception:
        print('ERROR: Not enough privileges to do this.')
        print('You have to belong to the "gpio" and "spidev" user groups.')
        print('If this occurred during reboot/shutdown, you need to run '
              'these commands as root (e.g. with sudo).')
        print(str(exception))

    except KeyboardInterrupt:
        print('System exit.')

    finally:
        # make sure the GPIOs are de-configured properly
        with suppress(AttributeError):
            interface.stop()
        for led_gpio in LEDS.values():
            turn_off(led_gpio)
        LEDS.clear()
        GPIO.cleanup()


class InterfaceBase:
    """Basic data structures of an interface"""
    def __init__(self, config_dict):
        self.config = config_dict
        # data structure to count photocell ON events for rpm meter
        self.meter_events = deque(maxlen=3)
        # temporary GPIO dict (can be populated in hardware_setup)
        self.gpios = dict(working_led=None)
        # initialize machine state
        self.status = dict(wedge_0005=15, wedge_0075=15,
                           working=OFF, water=OFF, air=OFF, motor=OFF,
                           pump=OFF, sensor=OFF, signals=[],
                           emergency_stop=OFF)

    def __str__(self):
        return self.config['name']

    @property
    def emergency_stop_state(self):
        """Check whether emergency stop was triggered"""
        return self.status['emergency_stop']

    @emergency_stop_state.setter
    def emergency_stop_state(self, state):
        """Set the emergency stop state"""
        self.status['emergency_stop'] = True if state else False

    @property
    def is_working(self):
        """Get the machine working status"""
        return self.status['working']

    @is_working.setter
    def is_working(self, state):
        """Set the machine working state"""
        working_status = True if state else False
        if working_status:
            turn_on(self.gpios['working_led'])
        else:
            turn_off(self.gpios['working_led'])
        self.status['working'] = working_status

    @property
    def pump_working(self):
        """Get the pump working status"""
        return self.status['pump']

    @pump_working.setter
    def pump_working(self, state):
        """Set the pump working state"""
        self.status['pump'] = True if state else False

    @property
    def punch_mode(self):
        """Check if interface is in punching mode"""
        return self.config.get('punch_mode')

    @property
    def signals(self):
        """Get the current signals."""
        return self.status['signals']

    @signals.setter
    def signals(self, source):
        """Set the current signals."""
        self.status['signals'] = source

    @property
    def sensor_state(self):
        """Get the sensor state"""
        return self.status['sensor']

    @sensor_state.setter
    def sensor_state(self, state):
        """Update the sensor state"""
        self.status['sensor'] = True if state else False

    @property
    def current_status(self):
        """Get the most current status."""
        status = dict()
        status.update(self.status)
        status.update(speed='{}rpm'.format(self.rpm()))
        return status

    def wait_for_sensor(self, new_state, timeout=None):
        """Wait until the machine cycle sensor changes its state
        to the desired value (True or False).
        If no state change is registered in the given time,
        raise MachineStopped."""
        start_time = time.time()
        timeout = timeout if timeout else self.config['sensor_timeout']
        while self.sensor_state != new_state:
            if time.time() - start_time > timeout:
                raise librpi2caster.MachineStopped
            # wait 10ms to ease the load on the CPU
            time.sleep(0.01)

    def rpm(self):
        """Speed meter for rpi2casterd"""
        events = self.meter_events
        sensor_timeout = self.config['sensor_timeout']
        try:
            # how long in seconds is it from the first to last event?
            duration = events[-1] - events[0]
            if not duration or duration > sensor_timeout:
                # single event or waited too long
                return 0
            # 3 timestamps = 2 rotations
            per_second = (len(events) - 1) / duration
            rpm = round(per_second * 60, 2)
            return rpm
        except IndexError:
            # not enough events / measurement points
            return 0

    def update_pump_and_wedges(self):
        """Check the wedge positions and return them."""
        def found(code):
            """check if code was found in a combination"""
            return set(code).issubset(self.signals)

        # check 0075 wedge position and determine the pump status:
        # find the earliest row number or default to 15
        if found(['0075']) or found('NK'):
            # 0075 always turns the pump on
            self.pump_working = ON
            for pos in range(1, 15):
                if str(pos) in self.signals:
                    self.status['wedge_0075'] = pos
                    break
            else:
                self.status['wedge_0075'] = 15

        elif found(['0005']) or found('NJ'):
            # 0005 without 0075 turns the pump off
            self.pump_working = OFF

        # check 0005 wedge position:
        # find the earliest row number or default to 15
        if found(['0005']) or found('NJ'):
            for pos in range(1, 15):
                if str(pos) in self.signals:
                    self.status['wedge_0005'] = pos
                    break
            else:
                self.status['wedge_0005'] = 15


class Interface(InterfaceBase):
    """Hardware control interface"""
    output, gpios = None, dict()

    def __init__(self, config_dict):
        super().__init__(config_dict)
        self.hardware_setup(self.config)

    def hardware_setup(self, config):
        """Configure the inputs and outputs.
        Raise ConfigurationError if output name is not recognized,
        or modules supporting the hardware backends cannot be imported."""
        def update_sensor(sensor_gpio):
            """Update the RPM event counter"""
            self.sensor_state = get_state(sensor_gpio)
            print('Photocell sensor goes {}'
                  .format('ON' if self.sensor_state else 'OFF'))
            if self.sensor_state:
                self.meter_events.append(time.time())

        def update_emergency_stop(emergency_stop_gpio):
            """Check and update the emergency stop status"""
            self.emergency_stop_state = get_state(emergency_stop_gpio)
            print('Emergency stop button pressed!')

        bouncetime = config['debounce_milliseconds']
        gpios = dict(sensor=setup_gpio('sensor_gpio', GPIO.IN, edge=GPIO.BOTH,
                                       callbk=update_sensor,
                                       bouncetime=bouncetime),
                     emergency_stop=setup_gpio('emergency_stop_gpio', GPIO.IN,
                                               callbk=update_emergency_stop,
                                               edge=GPIO.RISING,
                                               bouncetime=bouncetime),
                     mode_detect=setup_gpio('mode_detect_gpio',
                                            GPIO.IN, GPIO.PUD_UP),
                     error_led=setup_gpio('error_led_gpio', GPIO.OUT),
                     working_led=setup_gpio('working_led_gpio', GPIO.OUT),
                     air=setup_gpio('air_gpio', GPIO.OUT),
                     water=setup_gpio('water_gpio', GPIO.OUT),
                     motor_stop=setup_gpio('motor_stop_gpio', GPIO.OUT),
                     motor_start=setup_gpio('motor_start_gpio', GPIO.OUT))

        # does the interface offer the motor start/stop capability?
        motor_feature = gpios.get('motor_start') and gpios.get('motor_stop')
        self.config['has_motor_control'] = bool(motor_feature)

        # use a GPIO pin for sensing punch/cast mode
        self.config['punch_mode'] = get_state(gpios['mode_detect'])

        # output setup:
        try:
            output_name = config['output_driver']
            if output_name == 'smbus':
                from rpi2casterd.smbus import SMBusOutput as output
            elif output_name == 'wiringpi':
                from rpi2casterd.wiringpi import WiringPiOutput as output
            else:
                raise NameError
            self.output = output(config)
        except NameError:
            raise librpi2caster.ConfigurationError('Unknown output: {}.'
                                                   .format(output_name))
        except ImportError:
            raise librpi2caster.ConfigurationError('{}: module not installed'
                                                   .format(output_name))
        self.gpios = gpios

    @handle_machine_stop
    def start(self, testing_mode=False):
        """Starts the machine. When casting, check if it's running."""
        # check if the interface is already busy
        if self.is_working:
            message = 'Cannot do that - the machine is already working.'
            raise librpi2caster.InterfaceBusy(message)
        # reset the RPM counter
        self.meter_events.clear()
        # reset the emergency stop status
        self.emergency_stop_state = OFF
        # turn on the compressed air
        with suppress(NotImplementedError):
            self.air_control(ON)
        if self.punch_mode or testing_mode:
            # can start working right away
            pass
        else:
            # turn on the cooling water and motor, check the machine rotation
            # if MachineStopped is raised, it'll bubble up from here
            with suppress(NotImplementedError):
                self.water_control(ON)
            with suppress(NotImplementedError):
                self.motor_control(ON)
            # check machine rotation
            timeout = self.config['startup_timeout']
            for _ in range(3, 0, -1):
                self.wait_for_sensor(ON, timeout=timeout)
                self.wait_for_sensor(OFF, timeout=timeout)
        # properly initialized => mark it as working
        self.is_working = True

    def stop(self, testing_mode=False):
        """Stop the machine, making sure that the pump is disengaged."""
        if self.is_working:
            self.pump_control(OFF)
            self.valves_control(OFF)
            self.signals = []
            if self.punch_mode or testing_mode:
                # turn off the air only
                pass
            else:
                # turn off the motor
                with suppress(NotImplementedError):
                    self.motor_control(OFF)
                # turn off the cooling water
                with suppress(NotImplementedError):
                    self.water_control(OFF)
            with suppress(NotImplementedError):
                # turn off the machine air supply
                self.air_control(OFF)
            # release the interface so others can claim it
            self.is_working = False

    def machine_control(self, state=None, testing_mode=False):
        """Machine and interface control.
        If no state or state is None, return the current working state.
        If state evaluates to True, start the machine.
        If state evaluates to False, stop (and try to stop the pump).
        """
        if state is None:
            pass
        elif state:
            self.start(testing_mode)
        else:
            self.stop(testing_mode)
        return self.is_working

    def valves_control(self, state=None):
        """Turn valves on or off, check valve status.
        Accepts signals (turn on), False (turn off) or None (get the status)"""
        if state is None:
            # get the status
            pass
        elif not state:
            # False, 0, empty container etc.
            self.output.valves_off()
        else:
            # got the signals
            codes = parse_signals(state)
            self.output.valves_on(codes)
            self.signals = codes
            self.update_pump_and_wedges()
        return self.signals

    @handle_machine_stop
    def motor_control(self, state=None):
        """Motor control:
            no state or None = get the motor state,
            anything evaluating to True or False = turn on or off"""
        if state is None:
            # do nothing
            retval = self.status['motor']
        elif state:
            start_gpio = self.gpios['motor_start']
            if start_gpio:
                turn_on(start_gpio, raise_exception=True)
                time.sleep(0.5)
                turn_off(start_gpio)
            self.status['motor'] = ON
            retval = ON
        else:
            stop_gpio = self.gpios['motor_stop']
            if stop_gpio:
                turn_on(stop_gpio, raise_exception=True)
                time.sleep(0.5)
                turn_off(stop_gpio)
            self.status['motor'] = OFF
            self.meter_events.clear()
            retval = OFF
        return retval

    def air_control(self, state=None):
        """Air supply control: master compressed air solenoid valve.
            no state or None = get the air state,
            anything evaluating to True or False = turn on or off"""
        if state is None:
            retval = self.status['air']
        elif state:
            turn_on(self.gpios['air'], raise_exception=True)
            self.status['air'] = ON
            retval = ON
        else:
            turn_off(self.gpios['air'], raise_exception=True)
            self.status['air'] = OFF
            retval = OFF
        return retval

    def water_control(self, state=None):
        """Cooling water control:
            no state or None = get the water valve state,
            anything evaluating to True or False = turn on or off"""
        if state is None:
            retval = self.status['water']
        elif state:
            turn_on(self.gpios['water'], raise_exception=True)
            self.status['water'] = ON
            retval = ON
        else:
            turn_off(self.gpios['water'], raise_exception=True)
            self.status['water'] = OFF
            retval = OFF
        return retval

    @handle_machine_stop
    def pump_control(self, state=None):
        """No state: get the pump status.
        Anything evaluating to True or False: start or stop the pump"""
        def start():
            """Start the pump."""
            # get the current 0075 wedge position and preserve it
            wedge_0075 = self.status['wedge_0075']
            self.send_signals('NKS0075{}'.format(wedge_0075))

        def stop():
            """Stop the pump if it is working.
            This function will send the pump stop combination (NJS 0005) twice
            to make sure that the pump is turned off.
            In case of failure, repeat."""
            if not self.pump_working:
                return

            if self.is_working:
                turn_off(self.gpios['working_led'])
            turn_on(self.gpios['error_led'])

            # don't change the current 0005 wedge position
            wedge_0005 = self.status['wedge_0005']
            stop_code = 'NJS0005{}'.format(wedge_0005)

            # use longer timeout
            timeout = self.config['pump_stop_timeout']

            # try as long as necessary
            while self.pump_working:
                self.send_signals(stop_code, timeout=timeout)
                self.send_signals(stop_code, timeout=timeout)

            # finished; emergency LED off, working LED on if needed
            turn_off(self.gpios['error_led'])
            if self.is_working:
                turn_on(self.gpios['working_led'])

        if state is None:
            pass
        elif state:
            start()
        else:
            stop()
        return self.pump_working

    def justification(self, galley_trip=False,
                      wedge_0005=None, wedge_0075=None):
        """Single/double justification and 0075/0005 wedge control.

        If galley_trip is desired, put the line to the galley (0075+0005),
        setting the wedges to their new positions (if specified),
        or keeping the current positions.

        Otherwise, determine if the wedges change positions
        and set them if needed.

        This function checks if the pump is currently active, and sends
        the signals in a sequence preserving the pump status
        (if the pump was off, it will be off, and vice versa).
        """
        current_0005 = self.status['wedge_0005']
        current_0075 = self.status['wedge_0075']
        new_0005 = wedge_0005 or current_0005
        new_0075 = wedge_0075 or current_0075

        if galley_trip:
            # double justification: line out + set wedges
            if self.pump_working:
                self.send_signals('NKJS 0075 0005 {}'.format(new_0005))
                self.send_signals('NKS 0075 {}'.format(new_0075))
            else:
                self.send_signals('NKJS 0075 0005{}'.format(new_0075))
                self.send_signals('NJS 0005 {}'.format(new_0005))

        elif new_0005 == current_0005 and new_0075 == current_0075:
            # no need to do anything
            return

        else:
            # single justification = no galley trip
            if self.pump_working:
                self.send_signals('NJS 0005 {}'.format(new_0005))
                self.send_signals('NKS 0075 {}'.format(new_0075))
            else:
                self.send_signals('NKS 0075 {}'.format(new_0075))
                self.send_signals('NJS 0005 {}'.format(new_0005))

    def send_signals(self, signals, timeout=None, testing_mode=False):
        """Send the signals to the caster/perforator.
        This method performs a single-dispatch on current operation mode:
            casting: sensor ON, valves ON, sensor OFF, valves OFF;
            punching: valves ON, wait t1, valves OFF, wait t2
            testing: valves OFF, valves ON

        In the punching mode, if there are less than two signals,
        an additional O+15 signal will be activated. Otherwise the paper ribbon
        advance mechanism won't work."""
        @handle_machine_stop
        def cast(codes):
            """Monotype composition caster.

            Wait for sensor to go ON, turn on the valves,
            wait for sensor to go OFF, turn off the valves.
            """
            if not self.is_working:
                raise librpi2caster.InterfaceNotStarted

            # skip O15 when casting
            sigs = [s for s in codes if s != 'O15']
            # allow the use of a custom timeout
            wait = timeout or self.config['sensor_timeout']
            # machine control cycle
            self.wait_for_sensor(ON, timeout=wait)
            self.valves_control(sigs)
            self.wait_for_sensor(OFF, timeout=wait)
            self.valves_control(OFF)

        @handle_machine_stop
        def test(codes):
            """Turn off any previous combination, then send signals."""
            if not self.is_working:
                self.start(ON)

            # change the active combination
            self.valves_control(OFF)
            self.valves_control(codes)

        @handle_machine_stop
        def punch(codes):
            """Timer-driven ribbon perforator."""
            if not self.is_working:
                self.start()

            # add missing O15 if less than 2 signals
            sigs = codes if len(codes) >= 2 else [*codes, 'O15']
            # timer-driven operation
            self.valves_control(sigs)
            time.sleep(self.config['punching_on_time'])
            self.valves_control(OFF)
            time.sleep(self.config['punching_off_time'])

        if not signals:
            # this tells the interface to turn off all the valves
            self.valves_control(OFF)
            return
        method = test if testing_mode else punch if self.punch_mode else cast
        method(signals)


if __name__ == '__main__':
    main()
