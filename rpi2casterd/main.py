# -*- coding: utf-8 -*-
"""rpi2casterd: hardware control daemon for the rpi2caster software.

This program runs on a Raspberry Pi or a similar single-board computer
and listens on its address(es) on a specified port using the HTTP protocol.
It communicates with client(s) via a JSON API and controls the machine
using selectable backend libraries for greater configurability.
"""
from collections import deque, OrderedDict
from contextlib import contextmanager, suppress
from functools import wraps
import configparser
import logging
import signal
import subprocess
import time

import librpi2caster
from flask import Flask, abort, jsonify
from flask.globals import request
from gpiozero import Button, LED, GPIOPinMissing, GPIOPinInUse

LOG = logging.getLogger('rpi2casterd')
ALL_METHODS = GET, PUT, POST, DELETE = 'GET', 'PUT', 'POST', 'DELETE'
IN, OUT = ON, OFF = True, False
OUTPUT_SIGNALS = tuple(['0075', 'S', '0005', *'ABCDEFGHIJKLMN',
                        *(str(x) for x in range(1, 15)), 'O15'])

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
CFG.read(['/usr/lib/rpi2casterd/rpi2casterd.conf', '/etc/rpi2casterd.conf'])


def journald_setup():
    """Set up and start journald logging"""
    with suppress(ImportError):
        from systemd.journal import JournalHandler
        journal_handler = JournalHandler()
        log_entry_format = '[%(levelname)s] %(message)s'
        journal_handler.setFormatter(logging.Formatter(log_entry_format))
        LOG.setLevel(logging.DEBUG)
        LOG.addHandler(journal_handler)


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
    def signals(input_string):
        """Convert 'a,b,c,d,e' -> ['A', 'B', 'C', 'D', 'E'].
        Allow only known defined signals."""
        raw = [x.strip().upper() for x in input_string.split(',')]
        return [x for x in raw if x in OUTPUT_SIGNALS]

    def integer(input_string):
        """Convert a decimal, octal, binary or hexadecimal string to int"""
        with suppress(TypeError):
            return int(input_string, 0)

    def get(parameter, convert=str):
        """Gets a value from a specified source for a given parameter,
        converts it to a desired data type"""
        return convert(source.get(parameter))

    def address_and_port(input_string):
        """Get an IP or DNS address and a port"""
        try:
            address, _port = input_string.split(':')
            port = int(_port)
        except ValueError:
            address, port = input_string, 23017
        return address, port

    config = OrderedDict()

    # get timings
    config['name'] = get('name', str)
    config['address'], config['port'] = get('listen_address', address_and_port)
    config['startup_timeout'] = get('startup_timeout', float)
    config['sensor_timeout'] = get('sensor_timeout', float)
    config['pump_stop_timeout'] = get('pump_stop_timeout', float)
    config['punching_on_time'] = get('punching_on_time', float)
    config['punching_off_time'] = get('punching_off_time', float)

    # determine the output driver and settings
    config['output_driver'] = get('output_driver').lower()
    config['i2c_bus'] = get('i2c_bus', integer)
    config['mcp0_address'] = get('mcp0_address', integer)
    config['mcp1_address'] = get('mcp1_address', integer)
    valves = dict(valve1=get('valve1', signals), valve2=get('valve2', signals),
                  valve3=get('valve3', signals), valve4=get('valve4', signals))
    config['signal_mappings'] = valves
    return config


def daemon_setup():
    """Configure the "ready" LED and shutdown/reboot buttons"""
    def shutdown():
        """Shut the system down"""
        command = CFG.defaults().get('shutdown_command')
        LOG.info('Shutting down...')
        with suppress(AttributeError):
            GPIO.ready_led.blink(0.5, 0.5)
        subprocess.run([x.strip() for x in command.split(' ')])

    def reboot():
        """Restart the system"""
        command = CFG.defaults().get('reboot_command')
        LOG.info('Rebooting...')
        with suppress(AttributeError):
            GPIO.ready_led.blink(0.2, 0.2)
        subprocess.run([x.strip() for x in command.split(' ')])

    def signal_handler(*_):
        """Exit gracefully if SIGINT or SIGTERM received"""
        raise KeyboardInterrupt

    # set up the GPIOs for the interface
    GPIO.reboot_button.when_held = reboot
    GPIO.shutdown_button.when_held = shutdown
    # register callbacks for signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)


def pin(name, direction, **kwargs):
    """Set up an input or output pin"""
    gpio_string = CFG.defaults().get('{}_gpio'.format(name)).strip()
    with suppress(TypeError, ValueError, GPIOPinMissing, GPIOPinInUse):
        gpio_number = int(gpio_string)
        device = Button if direction == IN else LED
        return device(gpio_number, **kwargs)


def main():
    """Starts the application. Contains web API subroutines."""
    journald_setup()
    interface = None
    config = CFG.defaults()
    try:
        # initialize hardware
        daemon_setup()
        # interface configuration
        settings = parse_configuration(config)
        interface = Interface(settings)
        GPIO.ready_led.on()
        # start the web API for communicating with client
        interface.webapi()

    except KeyError as exception:
        raise librpi2caster.ConfigurationError(exception)

    except (OSError, PermissionError, RuntimeError) as exception:
        LOG.error('ERROR: Not enough privileges to do this.')
        LOG.info('You have to belong to the "gpio" and "spidev" user groups. '
                 'If this occurred during reboot/shutdown, you need to run '
                 'these commands as root (e.g. with sudo).')
        LOG.error(str(exception))

    except KeyboardInterrupt:
        LOG.info('System exit due to ctrl-C keypress.')

    finally:
        # make sure the GPIOs are de-configured properly
        with suppress(AttributeError):
            interface.stop()
        GPIO.cleanup()


class InterfaceBase:
    """Basic data structures of an interface"""
    def __init__(self, config_dict):
        self.config = config_dict
        self.output = None
        # data structure to count photocell ON events for rpm meter
        self.meter_events = deque(maxlen=3)
        # initialize machine state
        self.status = dict(wedge_0005=15, wedge_0075=15, valves=OFF,
                           machine=OFF, motor=OFF, sensor=OFF, signals=[],
                           testing_mode=OFF, emergency_stop=OFF)
        self.hardware_setup()

    def __str__(self):
        return self.config.get('name', 'Monotype composition caster')

    @property
    def is_working(self):
        """Get the machine working status"""
        return self.status.get('machine')

    @is_working.setter
    def is_working(self, state):
        """Set the machine working state"""
        self.status.update(machine=bool(state))

    @property
    def punch_mode(self):
        """Check if interface is in punching mode"""
        return self.config.get('punch_mode')

    @property
    def testing_mode(self):
        """Check if interface is in testing mode"""
        return self.status.get('testing_mode')

    @testing_mode.setter
    def testing_mode(self, state):
        """Update the testing mode"""
        self.status.update(testing_mode=bool(state))

    @property
    def signals(self):
        """Get the current signals."""
        return self.status.get('signals')

    @signals.setter
    def signals(self, source):
        """Set the current signals."""
        message = 'Signals received: {}'.format(''.join([s for s in source]))
        LOG.debug(message)
        codes = parse_signals(source)
        # do some changes based on mode
        if self.testing_mode:
            signals = codes
        elif self.punch_mode:
            signals = (codes if len(codes) >= 2
                       else codes if 'O15' in codes else [*codes, 'O15'])
        else:
            signals = [s for s in codes if s != 'O15']
        message = 'Sending signals: {}'.format(' '.join(signals))
        LOG.debug(message)
        self.status.update(signals=signals)

    @property
    def pump(self):
        """Check the pump state"""
        return self.status.get('pump')

    @pump.setter
    def pump(self, state):
        """Set the pump state"""
        self.status.update(pump=bool(state))

    @property
    def emergency_stop(self):
        """Get the emergency stop state"""
        return self.status.get('emergency_stop')

    @emergency_stop.setter
    def emergency_stop(self, state):
        """Set the emergency stop state"""
        self.status.update(emergency_stop=bool(state))

    @staticmethod
    def hardware_setup():
        """Nothing to do."""
        pass

    def wait_for_sensor(self, new_state, timeout=None):
        """Wait until the machine cycle sensor changes its state
        to the desired value (True or False).
        If no state change is registered in the given time,
        raise MachineStopped."""
        message = 'Waiting for sensor state {}'.format(new_state)
        LOG.debug(message)
        start_time = time.time()
        timeout = timeout if timeout else self.config.get('sensor_timeout', 5)
        while GPIO.sensor.value != new_state:
            timed_out = time.time() - start_time > timeout
            if GPIO.estop_button.value or self.emergency_stop or timed_out:
                raise librpi2caster.MachineStopped
            # wait 5ms to ease the load on the CPU
            time.sleep(0.005)

    def rpm(self):
        """Speed meter for rpi2casterd"""
        events = self.meter_events
        sensor_timeout = self.config.get('sensor_timeout', 5)
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

        # check the previous wedge positions and pump state
        pos_0075 = self.status.get('wedge_0075')
        pos_0005 = self.status.get('wedge_0005')
        pump_working = self.status.get('pump')
        # check 0005 wedge position:
        # find the earliest row number or default to 15
        if found(['0005']) or found('NJ'):
            pump_working = OFF
            for pos in range(1, 15):
                if str(pos) in self.signals:
                    pos_0005 = pos
                    break
            else:
                pos_0005 = 15

        # check 0075 wedge position and determine the pump status:
        # find the earliest row number or default to 15
        if found(['0075']) or found('NK'):
            # 0075 always turns the pump on
            pump_working = ON
            for pos in range(1, 15):
                if str(pos) in self.signals:
                    pos_0075 = pos
                    break
            else:
                pos_0075 = 15

        self.status.update(pump=bool(pump_working),
                           wedge_0075=pos_0075, wedge_0005=pos_0005)


class Interface(InterfaceBase):
    """Hardware control interface"""
    def webapi(self):
        """JSON web API for communicating with the casting software."""
        def handle_request(routine):
            """Boilerplate code for the flask API functions,
            used for handling requests to interfaces."""
            @wraps(routine)
            def wrapper(*args, **kwargs):
                """wraps the routine"""
                response = OrderedDict()
                try:
                    # does the function return any json-ready parameters?
                    outcome = routine(*args, **kwargs) or {}
                    # if caught no exceptions, all went well => return success
                    response.update(success=True, **outcome)
                except KeyError:
                    abort(404)
                except NotImplementedError:
                    abort(501)
                except (librpi2caster.InterfaceNotStarted,
                        librpi2caster.InterfaceBusy,
                        librpi2caster.MachineStopped) as exc:
                    # HTTP response with an error code
                    response.update(success=False, error_code=exc.code,
                                    error_name=exc.message)
                return jsonify(response)

            return wrapper

        @handle_request
        def index():
            """Get or change the interface's current status."""
            if request.method in (POST, PUT):
                request_data = request.get_json() or {}
                self.status.update(**request_data)
            status = self.status
            status.update(speed='{}rpm'.format(self.rpm()),
                          **GPIO.get_values())
            return status

        @handle_request
        def config():
            """Get or change the interface's configuration"""
            if request.method in (POST, PUT):
                request_data = request.get_json() or {}
                self.config.update(request_data)
            return self.config

        @handle_request
        def signals():
            """Sends the signals to the machine.
            GET: gets the current signals,
            PUT/POST: sends the signals to the machine."""
            if request.method in (POST, PUT):
                request_data = request.get_json() or dict()
                codes = request_data.get('signals') or []
                timeout = request_data.get('timeout')
                self.send_signals(codes, timeout)
            elif request.method == DELETE:
                self.valves_control(OFF)
            return dict(signals=self.signals)

        @handle_request
        def control(device):
            """Change or check the status of one of the
            machine/interface's controls:
                -caster's pump,
                -caster's motor (using two relays),
                -compressed air supply,
                -cooling water supply,
                -solenoid valves.

            GET checks the device's state,
            DELETE turns the device off, PUT turns it on
            POST turns on (state=True), off (state=False)
            """
            # find a suitable interface method, otherwise it's not implemented
            # handle_request will reply 501
            method_name = '{}_control'.format(device)
            request_data = request.get_json() or {}
            device_state = request_data.get('state')
            try:
                routine = getattr(self, method_name)
            except AttributeError:
                raise NotImplementedError
            # we're sure that we have a method
            if request.method == POST and device_state is not None:
                routine(bool(device_state))
            elif request.method == PUT:
                routine(ON)
            elif request.method == DELETE:
                routine(OFF)
            # always return the current state of the controlled device
            return dict(active=self.status.get(device))

        app = Flask('rpi2casterd')
        app.route('/', methods=ALL_METHODS)(index)
        app.route('/config', methods=ALL_METHODS)(config)
        app.route('/signals', methods=ALL_METHODS)(signals)
        app.route('/<device>', methods=ALL_METHODS)(control)
        app.run(self.config.get('address'), self.config.get('port'))

    def hardware_setup(self):
        """Configure the inputs and outputs.
        Raise ConfigurationError if output name is not recognized,
        or modules supporting the hardware backends cannot be imported."""
        def update_rpm_meter():
            """Update the RPM event counter"""
            LOG.debug('Photocell sensor activated')
            self.meter_events.append(time.time())

        def update_emergency_stop():
            """Check and update the emergency stop status"""
            LOG.error('Emergency stop button pressed!')
            self.emergency_stop = ON

        # register callbacks
        GPIO.sensor.when_pressed = update_rpm_meter
        GPIO.estop_button.when_pressed = update_emergency_stop

        # does the interface offer the motor start/stop capability?
        motor_feature = GPIO.motor_start and GPIO.motor_stop
        self.config['has_motor_control'] = bool(motor_feature)

        # use a GPIO pin for sensing punch/cast mode
        self.config['punch_mode'] = not bool(GPIO.mode_detect.value)

        # output setup:
        try:
            output_name = self.config.get('output_driver')
            if output_name == 'smbus':
                from rpi2casterd.smbus import SMBusOutput as output
            elif output_name == 'wiringpi':
                from rpi2casterd.wiringpi import WiringPiOutput as output
            else:
                raise NameError
            self.output = output(self.config)
        except NameError:
            raise librpi2caster.ConfigurationError('Unknown output: {}.'
                                                   .format(output_name))
        except ImportError:
            raise librpi2caster.ConfigurationError('{}: module not installed'
                                                   .format(output_name))

    @contextmanager
    def handle_machine_stop(self, suppress_stop=False):
        """Make sure that when MachineStopped occurs inside the contextmanager,
        the machine will be stopped and the exception raised."""
        def check_estop():
            """if emergency stop is active, raise MachineStopped"""
            if not suppress_stop:
                if self.emergency_stop or GPIO.estop_button.value:
                    raise librpi2caster.MachineStopped

        try:
            check_estop()
            yield
            check_estop()
        except librpi2caster.MachineStopped:
            if not suppress_stop:
                self.stop()
                raise

    def start(self):
        """Starts the machine. When casting, check if it's running."""
        # check if the interface is already busy
        LOG.info('Starting the machine...')
        if self.is_working:
            message = 'Cannot do that - the machine is already working.'
            LOG.error(message)
            raise librpi2caster.InterfaceBusy(message)
        self.is_working = True
        GPIO.error_led.value, GPIO.working_led.value = ON, ON
        # reset the RPM counter
        self.meter_events.clear()
        # turn on the compressed air
        self.air_control(ON)
        if self.punch_mode or self.testing_mode:
            # automatically reset the emergency stop if it was engaged
            self.emergency_stop = OFF
        else:
            # turn on the cooling water and motor, check the rotation
            # if MachineStopped is raised, it'll bubble up from here
            self.water_control(ON)
            self.motor_control(ON)
            # check machine rotation
            timeout = self.config.get('startup_timeout', 5)
            for _ in range(3):
                with self.handle_machine_stop():
                    self.wait_for_sensor(ON, timeout=timeout)
                    self.wait_for_sensor(OFF, timeout=timeout)
        LOG.info('Machine started.')
        # properly initialized => mark it as working
        GPIO.error_led.value = OFF

    def stop(self):
        """Stop the machine, making sure that the pump is disengaged."""
        LOG.debug('Checking if the machine is working...')
        if self.is_working:
            LOG.debug('The machine was working.')
            LOG.info('Stopping the machine...')
            # orange LED for stopping sequence
            GPIO.error_led.value, GPIO.working_led.value = ON, ON
            # store the emergency stop state;
            # temporarily ooverride for pump stop
            estop, self.emergency_stop = self.emergency_stop, OFF
            while self.pump:
                # force turning the pump off
                LOG.info('Stopping the pump...')
                self.pump_stop(suppress_stop=True)
                LOG.info('Pump stopped.')
            # turn all off
            self.valves_control(OFF)
            self.signals = []
            if not self.punch_mode and not self.testing_mode:
                # turn off the motor and cooling water
                self.motor_control(OFF)
                self.water_control(OFF)
            # turn off the machine air supply
            self.air_control(OFF)
            # turn off the red/green/orange LED
            GPIO.error_led.value, GPIO.working_led.value = OFF, OFF
            # release the interface so others can claim it
            self.is_working = False
            self.testing_mode = False
            LOG.info('Machine stopped.')
            # reset the emergency stop state so it has to be cleared in client
            self.emergency_stop = estop

    def pump_start(self):
        """Start the pump."""
        # get the current 0075 wedge position and preserve it
        if not self.pump:
            wedge_0075 = self.status['wedge_0075']
            self.send_signals('NKS0075{}'.format(wedge_0075))

    def pump_stop(self, suppress_stop=False):
        """Stop the pump if it is working.
        This function will send the pump stop combination (NJS 0005) twice
        to make sure that the pump is turned off.
        In case of failure, repeat."""
        if not self.pump:
            return
        # don't change the current 0005 wedge position
        wedge_0005 = self.status['wedge_0005']
        stop_code = 'NJS0005{}'.format(wedge_0005)

        # use longer timeout
        timeout = self.config.get('pump_stop_timeout')

        # store previous LED states; light the red error LED only
        error_led, working_led = GPIO.error_led.value, GPIO.working_led.value
        GPIO.error_led.value, GPIO.working_led.value = ON, OFF

        # suppressing stop means that in case of emergency stop, or when
        # stopping the machine, the routine will go on regardless of any
        # emergency stop button presses, sensor timeouts
        # or client emergency stop calls
        # try as long as necessary
        while self.pump:
            with suppress(librpi2caster.InterfaceNotStarted):
                with self.handle_machine_stop(suppress_stop=suppress_stop):
                    self.send_signals(stop_code, timeout=timeout)
                    self.send_signals(stop_code, timeout=timeout)

        # finished; reset LEDs
        GPIO.error_led.value, GPIO.working_led.value = error_led, working_led

    def emergency_stop_control(self, state):
        """Emergency stop: state=ON to activate, OFF to clear"""
        self.emergency_stop = state

    def machine_control(self, state):
        """Machine and interface control.
        If no state or state is None, return the current working state.
        If state evaluates to True, start the machine.
        If state evaluates to False, stop (and try to stop the pump).
        """
        if state:
            self.start()
        else:
            self.stop()

    def valves_control(self, state):
        """Turn valves on or off, check valve status.
        Accepts signals (turn on), False (turn off) or None (get the status)"""
        if state:
            # got the signals
            self.output.valves_on(self.signals)
            self.update_pump_and_wedges()
            self.status.update(valves=ON)
        else:
            self.output.valves_off()
            self.status.update(valves=OFF)

    def motor_control(self, state):
        """Motor control:
        no state or None = get the motor state,
        anything evaluating to True or False = turn on or off"""
        new_state = bool(state)
        output = GPIO.motor_start if new_state else GPIO.motor_stop
        with suppress(AttributeError):
            output.on()
            time.sleep(0.2)
            output.off()
        self.status.update(motor=new_state)
        self.meter_events.clear()

    @staticmethod
    def air_control(state):
        """Air supply control: master compressed air solenoid valve.
        no state or None = get the air state,
        anything evaluating to True or False = turn on or off"""
        with suppress(AttributeError):
            GPIO.air.value = bool(state)

    @staticmethod
    def water_control(state):
        """Cooling water control:
        no state or None = get the water valve state,
        anything evaluating to True or False = turn on or off"""
        with suppress(AttributeError):
            GPIO.water.value = bool(state)

    def pump_control(self, state):
        """No state: get the pump status.
        Anything evaluating to True or False: start or stop the pump"""
        if state:
            self.pump_start()
        else:
            self.pump_stop()

    def send_signals(self, signals, timeout=None):
        """Send the signals to the caster/perforator.
        This method performs a single-dispatch on current operation mode:
            casting: sensor ON, valves ON, sensor OFF, valves OFF;
            punching: valves ON, wait t1, valves OFF, wait t2
            testing: valves OFF, valves ON

        In the punching mode, if there are less than two signals,
        an additional O+15 signal will be activated. Otherwise the paper ribbon
        advance mechanism won't work."""
        def cast():
            """Monotype composition caster.

            Wait for sensor to go ON, turn on the valves,
            wait for sensor to go OFF, turn off the valves.
            """
            # the interface must be started beforehand if we want to cast
            if not self.is_working:
                raise librpi2caster.InterfaceNotStarted
            # allow the use of a custom timeout
            wait = timeout or self.config['sensor_timeout']
            # machine control cycle
            self.wait_for_sensor(ON, timeout=wait)
            self.valves_control(ON)
            self.wait_for_sensor(OFF, timeout=wait)
            self.valves_control(OFF)

        def test():
            """Turn off any previous combination, then send signals."""
            with suppress(librpi2caster.InterfaceBusy):
                self.start()
            # change the active combination
            self.valves_control(OFF)
            self.valves_control(ON)

        def punch():
            """Timer-driven ribbon perforator."""
            with suppress(librpi2caster.InterfaceBusy):
                self.start()
            # timer-driven operation
            self.valves_control(ON)
            time.sleep(self.config['punching_on_time'])
            self.valves_control(OFF)
            time.sleep(self.config['punching_off_time'])

        self.signals = signals
        rtn = test if self.testing_mode else punch if self.punch_mode else cast
        with self.handle_machine_stop():
            # stop the machine when emergency stop or sensor timeout happens
            # then bubble the exception up
            rtn()


class GPIOCollection:
    """Input/output group with iteration and getting attributes by name"""
    ready_led, error_led, working_led = None, None, None
    motor_start, motor_stop, air, water = None, None, None, None
    sensor, estop_button, mode_detect = None, None, None
    shutdown_button, reboot_button = None, None

    def __init__(self):
        LOG.debug('Initializing general purpose input/outputs (GPIOs)...')
        bouncetime = float(CFG.defaults().get('debounce_milliseconds')) / 1000
        ins = dict(shutdown_button=pin('shutdown', IN, hold_time=2),
                   reboot_button=pin('reboot', IN, hold_time=2),
                   sensor=pin('sensor', IN, pull_up=False,
                              bounce_time=bouncetime),
                   estop_button=pin('emergency_stop', IN, pull_up=False,
                                    bounce_time=0.1),
                   mode_detect=pin('mode_detect', IN))
        outs = dict(working_led=pin('working_led', OUT),
                    error_led=pin('error_led', OUT),
                    ready_led=pin('ready_led', OUT),
                    air=pin('air', OUT), water=pin('water', OUT),
                    motor_start=pin('motor_start', OUT),
                    motor_stop=pin('motor_stop', OUT))
        self.inputs = ins
        self.outputs = outs
        self.__dict__.update(**ins, **outs)
        LOG.debug('GPIO initialization complete.')

    def get_values(self):
        """Get the current state of all GPIOs"""
        state = dict()
        state.update({name: gpio.value for name, gpio in self.inputs.items()})
        state.update({name: gpio.value for name, gpio in self.outputs.items()})
        return state

    def all_off(self):
        """Turn all outputs off"""
        for gpio in self.outputs.values():
            with suppress(AttributeError):
                gpio.off()

    def cleanup(self):
        """Clean up all GPIOs"""
        LOG.info('Turning GPIOs off and releasing them...')
        self.all_off()
        for name, gpio in self.outputs.items():
            with suppress(AttributeError):
                gpio.close()
            self.__dict__.pop(name)
        for name, gpio in self.inputs.items():
            with suppress(AttributeError):
                gpio.close()
            self.__dict__.pop(name)
        self.inputs.clear()
        self.outputs.clear()
        LOG.info('GPIOs off and released.')


GPIO = GPIOCollection()


if __name__ == '__main__':
    main()
