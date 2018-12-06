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

# Initialize the application
GPIO.setmode(GPIO.BCM)
LEDS = dict()


def setup_gpio(name, direction, pull=None, callbk=None,
               edge=GPIO.FALLING, bouncetime=50):
    """Set up a GPIO input/output"""
    gpio_string = CFG.defaults().get(name).strip()
    with suppress(TypeError, ValueError):
        gpio = int(gpio_string)
        if not gpio:
            return None
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
    if gpio:
        GPIO.output(gpio, ON)
    elif raise_exception:
        raise NotImplementedError


def turn_off(gpio, raise_exception=False):
    """Turn off a specified GPIO output"""
    if gpio:
        GPIO.output(gpio, OFF)
    elif raise_exception:
        raise NotImplementedError


def get_state(gpio):
    """Get the state of a GPIO input or output"""
    return GPIO.input(gpio)


def toggle(gpio):
    """Change the state of a GPIO output"""
    current_state = GPIO.input(gpio)
    GPIO.output(gpio, not current_state)


def ready_led_blink():
    """Blinks the LED"""
    led_gpio = LEDS.get('ready')
    if not led_gpio:
        return
    for _ in range(6):
        toggle(led_gpio)
        time.sleep(0.3)


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

    config = OrderedDict()

    # get timings
    config['name'] = get('name', str)
    config['startup_timeout'] = get('startup_timeout', float)
    config['sensor_timeout'] = get('sensor_timeout', float)
    config['pump_stop_timeout'] = get('pump_stop_timeout', float)
    config['punching_on_time'] = get('punching_on_time', float)
    config['punching_off_time'] = get('punching_off_time', float)

    # time (in milliseconds) for software debouncing
    config['debounce_milliseconds'] = get('debounce_milliseconds', int)

    # determine the output driver and settings
    config['output_driver'] = get('output_driver').lower()
    config['i2c_bus'] = get('i2c_bus', integer)
    config['mcp0_address'] = get('mcp0_address', integer)
    config['mcp1_address'] = get('mcp1_address', integer)
    valves = dict(valve1=get('valve1', signals), valve2=get('valve2', signals),
                  valve3=get('valve3', signals), valve4=get('valve4', signals))
    config['signal_mappings'] = valves
    return config


def handle_machine_stop(routine):
    """Ensure that when MachineStopped occurs, the interface will run
    its stop() method."""
    @wraps(routine)
    def wrapper(interface, *args, **kwargs):
        """wraps the routine"""
        interface.emergency_stop_control(interface.emergency_stop)
        try:
            return routine(interface, *args, **kwargs)
        finally:
            interface.emergency_stop_control(interface.emergency_stop)
    return wrapper


def daemon_setup():
    """Configure the "ready" LED and shutdown/reboot buttons"""
    def shutdown(*_):
        """Shut the system down"""
        command = CFG.defaults().get('shutdown_command')
        print('Shutdown button pressed. Hold down for 2s to shut down...')
        time.sleep(2)
        # the button is between GPIO and GND i.e. pulled up - negative logic
        if not get_state(shdn):
            print('Shutting down...')
            ready_led_blink()
            subprocess.run([x.strip() for x in command.split(' ')])

    def reboot(*_):
        """Restart the system"""
        command = CFG.defaults().get('reboot_command')
        print('Reboot button pressed. Hold down for 2s to reboot...')
        time.sleep(2)
        # the button is between GPIO and GND i.e. pulled up - negative logic
        if not get_state(reset):
            print('Rebooting...')
            ready_led_blink()
            subprocess.run([x.strip() for x in command.split(' ')])

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


def main():
    """Starts the application. Contains web API subroutines."""
    interface = None
    config = CFG.defaults()
    listen_address = config.get('listen_address')
    try:
        # initialize hardware
        daemon_setup()
        # interface configuration
        settings = parse_configuration(config)
        interface = Interface(settings)
        # all configured - it's ready to work
        ready_led_gpio = LEDS.get('ready')
        turn_on(ready_led_gpio)
        # start the web API for communicating with client
        interface.webapi(listen_address)

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
        self.output = None
        # data structure to count photocell ON events for rpm meter
        self.meter_events = deque(maxlen=3)
        # temporary GPIO dict (can be populated in hardware_setup)
        self.gpios = dict(working_led=None, error_led=None)
        # initialize machine state
        self.status = dict(wedge_0005=15, wedge_0075=15, valves=OFF,
                           machine=OFF, water=OFF, air=OFF, motor=OFF,
                           pump=OFF, sensor=OFF, signals=[],
                           emergency_stop=OFF, testing_mode=OFF,
                           working_led=OFF, error_led=OFF)
        self.hardware_setup()

    def __str__(self):
        return self.config['name']

    @property
    def is_working(self):
        """Get the machine working status"""
        return self.status['machine']

    @is_working.setter
    def is_working(self, state):
        """Set the machine working state"""
        self.update_status(machine=bool(state))

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
        self.update_status(testing_mode=bool(state))

    @property
    def signals(self):
        """Get the current signals."""
        return self.status['signals']

    @signals.setter
    def signals(self, source):
        """Set the current signals."""
        print('Signals received: {}'.format(''.join([s for s in source])))
        codes = parse_signals(source)
        # do some changes based on mode
        if self.punch_mode:
            signals = (codes if len(codes) >= 2
                       else codes if 'O15' in codes else [*codes, 'O15'])
        elif self.testing_mode:
            signals = codes
        else:
            signals = [s for s in codes if s != 'O15']
        print('Sending signals: {}'.format(' '.join(signals)))
        self.update_status(signals=signals)

    @property
    def pump(self):
        """Check the pump state"""
        return self.status.get('pump')

    @pump.setter
    def pump(self, state):
        """Set the pump state"""
        self.update_status(pump=bool(state))

    @property
    def sensor_state(self):
        """Get the sensor state"""
        return self.status['sensor']

    @sensor_state.setter
    def sensor_state(self, state):
        """Update the sensor state"""
        self.update_status(sensor=bool(state))

    @property
    def emergency_stop(self):
        """Get the emergency stop state"""
        return self.status.get('emergency_stop')

    @emergency_stop.setter
    def emergency_stop(self, state):
        """Set the emergency stop state"""
        self.update_status(emergency_stop=bool(state))

    @staticmethod
    def hardware_setup():
        """Nothing to do."""
        pass

    @staticmethod
    def stop():
        """Nothing to do."""
        pass

    def wait_for_sensor(self, new_state, timeout=None):
        """Wait until the machine cycle sensor changes its state
        to the desired value (True or False).
        If no state change is registered in the given time,
        raise MachineStopped."""
        print('Waiting for sensor state {}'.format(new_state))
        start_time = time.time()
        timeout = timeout if timeout else self.config['sensor_timeout']
        while self.sensor_state != new_state:
            timed_out = time.time() - start_time > timeout
            if self.emergency_stop or timed_out:
                raise librpi2caster.MachineStopped
            # wait 5ms to ease the load on the CPU
            time.sleep(0.005)

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

    def update_status(self, **kwargs):
        """Updates the machine status"""
        self.status.update(**kwargs)

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

        self.update_status(pump=bool(pump_working),
                           wedge_0075=pos_0075, wedge_0005=pos_0005)


class Interface(InterfaceBase):
    """Hardware control interface"""
    def webapi(self, listen_address):
        """JSON web API for communicating with the casting software."""
        def get_address_and_port():
            """Get an IP or DNS address and a port"""
            try:
                address, _port = listen_address.split(':')
                port = int(_port)
            except ValueError:
                address, port = listen_address, 23017
            return address, port

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
                        librpi2caster.InterfaceBusy) as exc:
                    # HTTP response with an error code
                    response.update(success=False, error_code=exc.code,
                                    error_name=exc.message)
                except librpi2caster.MachineStopped as exc:
                    self.stop()
                    response.update(success=False, error_code=exc.code,
                                    error_name=exc.message)
                return jsonify(response)

            return wrapper

        @handle_request
        def index():
            """Get the read-only information about the interface.
            Return the JSON-encoded dictionary with
                status: current interface state,
                settings: static configuration (in /etc/rpi2casterd.conf)
            """
            if request.method in (POST, PUT):
                request_data = request.get_json() or {}
                self.update_status(**request_data)
            status = self.status
            status.update(speed='{}rpm'.format(self.rpm()))
            return status

        @handle_request
        def config():
            """Return or change the interface configuration"""
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
                routine(device_state)
            elif request.method == PUT:
                routine(ON)
            elif request.method == DELETE:
                routine(OFF)
            # always return the current state of the controlled device
            return dict(active=self.status.get(device))

        app = Flask('rpi2casterd')
        all_methods = ('GET', 'PUT', 'POST', 'DELETE')
        app.route('/', methods=all_methods)(index)
        app.route('/config', methods=all_methods)(config)
        app.route('/signals', methods=all_methods)(signals)
        app.route('/<device>', methods=all_methods)(control)

        address, port = get_address_and_port()
        app.run(address, port)

    def hardware_setup(self):
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
            state = get_state(emergency_stop_gpio)
            if state:
                print('Emergency stop button pressed!')
                self.emergency_stop = ON

        bouncetime = self.config['debounce_milliseconds']
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
        self.config['punch_mode'] = bool(get_state(gpios['mode_detect']))

        # output setup:
        try:
            output_name = self.config['output_driver']
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
        self.gpios = gpios
        LEDS.update(error=gpios['error_led'], working=gpios['working_led'])

    @handle_machine_stop
    def start(self):
        """Starts the machine. When casting, check if it's running."""
        # check if the interface is already busy
        if self.is_working:
            message = 'Cannot do that - the machine is already working.'
            raise librpi2caster.InterfaceBusy(message)
        self.is_working = True
        self.working_led = ON
        self.error_led = ON
        # reset the RPM counter
        self.meter_events.clear()
        # turn on the compressed air
        self.air_control(ON)
        if self.punch_mode or self.testing_mode:
            # automatically reset the emergency stop if it was engaged
            self.emergency_stop = OFF
        else:
            # turn on the cooling water and motor, check the machine rotation
            # if MachineStopped is raised, it'll bubble up from here
            self.water_control(ON)
            self.motor_control(ON)
            # check machine rotation
            timeout = self.config['startup_timeout']
            for _ in range(3):
                self.wait_for_sensor(ON, timeout=timeout)
                self.wait_for_sensor(OFF, timeout=timeout)
        # properly initialized => mark it as working
        self.error_led = OFF

    def stop(self):
        """Stop the machine, making sure that the pump is disengaged."""
        if self.is_working:
            # orange LED for stopping sequence
            self.working_led = ON
            self.error_led = ON
            # store the emergency stop state;
            # temporarily ooverride for pump stop
            estop, self.emergency_stop = self.emergency_stop, OFF
            while self.pump:
                # force turning the pump off
                with suppress(librpi2caster.MachineStopped):
                    print('Stopping the pump...')
                    self.pump_control(OFF)
                    print('Pump stopped.')
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
            self.error_led = OFF
            self.working_led = OFF
            # release the interface so others can claim it
            self.is_working = False
            self.testing_mode = False
            # reset the emergency stop state so it has to be cleared in client
            self.emergency_stop = estop

    @property
    def error_led(self):
        """Get the current error LED state"""
        return self.status.get('error_led')

    @error_led.setter
    def error_led(self, state):
        """Turn the error LED on or off"""
        if state:
            turn_on(self.gpios['error_led'])
        else:
            turn_off(self.gpios['error_led'])
        self.update_status(error_led=bool(state))

    @property
    def working_led(self):
        """Get the current working LED state"""
        return self.status.get('working_led')

    @working_led.setter
    def working_led(self, state):
        """Turn the error LED on or off"""
        if state:
            turn_on(self.gpios['working_led'])
        else:
            turn_off(self.gpios['working_led'])
        self.update_status(working_led=bool(state))

    def emergency_stop_control(self, state):
        """Emergency stop: state=ON to activate, OFF to clear"""
        self.emergency_stop = state
        if state:
            self.stop()
            raise librpi2caster.MachineStopped

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
            self.update_status(valves=ON)
        else:
            self.output.valves_off()
            self.update_status(valves=OFF)

    def motor_control(self, state):
        """Motor control:
            no state or None = get the motor state,
            anything evaluating to True or False = turn on or off"""
        if state:
            start_gpio = self.gpios['motor_start']
            if start_gpio:
                turn_on(start_gpio)
                time.sleep(0.2)
                turn_off(start_gpio)
            self.update_status(motor=ON)
        else:
            stop_gpio = self.gpios['motor_stop']
            if stop_gpio:
                turn_on(stop_gpio)
                time.sleep(0.2)
                turn_off(stop_gpio)
            self.update_status(motor=OFF)
            self.meter_events.clear()

    def air_control(self, state):
        """Air supply control: master compressed air solenoid valve.
            no state or None = get the air state,
            anything evaluating to True or False = turn on or off"""
        if state:
            turn_on(self.gpios['air'])
            self.update_status(air=ON)
        else:
            turn_off(self.gpios['air'])
            self.update_status(air=OFF)

    def water_control(self, state):
        """Cooling water control:
            no state or None = get the water valve state,
            anything evaluating to True or False = turn on or off"""
        if state:
            turn_on(self.gpios['water'])
            self.update_status(water=ON)
        else:
            turn_off(self.gpios['water'])
            self.update_status(water=OFF)

    def pump_control(self, state):
        """No state: get the pump status.
        Anything evaluating to True or False: start or stop the pump"""
        def start():
            """Start the pump."""
            # get the current 0075 wedge position and preserve it
            if not self.pump:
                wedge_0075 = self.status['wedge_0075']
                self.send_signals('NKS0075{}'.format(wedge_0075))

        def stop():
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
            timeout = self.config['pump_stop_timeout']

            # store previous LED states; light the red error LED only
            self.error_led, error_led = ON, self.error_led
            self.working_led, working_led = OFF, self.working_led

            # try as long as necessary
            with suppress(librpi2caster.InterfaceNotStarted):
                while self.status['pump']:
                    self.send_signals(stop_code, timeout=timeout)
                    self.send_signals(stop_code, timeout=timeout)

            # finished; reset LEDs
            self.error_led, self.working_led = error_led, working_led
        if state:
            start()
        else:
            stop()

    @handle_machine_stop
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
        rtn()


if __name__ == '__main__':
    main()
