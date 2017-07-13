# -*- coding: utf-8 -*-
"""rpi2casterd: hardware control daemon for the rpi2caster software.

This program runs on a Raspberry Pi or a similar single-board computer
and listens on its address(es) on a specified port using the HTTP protocol.
It communicates with client(s) via a JSON API and controls the machine
using selectable backend libraries for greater configurability.
"""
from collections import deque, OrderedDict
from contextlib import suppress
from functools import partial, wraps
import configparser
import signal
import subprocess
import time
import RPi.GPIO as GPIO

import librpi2caster
from librpi2caster import ON, OFF, HMN, KMN, UNITSHIFT, CASTING, PUNCHING

from rpi2casterd.webapi import INTERFACES, APP

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
                i2c_bus='1', mcp0_address='0x20', mcp1_address='0x21',
                valve1='N,M,L,K,J,I,H,G',
                valve2='F,S,E,D,0075,C,B,A',
                valve3='1,2,3,4,5,6,7,8',
                valve4='9,10,11,12,13,14,0005,O15',
                supported_operation_modes=', '.join((CASTING, PUNCHING)),
                supported_row16_modes=', '.join((HMN, KMN, UNITSHIFT)))
CFG = configparser.ConfigParser(defaults=DEFAULTS)
CFG.read(CONFIGURATION_PATH)

# Initialize the application
GPIO.setmode(GPIO.BCM)
LEDS = dict()


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


def teardown():
    """Unregister the exported GPIOs"""
    # cleanup the registered interfaces
    for interface_id, interface in INTERFACES.items():
        interface.machine_control(OFF)
        INTERFACES[interface_id] = None
    INTERFACES.clear()
    # turn off and cleanup the LEDs
    for led_name, led_gpio in LEDS.items():
        turn_off(led_gpio)
        LEDS[led_name] = None
    LEDS.clear()
    GPIO.cleanup()


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
        return [x for x in raw if x in librpi2caster.OUTPUT_SIGNALS]

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


def parse_configuration(source):
    """Get the interface parameters from a config parser section"""
    config = OrderedDict()
    # caster name
    config['name'] = get('name', source, str)
    # supported operation and row 16 addressing modes
    modes = get('supported_operation_modes', source, 'strings')
    row16_modes = get('supported_row16_modes', source, 'strings')
    config['supported_operation_modes'] = modes
    config['supported_row16_modes'] = row16_modes
    config['default_operation_mode'] = modes[0]

    # determine the output driver
    config['output_driver'] = get('output_driver', source, 'lcstring')

    # get timings
    config['startup_timeout'] = get('startup_timeout', source, float)
    config['sensor_timeout'] = get('sensor_timeout', source, float)
    config['pump_stop_timeout'] = get('pump_stop_timeout', source, float)
    config['punching_on_time'] = get('punching_on_time', source, float)
    config['punching_off_time'] = get('punching_off_time', source, float)

    # interface settings: control GPIOs
    config['sensor_gpio'] = get('sensor_gpio', source, 'inone')
    config['error_led_gpio'] = get('error_led_gpio', source, 'inone')
    config['working_led_gpio'] = get('working_led_gpio', source, 'inone')
    config['emergency_stop_gpio'] = get('emergency_stop_gpio', source, 'inone')
    config['motor_start_gpio'] = get('motor_start_gpio', source, 'inone')
    config['motor_stop_gpio'] = get('motor_stop_gpio', source, 'inone')
    config['water_gpio'] = get('water_gpio', source, 'inone')
    config['air_gpio'] = get('air_gpio', source, 'inone')

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
            if GPIO.event_detected(interface.gpios['emergency_stop']):
                raise librpi2caster.MachineStopped

        try:
            # unfortunately we cannot abort the routine
            check_emergency_stop()
            retval = routine(interface, *args, **kwargs)
            check_emergency_stop()
            return retval
        except (librpi2caster.MachineStopped, KeyboardInterrupt):
            interface.machine_control(OFF)
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

    def setup_gpio(name, direction, pull=None, callbk=None, edge=GPIO.FALLING):
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
                    GPIO.add_event_detect(gpio, edge, callbk, bouncetime=50)
                except RuntimeError:
                    GPIO.add_event_callback(gpio, callbk)
                except TypeError:
                    pass
        return gpio

    # set up the ready LED and shutdown/reboot buttons, if possible
    LEDS['ready'] = setup_gpio('ready_led_gpio', GPIO.OUT)
    shdn = setup_gpio('shutdown_gpio', GPIO.IN, GPIO.PUD_UP, shutdown)
    reset = setup_gpio('reboot_gpio', GPIO.IN, GPIO.PUD_UP, reboot)
    # register callbacks for signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)


def interface_setup():
    """Setup the interfaces"""
    # greedily instantiate the interfaces
    for name, section in CFG.items():
        if name.lower() == 'default':
            # don't treat this as an interface
            continue
        try:
            settings = parse_configuration(section)
        except KeyError as exception:
            raise librpi2caster.ConfigurationError(exception)
        interface = Interface(settings)
        INTERFACES[name.lower().strip()] = interface


def main():
    """Starts the application"""
    try:
        # get the listen address and port
        config = CFG.defaults()
        address, port = get('listen_address', config, 'address')
        # initialize hardware
        daemon_setup()
        interface_setup()
        # all configured - it's ready to work
        ready_led_gpio = LEDS.get('ready')
        turn_on(ready_led_gpio)
        # start the web application
        APP.run(address, port)

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
        teardown()


class InterfaceBase:
    """Basic data structures of an interface"""
    def __init__(self, config_dict):
        self.config = config_dict
        # initialize the interface with empty state
        default_operation_mode = self.config['default_operation_mode']
        # data structure to count photocell ON events for rpm meter
        self.meter_events = deque(maxlen=3)
        # temporary GPIO dict (can be populated in hardware_setup)
        self.gpios = dict(working_led=None)
        # initialize machine state
        self.status = dict(wedge_0005=15, wedge_0075=15, testing_mode=OFF,
                           working=OFF, water=OFF, air=OFF, motor=OFF,
                           pump=OFF, sensor=OFF, signals=[],
                           current_operation_mode=default_operation_mode,
                           current_row16_mode=OFF)

    def __str__(self):
        return self.config['name']

    @property
    def operation_mode(self):
        """Get the current operation mode"""
        return self.status['current_operation_mode']

    @operation_mode.setter
    def operation_mode(self, mode):
        """Set the operation mode to a new value"""
        self.check_if_busy()
        if not mode:
            default_operation_mode = self.config['default_operation_mode']
            self.status['current_operation_mode'] = default_operation_mode
        elif mode in self.config['supported_operation_modes']:
            self.status['current_operation_mode'] = mode
        else:
            raise librpi2caster.UnsupportedMode(mode)

    @property
    def row16_mode(self):
        """Get the current row 16 addressing mode"""
        return self.status['current_row16_mode']

    @row16_mode.setter
    def row16_mode(self, mode):
        """Set the row 16 addressing mode to a new value"""
        # prevent mode change when machine is running
        self.check_if_busy()
        if not mode:
            # allow to turn it off in any case
            self.status['current_row16_mode'] = OFF
            return
        elif mode not in (HMN, KMN, UNITSHIFT):
            return
        if self.is_casting:
            # allow only supported row 16 addressing modes
            if mode in self.config['supported_row16_modes']:
                self.status['current_row16_mode'] = mode
            else:
                raise librpi2caster.UnsupportedRow16Mode(mode)
        else:
            self.status['current_row16_mode'] = mode

    @property
    def testing_mode(self):
        """Temporary testing mode"""
        return self.status['testing_mode']

    @testing_mode.setter
    def testing_mode(self, state):
        """Set the testing mode on the interface"""
        self.check_if_busy()
        self.status['testing_mode'] = True if state else False

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
    def is_casting(self):
        """Check if interface is in casting mode"""
        return self.operation_mode == CASTING

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

    @handle_machine_stop
    def wait_for_sensor(self, new_state, timeout=None):
        """Wait until the machine cycle sensor changes its state
        to the desired value (True or False).
        If no state change is registered in the given time,
        raise MachineStopped."""
        start_time = time.time()
        timeout = timeout or self.config['sensor_timeout']
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

    def check_if_busy(self):
        """Check if the interface is already working. If so,
        raise InterfaceBusy."""
        if self.is_working:
            message = 'Cannot do that - the machine is already working.'
            raise librpi2caster.InterfaceBusy(message)

    def check_rotation(self, revolutions=3):
        """Check whether the machine is turning.
        The machine must typically go 3 revolutions of the main shaft."""
        timeout = self.config['startup_timeout']
        for _ in range(revolutions, 0, -1):
            self.wait_for_sensor(ON, timeout=timeout)
            self.wait_for_sensor(OFF, timeout=timeout)

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
    output = None
    gpios = None
    gpio_definitions = dict(sensor=GPIO.IN, emergency_stop=GPIO.IN,
                            error_led=GPIO.OUT, working_led=GPIO.OUT,
                            air=GPIO.OUT, water=GPIO.OUT,
                            motor_stop=GPIO.OUT, motor_start=GPIO.OUT)

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
            if self.sensor_state:
                self.meter_events.append(time.time())

        # set up the controls
        self.gpios = dict()
        for gpio_name, direction in self.gpio_definitions.items():
            gpio_config_name = '{}_gpio'.format(gpio_name)
            gpio_number = config[gpio_config_name]
            self.gpios[gpio_name] = gpio_number
            # skip 0 or None
            if gpio_number:
                GPIO.setup(gpio_number, direction)

        with suppress(TypeError, RuntimeError):
            # register an event detection on emergency stop event
            GPIO.add_event_detect(self.gpios['emergency_stop'], GPIO.FALLING,
                                  bouncetime=config['debounce_milliseconds'])
        try:
            # register a callback to update the RPM meter
            GPIO.add_event_detect(self.gpios['sensor'], GPIO.BOTH,
                                  callback=update_sensor,
                                  bouncetime=config['debounce_milliseconds'])
        except RuntimeError:
            # event already registered
            GPIO.add_event_callback(self.gpios['sensor'], update_sensor)
        except TypeError:
            # sensor is not necessary for e.g. perforator interfaces
            pass

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

    def machine_control(self, state=None):
        """Machine and interface control.
        If no state or state is None, return the current working state.
        If state evaluates to True, start the machine.
        If state evaluates to False, stop (and try to stop the pump).
        """
        def start():
            """Start the machine.
            Casting requires that the machine is running before proceeding."""
            self.check_if_busy()
            # reset the RPM counter
            self.meter_events.clear()
            # turn on the compressed air
            with suppress(NotImplementedError):
                self.air_control(ON)
            # make sure the machine is turning before proceeding
            if self.is_casting and not self.testing_mode:
                # turn on the cooling water and motor
                with suppress(NotImplementedError):
                    self.water_control(ON)
                with suppress(NotImplementedError):
                    self.motor_control(ON)
                self.check_rotation()
            # properly initialized => mark it as working
            self.is_working = True

        def stop():
            """Stop the machine."""
            if self.is_working:
                self.pump_control(OFF)
                self.valves_control(OFF)
                self.signals = []
                if self.is_casting and not self.testing_mode:
                    with suppress(NotImplementedError):
                        self.motor_control(OFF)
                    with suppress(NotImplementedError):
                        self.water_control(OFF)
                with suppress(NotImplementedError):
                    self.air_control(OFF)
                # release the interface so others can claim it
                self.is_working = False
            self.testing_mode = False

        if state is None:
            pass
        elif state:
            start()
        else:
            stop()
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
            parse = librpi2caster.parse_signals
            codes = parse(state, self.operation_mode,
                          self.row16_mode, self.testing_mode)
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
            return self.status['motor']
        elif state:
            start_gpio = self.gpios['motor_start']
            turn_on(start_gpio, raise_exception=True)
            time.sleep(0.5)
            turn_off(start_gpio)
            self.status['motor'] = ON
            return ON
        else:
            stop_gpio = self.gpios['motor_stop']
            turn_on(stop_gpio, raise_exception=True)
            time.sleep(0.5)
            turn_off(stop_gpio)
            self.status['motor'] = OFF
            self.meter_events.clear()
            return OFF

    def air_control(self, state=None):
        """Air supply control: master compressed air solenoid valve.
            no state or None = get the air state,
            anything evaluating to True or False = turn on or off"""
        if state is None:
            return self.status['air']
        elif state:
            turn_on(self.gpios['air'], raise_exception=True)
            self.status['air'] = ON
            return ON
        else:
            turn_off(self.gpios['air'], raise_exception=True)
            self.status['air'] = OFF
            return OFF

    def water_control(self, state=None):
        """Cooling water control:
            no state or None = get the water valve state,
            anything evaluating to True or False = turn on or off"""
        if state is None:
            return self.status['water']
        elif state:
            turn_on(self.gpios['water'], raise_exception=True)
            self.status['water'] = ON
            return ON
        else:
            turn_off(self.gpios['water'], raise_exception=True)
            self.status['water'] = OFF
            return OFF

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

    def send_signals(self, signals, repetitions=None, timeout=None):
        """Send the signals to the caster/perforator.
        This method performs a single-dispatch on current operation mode:
            casting: sensor ON, valves ON, sensor OFF, valves OFF;
            punching: valves ON, wait t1, valves OFF, wait t2
            testing: valves OFF, valves ON

        In the punching mode, if there are less than two signals,
        an additional O+15 signal will be activated. Otherwise the paper ribbon
        advance mechanism won't work."""
        if not signals:
            # this tells the interface to turn off all the valves
            self.valves_control(OFF)
            return
        cast = partial(self.cast, timeout=timeout)
        send_routine = (self.test if self.testing_mode
                        else cast if self.is_casting else self.punch)
        for _ in range(repetitions or 1):
            send_routine(signals)

    def cast(self, codes, timeout=None):
        """Monotype composition caster.

        Wait for sensor to go ON, turn on the valves,
        wait for sensor to go OFF, turn off the valves.
        """
        if not self.is_working:
            raise librpi2caster.InterfaceNotStarted

        # allow the use of a custom timeout
        timeout = timeout or self.config['sensor_timeout']
        # machine control cycle
        self.wait_for_sensor(ON, timeout=timeout)
        self.valves_control(codes)
        self.wait_for_sensor(OFF, timeout=timeout)
        self.valves_control(OFF)

    def test(self, codes):
        """Turn off any previous combination, then send signals.
        """
        if not self.is_working:
            self.machine_control(ON)

        # change the active combination
        self.valves_control(OFF)
        self.valves_control(codes)

    def punch(self, codes):
        """Timer-driven ribbon perforator.

        Turn on the valves, wait the "punching_on_time",
        then turn off the valves and wait for them to go down
        ("punching_off_time").
        """
        if not self.is_working:
            self.machine_control(ON)

        # timer-driven operation
        self.valves_control(codes)
        time.sleep(self.config['punching_on_time'])
        self.valves_control(OFF)
        time.sleep(self.config['punching_off_time'])

if __name__ == '__main__':
    main()
