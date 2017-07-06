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
import RPi.GPIO as GPIO

from rpi2casterd import exceptions as exc
from rpi2casterd.webapi import INTERFACES, APP

# signals to send to the valves
OUTPUT_SIGNALS = tuple(['0075', 'S', '0005', *'ABCDEFGHIJKLMN',
                        *(str(x) for x in range(1, 15)), 'O15'])

# Where to look for config?
CONFIGURATION_PATH = '/etc/rpi2casterd.conf'
DEFAULTS = dict(listen_address='0.0.0.0:23017', output_driver='smbus',
                shutdown_gpio='24', shutdown_command='shutdown -h now',
                reboot_gpio='23', reboot_command='shutdown -r now',
                startup_timeout='30', sensor_timeout='5',
                pump_stop_timeout='120',
                punching_on_time='0.2', punching_off_time='0.3',
                debounce_milliseconds='25',
                ready_led_gpio='18', sensor_gpio='17',
                working_led_gpio='25', error_led_gpio='26',
                air_gpio='19', water_gpio='13', emergency_stop_gpio='22',
                motor_start_gpio='5', motor_stop_gpio='6',
                i2c_bus='1', mcp0_address='0x20', mcp1_address='0x21',
                valve1='N,M,L,K,J,I,H,G',
                valve2='F,S,E,D,0075,C,B,A',
                valve3='1,2,3,4,5,6,7,8',
                valve4='9,10,11,12,13,14,0005,O15',
                supported_operation_modes='casting, punching',
                supported_row16_modes='HMN, KMN, unit shift')
CFG = configparser.ConfigParser(defaults=DEFAULTS)
CFG.read(CONFIGURATION_PATH)

# Status for readability
ON, OFF = True, False

# Initialize the application
GPIO.setmode(GPIO.BCM)
LEDS = dict()


def turn_on(gpio):
    """Turn on a specified GPIO output"""
    GPIO.output(gpio, ON)


def turn_off(gpio):
    """Turn off a specified GPIO output"""
    GPIO.output(gpio, OFF)


def get_state(gpio):
    """Get the state of a GPIO input or output"""
    return GPIO.input(gpio)


def blink(gpio=None, seconds=0.5, times=3):
    """Blinks the LED"""
    led_gpio = LEDS.get(gpio)
    if not led_gpio:
        return
    for _ in range(times):
        turn_off(led_gpio)
        time.sleep(seconds)
        turn_on(led_gpio)
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
    return convert(source[parameter])


def parse_configuration(source):
    """Get the interface parameters from a config parser section"""
    def signals(input_string):
        """Convert 'a,b,c,d,e' -> ['A', 'B', 'C', 'D', 'E'].
        Allow only known defined signals."""
        raw = [x.strip().upper() for x in input_string.split(',')]
        return [x for x in raw if x in OUTPUT_SIGNALS]

    def strings(input_string):
        """Convert 'abc , def, 012' -> ['abc', 'def', '012']
        (no case change; strip whitespace)."""
        return [x.strip() for x in input_string.split(',')]

    def lcstring(input_string):
        """Return a lowercase string stripped of all whitespace"""
        return input_string.strip().lower()

    def anyint(input_string):
        """Convert a decimal, octal, binary or hexadecimal string to integer"""
        return int(lcstring(input_string), 0)

    config = OrderedDict()
    # supported operation and row 16 addressing modes
    modes = get('supported_operation_modes', source, strings)
    row16_modes = get('supported_row16_modes', source, strings)
    config['supported_operation_modes'] = modes
    config['supported_row16_modes'] = row16_modes
    config['default_operation_mode'] = modes[0]
    config['default_row16_mode'] = None

    # determine the output driver
    config['output_driver'] = get('output_driver', source, lcstring)

    # get timings
    config['startup_timeout'] = get('startup_timeout', source, float)
    config['sensor_timeout'] = get('sensor_timeout', source, float)
    config['pump_stop_timeout'] = get('pump_stop_timeout', source, float)
    config['punching_on_time'] = get('punching_on_time', source, float)
    config['punching_off_time'] = get('punching_off_time', source, float)

    # interface settings: control GPIOs
    config['sensor_gpio'] = get('sensor_gpio', source, int)
    config['error_led_gpio'] = get('error_led_gpio', source, int)
    config['working_led_gpio'] = get('working_led_gpio', source, int)
    config['emergency_stop_gpio'] = get('emergency_stop_gpio', source, int)
    config['motor_start_gpio'] = get('motor_start_gpio', source, int)
    config['motor_stop_gpio'] = get('motor_stop_gpio', source, int)
    config['water_gpio'] = get('water_gpio', source, int)
    config['air_gpio'] = get('air_gpio', source, int)

    # time (in milliseconds) for software debouncing
    config['debounce_milliseconds'] = get('debounce_milliseconds',
                                          source, int)

    # interface settings: output
    config['i2c_bus'] = get('i2c_bus', source, anyint)
    config['mcp0_address'] = get('mcp0_address', source, anyint)
    config['mcp1_address'] = get('mcp1_address', source, anyint)
    config['signal_mappings'] = dict(valve1=get('valve1', source, signals),
                                     valve2=get('valve2', source, signals),
                                     valve3=get('valve3', source, signals),
                                     valve4=get('valve4', source, signals))

    # configuration ready to ship
    return config


def ordered_signals(source):
    """Returns a list of arranged signals ready for display"""
    arranged = deque(s for s in OUTPUT_SIGNALS if s in source)
    # put NI, NL, NK, NJ, NKJ etc. at the front
    if 'N' in arranged:
        for other in 'JKLI':
            if other in source:
                arranged.remove('N')
                arranged.remove(other)
                arranged.appendleft(other)
                arranged.appendleft('N')
    return list(arranged)


def handle_machine_stop(routine):
    """Ensure that when MachineStopped occurs, the interface will run
    its stop() method."""
    @wraps(routine)
    def wrapper(interface, *args, **kwargs):
        """wraps the routine"""
        def check_emergency_stop():
            """check if the emergency stop button registered any events"""
            if GPIO.event_detected(interface.gpios['emergency_stop']):
                raise exc.MachineStopped

        try:
            # unfortunately we cannot abort the routine
            check_emergency_stop()
            retval = routine(interface, *args, **kwargs)
            check_emergency_stop()
            return retval
        except (exc.MachineStopped, KeyboardInterrupt):
            interface.machine_control(OFF)
            raise exc.MachineStopped
    return wrapper


def daemon_setup():
    """Configure the "ready" LED and shutdown/reboot buttons"""
    def command(input_string):
        """Operating system command: string -> accepted by subprocess.run"""
        chunks = input_string.split(' ')
        return [x.strip() for x in chunks]

    def shutdown(*_):
        """Shut the system down"""
        print('Shutdown button pressed. Hold down for 2s to shut down...')
        time.sleep(2)
        # the button is between GPIO and GND i.e. pulled up - negative logic
        if not get_state(shutdown_gpio):
            print('Shutting down...')
            blink('ready')
            cmd = config.get('shutdown_command')
            subprocess.run(command(cmd))

    def reboot(*_):
        """Restart the system"""
        print('Reboot button pressed. Hold down for 2s to reboot...')
        time.sleep(2)
        # the button is between GPIO and GND i.e. pulled up - negative logic
        if not get_state(reboot_gpio):
            print('Rebooting...')
            blink('ready')
            cmd = config.get('reboot_command')
            subprocess.run(command(cmd))

    def signal_handler(*_):
        """Exit gracefully if SIGINT or SIGTERM received"""
        raise KeyboardInterrupt

    config = CFG.defaults()
    # set the LED up
    ready_led_gpio = get('ready_led_gpio', config, int)
    GPIO.setup(ready_led_gpio, GPIO.OUT)
    LEDS['ready'] = ready_led_gpio

    # set the buttons up
    shutdown_gpio = get('shutdown_gpio', config, int)
    reboot_gpio = get('reboot_gpio', config, int)
    GPIO.setup(shutdown_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(reboot_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # register callbacks for shutdown and reboot
    # if some callback was already registered, hook up another function
    try:
        GPIO.add_event_detect(shutdown_gpio, GPIO.FALLING,
                              callback=shutdown, bouncetime=50)
    except RuntimeError:
        GPIO.add_event_callback(shutdown_gpio, shutdown)
    try:
        GPIO.add_event_detect(reboot_gpio, GPIO.FALLING,
                              callback=reboot, bouncetime=50)
    except RuntimeError:
        GPIO.add_event_callback(reboot_gpio, reboot)

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
            raise exc.ConfigurationError(exception)
        interface = Interface(settings)
        INTERFACES[name.lower().strip()] = interface


def main():
    """Starts the application"""
    def address_and_port(input_string):
        """Get an IP or DNS address and a port"""
        try:
            address, _port = input_string.split(':')
            port = int(_port)
        except ValueError:
            address = input_string
            port = 23017
        return address, port

    try:
        # get the listen address and port
        config = CFG.defaults()
        address, port = get('listen_address', config, address_and_port)
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
        default_row16_mode = self.config['default_row16_mode']
        self.status = dict(wedge_0005=15, wedge_0075=15, testing_mode=False,
                           working=False, water=False, air=False,
                           motor=False, pump=False, sensor=False,
                           current_operation_mode=default_operation_mode,
                           current_row16_mode=default_row16_mode)

    @property
    def operation_mode(self):
        """Get the current operation mode"""
        default_mode = self.config['default_operation_mode']
        return self.status.get('current_operation_mode', default_mode)

    @operation_mode.setter
    def operation_mode(self, mode):
        """Set the operation mode to a new value"""
        if not mode:
            return
        elif self.status['working']:
            raise exc.InterfaceBusy('Machine is working - cannot change mode')
        elif mode == 'reset':
            default_operation_mode = self.config['default_operation_mode']
            self.status['current_operation_mode'] = default_operation_mode
        elif mode in self.config['supported_operation_modes']:
            self.status['current_operation_mode'] = mode
        else:
            raise exc.UnsupportedMode(mode)

    @property
    def row16_mode(self):
        """Get the current row 16 addressing mode"""
        return self.status['current_row16_mode']

    @row16_mode.setter
    def row16_mode(self, mode):
        """Set the row 16 addressing mode to a new value"""
        if mode not in (None, 'reset', 'HMN', 'KMN', 'unit shift'):
            return
        if self.status['working']:
            raise exc.InterfaceBusy('Machine is working - cannot change mode')
        if mode == 'reset':
            default_row16_mode = self.config['default_row16_mode']
            self.status['current_row16_mode'] = default_row16_mode
        elif mode is None:
            # allow to turn it off in any case
            self.status['current_row16_mode'] = mode
        elif self.operation_mode == 'casting':
            # allow only supported row 16 addressing modes
            if mode in self.config['supported_row16_modes']:
                self.status['current_row16_mode'] = mode
            else:
                raise exc.UnsupportedRow16Mode(mode)
        elif mode in ('HMN', 'KMN', 'unit shift'):
            # operation mode is testing (None) or punching
            self.status['current_row16_mode'] = mode

    @property
    def testing_mode(self):
        """Temporary testing mode"""
        return self.status['testing_mode']

    @testing_mode.setter
    def testing_mode(self, state):
        """Set the testing mode on the interface"""
        if self.status['working']:
            raise exc.InterfaceBusy('Machine is working - cannot change mode')
        self.status['testing_mode'] = True if state else False


class Interface(InterfaceBase):
    """Hardware control interface"""
    name = 'Raspberry Pi interface'
    gpio_definitions = dict(sensor=GPIO.IN, emergency_stop=GPIO.IN,
                            error_led=GPIO.OUT, working_led=GPIO.OUT,
                            air=GPIO.OUT, water=GPIO.OUT,
                            motor_stop=GPIO.OUT, motor_start=GPIO.OUT)

    def __init__(self, config_dict):
        super().__init__(config_dict)
        # GPIO definitions (after setup, these will be actual GPIO numbers)
        self.gpios = dict()
        # store the current signals
        self.signals = []
        # output driver (will be initialized in hardware_setup)
        self.output = None
        # data structure to count photocell ON events for rpm meter
        self.meter_events = deque(maxlen=3)
        # configure the hardware
        self.hardware_setup(self.config)

    def __str__(self):
        return self.name

    def hardware_setup(self, config):
        """Configure the inputs and outputs.
        Raise ConfigurationError if output name is not recognized,
        or modules supporting the hardware backends cannot be imported."""
        def update_sensor(sensor_gpio):
            """Update the RPM event counter"""
            sensor_state = get_state(sensor_gpio)
            self.status['sensor'] = bool(sensor_state)
            if sensor_state:
                self.meter_events.append(time.time())

        # set up the controls
        for gpio_name, direction in self.gpio_definitions.items():
            gpio_config_name = '{}_gpio'.format(gpio_name)
            gpio_number = config[gpio_config_name]
            # configure the GPIO
            GPIO.setup(gpio_number, direction)
            self.gpios[gpio_name] = gpio_number

        with suppress(RuntimeError):
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
            raise exc.ConfigurationError('Unknown output: {}.'
                                         .format(output_name))
        except ImportError:
            raise exc.ConfigurationError('Module not installed for {}'
                                         .format(output_name))

    @handle_machine_stop
    def wait_for_sensor(self, new_state, timeout=None):
        """Wait until the machine cycle sensor changes its state
        to the desired value (True or False).
        If no state change is registered in the given time,
        raise MachineStopped."""
        start_time = time.time()
        timeout = timeout or self.config['sensor_timeout']
        while self.status['sensor'] != new_state:
            if time.time() - start_time > timeout:
                raise exc.MachineStopped
            # wait 10ms to ease the load on the CPU
            time.sleep(0.01)

    def machine_control(self, state=None):
        """Machine and interface control.
        If no state or state is None, return the current working state.
        If state evaluates to True, start the machine.
        If state evaluates to False, stop (and try to stop the pump).
        """
        def start():
            """Start the machine.
            Casting requires that the machine is running before proceeding."""
            # don't let anyone else initialize an interface already initialized
            if self.status['working']:
                raise exc.InterfaceBusy

            # reset the RPM counter
            self.meter_events.clear()
            # turn on the compressed air
            self.air_control(ON)
            # make sure the machine is turning before proceeding
            if self.operation_mode == 'casting' and not self.testing_mode:
                # turn on the cooling water and motor
                self.water_control(ON)
                self.motor_control(ON)
                self.check_rotation()
            # properly initialized => mark it as working
            turn_on(self.gpios['working_led'])
            self.status['working'] = True

        def stop():
            """Stop the machine."""
            if self.status['working']:
                self.pump_control(OFF)
                self.valves_control(OFF)
                self.signals = []
                if self.operation_mode == 'casting' and not self.testing_mode:
                    self.motor_control(OFF)
                    self.water_control(OFF)
                self.air_control(OFF)
                turn_off(self.gpios['working_led'])
                # release the interface so others can claim it
                self.status['working'] = False
            self.testing_mode = False

        if state is None:
            pass
        elif state:
            start()
        else:
            stop()
        return self.status['working']

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

    @property
    def current_status(self):
        """Get the most current status."""
        status = dict()
        status.update(self.status)
        status.update(speed='{}rpm'.format(self.rpm()))
        status.update(signals=self.signals)
        return status

    def check_pump(self):
        """Check if the pump is working or not"""
        def found(code):
            """check if code was found in a combination"""
            return set(code).issubset(self.signals)

        # cache this to avoid double dictionary lookup for each check
        if found(['0075']) or found('NK'):
            return True
        elif found(['0005']) or found('NJ'):
            return False
        else:
            # state does not change
            return self.status['pump']

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

        # first check the pump status
        if found(['0075']) or found('NK'):
            self.status['pump'] = True
        elif found(['0005']) or found('NJ'):
            self.status['pump'] = False

        # check 0075: find the earliest row number or default to 15
        if found(['0075']) or found('NK'):
            for pos in range(1, 15):
                if str(pos) in self.signals:
                    self.status['wedge_0075'] = pos
                    break
            else:
                self.status['wedge_0075'] = 15

        # check 0005: find the earliest row number or default to 15
        if found(['0005']) or found('NJ'):
            for pos in range(1, 15):
                if str(pos) in self.signals:
                    self.status['wedge_0005'] = pos
                    break
            else:
                self.status['wedge_0005'] = 15

    def valves_control(self, state):
        """Turn valves on or off, check valve status.
        Accepts signals (turn on), False (turn off) or None (get the status)"""
        if state:
            self.output.valves_on(state)
            self.update_pump_and_wedges()
            self.signals = ordered_signals(state)
        elif state is None:
            pass
        else:
            self.output.valves_off()
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
            turn_on(start_gpio)
            time.sleep(0.5)
            turn_off(start_gpio)
            self.status['motor'] = True
            return True
        else:
            stop_gpio = self.gpios['motor_stop']
            turn_on(stop_gpio)
            time.sleep(0.5)
            turn_off(stop_gpio)
            self.status['motor'] = False
            self.meter_events.clear()
            return False

    def air_control(self, state=None):
        """Air supply control: master compressed air solenoid valve.
            no state or None = get the air state,
            anything evaluating to True or False = turn on or off"""
        if state is None:
            return self.status['air']
        elif state:
            turn_on(self.gpios['air'])
            self.status['air'] = True
            return True
        else:
            turn_off(self.gpios['air'])
            self.status['air'] = False
            return False

    def water_control(self, state=None):
        """Cooling water control:
            no state or None = get the water valve state,
            anything evaluating to True or False = turn on or off"""
        if state is None:
            return self.status['water']
        elif state:
            turn_on(self.gpios['water'])
            self.status['water'] = True
            return True
        else:
            turn_off(self.gpios['water'])
            self.status['water'] = False
            return False

    @handle_machine_stop
    def pump_control(self, state=None):
        """No state: get the pump status.
        Anything evaluating to True or False: start or stop the pump"""
        def start():
            """Start the pump."""
            pump_start_code = ['N', 'K', 'S', '0075']
            # get the current 0075 wedge position and preserve it
            wedge_position = self.status['wedge_0075']
            pump_start_code.append(str(wedge_position))
            # start the pump
            self.send_signals(pump_start_code)

        def stop():
            """Stop the pump if it is working.
            This function will send the pump stop combination (NJS 0005) twice
            to make sure that the pump is turned off.
            In case of failure, repeat."""
            if not self.status['pump']:
                # that means the pump is not working, so why stop it?
                return

            # turn the emergency LED on, working LED off if needed
            working_led = self.gpios['working_led']
            working_led_state = get_state(working_led)
            if working_led_state:
                turn_off(self.gpios['working_led'])
            turn_on(self.gpios['error_led'])
            pump_stop_code = ['N', 'J', 'S', '0005']

            # don't change the current 0005 wedge position
            wedge_position = self.status['wedge_0005']
            pump_stop_code.append(str(wedge_position))

            # use longer timeout
            timeout = self.config['pump_stop_timeout']

            # try as long as necessary
            while self.status['pump']:
                self.send_signals(pump_stop_code, timeout=timeout)
                self.send_signals(pump_stop_code, timeout=timeout)

            # finished; emergency LED off, working LED on if needed
            turn_off(self.gpios['error_led'])
            if working_led_state:
                turn_on(working_led)

        if state is None:
            pass
        elif state:
            start()
        else:
            stop()
        return self.status['pump']

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
        def send_double(code):
            """Send a double justification sequence i.e. 0075+0005"""
            self.send_signals([*'NKJS', '0075', '0005', str(code)])

        def send_0005():
            """Send a 0005+code"""
            self.send_signals([*'NJS', '0005', str(new_0005)])

        def send_0075():
            """Send a 0005+code"""
            self.send_signals([*'NKS', '0075', str(new_0075)])

        pump_working = self.status['pump']
        current_0005 = self.status['wedge_0005']
        current_0075 = self.status['wedge_0075']
        new_0005 = wedge_0005 or current_0005
        new_0075 = wedge_0075 or current_0075

        if galley_trip:
            # double justification: line out + set wedges
            if pump_working:
                send_double(new_0005)
                send_0075()
            else:
                send_double(new_0075)
                send_0005()

        elif new_0005 == current_0005 and new_0075 == current_0075:
            # no need to do anything
            return

        else:
            # single justification = no galley trip
            if pump_working:
                # if no change, skip
                send_0005()
                send_0075()
            else:
                send_0075()
                send_0005()

    def prepare_signals(self, input_signals):
        """Prepare the incoming signals for casting, testing or punching."""
        def parse_signals(source):
            """Parse the incoming signals iterable into useful signals"""
            def find(value):
                """Detect and dispatch known signals in source string"""
                nonlocal _source
                string = str(value)
                if string in _source:
                    _source = _source.replace(string, '')
                    return True
                else:
                    return False

            # make sure it's an uppercase string
            try:
                _source = source.upper()
            except AttributeError:
                _source = ''.join(str(x) for x in source).upper()
            # read the signals to know what's inside
            input_signals = tuple(['0005', '0075',
                                   *(str(x) for x in range(16, 0, -1)),
                                   *'ABCDEFGHIJKLMNOS'])
            return {s for s in input_signals if find(s)}

        def strip_16():
            """Get rid of the "16" signal and replace it with "15"."""
            if '16' in parsed_signals:
                parsed_signals.discard('16')
                parsed_signals.add('15')

        def convert_hmn():
            """HMN addressing mode - developed by Monotype, based on KMN.
            Uncommon."""
            # NI, NL, M -> add H -> HNI, HNL, HM
            # H -> add N -> HN
            # N -> add M -> MN
            # O -> add HMN
            # {ABCDEFGIJKL} -> add HM -> HM{ABCDEFGIJKL}

            # earlier rows than 16 won't trigger the attachment -> early return
            for i in range(1, 16):
                if str(i) in parsed_signals:
                    return

            columns = 'NI', 'NL', 'H', 'M', 'N', 'O'
            extras = 'H', 'H', 'N', 'H', 'M', 'HMN'
            if '16' in parsed_signals:
                parsed_signals.discard('16')
                for column, extra in zip(columns, extras):
                    if parsed_signals.issuperset(column):
                        parsed_signals.update(extra)
                        return
                parsed_signals.update('HM')

        def convert_kmn():
            """KMN addressing mode - invented by a British printshop.
            Very uncommon."""
            # NI, NL, M -> add K -> KNI, KNL, KM
            # K -> add N -> KN
            # N -> add M -> MN
            # O -> add KMN
            # {ABCDEFGHIJL} -> add KM -> KM{ABCDEFGHIJL}

            # earlier rows than 16 won't trigger the attachment -> early return
            for i in range(1, 16):
                if str(i) in parsed_signals:
                    return

            columns = 'NI', 'NL', 'K', 'M', 'N', 'O'
            extras = 'K', 'K', 'N', 'K', 'M', 'HMN'
            if '16' in parsed_signals:
                parsed_signals.discard('16')
                for column, extra in zip(columns, extras):
                    if parsed_signals.issuperset(column):
                        parsed_signals.update(extra)
                        return
                parsed_signals.update('KM')

        def convert_unitshift():
            """Unit-shift addressing mode - rather common,
            designed by Monotype and introduced in 1963"""
            if 'D' in parsed_signals:
                # when the attachment is on, the D signal is routed
                # to unit-shift activation piston instead of column D air pin
                # this pin is activated by EF combination instead
                parsed_signals.discard('D')
                parsed_signals.update('EF')
            if '16' in parsed_signals:
                # use unit shift if the row signal is 16
                # make it possible to shift the diecase on earlier rows
                parsed_signals.update('D')
                parsed_signals.discard('16')

        def convert_o15():
            """Change O and 15 to a combined O+15 signal"""
            for sig in ('O', '15'):
                if sig in parsed_signals:
                    parsed_signals.discard(sig)
                    parsed_signals.add('O15')

        def strip_o15():
            """For casting, don't use O+15"""
            parsed_signals.discard('O15')

        def add_missing_o15():
            """If length of signals is less than 2, add an O+15,
            so that when punching, the ribbon will be advanced properly."""
            if len(parsed_signals) < 2:
                parsed_signals.add('O15')

        parsed_signals = parse_signals(input_signals)
        # based on row 16 addressing mode,
        # decide which signal conversion should be applied
        row16_conv = {None: strip_16, 'unit shift': convert_unitshift,
                      'HMN': convert_hmn, 'KMN': convert_kmn}[self.row16_mode]
        row16_conv()
        # based on the operation mode, strip, convert or add O/15 signals
        # casting: strip (as it's not used),
        # punching: add if less than 2 signals,
        # testing: convert O or 15 to O+15 which will be sent
        mode_conv = (convert_o15 if self.testing_mode
                     else add_missing_o15 if self.operation_mode == 'punching'
                     else strip_o15)
        mode_conv()
        return parsed_signals

    def send_signals(self, signals, timeout=None):
        """Send the signals to the caster/perforator.
        This method performs a single-dispatch on current operation mode:
            casting: sensor ON, valves ON, sensor OFF, valves OFF;
            punching: valves ON, wait t1, valves OFF, wait t2
            testing: valves OFF, valves ON

        In the punching mode, if there are less than two signals,
        an additional O+15 signal will be activated. Otherwise the paper ribbon
        advance mechanism won't work."""
        if self.testing_mode:
            self.test(signals)
        elif self.operation_mode == 'casting':
            self.cast(signals, timeout=timeout)
        elif self.operation_mode == 'punching':
            self.punch(signals)

    def cast(self, input_signals, timeout=None):
        """Monotype composition caster.

        Wait for sensor to go ON, turn on the valves,
        wait for sensor to go OFF, turn off the valves.
        """
        if not self.status['working']:
            raise exc.InterfaceNotStarted

        codes = self.prepare_signals(input_signals)
        # allow the use of a custom timeout
        timeout = timeout or self.config['sensor_timeout']
        # machine control cycle
        self.wait_for_sensor(ON, timeout=timeout)
        self.valves_control(codes)
        self.wait_for_sensor(OFF, timeout=timeout)
        self.valves_control(OFF)

    def test(self, input_signals):
        """Turn off any previous combination, then send signals.
        """
        if not self.status['working']:
            self.machine_control(True)

        codes = self.prepare_signals(input_signals)
        # change the active combination
        self.valves_control(OFF)
        self.valves_control(codes)

    def punch(self, input_signals):
        """Timer-driven ribbon perforator.

        Turn on the valves, wait the "punching_on_time",
        then turn off the valves and wait for them to go down
        ("punching_off_time").
        """
        if not self.status['working']:
            self.machine_control(True)

        codes = self.prepare_signals(input_signals)
        # timer-driven operation
        self.valves_control(codes)
        time.sleep(self.config['punching_on_time'])
        self.valves_control(OFF)
        time.sleep(self.config['punching_off_time'])

if __name__ == '__main__':
    main()
