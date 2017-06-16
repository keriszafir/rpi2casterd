# -*- coding: utf-8 -*-
"""hardware drivers for rpi2caster"""

import configparser
import io
import functools
import os
import select
import signal
import subprocess
import time

from flask import Flask, abort, jsonify
from flask.globals import request
import RPi.GPIO as GPIO

import rpi2caster_driver.converters as converters

# Where to look for config?
CONFIGURATION_PATH = '/etc/rpi2caster-driver.conf'

# Where to listen to?
DEFAULT_ADDRESS = '127.0.0.1'
DEFAULT_PORT = 23017

# Bus parameters
I2C_BUS = 1
MCP0, MCP1 = 0x20, 0x21

# Default GPIO parameters
DEFAULT_SENSOR_GPIO = 17  # photocell input
DEFAULT_SHDN_GPIO = 24  # shutdown button input, pulled up
DEFAULT_REBOOT_GPIO = 23  # reboot button input, pulled up
DEFAULT_STOP_GPIO = 22  # emergency stop button input, pulled up
DEFAULT_LED_GPIO = 18  # "system ready" LED output

# Default timings
DEFAULT_CASTING_STARTUP_TIMEOUT = 30  # during startup rotation check
DEFAULT_CASTING_SENSOR_TIMEOUT = 5  # during normal casting
DEFAULT_CASTING_PUMP_STOP_TIMEOUT = 120  # during pump stop
DEFAULT_PUNCHING_ON_TIME = 0.2  # how long to keep the air flowing / punches up
DEFAULT_PUNCHING_OFF_TIME = 0.3  # how long to wait before next combination
DEFAULT_DEBOUNCE_TIME = 25  # milliseconds for software de-bouncing

# Status symbols for convenience
AIR_ON, AIR_OFF = True, False
# Working modes
ALL_MODES = TESTING, CASTING, PUNCHING = 0, 1, 2
# Row 16 addressing modes
ALL_ROW16_MODES = ROW16_OFF, ROW16_HMN, ROW16_KMN, ROW16_UNITSHIFT = 0, 1, 2, 3

# Default signals arrangement
SIGNALS = ['O15', *'NMLKJIHGFSED', '0075', *'CBA',
           *'123456789', '10', '11', '12', '13', '14', '0005']

# Control combinations for the caster
PUMP_STOP = ['N', 'J', 'S', '0005']
PUMP_START = ['N', 'K', 'S', '0075']
DOUBLE_JUSTIFICATION = ['N', 'K', 'J', 'S', '0075', '0005']

# Registered interfaces name_prefix : instance mapping
INTERFACES = {}
# Registered (exported) GPIOs
GPIOS = []

# Configuration engine
CFG = configparser.ConfigParser(converters={'list': converters.str_list,
                                            'intlist': converters.int_list})
CFG.read(CONFIGURATION_PATH)

# Initialize the application
GPIO.setmode(GPIO.BCM)
APP = Flask(__name__)


class HWConfigError(Exception):
    """configuration error: wrong name or cannot import module"""


class MachineStopped(Exception):
    """machine not turning exception"""


class SysfsSensor:
    """Optical cycle sensor using kernel sysfs interface"""
    name = 'Kernel SysFS interface for photocell sensor GPIO'
    value_file = None
    last_state = True

    def __init__(self, gpio):
        self.gpio = gpio
        self.bounce_time = CFG['Control'].getfloat(
            'bounce_time', fallback=DEFAULT_DEBOUNCE_TIME) * 0.001
        self.setup()

    def wait_for(self, new_state, timeout):
        """
        Waits until the sensor is in the desired state.
        new_state = True or False.
        timeout means that if no signals in given time,
        raise MachineStopped.
        force_cycle means that if last_state == new_state,
        a full cycle must pass before exit.
        Uses software debouncing set at 5ms
        """
        def get_state():
            """Reads current input state"""
            gpiostate.seek(0)
            # File can contain "1\n" or "0\n"; convert it to boolean
            return bool(int(gpiostate.read().strip()))

        # Set debounce time to now
        debounce = time.time()
        # Prevent sudden exit if the current state is the desired state
        with io.open(self.value_file, 'r') as gpiostate:
            if get_state() == new_state:
                self.last_state = not new_state
        with io.open(self.value_file, 'r') as gpiostate:
            while True:
                try:
                    signals = select.epoll()
                    signals.register(gpiostate, select.POLLPRI)
                    while self.last_state != new_state:
                        # Keep polling or raise MachineStopped on timeout
                        if signals.poll(timeout):
                            state = get_state()
                            # Input bounce time is given in milliseconds
                            if time.time() - debounce > self.bounce_time:
                                self.last_state = state
                            debounce = time.time()
                        else:
                            raise MachineStopped

                    # state changed
                    return

                except RuntimeError:
                    continue

    def setup(self):
        """configure_sysfs_interface(gpio):

        Sets up the sysfs interface for reading events from GPIO
        (general purpose input/output). Checks if path/file is readable.
        Returns the value and edge filenames for this GPIO.
        """
        # Set up an input polling file for machine cycle sensor:
        sysfs_root = '/sys/class/gpio'
        export_path = '{}/export'.format(sysfs_root)
        value_path = '{}/gpio{}/value'.format(sysfs_root, self.gpio)
        dir_path = '{}/gpio{}/direction'.format(sysfs_root, self.gpio)
        edge_path = '{}/gpio{}/edge'.format(sysfs_root, self.gpio)

        # Export the GPIO pin to sysfs
        with io.open(export_path, 'w') as export_file:
            export_file.write('{}'.format(self.gpio))

        # set the GPIO as input
        with io.open(dir_path, 'w') as direction_file:
            direction_file.write('in')

        # set the GPIO to generate interrupts on both edges
        with io.open(edge_path, 'w') as edge_file:
            edge_file.write('both')

        with io.open(edge_path, 'r') as edge_file:
            if 'both' not in edge_file.read():
                message = ('GPIO {} must be set to generate interrupts '
                           'on both rising AND falling edge!')
                raise OSError(19, message.format(self.gpio), edge_path)

        # verify that the GPIO value file is accessible
        if not os.access(value_path, os.R_OK):
            message = ('GPIO value file does not exist or cannot be read. '
                       'You must export the GPIO no {} as input first!')
            raise OSError(13, message.format(self.gpio), value_path)

        # register the GPIO for future teardown
        GPIOS.append(self.gpio)
        self.value_file = value_path


class RPiGPIOSensor:
    """Simple RPi.GPIO input driver for photocell"""
    name = 'RPi.GPIO input driver'

    def __init__(self, gpio):
        self.gpio = gpio
        self.bounce_time = CFG['Control'].getfloat(
            'bounce_time', fallback=DEFAULT_DEBOUNCE_TIME)
        self.setup()

    def setup(self):
        """Initial configuration."""
        GPIO.setup(self.gpio, GPIO.IN)

    def wait_for(self, new_state, timeout):
        """Use interrupt handlers in RPi.GPIO for triggering the change"""
        change = GPIO.RISING if new_state else GPIO.FALLING
        while True:
            try:
                channel = GPIO.wait_for_edge(self.gpio, change,
                                             timeout=timeout*1000,
                                             bouncetime=self.bounce_time)
                if channel is None:
                    raise MachineStopped
                else:
                    return
            except RuntimeError:
                # In case RuntimeError: Error waiting for edge is raised...
                continue
            except (KeyboardInterrupt, EOFError):
                # Emergency stop by keyboard
                raise MachineStopped


class Interface:
    """Hardware control interface"""
    mode, row16_mode = TESTING, ROW16_OFF
    sensor, output = None, None
    manual_advance = False
    busy, pump_working = False, False
    current_signals = []

    def __init__(self, config_section):
        # get the interface configuration
        self.config = config_section
        self.hardware_setup()

    @property
    @functools.lru_cache(32)
    def supported_modes(self):
        """Get a list of supported operation modes
        (testing/casting/punching)"""
        return self.config.getintlist('supported_modes', ALL_MODES)

    @property
    @functools.lru_cache(32)
    def supported_row16_modes(self):
        """Get a list of supported row 16 addressing modes
        (off, HMN, KMN, unit shift)"""
        return self.config.getintlist('supported_row16_modes', ALL_ROW16_MODES)

    @property
    @functools.lru_cache(32)
    def timings(self):
        """Get the times for the interface:
            startup_timeout: max time the interface is allowed to stall
                             during startup in casting mode,
            sensor_timeout:  as above but when casting,
            pump_stop_timeout: as above, but when stopping the pump,
            on_time: valves on time in automatic punching,
            off_time: valves off time in automatic punching."""

        casting_startup_timeout = self.config.getint(
            'startup_timeout', fallback=DEFAULT_CASTING_STARTUP_TIMEOUT)

        casting_sensor_timeout = self.config.getint(
            'sensor_timeout', fallback=DEFAULT_CASTING_SENSOR_TIMEOUT)

        casting_pump_stop_timeout = self.config.getint(
            'pump_stop_timeout', fallback=DEFAULT_CASTING_PUMP_STOP_TIMEOUT)

        punching_on_time = self.config.getfloat(
            'on_time', fallback=DEFAULT_PUNCHING_ON_TIME)

        punching_off_time = self.config.getfloat(
            'off_time', fallback=DEFAULT_PUNCHING_OFF_TIME)

        return dict(casting_startup_timeout=casting_startup_timeout,
                    casting_sensor_timeout=casting_sensor_timeout,
                    casting_pump_stop_timeout=casting_pump_stop_timeout,
                    punching_on_time=punching_on_time,
                    punching_off_time=punching_off_time)

    def status(self):
        """Get the interface current status"""
        return dict(success=True, busy=self.busy,
                    pump_working=self.pump_working,
                    signals=self.current_signals)

    def get_config(self):
        """Get the interface configuration"""
        return dict(success=True,
                    mode=self.mode, row16_mode=self.row16_mode,
                    supported_modes=self.supported_modes,
                    supported_row16_modes=self.supported_row16_modes)

    def set_config(self, data):
        """Change the interface configuration.
        If a mode or row 16 mode is specified, it'll be checked against the
        supported modes (failing that, the method will fail).

        The setup is atomic: no parameters are changed if any fails.
        """
        # cache the old parameters
        new_mode, new_row16_mode = self.mode, self.row16_mode
        # update the operation mode
        mode = data.get('mode')
        if mode in self.supported_modes:
            new_mode = mode
        elif mode is not None:
            return dict(success=False, error='unsupported_mode')
        # update the row 16 addressing mode
        row16_mode = data.get('row16_mode')
        if row16_mode in self.supported_row16_modes:
            new_row16_mode = row16_mode
        elif row16_mode is not None:
            return dict(success=False, error='unsupported_row16_mode')
        # update manual/automatic ribbon advance in punching mode
        manual_advance = data.get('ribbon_advance')
        if manual_advance is not None:
            self.manual_advance = bool(manual_advance)
        # store the new configuration
        self.mode, self.row16_mode = new_mode, new_row16_mode
        return dict(success=True)

    def hardware_setup(self):
        """Return a HardwareBackend namedtuple with sensor and driver.
        Raise ConfigurationError if sensor or output name is not recognized,
        or modules supporting the hardware backends cannot be imported."""
        # look up the sensor factory function if casting mode is available
        # otherwise skip it as sensor will not be used
        if CASTING in self.supported_modes:
            sensor_gpio = self.config.getint('sensor_gpio',
                                             fallback=DEFAULT_SENSOR_GPIO)
            try:
                sensor_name = CFG['Control'].get('sensor', 'sysfs').lower()
                sensor = (SysfsSensor if sensor_name == 'sysfs'
                          else RPiGPIOSensor if sensor_name == 'rpi_gpio'
                          else RPiGPIOSensor if sensor_name == 'gpio'
                          else None)
                self.sensor = sensor(sensor_gpio)
            except TypeError:
                raise HWConfigError('Unknown sensor: {}.'.format(sensor_name))
        # the same with output, but for all modes unconditionally:
        try:
            mcp0_address = self.config.getint('mcp0_address', fallback=MCP0)
            mcp1_address = self.config.getint('mcp1_address', fallback=MCP1)
            i2c_bus = self.config.getint('i2c_bus', fallback=I2C_BUS)
            signals = self.config.getlist('signals', fallback=[]) or SIGNALS
            output_name = CFG['Control'].get('output', 'smbus').lower()
            if output_name == 'smbus':
                from rpi2caster_driver.smbus import SMBusOutput as output
            elif output_name == 'wiringpi':
                from rpi2caster_driver.wiringpi import WiringPiOutput as output
            self.output = output(signals, mcp0_address, mcp1_address, i2c_bus)
        except NameError:
            raise HWConfigError('Unknown output: {}.'.format(output_name))
        except ImportError:
            raise HWConfigError('Module not installed for {}'
                                .format(output_name))

    def start(self):
        """Start the machine.
        Casting requires that the machine is running before proceeding."""
        if self.busy:
            return dict(success=False, error='busy')
        self.busy = True
        if self.mode == CASTING:
            timeout = self.timings['casting_startup_timeout']
            cycles = 3
            try:
                self.sensor.wait_for(AIR_ON, timeout=timeout)
                for _ in range(cycles, 0, -1):
                    self.sensor.wait_for(AIR_ON, timeout=timeout)
            except MachineStopped:
                self.busy = False
                return dict(success=False, error='machine_stopped')
        return dict(success=True)

    def stop(self):
        """Stop the machine."""
        self.output.valves_off()
        if self.mode == CASTING:
            self.pump_stop()
        # cut the air off in all modes
        self.output.valves_off()
        self.busy = False
        return dict(success=True)

    def check_pump(self):
        """Check if the pump is working or not"""
        if set(PUMP_STOP).issubset(self.current_signals):
            # pump going OFF
            return False
        elif set(PUMP_START).issubset(self.current_signals):
            # pump going ON
            return True
        else:
            # state does not change
            return self.pump_working

    def pump_stop(self):
        """Stop the pump if it is working"""
        if self.pump_working:
            timeout = self.timings['casting_pump_stop_timeout']
            self.send_signals(PUMP_STOP, timeout=timeout)

    def send_signals(self, signals, timeout=None):
        """Send the signals to the caster/perforator.
        Based on mode:
            casting: sensor ON, valves ON, sensor OFF, valves OFF;
            punching: valves ON, wait t1, valves OFF, wait t2
            testing: valves OFF, valves ON

        In the punching mode, if there are less than two signals,
        an additional O+15 signal will be activated. Otherwise the paper ribbon
        advance mechanism won't work."""
        timeout = timeout or self.timings['casting_sensor_timeout']
        # first adjust the signals based on the row16 addressing mode
        conversions = {ROW16_OFF: lambda x: x,
                       ROW16_HMN: converters.convert_hmn,
                       ROW16_KMN: converters.convert_kmn,
                       ROW16_UNITSHIFT: converters.convert_unitshift}
        conversion = conversions[self.row16_mode]
        codes = signals if self.mode == TESTING else conversion(signals)
        self.current_signals = codes
        if self.mode == CASTING:
            # casting: sensor-driven valves on and off
            try:
                self.sensor.wait_for(AIR_ON, timeout=timeout)
                self.output.valves_on(codes)
                self.sensor.wait_for(AIR_OFF, timeout=timeout)
                self.output.valves_off()
                self.pump_working = self.check_pump()
                return dict(success=True, signals=codes)
            except MachineStopped:
                # run recursively if needed
                self.pump_stop()
                return dict(success=False, error='machine_stopped')
        elif self.mode == PUNCHING and self.manual_advance:
            # semi-automatic perforator (advanced by keypress)
            self.output.valves_off()
            time.sleep(self.timings['punching_off_time'])
            self.output.valves_on(codes)
            return dict(success=True, signals=codes)
        elif self.mode == PUNCHING:
            # timer-driven perforator
            self.output.valves_on(codes)
            time.sleep(self.timings['punching_on_time'])
            self.output.valves_off(codes)
            time.sleep(self.timings['punching_off_time'])
            return dict(success=True, signals=codes)
        else:
            # testing mode
            self.output.valves_off()
            self.output.valves_on(codes)
            return dict(success=True, signals=codes)


def teardown():
    """Unregister the exported GPIOs"""
    for gpio_number in GPIOS:
        with io.open('/sys/class/gpio/unexport', 'w') as unexport_file:
            unexport_file.write(str(gpio_number))
    GPIO.cleanup()


def handle_exceptions(routine):
    """Run a routine with exception handling"""
    @functools.wraps(routine)
    def wrapper(*args, **kwargs):
        """wraps the routine"""
        try:
            return routine(*args, **kwargs)

        except (OSError, PermissionError, RuntimeError):
            print('ERROR: You must run this program as root!')

        except KeyboardInterrupt:
            print('System exit.')

        finally:
            # make sure the GPIOs are de-configured properly
            teardown()
    return wrapper


@APP.route('/interfaces', methods=('GET',))
def list_interfaces():
    """Lists available interfaces"""
    return jsonify([i for i in INTERFACES])


@APP.route('/interfaces/<prefix>/config', methods=('GET', 'POST'))
def configuration(prefix):
    """GET: reads the interface configuration,
    POST: changes the configuration"""
    try:
        interface = INTERFACES[prefix]
        if request.method == 'GET':
            return jsonify(interface.get_config())
        elif request.method == 'POST':
            return jsonify(interface.set_config(request.json))
    except KeyError:
        abort(404)


@APP.route('/interfaces/<prefix>/status')
def get_status(prefix):
    """Gets the current interface status"""
    try:
        interface = INTERFACES[prefix]
        return jsonify(interface.status())
    except KeyError:
        abort(404)


@APP.route('/interfaces/<prefix>/start')
def start_machine(prefix):
    """Starts the machine"""
    try:
        interface = INTERFACES[prefix]
        return jsonify(interface.start())
    except KeyError:
        # the interface does not exist
        abort(404)


@APP.route('/interfaces/<prefix>/stop')
def stop_machine(prefix):
    """Stops the machine"""
    try:
        interface = INTERFACES[prefix]
        return jsonify(interface.stop())
    except KeyError:
        # the interface does not exist
        abort(404)


@APP.route('/interfaces/<prefix>/send', methods=('POST',))
def send_signals(prefix):
    """Sends the signals to the machine"""
    try:
        interface = INTERFACES[prefix]
        signals = request.json.get('signals')
        return jsonify(interface.send_signals(signals))
    except KeyError:
        abort(404)


def daemon_setup():
    """Configure the "ready" LED and shutdown/reboot buttons"""
    def shutdown(*_):
        """Shut the system down"""
        print('Shutdown button pressed. Hold down for 2s to shut down...')
        time.sleep(2)
        # the button is between GPIO and GND i.e. pulled up - negative logic
        if not GPIO.input(shdn_gpio):
            print('Shutting down...')
            blink()
            subprocess.run(['shutdown', '-h', 'now'])

    def reboot(*_):
        """Restart the system"""
        print('Reboot button pressed. Hold down for 2s to reboot...')
        time.sleep(2)
        # the button is between GPIO and GND i.e. pulled up - negative logic
        if not GPIO.input(reboot_gpio):
            print('Rebooting...')
            blink()
            subprocess.run(['shutdown', '-r', 'now'])

    def blink(seconds=0.5, number=3):
        """Blinks the LED"""
        for _ in range(number):
            GPIO.output(led_gpio, 0)
            time.sleep(seconds)
            GPIO.output(led_gpio, 1)
            time.sleep(seconds)

    def signal_handler(*_):
        """Exit gracefully if SIGINT or SIGTERM received"""
        raise KeyboardInterrupt

    config = CFG['Control']
    # set the LED up
    led_gpio = config.getint('led_gpio', DEFAULT_LED_GPIO)
    GPIO.setup(led_gpio, GPIO.OUT)
    GPIO.output(led_gpio, 1)
    # set the buttons up
    shdn_gpio = config.getint('shutdown_gpio', fallback=DEFAULT_SHDN_GPIO)
    reboot_gpio = config.getint('reboot_gpio', fallback=DEFAULT_REBOOT_GPIO)
    GPIO.setup(shdn_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(reboot_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # register callbacks for shutdown and reboot
    event_detect = GPIO.add_event_detect
    event_detect(shdn_gpio, GPIO.FALLING, callback=shutdown, bouncetime=200)
    event_detect(reboot_gpio, GPIO.FALLING, callback=reboot, bouncetime=200)
    # Register callbacks for signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)


def interface_setup():
    """Setup the interfaces"""
    # greedily instantiate the interfaces
    for section in CFG.sections():
        section_name = section.lower().strip()
        if section_name.startswith('interface'):
            interface = Interface(CFG[section])
            # register this interface and keep a reference to it
            interface_id = section_name.replace('interface', '').strip()
            INTERFACES[interface_id] = interface


@handle_exceptions
def main():
    """Starts the application"""
    daemon_setup()
    interface_setup()
    host = CFG['Control'].get('host', fallback=DEFAULT_ADDRESS)
    port = CFG['Control'].getint('port', fallback=DEFAULT_PORT)
    APP.run(host, port)


if __name__ == '__main__':
    main()
