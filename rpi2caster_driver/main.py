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
import RPi.GPIO as GPIO

import rpi2caster_driver.converters as converters
from rpi2caster_driver.webapi import INTERFACES, APP

# Where to look for config?
CONFIGURATION_PATH = '/etc/rpi2caster-driver.conf'
DEFAULTS = dict(listen_address='127.0.0.1:23017',
                sensor_driver='rpi_gpio', output_driver='smbus',
                shutdown_gpio='24', shutdown_command='shutdown -h now',
                reboot_gpio='23', reboot_command='shutdown -r now',
                casting_startup_timeout='30',
                casting_sensor_timeout='5',
                pump_stop_timeout='120',
                punching_on_time='0.2', punching_off_time='0.3',
                input_bounce_time='0.025',
                led_gpio='18', sensor_gpio='17',
                i2c_bus='1', mcp0='0x20', mcp1='0x21',
                valve1='N,M,L,K,J,I,H,G',
                valve2='F,S,E,D,0075,C,B,A',
                valve3='1,2,3,4,5,6,7,8',
                valve4='9,10,11,12,13,14,0005,O15',
                supported_modes='0,1,2,3',
                supported_row16_modes='0,1,2,3')
CONVERTERS = {'signals': converters.sig_list, 'intlist': converters.int_list}
CFG = configparser.ConfigParser(defaults=DEFAULTS, converters=CONVERTERS,
                                default_section='default')
CFG.read(CONFIGURATION_PATH)

# Status symbols for convenience
AIR_ON, AIR_OFF = True, False
# Working modes
ALL_MODES = TESTING, CASTING, PUNCHING, MANUAL_PUNCHING = 0, 1, 2, 3
# Row 16 addressing modes
ALL_ROW16_MODES = ROW16_OFF, ROW16_HMN, ROW16_KMN, ROW16_UNITSHIFT = 0, 1, 2, 3

# Control combinations for the caster
PUMP_STOP = ['N', 'J', 'S', '0005']
PUMP_START = ['N', 'K', 'S', '0075']
DOUBLE_JUSTIFICATION = ['N', 'K', 'J', 'S', '0075', '0005']

# Registered (exported) GPIOs
GPIOS = []
# Initialize the application
GPIO.setmode(GPIO.BCM)


class HWConfigError(Exception):
    """configuration error: wrong name or cannot import module"""


class MachineStopped(Exception):
    """machine not turning exception"""


class SysfsSensor:
    """Optical cycle sensor using kernel sysfs interface"""
    name = 'Kernel SysFS interface for photocell sensor GPIO'
    last_state = True

    def __init__(self, config):
        self.gpio = config['sensor_gpio']
        self.bounce_time = config['input_bounce_time']
        self.value_file = self.setup()
        GPIOS.append(self.gpio)

    def __str__(self):
        return self.name

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
        return value_path


class RPiGPIOSensor:
    """Simple RPi.GPIO input driver for photocell"""
    name = 'RPi.GPIO input driver'

    def __init__(self, config):
        self.gpio = config['sensor_gpio']
        self.bounce_time = config['input_bounce_time']
        self.setup()

    def __str__(self):
        return self.name

    def setup(self):
        """Initial configuration."""
        GPIO.setup(self.gpio, GPIO.IN)

    def wait_for(self, new_state, timeout):
        """Use interrupt handlers in RPi.GPIO for triggering the change"""
        change = GPIO.RISING if new_state else GPIO.FALLING
        millis = int(self.bounce_time * 1000)
        while True:
            try:
                # all times are in milliseconds
                channel = GPIO.wait_for_edge(self.gpio, change,
                                             timeout=timeout*1000,
                                             bouncetime=millis)
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
    sensor, output = None, None

    def __init__(self, config_dict):
        self.config = config_dict
        self.status = dict(busy=False, pump_working=False, signals=[])
        self.hardware_setup()

    @property
    def mode(self):
        """Get an operation mode"""
        return self.config['mode']

    @property
    def row16_mode(self):
        """Get a row 16 addressing mode"""
        return self.config['row16_mode']

    @property
    def busy(self):
        """Interface busy status"""
        return self.status['busy']

    @busy.setter
    def busy(self, state):
        """Busy setter"""
        self.status['busy'] = True if state else False

    def set_config(self, data):
        """Change the interface configuration.
        If a mode or row 16 mode is specified, it'll be checked against the
        supported modes (failing that, the method will fail).

        The setup is atomic: no parameters are changed if any fails.
        """
        # cannot reconfigure the interface on the fly
        if self.busy:
            return dict(success=False, error='busy')
        # work on the configuration dictionary
        config = self.config
        # cache the old parameters
        new_mode, new_row16_mode = config['mode'], config['row16_mode']
        # update the operation mode
        mode = data.get('mode')
        if mode in config['supported_modes']:
            new_mode = mode
        elif mode is not None:
            return dict(success=False, error='unsupported_mode')
        # update the row 16 addressing mode
        row16_mode = data.get('row16_mode')
        if row16_mode in config['supported_row16_modes']:
            new_row16_mode = row16_mode
        elif row16_mode is not None:
            return dict(success=False, error='unsupported_row16_mode')
        # store the new configuration
        config.update(dict(mode=new_mode, row16_mode=new_row16_mode))

    def hardware_setup(self):
        """Return a HardwareBackend namedtuple with sensor and driver.
        Raise ConfigurationError if sensor or output name is not recognized,
        or modules supporting the hardware backends cannot be imported."""
        # cache the interface configuration
        config = self.config
        # set up the sensor - either sysfs or RPi.GPIO
        try:
            driver = config['sensor_driver']
            sensor = (SysfsSensor if driver == 'sysfs'
                      else RPiGPIOSensor if driver == 'rpi_gpio'
                      else RPiGPIOSensor if driver == 'gpio'
                      else None)
            self.sensor = sensor(config)
        except TypeError:
            raise HWConfigError('Unknown sensor: {}.'.format(driver))
        # output setup:
        try:
            output_name = config['output_driver']
            if output_name == 'smbus':
                from rpi2caster_driver.smbus import SMBusOutput as output
            elif output_name == 'wiringpi':
                from rpi2caster_driver.wiringpi import WiringPiOutput as output
            self.output = output(config)
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
            return self.check_rotation()

    def stop(self):
        """Stop the machine."""
        self.valves_off()
        if self.mode == CASTING:
            self.pump_stop()
        # cut the air off in all modes
        self.valves_off()
        self.busy = False

    def check_pump(self):
        """Check if the pump is working or not"""
        signals = self.status['signals']
        if set(PUMP_STOP).issubset(signals):
            # pump going OFF
            return False
        elif set(PUMP_START).issubset(signals):
            # pump going ON
            return True
        else:
            # state does not change
            return self.status['pump_working']

    def check_rotation(self):
        """Check whether the machine is turning. Measure the speed."""
        timeout = self.config['casting_startup_timeout']
        cycles = 3
        try:
            start_time = time.time()
            for _ in range(cycles, 0, -1):
                self.sensor.wait_for(AIR_ON, timeout=timeout)
                self.sensor.wait_for(AIR_OFF, timeout=timeout)
            end_time = time.time()
            duration = end_time - start_time
            # how fast is the machine turning?
            rpm = round(cycles / duration, 2)
            return dict(speed='{}rpm'.format(rpm))

        except MachineStopped:
            self.busy = False
            return dict(error='machine_stopped')

    def pump_stop(self):
        """Stop the pump if it is working"""
        if self.status['pump_working']:
            timeout = self.config['pump_stop_timeout']
            self.send_signals(PUMP_STOP, timeout=timeout)

    def valves_off(self):
        """Turn all valves off"""
        self.output.valves_off()

    def valves_on(self, signals):
        """proxy for output's valves_on method"""
        _signals = [str(s).upper() for s in signals]
        self.output.valves_on(_signals)

    def send_signals(self, signals, timeout=None):
        """Send the signals to the caster/perforator.
        Based on mode:
            casting: sensor ON, valves ON, sensor OFF, valves OFF;
            punching: valves ON, wait t1, valves OFF, wait t2
            testing: valves OFF, valves ON

        In the punching mode, if there are less than two signals,
        an additional O+15 signal will be activated. Otherwise the paper ribbon
        advance mechanism won't work."""
        timeout = timeout or self.config['casting_sensor_timeout']
        mode, row16_mode = self.config['mode'], self.config['row16_mode']
        # first adjust the signals based on the row16 addressing mode
        conversions = {ROW16_OFF: lambda x: x,
                       ROW16_HMN: converters.convert_hmn,
                       ROW16_KMN: converters.convert_kmn,
                       ROW16_UNITSHIFT: converters.convert_unitshift}
        conversion = conversions[row16_mode]
        codes = signals if mode == TESTING else conversion(signals)
        self.status.update(signals=codes)
        if mode == CASTING:
            # casting: sensor-driven valves on and off
            try:
                self.sensor.wait_for(AIR_ON, timeout=timeout)
                self.valves_on(codes)
                self.sensor.wait_for(AIR_OFF, timeout=timeout)
                self.valves_off()
                self.status.update(pump_working=self.check_pump())
            except MachineStopped:
                # run recursively if needed
                self.pump_stop()
                return dict(error='machine_stopped')

        elif mode == MANUAL_PUNCHING:
            # semi-automatic perforator (advanced by keypress)
            self.valves_on(codes)
            time.sleep(self.config['punching_on_time'])
            self.valves_off()

        elif mode == PUNCHING:
            # timer-driven perforator
            self.valves_on(codes)
            time.sleep(self.config['punching_on_time'])
            self.valves_off()
            time.sleep(self.config['punching_off_time'])

        elif mode == TESTING:
            # send signals to valves and keep them on
            self.valves_off()
            self.valves_on(codes)

        else:
            return dict(error='unknown_mode')

        # all went well
        return self.status


def teardown():
    """Unregister the exported GPIOs"""
    # cleanup the registered interfaces
    # ugly-iterate over a list of keys because we mutate the dict
    interface_ids = [i for i in INTERFACES]
    for interface_id in interface_ids:
        interface = INTERFACES[interface_id]
        interface.valves_off()
        INTERFACES.pop(interface_id)
    # cleanup the registered GPIOs
    for gpio_number in GPIOS[:]:
        with io.open('/sys/class/gpio/unexport', 'w') as unexport_file:
            unexport_file.write(str(gpio_number))
        GPIOS.pop(gpio_number)
    GPIO.cleanup()


def handle_exceptions(routine):
    """Run a routine with exception handling"""
    @functools.wraps(routine)
    def wrapper(*args, **kwargs):
        """wraps the routine"""
        try:
            return routine(*args, **kwargs)

        except (OSError, PermissionError, RuntimeError) as exc:
            print('ERROR: You must run this program as root!')
            print(str(exc))

        except KeyboardInterrupt:
            print('System exit.')

        finally:
            # make sure the GPIOs are de-configured properly
            teardown()
    return wrapper


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
            cmd = config.get('shutdown_command')
            subprocess.run(converters.command(cmd))

    def reboot(*_):
        """Restart the system"""
        print('Reboot button pressed. Hold down for 2s to reboot...')
        time.sleep(2)
        # the button is between GPIO and GND i.e. pulled up - negative logic
        if not GPIO.input(reboot_gpio):
            print('Rebooting...')
            blink()
            cmd = config.get('reboot_command')
            subprocess.run(converters.command(cmd))

    def blink(seconds=0.5, number=3):
        """Blinks the LED"""
        for _ in range(number):
            GPIO.output(led_gpio, 0)
            time.sleep(seconds)
            GPIO.output(led_gpio, 1)
            time.sleep(seconds)

    def signal_handler(*_):
        """Exit gracefully if SIGINT or SIGTERM received"""
        blink(0.2, 5)
        raise KeyboardInterrupt

    config = CFG.defaults()
    # set the LED up
    led_gpio = int(config.get('led_gpio'))
    GPIO.setup(led_gpio, GPIO.OUT)
    GPIO.output(led_gpio, 1)
    # set the buttons up
    shdn_gpio = int(config.get('shutdown_gpio'))
    reboot_gpio = int(config.get('reboot_gpio'))
    # debounce time in milliseconds
    millis = int(float(config.get('input_bounce_time')) * 1000)
    GPIO.setup(shdn_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(reboot_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # register callbacks for shutdown and reboot
    event_detect = GPIO.add_event_detect
    event_detect(shdn_gpio, GPIO.FALLING, callback=shutdown, bouncetime=millis)
    event_detect(reboot_gpio, GPIO.FALLING, callback=reboot, bouncetime=millis)
    # Register callbacks for signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)


def interface_setup():
    """Setup the interfaces"""
    # greedily instantiate the interfaces
    for name, section in CFG.items():
        if name.lower == 'default':
            # don't treat this as an interface
            continue
        settings = converters.parse_configuration(section)
        interface = Interface(settings)
        INTERFACES[name.lower().strip()] = interface


@handle_exceptions
def main():
    """Starts the application"""
    daemon_setup()
    interface_setup()
    try:
        address, _port = CFG.defaults().get('listen_address').split(':')
        port = int(_port)
    except ValueError:
        address = CFG.defaults().get('listen_address')
        port = 23017
    APP.run(address, port)


if __name__ == '__main__':
    main()
