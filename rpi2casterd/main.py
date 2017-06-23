# -*- coding: utf-8 -*-
"""rpi2casterd: hardware control daemon for the rpi2caster software.

This program runs on a Raspberry Pi or a similar single-board computer
and listens on its address(es) on a specified port using the HTTP protocol.
It communicates with client(s) via a JSON API and controls the machine
using selectable backend libraries for greater configurability.
"""

from collections import OrderedDict
import configparser
import functools
import signal
import subprocess
import time
import RPi.GPIO as GPIO

from rpi2casterd import exceptions as exc
from rpi2casterd import converters as cv
from rpi2casterd.webapi import INTERFACES, APP

# Where to look for config?
CONFIGURATION_PATH = '/etc/rpi2casterd.conf'
DEFAULTS = dict(listen_address='127.0.0.1:23017',
                sensor_driver='rpi_gpio', output_driver='smbus',
                shutdown_gpio='24', shutdown_command='shutdown -h now',
                reboot_gpio='23', reboot_command='shutdown -r now',
                startup_timeout='30', sensor_timeout='5',
                pump_stop_timeout='120',
                punching_on_time='0.2', punching_off_time='0.3',
                debounce_milliseconds='25',
                led_gpio='18', sensor_gpio='17',
                i2c_bus='1', mcp0_address='0x20', mcp1_address='0x21',
                valve1='N,M,L,K,J,I,H,G',
                valve2='F,S,E,D,0075,C,B,A',
                valve3='1,2,3,4,5,6,7,8',
                valve4='9,10,11,12,13,14,0005,O15',
                supported_modes='0,1,2,3',
                supported_row16_modes='0,1,2,3')
CFG = configparser.ConfigParser(defaults=DEFAULTS)
CFG.read(CONFIGURATION_PATH)

# Status symbols for convenience
AIR_ON, AIR_OFF = True, False
ON, OFF = True, False
# Working modes
ALL_MODES = 'testing', 'casting', 'punching', 'manual punching'
TESTING, CASTING, PUNCHING, MANUAL_PUNCHING = ALL_MODES
# Row 16 addressing modes
ALL_ROW16_MODES = 'off', 'HMN', 'KMN', 'unit shift'
ROW16_OFF, ROW16_HMN, ROW16_KMN, ROW16_UNITSHIFT = ALL_ROW16_MODES

# Control combinations for the caster
PUMP_STOP = ['N', 'J', 'S', '0005']
PUMP_START = ['N', 'K', 'S', '0075']
DOUBLE_JUSTIFICATION = ['N', 'K', 'J', 'S', '0075', '0005']

# Initialize the application
GPIO.setmode(GPIO.BCM)


def check_mode(routine):
    """Check if the interface supports the desired operation mode"""
    @functools.wraps(routine)
    def wrapper(interface, *args, **kwargs):
        """wraps the routine"""
        mode = kwargs.get('mode')
        if mode is not None:
            raise exc.UnsupportedMode(mode)
        return routine(interface, *args, **kwargs)
    return wrapper


def check_row16_mode(routine):
    """Check if the interface supports the desired row 16 addressing mode"""
    @functools.wraps(routine)
    def wrapper(interface, *args, **kwargs):
        """wraps the routine"""
        row16_mode = kwargs.get('row16_mode')
        if row16_mode is not None:
            raise exc.UnsupportedRow16Mode(row16_mode)
        return routine(interface, *args, **kwargs)
    return wrapper


def teardown():
    """Unregister the exported GPIOs"""
    # cleanup the registered interfaces
    # ugly-iterate over a list of keys because we mutate the dict
    interface_ids = [i for i in INTERFACES]
    for interface_id in interface_ids:
        interface = INTERFACES[interface_id]
        interface.valves_off()
        INTERFACES.pop(interface_id)
    GPIO.cleanup()


def handle_exceptions(routine):
    """Run a routine with exception handling"""
    @functools.wraps(routine)
    def wrapper(*args, **kwargs):
        """wraps the routine"""
        try:
            return routine(*args, **kwargs)

        except (OSError, PermissionError, RuntimeError) as exception:
            print('ERROR: You must run this program as root!')
            print(str(exception))

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
        if not GPIO.input(shutdown_gpio):
            print('Shutting down...')
            blink()
            cmd = config.get('shutdown_command')
            subprocess.run(cv.command(cmd))

    def reboot(*_):
        """Restart the system"""
        print('Reboot button pressed. Hold down for 2s to reboot...')
        time.sleep(2)
        # the button is between GPIO and GND i.e. pulled up - negative logic
        if not GPIO.input(reboot_gpio):
            print('Rebooting...')
            blink()
            cmd = config.get('reboot_command')
            subprocess.run(cv.command(cmd))

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
    led_gpio = cv.get('led_gpio', config, int)
    GPIO.setup(led_gpio, GPIO.OUT)
    GPIO.output(led_gpio, 1)
    # set the buttons up
    shutdown_gpio = cv.get('shutdown_gpio', config, int)
    reboot_gpio = cv.get('reboot_gpio', config, int)
    GPIO.setup(shutdown_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(reboot_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # debounce time in milliseconds
    millis = cv.get('debounce_milliseconds', config, int)
    # register callbacks for shutdown and reboot
    ev_det = GPIO.add_event_detect
    ev_det(shutdown_gpio, GPIO.FALLING, callback=shutdown, bouncetime=millis)
    ev_det(reboot_gpio, GPIO.FALLING, callback=reboot, bouncetime=millis)
    # Register callbacks for signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)


def interface_setup():
    """Setup the interfaces"""
    # greedily instantiate the interfaces
    for name, section in CFG.items():
        if name.lower() == 'default':
            # don't treat this as an interface
            continue
        settings = cv.parse_configuration(section)
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


class Interface:
    """Hardware control interface"""
    def __init__(self, config_dict):
        self.config = config_dict
        # initialize the interface with empty state
        self.state = OrderedDict(signals=[], wedge_0005=15, wedge_0075=15,
                                 working=False, water=False, air=False,
                                 motor=False, pump=False)
        # GPIO definitions (after setup, these will be actual GPIO numbers)
        self.gpios = dict(sensor=GPIO.IN, emergency_stop=GPIO.IN,
                          error_led=GPIO.OUT, air=GPIO.OUT, water=GPIO.OUT,
                          motor_stop=GPIO.OUT, motor_start=GPIO.OUT)
        self.output = None
        # configure the hardware
        self.hardware_setup()

    def get_status(self):
        """Returns the interface's current status"""
        state = self.state
        signals = state['signals']
        state.update(signals=cv.ordered_signals(signals))
        return state

    def hardware_setup(self):
        """Configure the inputs and outputs.
        Raise ConfigurationError if output name is not recognized,
        or modules supporting the hardware backends cannot be imported."""
        config = self.config

        # set up the controls
        gpio_settings = dict()
        for gpio_name, direction in self.gpios.items():
            gpio_config_name = '{}_gpio'.format(gpio_name)
            gpio_number = config[gpio_config_name]
            # configure the GPIO
            GPIO.setup(gpio_number, direction)
            gpio_settings[gpio_name] = gpio_number
        self.gpios.update(gpio_settings)

        # output setup:
        try:
            output_name = config['output_driver']
            if output_name == 'smbus':
                from rpi2casterd.smbus import SMBusOutput as output
            elif output_name == 'wiringpi':
                from rpi2casterd.wiringpi import WiringPiOutput as output
            self.output = output(config)
        except NameError:
            raise exc.HWConfigError('Unknown output: {}.'.format(output_name))
        except ImportError:
            raise exc.HWConfigError('Module not installed for {}'
                                    .format(output_name))

    def wait_for(self, new_state, timeout=None):
        """Wait until the machine cycle sensor changes its state
        to the desired value (True or False).
        If no state change is registered in the given time,
        raise MachineStopped."""
        sensor_gpio = self.gpios['sensor']
        change = GPIO.RISING if new_state else GPIO.FALLING
        debounce_milliseconds = self.config['debounce_milliseconds']
        timeout_milliseconds = int(timeout * 1000)
        while True:
            try:
                # all times are in milliseconds
                channel = GPIO.wait_for_edge(sensor_gpio, change,
                                             timeout=timeout_milliseconds,
                                             bouncetime=debounce_milliseconds)
                if channel is None:
                    raise exc.MachineStopped
                else:
                    return
            except RuntimeError:
                # In case RuntimeError: Error waiting for edge is raised...
                continue
            except (KeyboardInterrupt, EOFError):
                # Emergency stop by keyboard
                raise exc.MachineStopped

    @check_mode
    def start(self, mode):
        """Start the machine.
        Casting requires that the machine is running before proceeding."""
        # don't let anyone else initialize an interface already initialized
        if self.state['working']:
            raise exc.InterfaceBusy
        # turn on the compressed air
        self.air_control(ON)
        # make sure the machine is turning before proceeding
        if mode == CASTING:
            # turn on the cooling water and motor
            self.water_control(ON)
            self.motor_control(ON)
            try:
                self.check_rotation()
            except exc.MachineStopped:
                # cleanup and pass the exception
                self.stop(mode)
                raise
        # properly initialized => mark it as working
        self.state['working'] = True

    @check_mode
    def stop(self, mode):
        """Stop the machine."""
        self.pump_stop()
        self.valves_off()
        self.state['signals'] = []
        if mode == CASTING:
            self.motor_control(OFF)
            self.water_control(OFF)
        self.air_control(OFF)
        # release the interface so others can claim it
        self.state['working'] = False

    def check_pump(self):
        """Check if the pump is working or not"""
        def found(code):
            """check if code was found in a combination"""
            return set(code).issubset(signals)

        # cache this to avoid double dictionary lookup for each check
        signals = self.state['signals']
        if found(['0075']) or found('NK'):
            return True
        elif found(['0005']) or found('NJ'):
            return False
        else:
            # state does not change
            return self.state['pump']

    def check_rotation(self):
        """Check whether the machine is turning. Measure the speed."""
        timeout = self.config['startup_timeout']
        cycles = 3
        start_time = time.time()
        for _ in range(cycles, 0, -1):
            self.wait_for(AIR_ON, timeout=timeout)
            self.wait_for(AIR_OFF, timeout=timeout)
        end_time = time.time()
        duration = end_time - start_time
        # how fast is the machine turning?
        rpm = round(cycles / duration, 2)
        return dict(speed='{}rpm'.format(rpm))

    def check_wedge_positions(self):
        """Check the wedge positions and return them."""
        def found(code):
            """check if code was found in a combination"""
            return set(code).issubset(signals)

        signals = self.state['signals']

        # check 0075: find the earliest row number or default to 15
        if found(['0075']) or found('NK'):
            for pos in range(1, 15):
                if str(pos) in signals:
                    pos_0075 = pos
                    break
            else:
                pos_0075 = 15
        else:
            # no change = current state
            pos_0075 = self.state['wedge_0075']

        # check 0005: find the earliest row number or default to 15
        if found(['0005']) or found('NJ'):
            for pos in range(1, 15):
                if str(pos) in signals:
                    pos_0005 = pos
                    break
            else:
                pos_0005 = 15
        else:
            # no change = current state
            pos_0005 = self.state['wedge_0005']

        # we know them now
        return dict(wedge_0075=pos_0075, wedge_0005=pos_0005)

    def pump_stop(self):
        """Stop the pump if it is working.
        This function will send the pump stop combination (NJS 0005) twice
        to make sure that the pump is turned off.
        In case of failure, repeat."""
        timeout = self.config['pump_stop_timeout']
        while self.state['pump']:
            try:
                # first time
                self.wait_for(AIR_ON, timeout=timeout)
                self.valves_on(PUMP_STOP)
                self.wait_for(AIR_OFF, timeout=timeout)
                self.valves_off()
                # second time
                self.wait_for(AIR_ON, timeout=timeout)
                self.valves_on(PUMP_STOP)
                self.wait_for(AIR_OFF, timeout=timeout)
                self.valves_off()
                # successfully stopped
                self.state['pump'] = False
            except exc.MachineStopped:
                self.valves_off()
        # the 0005 wedge position changes as well, so update it
        self.state.update(wedge_0005=15)

    def valves_off(self):
        """Turn all valves off"""
        self.output.valves_off()

    def valves_on(self, signals):
        """proxy for output's valves_on method"""
        self.output.valves_on(signals)
        self.state['signals'] = signals

    def motor_control(self, value=None):
        """Motor control:
            no value or None = get the motor state,
            anything evaluating to True or False = turn on or off"""
        if value is not None:
            self.state['motor'] = True if value else False
        return dict(motor=self.state['motor'])

    def air_control(self, value=None):
        """Air supply control: master compressed air solenoid valve.
            no value or None = get the air state,
            anything evaluating to True or False = turn on or off"""
        if value is not None:
            self.state['air'] = True if value else False
        return dict(air=self.state['air'])

    def water_control(self, value=None):
        """Cooling water control:
            no value or None = get the water valve state,
            anything evaluating to True or False = turn on or off"""
        if value is not None:
            self.state['water'] = True if value else False
        return dict(water=self.state['water'])

    @check_mode
    @check_row16_mode
    def send_signals(self, signals,
                     mode=CASTING, row16_mode=ROW16_OFF, timeout=None):
        """Send the signals to the caster/perforator.
        Based on mode:
            casting: sensor ON, valves ON, sensor OFF, valves OFF;
            punching: valves ON, wait t1, valves OFF, wait t2
            testing: valves OFF, valves ON

        In the punching mode, if there are less than two signals,
        an additional O+15 signal will be activated. Otherwise the paper ribbon
        advance mechanism won't work."""
        # make sure the interface is initialized
        if not self.state['working']:
            raise exc.InterfaceNotStarted
        # allow using a custom timeout
        timeout = timeout or self.config['sensor_timeout']

        # first adjust the signals based on the row16 addressing mode
        conversions = {ROW16_OFF: cv.strip_16,
                       ROW16_HMN: cv.convert_hmn,
                       ROW16_KMN: cv.convert_kmn,
                       ROW16_UNITSHIFT: cv.convert_unitshift}
        signals = conversions[row16_mode](signals)
        signals = cv.convert_o15(signals)

        if mode == CASTING:
            # casting: sensor-driven valves on and off
            signals = cv.strip_o15(signals)
            try:
                self.wait_for(AIR_ON, timeout=timeout)
                self.valves_on(signals)
                self.state.update(pump=self.check_pump())
                self.wait_for(AIR_OFF, timeout=timeout)
                self.valves_off()
            except exc.MachineStopped:
                # run recursively if needed
                self.pump_stop()
                raise

        elif mode == MANUAL_PUNCHING:
            # semi-automatic perforator (advanced by keypress)
            signals = cv.add_missing_o15(signals)
            self.valves_on(signals)
            time.sleep(self.config['punching_on_time'])
            self.valves_off()

        elif mode == PUNCHING:
            # timer-driven perforator
            signals = cv.add_missing_o15(signals)
            self.valves_on(signals)
            time.sleep(self.config['punching_on_time'])
            self.valves_off()
            time.sleep(self.config['punching_off_time'])

        elif mode == TESTING:
            # send signals to valves and keep them on
            self.valves_off()
            self.valves_on(signals)

        self.state.update(self.check_wedge_positions)


if __name__ == '__main__':
    main()
