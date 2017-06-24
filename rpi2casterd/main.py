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
DEFAULTS = dict(listen_address='0.0.0.0:23017', output_driver='smbus',
                shutdown_gpio='24', shutdown_command='shutdown -h now',
                reboot_gpio='23', reboot_command='shutdown -r now',
                startup_timeout='30', sensor_timeout='5',
                pump_stop_timeout='120',
                punching_on_time='0.2', punching_off_time='0.3',
                debounce_milliseconds='25',
                led_gpio='18', sensor_gpio='17', error_led_gpio='26',
                air_gpio='19', water_gpio='13', emergency_stop_gpio='22',
                motor_start_gpio='5', motor_stop_gpio='6',
                i2c_bus='1', mcp0_address='0x20', mcp1_address='0x21',
                valve1='N,M,L,K,J,I,H,G',
                valve2='F,S,E,D,0075,C,B,A',
                valve3='1,2,3,4,5,6,7,8',
                valve4='9,10,11,12,13,14,0005,O15',
                supported_modes='testing, casting, punching, manual punching',
                supported_row16_modes='off, HMN, KMN, unit shift')
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
    # ugly-iterate over a list of keys because we mutate the dict
    interface_ids = [i for i in INTERFACES]
    for interface_id in interface_ids:
        interface = INTERFACES[interface_id]
        interface.valves_off()
        INTERFACES.pop(interface_id)
    for led in LEDS.values():
        turn_off(led)
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
            blink('ready')
            cmd = config.get('shutdown_command')
            subprocess.run(cv.command(cmd))

    def reboot(*_):
        """Restart the system"""
        print('Reboot button pressed. Hold down for 2s to reboot...')
        time.sleep(2)
        # the button is between GPIO and GND i.e. pulled up - negative logic
        if not GPIO.input(reboot_gpio):
            print('Rebooting...')
            blink('ready')
            cmd = config.get('reboot_command')
            subprocess.run(cv.command(cmd))

    def signal_handler(*_):
        """Exit gracefully if SIGINT or SIGTERM received"""
        raise KeyboardInterrupt

    config = CFG.defaults()
    # set the LED up
    led_gpio = cv.get('led_gpio', config, int)
    GPIO.setup(led_gpio, GPIO.OUT)
    LEDS['ready'] = led_gpio
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
    # all configured - it's ready to work
    ready_led_gpio = LEDS.get('ready')
    turn_on(ready_led_gpio)
    APP.run(address, port)


class Interface:
    """Hardware control interface"""
    def __init__(self, config_dict):
        self.config = config_dict
        self.mode, self.row16_mode = None, None
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

    def __str__(self):
        return 'Raspberry Pi interface ({})'.format(self.output.name)

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
            else:
                raise NameError
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

    def start(self, mode=None, row16_mode=None):
        """Start the machine.
        Casting requires that the machine is running before proceeding."""
        # don't let anyone else initialize an interface already initialized
        if self.state['working']:
            raise exc.InterfaceBusy

        # check and update the operation mode
        if mode in self.config['supported_modes']:
            self.mode = mode
        elif not mode:
            self.mode = self.config['default_mode']
        else:
            raise exc.UnsupportedMode(mode)

        # check and update the row 16 addressing mode
        # testing and punching can use all modes
        all_row16_modes = ('off', 'HMN', 'KMN', 'unit shift')
        if row16_mode in self.config['supported_row16_modes']:
            self.row16_mode = row16_mode
        elif self.mode != 'casting' and row16_mode in all_row16_modes:
            self.row16_mode = row16_mode
        elif not row16_mode:
            self.row16_mode = self.config['default_row16_mode']
        else:
            raise exc.UnsupportedRow16Mode(row16_mode)

        # turn on the compressed air
        self.air_control(ON)
        # make sure the machine is turning before proceeding
        if self.mode == 'casting':
            # turn on the cooling water and motor
            self.water_control(ON)
            self.motor_control(ON)
            try:
                self.check_rotation()
            except exc.MachineStopped:
                # cleanup and pass the exception
                self.stop()
                raise
        # properly initialized => mark it as working
        self.state['working'] = True

    def stop(self):
        """Stop the machine."""
        self.pump_stop()
        self.valves_off()
        self.state['signals'] = []
        if self.mode == 'casting':
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
            self.wait_for(ON, timeout=timeout)
            self.wait_for(OFF, timeout=timeout)
        end_time = time.time()
        duration = end_time - start_time
        # how fast is the machine turning?
        rpm = round(60 * cycles / duration, 2)
        self.state.update(speed='{}rpm'.format(rpm))

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
        # turn on the emergency LED
        turn_on(self.gpios['error_led'])
        pump_stop = ['N', 'J', 'S', '0005']
        while self.state['pump']:
            try:
                # first time
                self.wait_for(ON, timeout=timeout)
                self.valves_on(pump_stop)
                self.wait_for(OFF, timeout=timeout)
                self.valves_off()
                # second time
                self.wait_for(ON, timeout=timeout)
                self.valves_on(pump_stop)
                self.wait_for(OFF, timeout=timeout)
                self.valves_off()
                # successfully stopped
                self.state['pump'] = False
            except exc.MachineStopped:
                self.valves_off()
        # finished; LED off
        turn_off(self.gpios['error_led'])
        # the 0005 wedge position changes as well, so update it
        self.state.update(wedge_0005=15)

    def valves_off(self):
        """Turn all valves off"""
        self.output.valves_off()

    def valves_on(self, signals):
        """proxy for output's valves_on method"""
        self.output.valves_on(signals)
        ordered_signals = cv.ordered_signals(signals)
        self.state['signals'] = ordered_signals
        return ordered_signals

    def motor_control(self, value=None):
        """Motor control:
            no value or None = get the motor state,
            anything evaluating to True or False = turn on or off"""
        if value is None:
            # do nothing
            return self.state['motor']
        elif value:
            start_gpio = self.gpios['motor_start']
            turn_on(start_gpio)
            time.sleep(0.5)
            turn_off(start_gpio)
            self.state['motor'] = True
            return True
        else:
            stop_gpio = self.gpios['motor_stop']
            turn_on(stop_gpio)
            time.sleep(0.5)
            turn_off(stop_gpio)
            self.state['motor'] = False
            return False

    def air_control(self, value=None):
        """Air supply control: master compressed air solenoid valve.
            no value or None = get the air state,
            anything evaluating to True or False = turn on or off"""
        if value is None:
            return self.state['air']
        elif value:
            turn_on(self.gpios['air'])
            self.state['air'] = True
            return True
        else:
            turn_off(self.gpios['air'])
            self.state['air'] = False
            return False

    def water_control(self, value=None):
        """Cooling water control:
            no value or None = get the water valve state,
            anything evaluating to True or False = turn on or off"""
        if value is None:
            return self.state['water']
        elif value:
            turn_on(self.gpios['water'])
            self.state['water'] = True
            return True
        else:
            turn_off(self.gpios['water'])
            self.state['water'] = False
            return False

    def send_signals(self, signals, timeout=None):
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
        conversions = {'off': cv.strip_16,
                       'HMN': cv.convert_hmn,
                       'KMN': cv.convert_kmn,
                       'unit shift': cv.convert_unitshift}
        signals = conversions[self.row16_mode or 'off'](signals)
        signals = cv.convert_o15(signals)

        if self.mode == 'casting':
            # casting: sensor-driven valves on and off
            signals = cv.strip_o15(signals)
            try:
                self.wait_for(ON, timeout=timeout)
                self.valves_on(signals)
                self.state.update(pump=self.check_pump())
                self.wait_for(OFF, timeout=timeout)
                self.valves_off()
            except exc.MachineStopped:
                # run recursively if needed
                self.pump_stop()
                raise

        elif self.mode == 'manual punching':
            # semi-automatic perforator (advanced by keypress)
            signals = cv.add_missing_o15(signals)
            self.valves_on(signals)
            time.sleep(self.config['punching_on_time'])
            self.valves_off()

        elif self.mode == 'punching':
            # timer-driven perforator
            signals = cv.add_missing_o15(signals)
            self.valves_on(signals)
            time.sleep(self.config['punching_on_time'])
            self.valves_off()
            time.sleep(self.config['punching_off_time'])

        elif self.mode == 'testing':
            # send signals to valves and keep them on
            self.valves_off()
            self.valves_on(signals)

        self.state.update(self.check_wedge_positions())
        return signals


if __name__ == '__main__':
    main()
