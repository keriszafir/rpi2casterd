# -*- coding: utf-8 -*-
"""rpi2casterd: hardware control daemon for the rpi2caster software.

This program runs on a Raspberry Pi or a similar single-board computer
and listens on its address(es) on a specified port using the HTTP protocol.
It communicates with client(s) via a JSON API and controls the machine
using selectable backend libraries for greater configurability.
"""
from collections import deque
from contextlib import suppress
from functools import partial, wraps
import configparser
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
                ready_led_gpio='18', sensor_gpio='17',
                working_led_gpio='25', error_led_gpio='26',
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
    def shutdown(*_):
        """Shut the system down"""
        print('Shutdown button pressed. Hold down for 2s to shut down...')
        time.sleep(2)
        # the button is between GPIO and GND i.e. pulled up - negative logic
        if not get_state(shutdown_gpio):
            print('Shutting down...')
            blink('ready')
            cmd = config.get('shutdown_command')
            subprocess.run(cv.command(cmd))

    def reboot(*_):
        """Restart the system"""
        print('Reboot button pressed. Hold down for 2s to reboot...')
        time.sleep(2)
        # the button is between GPIO and GND i.e. pulled up - negative logic
        if not get_state(reboot_gpio):
            print('Rebooting...')
            blink('ready')
            cmd = config.get('reboot_command')
            subprocess.run(cv.command(cmd))

    def signal_handler(*_):
        """Exit gracefully if SIGINT or SIGTERM received"""
        raise KeyboardInterrupt

    config = CFG.defaults()
    # set the LED up
    ready_led_gpio = cv.get('ready_led_gpio', config, int)
    GPIO.setup(ready_led_gpio, GPIO.OUT)
    LEDS['ready'] = ready_led_gpio

    # set the buttons up
    shutdown_gpio = cv.get('shutdown_gpio', config, int)
    reboot_gpio = cv.get('reboot_gpio', config, int)
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
        settings = cv.parse_configuration(section)
        interface = Interface(settings)
        INTERFACES[name.lower().strip()] = interface


def main():
    """Starts the application"""
    try:
        # get the listen address and port
        config = CFG.defaults()
        address, port = cv.get('listen_address', config, cv.address_and_port)
        # initialize hardware
        daemon_setup()
        interface_setup()
        # all configured - it's ready to work
        ready_led_gpio = LEDS.get('ready')
        turn_on(ready_led_gpio)
        # start the web application
        APP.run(address, port)

    except (OSError, PermissionError, RuntimeError) as exception:
        print('ERROR: You must run this program as root!')
        print(str(exception))

    except KeyboardInterrupt:
        print('System exit.')

    finally:
        # make sure the GPIOs are de-configured properly
        teardown()


class Interface:
    """Hardware control interface"""
    gpio_definitions = dict(sensor=GPIO.IN, emergency_stop=GPIO.IN,
                            error_led=GPIO.OUT, working_led=GPIO.OUT,
                            air=GPIO.OUT, water=GPIO.OUT,
                            motor_stop=GPIO.OUT, motor_start=GPIO.OUT)

    def __init__(self, config_dict):
        config = self.config = config_dict
        # what modes can and will this interface work with?
        # start with default values
        modes = dict(default_operation_mode=config['default_mode'],
                     default_row16_mode=config['default_row16_mode'],
                     current_operation_mode=config['default_mode'],
                     current_row16_mode=config['default_row16_mode'],
                     supported_operation_modes=config['supported_modes'],
                     supported_row16_modes=config['supported_row16_modes'])
        self.modes = modes
        # initialize the interface with empty state
        self.state = dict(wedge_0005=15, wedge_0075=15,
                          working=False, water=False, air=False,
                          motor=False, pump=False, sensor=False)
        # GPIO definitions (after setup, these will be actual GPIO numbers)
        self.gpios = dict()
        # store the current signals
        self.signals = []
        # output driver (will be initialized in hardware_setup)
        self.output = None
        # data structure to count photocell ON events for rpm meter
        self.meter_events = deque(maxlen=3)
        # configure the hardware
        self.hardware_setup(config)

    def __str__(self):
        return 'Raspberry Pi interface ({})'.format(self.output.name)

    def hardware_setup(self, config):
        """Configure the inputs and outputs.
        Raise ConfigurationError if output name is not recognized,
        or modules supporting the hardware backends cannot be imported."""
        def update_sensor(sensor_gpio):
            """Update the RPM event counter"""
            sensor_state = get_state(sensor_gpio)
            self.state['sensor'] = bool(sensor_state)
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
            raise exc.HWConfigError('Unknown output: {}.'.format(output_name))
        except ImportError:
            raise exc.HWConfigError('Module not installed for {}'
                                    .format(output_name))

    @handle_machine_stop
    def wait_for_sensor(self, new_state, timeout=None):
        """Wait until the machine cycle sensor changes its state
        to the desired value (True or False).
        If no state change is registered in the given time,
        raise MachineStopped."""
        start_time = time.time()
        timeout = timeout or self.config['sensor_timeout']
        while self.state['sensor'] != new_state:
            if time.time() - start_time > timeout:
                raise exc.MachineStopped
            # wait 10ms to ease the load on the CPU
            time.sleep(0.01)

    def mode_control(self, operation_mode=None, row16_mode=None):
        """Get or set the interface operation and row 16 addressing modes."""
        modes = self.modes
        new_modes = dict()

        # check if the interface is in casting mode
        # this will be overriden if mode changes to something different
        casting_mode = modes['current_operation_mode'] == 'casting'

        # check and update the operation mode
        if operation_mode in modes['supported_operation_modes']:
            # change an operation mode, but don't commit it yet...
            new_modes['current_operation_mode'] = operation_mode
            # ...and check if it's going to be casting
            casting_mode = operation_mode == 'casting'
        elif operation_mode == 'reset':
            default_mode = modes['default_operation_mode']
            new_modes['current_operation_mode'] = default_mode
        elif operation_mode:
            raise exc.UnsupportedMode(operation_mode)

        # check and update the row 16 addressing mode
        # testing and punching can use all modes
        all_row16_modes = ('off', 'HMN', 'KMN', 'unit shift')
        if row16_mode in modes['supported_row16_modes']:
            # casting modes are limited by configuration
            new_modes['current_row16_mode'] = row16_mode
        elif not casting_mode and row16_mode in all_row16_modes:
            # allow any addressing mode for testing and manual/auto punching
            new_modes['current_row16_mode'] = row16_mode
        elif row16_mode == 'reset':
            default_row16_mode = modes['default_row16_mode']
            new_modes['current_row16_mode'] = default_row16_mode
        elif row16_mode:
            raise exc.UnsupportedRow16Mode(row16_mode)

        # no exceptions occurred, everything went OK = set new modes
        self.modes.update(new_modes)
        return self.modes

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
            if self.state['working']:
                raise exc.InterfaceBusy

            # reset the RPM counter
            self.meter_events.clear()
            # turn on the compressed air
            self.air_control(ON)
            # make sure the machine is turning before proceeding
            if self.modes['current_operation_mode'] == 'casting':
                # turn on the cooling water and motor
                self.water_control(ON)
                self.motor_control(ON)
                self.check_rotation()
            # properly initialized => mark it as working
            turn_on(self.gpios['working_led'])
            self.state['working'] = True

        def stop():
            """Stop the machine."""
            if not self.state['working']:
                # don't stop a non-working interface
                return
            self.pump_control(OFF)
            self.valves_control(OFF)
            self.signals = []
            if self.modes['current_operation_mode'] == 'casting':
                self.motor_control(OFF)
                self.water_control(OFF)
            self.air_control(OFF)
            turn_off(self.gpios['working_led'])
            # release the interface so others can claim it
            self.state['working'] = False

        if state is None:
            pass
        elif state:
            start()
        else:
            stop()
        return self.state['working']

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
            return self.state['pump']

    def check_rotation(self, revolutions=3):
        """Check whether the machine is turning.
        The machine must typically go 3 revolutions of the main shaft."""
        timeout = self.config['startup_timeout']
        for _ in range(revolutions, 0, -1):
            self.wait_for_sensor(ON, timeout=timeout)
            self.wait_for_sensor(OFF, timeout=timeout)

    def check_wedge_positions(self):
        """Check the wedge positions and return them."""
        def found(code):
            """check if code was found in a combination"""
            return set(code).issubset(self.signals)

        # check 0075: find the earliest row number or default to 15
        if found(['0075']) or found('NK'):
            for pos in range(1, 15):
                if str(pos) in self.signals:
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
                if str(pos) in self.signals:
                    pos_0005 = pos
                    break
            else:
                pos_0005 = 15
        else:
            # no change = current state
            pos_0005 = self.state['wedge_0005']

        # we know them now
        return dict(wedge_0075=pos_0075, wedge_0005=pos_0005)

    @handle_machine_stop
    def valves_control(self, state):
        """Turn valves on or off, check valve status.
        Accepts signals (turn on), False (turn off) or None (get the status)"""
        if state:
            self.output.valves_on(state)
            self.signals = cv.ordered_signals(state)
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
            return self.state['motor']
        elif state:
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
            self.meter_events.clear()
            return False

    @handle_machine_stop
    def air_control(self, state=None):
        """Air supply control: master compressed air solenoid valve.
            no state or None = get the air state,
            anything evaluating to True or False = turn on or off"""
        if state is None:
            return self.state['air']
        elif state:
            turn_on(self.gpios['air'])
            self.state['air'] = True
            return True
        else:
            turn_off(self.gpios['air'])
            self.state['air'] = False
            return False

    @handle_machine_stop
    def water_control(self, state=None):
        """Cooling water control:
            no state or None = get the water valve state,
            anything evaluating to True or False = turn on or off"""
        if state is None:
            return self.state['water']
        elif state:
            turn_on(self.gpios['water'])
            self.state['water'] = True
            return True
        else:
            turn_off(self.gpios['water'])
            self.state['water'] = False
            return False

    def pump_control(self, state=None):
        """No state: get the pump status.
        Anything evaluating to True or False: start or stop the pump"""
        def start():
            """Start the pump."""
            pump_start_code = ['N', 'K', 'S', '0075']
            # get the current 0075 wedge position and preserve it
            wedge_position = self.state['wedge_0075']
            pump_start_code.append(str(wedge_position))
            # start the pump
            self.send_signals(pump_start_code)

        def stop():
            """Stop the pump if it is working.
            This function will send the pump stop combination (NJS 0005) twice
            to make sure that the pump is turned off.
            In case of failure, repeat."""
            if not self.state['pump']:
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
            wedge_position = self.state['wedge_0005']
            pump_stop_code.append(str(wedge_position))

            # use longer timeout
            timeout = self.config['pump_stop_timeout']

            # try as long as necessary
            while self.state['pump']:
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
        return self.state['pump']

    def justification_wedge_control(self, wedge_0005=None, wedge_0075=None):
        """Get or set the current 0075 and 0005 wedge positions.
        The pump state is not changed in the process:
            pump previously working will work afterwards, and vice versa.
        """
        pump_working = self.state['pump']
        current_0005 = self.state['wedge_0005']
        current_0075 = self.state['wedge_0075']
        new_0005 = wedge_0005 or current_0005
        new_0075 = wedge_0075 or current_0075
        if new_0005 == current_0005 and new_0075 == current_0075:
            # no change
            return dict(wedge_0075=current_0075, wedge_0005=current_0005)
        if pump_working:
            # stop, then start
            self.send_signals(['N', 'J', 'S', '0005', str(new_0005)])
            self.send_signals(['N', 'K', 'S', '0075', str(new_0075)])
        else:
            # start, then stop
            self.send_signals(['N', 'K', 'S', '0075', str(new_0075)])
            self.send_signals(['N', 'J', 'S', '0005', str(new_0005)])
        return dict(wedge_0075=new_0075, wedge_0005=new_0005)

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

        # based on row 16 addressing mode,
        # decide which signal conversion should be applied
        row16_mode = self.modes['current_row16_mode']
        conversion = {'off': cv.strip_16,
                      'HMN': cv.convert_hmn,
                      'KMN': cv.convert_kmn,
                      'unit shift': cv.convert_unitshift}[row16_mode]
        signals = conversion(signals)
        # if O or 15 is found in signals, convert it to O15
        signals = cv.convert_o15(signals)

        # based on operation mode, decide what to do with the signals
        mode = self.modes['current_operation_mode']
        control_machine = {'casting': partial(self.cast, timeout=timeout),
                           'testing': self.test,
                           'punching': self.auto_punch,
                           'manual punching': self.manual_punch}[mode]

        # test/cast/punch the signals
        control_machine(signals)
        self.state.update(pump=self.check_pump())
        self.state.update(self.check_wedge_positions())

    def cast(self, codes, timeout=None):
        """Monotype composition caster.

        Wait for sensor to go ON, turn on the valves,
        wait for sensor to go OFF, turn off the valves."""
        casting_codes = cv.strip_o15(codes)
        timeout = timeout or self.config['sensor_timeout']
        self.wait_for_sensor(ON, timeout=timeout)
        self.valves_control(casting_codes)
        self.wait_for_sensor(OFF, timeout=timeout)
        self.valves_control(OFF)

    def test(self, codes):
        """Turn off any previous combination, then send signals."""
        self.valves_control(OFF)
        self.valves_control(codes)

    def auto_punch(self, codes):
        """Timer-driven ribbon perforator.

        Turn on the valves, wait the "punching_on_time",
        then turn off the valves and wait for them to go down
        ("punching_off_time")."""
        punching_codes = cv.add_missing_o15(codes)
        self.valves_control(punching_codes)
        time.sleep(self.config['punching_on_time'])
        self.valves_control(OFF)
        time.sleep(self.config['punching_off_time'])

    def manual_punch(self, codes):
        """Semi-automatic ribbon perforator.

        Turn on the valves, wait the "punching_on_time",
        turn off the valves and yield control back.
        The client will advance to the next combination."""
        punching_codes = cv.add_missing_o15(codes)
        self.valves_control(punching_codes)
        time.sleep(self.config['punching_on_time'])
        self.valves_control(OFF)


if __name__ == '__main__':
    main()
