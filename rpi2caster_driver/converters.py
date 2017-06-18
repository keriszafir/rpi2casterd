"""Converter and parser functions for rpi2caster driver"""

from collections import OrderedDict

COLUMNS = tuple('ABCDEFGHIJKLMNO')
ROWS = tuple(str(x) for x in range(16, 0, -1))
JUSTIFICATION = ('0005', '0075', 'S')
# all Monotype signals
SIGNALS = [*COLUMNS[:-1], *(str(x) for x in range(15)), 'O15', *JUSTIFICATION]


def sig_list(input_string):
    """Convert 'a,b,c,d,e' -> ['A', 'B', 'C', 'D', 'E']."""
    raw = [x.strip().upper() for x in input_string.split(',')]
    return [x for x in raw if x in SIGNALS]


def int_list(input_string):
    """Convert '1,2,3,4,5' -> [1, 2, 3, 4, 5]"""
    return [int(x.strip()) for x in input_string.split(',')]


def command(input_string):
    """Operating system command: string -> accepted by subprocess.run"""
    chunks = input_string.split(' ')
    return [x.strip() for x in chunks]


def parse_configuration(source):
    """Get the interface parameters from a config parser section"""
    try:
        config = OrderedDict()
        # supported operation and row 16 addressing modes
        modes = source.getintlist('supported_modes')
        row16_modes = source.getintlist('supported_row16_modes')
        config['supported_modes'] = modes
        config['mode'] = modes[0]
        config['supported_row16_modes'] = row16_modes
        config['row16_mode'] = row16_modes[0]

        # determine the sensor and output drivers
        config['sensor_driver'] = source.get('sensor_driver').lower().strip()
        config['output_driver'] = source.get('output_driver').lower().strip()

        # get timings
        config['casting_startup_timeout'] = source.getfloat('startup_timeout')
        config['casting_sensor_timeout'] = source.getfloat('sensor_timeout')
        config['pump_stop_timeout'] = source.getfloat('pump_stop_timeout')
        config['punching_on_time'] = source.getfloat('punching_on_time')
        config['punching_off_time'] = source.getfloat('punching_off_time')

        # interface settings: input
        config['sensor_gpio'] = source.getint('sensor_gpio')
        config['input_bounce_time'] = source.getfloat('input_bounce_time')

        # interface settings: output
        config['i2c_bus'] = source.getint('i2c_bus')
        config['mcp0_address'] = source.getint('mcp0_address')
        config['mcp1_address'] = source.getint('mcp1_address')
        config['signal_mappings'] = dict(valve1=source.getsignals('valve1'),
                                         valve2=source.getsignals('valve2'),
                                         valve3=source.getsignals('valve3'),
                                         valve4=source.getsignals('valve4'))

        # configuration ready to ship
        return config

    except KeyError as exc:
        return dict(error='configuration_error: {}'.format(exc))


def parse_signals(source):
    """Parse the incoming signals iterable into useful signals"""
    def find(value):
        """Detect and dispatch known signals in source string"""
        nonlocal signals
        string = str(value)
        if string in signals:
            signals = signals.replace(string, '')
            return True
        else:
            return False

    # make sure it's an uppercase string
    try:
        signals = source.upper()
    except AttributeError:
        signals = ''.join(str(x).upper() for x in source)
    # read the signals to know what's inside
    justification = [x for x in JUSTIFICATION if find(x)]
    rows = reversed([x for x in ROWS if find(x)]) or ['15']
    columns = [x for x in COLUMNS if find(x)] or ['O']
    # make NI, NL appear on the front
    for combo in ['NL', 'NI']:
        if set(combo).issubset(columns):
            columns = [*combo, *(c for c in columns if c not in combo)]
    return tuple([*columns, *rows, *justification])


def convert_hmn(signals):
    """HMN addressing mode - developed by Monotype, based on KMN.
    Uncommon."""
    # NI, NL, M -> add H -> HNI, HNL, HM
    # H -> add N -> HN
    # N -> add M -> MN
    # O -> add HMN
    # {ABCDEFGIJKL} -> add HM -> HM{ABCDEFGIJKL}

    # earlier rows than 16 won't trigger the attachment -> early return
    signals_set = {str(s).upper() for s in signals}
    for i in range(1, 16):
        if str(i) in signals_set:
            return signals_set

    columns = 'NI', 'NL', 'H', 'M', 'N', 'O'
    extras = 'H', 'H', 'N', 'H', 'M', 'HMN'
    if '16' in signals_set:
        for column, extra in zip(columns, extras):
            if column in signals_set:
                signals_set.update(extra)
                signals_set.discard('16')
                break
    return signals_set


def convert_kmn(signals):
    """KMN addressing mode - invented by a British printshop.
    Very uncommon."""
    # NI, NL, M -> add K -> KNI, KNL, KM
    # K -> add N -> KN
    # N -> add M -> MN
    # O -> add KMN
    # {ABCDEFGHIJL} -> add KM -> KM{ABCDEFGHIJL}

    # earlier rows than 16 won't trigger the attachment -> early return
    signals_set = {str(s).upper() for s in signals}
    for i in range(1, 16):
        if str(i) in signals_set:
            return signals_set

    columns = 'NI', 'NL', 'K', 'M', 'N', 'O'
    extras = 'K', 'K', 'N', 'K', 'M', 'HMN'
    if '16' in signals_set:
        for column, extra in zip(columns, extras):
            if column in signals_set:
                signals_set.update(extra)
                signals_set.discard('16')
                break
    return signals_set


def convert_unitshift(signals):
    """Unit-shift addressing mode - rather common,
    designed by Monotype and introduced in 1963"""
    signals_set = {str(s).upper() for s in signals}
    if 'D' in signals_set:
        # when the attachment is on, the D signal is routed
        # to unit-shift activation piston instead of column D air pin
        # this pin is activated by EF combination instead
        signals_set.discard('D')
        signals_set.update('EF')
    if '16' in signals_set:
        # use unit shift if the row signal is 16
        # make it possible to shift the diecase on earlier rows
        signals_set.update('D')
        signals_set.discard('16')
    return signals_set
