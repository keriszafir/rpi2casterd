"""Converter and parser functions for rpi2caster driver"""

from collections import OrderedDict

COLUMNS = tuple('ABCDEFGHIJKLMNO')
ROWS = tuple(str(x) for x in range(16, 0, -1))
JUSTIFICATION = ('0005', '0075', 'S')
# all Monotype signals
SIGNALS = [*COLUMNS[:-1], *(str(x) for x in range(15)), 'O15', *JUSTIFICATION]


def signals(input_string):
    """Convert 'a,b,c,d,e' -> ['A', 'B', 'C', 'D', 'E']."""
    raw = [x.strip().upper() for x in input_string.split(',')]
    return [x for x in raw if x in SIGNALS]


def integers(input_string):
    """Convert '1,2,3,4,5' -> [1, 2, 3, 4, 5]"""
    return [int(x.strip()) for x in input_string.split(',')]


def lcstring(input_string):
    """Return a lowercase string stripped of all whitespace"""
    return input_string.strip().lower()


def millis(input_string):
    """Get milliseconds from a fractional value in seconds"""
    seconds = float(lcstring(input_string))
    return int(seconds * 1000)


def anyint(input_string):
    """Convert a decimal, octal, binary or hexadecimal string to integer"""
    return int(lcstring(input_string), 0)


def command(input_string):
    """Operating system command: string -> accepted by subprocess.run"""
    chunks = input_string.split(' ')
    return [x.strip() for x in chunks]


def get(parameter, source, convert):
    """Gets a value from a specified source for a given parameter,
    converts it to a desired data type"""
    return convert(source[parameter])


def parse_configuration(source):
    """Get the interface parameters from a config parser section"""
    try:
        config = OrderedDict()
        # supported operation and row 16 addressing modes
        modes = get('supported_modes', source, integers)
        row16_modes = get('supported_row16_modes', source, integers)
        config['supported_modes'] = modes
        config['mode'] = modes[0]
        config['supported_row16_modes'] = row16_modes
        config['row16_mode'] = row16_modes[0]

        # determine the sensor and output drivers
        config['sensor_driver'] = get('sensor_driver', source, lcstring)
        config['output_driver'] = get('output_driver', source, lcstring)

        # get timings
        config['startup_timeout'] = get('startup_timeout', source, float)
        config['sensor_timeout'] = get('sensor_timeout', source, float)
        config['pump_stop_timeout'] = get('pump_stop_timeout', source, float)
        config['punching_on_time'] = get('punching_on_time', source, float)
        config['punching_off_time'] = get('punching_off_time', source, float)

        # interface settings: input
        config['sensor_gpio'] = get('sensor_gpio', source, int)
        config['input_bounce_time'] = get('input_bounce_time', source, float)

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

    except KeyError as exc:
        return dict(error='configuration_error: {}'.format(exc))


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
        _source = ''.join(str(x).upper() for x in source)
    # read the signals to know what's inside
    justification = [x for x in JUSTIFICATION if find(x)]
    rows = reversed([x for x in ROWS if find(x)]) or ['15']
    columns = [x for x in COLUMNS if find(x)] or ['O']
    # make NI, NL appear on the front
    for combo in ['NL', 'NI']:
        if set(combo).issubset(columns):
            columns = [*combo, *(c for c in columns if c not in combo)]
    return tuple([*columns, *rows, *justification])


def convert_hmn(source):
    """HMN addressing mode - developed by Monotype, based on KMN.
    Uncommon."""
    # NI, NL, M -> add H -> HNI, HNL, HM
    # H -> add N -> HN
    # N -> add M -> MN
    # O -> add HMN
    # {ABCDEFGIJKL} -> add HM -> HM{ABCDEFGIJKL}

    # earlier rows than 16 won't trigger the attachment -> early return
    signals_set = {str(s).upper() for s in source}
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


def convert_kmn(source):
    """KMN addressing mode - invented by a British printshop.
    Very uncommon."""
    # NI, NL, M -> add K -> KNI, KNL, KM
    # K -> add N -> KN
    # N -> add M -> MN
    # O -> add KMN
    # {ABCDEFGHIJL} -> add KM -> KM{ABCDEFGHIJL}

    # earlier rows than 16 won't trigger the attachment -> early return
    signals_set = {str(s).upper() for s in source}
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


def convert_unitshift(source):
    """Unit-shift addressing mode - rather common,
    designed by Monotype and introduced in 1963"""
    signals_set = {str(s).upper() for s in source}
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
