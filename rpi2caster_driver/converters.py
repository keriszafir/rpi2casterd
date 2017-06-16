"""Converter functions for rpi2caster driver"""


def str_list(input_string):
    """Convert 'a,b,c,d,e' -> ['a', 'b', 'c', 'd', 'e']"""
    return [x.strip() for x in input_string.split(',')]


def int_list(input_string):
    """Convert '1,2,3,4,5' -> [1, 2, 3, 4, 5]"""
    return [int(x.strip()) for x in input_string.split(',')]


def convert_hmn(signals):
    """HMN addressing mode - developed by Monotype, based on KMN.
    Uncommon."""
    # NI, NL, M -> add H -> HNI, HNL, HM
    # H -> add N -> HN
    # N -> add M -> MN
    # O -> add HMN
    # {ABCDEFGIJKL} -> add HM -> HM{ABCDEFGIJKL}

    # earlier rows than 16 won't trigger the attachment -> early return
    signals_set = set(signals)
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
    signals_set = set(signals)
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
    signals_set = set(signals)
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

