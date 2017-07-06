# -*- coding: utf-8 -*-
"""Exceptions for rpi2casterd"""


class MachineStopped(Exception):
    """machine not turning exception"""
    code = 0
    name = 'machine stopped'


class UnsupportedMode(Exception):
    """The operation mode is not supported by this interface."""
    code = 1
    name = 'unsupported operation mode'


class UnsupportedRow16Mode(Exception):
    """The row 16 addressing mode is not supported by this interface."""
    code = 2
    name = 'unsupported row 16 addressing mode'


class InterfaceBusy(Exception):
    """the interface was claimed by another client and cannot be used
    until it is released"""
    code = 3
    name = 'interface already in use'


class InterfaceNotStarted(Exception):
    """the interface was not started and cannot accept signals"""
    code = 4
    name = 'interface not initialized'


class ConfigurationError(Exception):
    """configuration error: wrong name or cannot import module"""
    code = 5
    name = 'hardware configuration error'


class NoUsefulSignals(Exception):
    """no useful signals found in the sequence."""
    code = 6
    name = 'no useful signals found'
