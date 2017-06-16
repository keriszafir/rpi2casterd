"""rpi2caster-driver - a hardware control daemon for rpi2caster interfaces
based on a Raspberry Pi microcomputer (and other single-board computers)."""

from .main import Controller, Interface, APP as app, setup, main

__version__ = '0.1.dev1'
__author__ = 'Christophe Slychan'
__author_email__ = 'krzysztof.slychan@gmail.com'
__github_url__ = 'http://github.com/elegantandrogyne/rpi2caster-driver'

__all__ = ['Controller', 'Interface', 'app', 'setup', 'main']
