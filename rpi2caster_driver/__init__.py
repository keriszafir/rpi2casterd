"""rpi2caster-driver - a hardware control daemon for rpi2caster interfaces
based on a Raspberry Pi microcomputer (and other single-board computers)."""

try:
    from .main import Controller, Interface, APP as app, setup, main
    __all__ = ['Controller', 'Interface', 'app', 'setup', 'main']
except ImportError:
    print('Some dependencies are not installed')

__version__ = '0.1.dev1'
__author__ = 'Christophe Slychan'
__author_email__ = 'krzysztof.slychan@gmail.com'
__github_url__ = 'http://github.com/elegantandrogyne/rpi2caster-driver'
