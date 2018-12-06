"""rpi2casterd setup: this software SHOULD be installed on a Raspberry Pi."""
from setuptools import setup

__version__ = '2.2.26'
__author__ = 'Christophe Catherine Slychan'
__author_email__ = 'krzysztof.slychan@gmail.com'
__github_url__ = 'http://github.com/elegantandrogyne/rpi2casterd'
__dependencies__ = ['RPi.GPIO >= 0.6.3', 'Flask >= 0.12',
                    'librpi2caster >= 2.0']

with open('README.rst', 'r') as readme_file:
    long_description = readme_file.read()

setup(name='rpi2casterd', version=__version__,
      description='Hardware control daemon for rpi2caster',
      long_description=long_description,
      url=__github_url__, author=__author__, author_email=__author_email__,
      license='MIT',
      packages=['rpi2casterd'], include_package_data=True,
      package_data={'rpi2casterd': ['data/*']},
      data_files=[('/etc/systemd/system', ['data/rpi2casterd.service']),
                  ('/usr/lib/rpi2casterd', ['data/rpi2casterd.conf'])],
      classifiers=['Development Status :: 5 - Production/Stable',
                   'Topic :: System :: Hardware :: Hardware Drivers',
                   'License :: OSI Approved :: MIT License',
                   'Natural Language :: English',
                   'Operating System :: POSIX :: Linux',
                   'Programming Language :: Python :: 3 :: Only',
                   'Framework :: Flask'],
      install_requires=__dependencies__, zip_safe=True,
      entry_points={'console_scripts': ['rpi2casterd = rpi2casterd.main:main']}
      )
