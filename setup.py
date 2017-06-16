from setuptools import setup
from rpi2caster_driver import __version__, __author__, __author_email__, __github_url__

with open('README.rst', 'r') as readme_file:
    long_description = readme_file.read()

setup(name='rpi2caster_driver',
      version=__version__,
      description='Raspberry Pi GPIO setup utility for rpi2caster',
      long_description=long_description,
      url=__github_url__,
      author=__author__,
      author_email=__author_email__,
      license='MIT',
      packages=['rpi2caster_driver'],
      include_package_data=True,
      package_data={'rpi2caster_driver': ['data/*']},
      data_files=[('/etc/systemd/system', ['data/rpi2caster-driver.service']),
                  ('/etc', ['data/rpi2caster-driver.conf'])],
      classifiers=['Development Status :: 5 - Production/Stable',
                   'License :: OSI Approved :: MIT',
                   'Natural Language :: English',
                   'Operating System :: POSIX :: Linux',
                   'Programming Language :: Python :: 3 :: Only',],
      entry_points={'console_scripts':['rpi2caster-driver = rpi2caster_driver.main:main']},
      install_requires=['RPi.GPIO'],
      zip_safe=True)
