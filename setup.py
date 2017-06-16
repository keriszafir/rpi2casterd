from setuptools import setup
from rpi2caster_driver import metadata

with open('README.rst', 'r') as readme_file:
    long_description = readme_file.read()

setup(name='rpi2caster_driver',
      version=metadata.version,
      description='Raspberry Pi GPIO setup utility for rpi2caster',
      long_description=long_description,
      url=metadata.github_url,
      author=metadata.author,
      author_email=metadata.author_email,
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
      install_requires=['RPi.GPIO >= 0.6.3', 'Flask >= 0.12'],
      zip_safe=True)
