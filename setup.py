from setuptools import setup
from rpi2casterd import metadata

with open('README.rst', 'r') as readme_file:
    long_description = readme_file.read()

setup(name='rpi2casterd',
      version=metadata.version,
      description='Hardware control daemon for rpi2caster',
      long_description=long_description,
      url=metadata.github_url,
      author=metadata.author,
      author_email=metadata.author_email,
      license='MIT',
      packages=['rpi2casterd'],
      include_package_data=True,
      package_data={'rpi2casterd': ['data/*']},
      data_files=[('/etc/systemd/system', ['data/rpi2casterd.service']),
                  ('/etc', ['data/rpi2casterd.conf'])],
      classifiers=['Development Status :: 5 - Production/Stable',
                   'License :: OSI Approved :: MIT',
                   'Natural Language :: English',
                   'Operating System :: POSIX :: Linux',
                   'Programming Language :: Python :: 3 :: Only',],
      entry_points={'console_scripts':['rpi2casterd = rpi2casterd.main:main']},
      install_requires=['RPi.GPIO >= 0.6.3', 'Flask >= 0.12'],
      zip_safe=True)
