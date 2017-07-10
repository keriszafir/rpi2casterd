rpi2casterd
===========

Hardware driver and web API for rpi2caster
------------------------------------------

This is a machine control daemon for the ``rpi2caster`` typesetting and casting software.
It is supposed to run on a Raspberry Pi (any model) with an output expander based on two
MCP23017 chips to provide 32 additional outputs. These are connected to solenoid valves,
which in turn send the pneumatic signals to a Monotype composition caster or tape punch.

This driver uses a ``RPi.GPIO`` library for GPIO control, 

There are several available output control backends:

1. SMBus (via ``smbus-cffi`` or ``smbus2`` package),
2. ``WiringPi`` library.

When ready to use, the daemon lights a LED on a specified "ready LED" GPIO.
An additional functionality of this daemon is control over the power and reboot buttons.
After one of these buttons is held for 2 seconds, the LED flashes and the shutdown or reboot
procedure begins.

The program uses ``Flask`` to provide a rudimentary JSON API for caster control.
It accepts POST requests to start and stop the machine, turn the valves on and off,
change the operation and row 16 addressing modes and send specified signals to the caster or perforator.
GET requests are used for obtaining the interface's state current, default and supported modes,
configuration, water/air/motor/pump/machine state, rotation speed and current justification wedge positions.

Initializing
------------

When starting to work with the interface, change its mode of operation or row 16 addressing.

If a new operation or row 16 addressing mode is unsupported by the interface's configuration
(for example, the device is meant for being installed on a caster without any row 16 attachments),
the ``modes`` request will get an error message, stating that a mode we want to use is not supported.

The ``casting`` operation mode limits the choice of row 16 addressing modes to the supported modes only,
whereas the ``punching`` mode can use all row 16 addressing modes.

Starting
--------

The interface needs to be started up in order to work. The startup procedure ensures that:

1. the interface is not busy - has not been claimed by any other client,
2. air and (for casting only) water and motor is turned on, if the hardware supports this,
3. (for casting) the machine is actually turning,
4. an "interface in use" LED (green) is turned on, if hardware supports this,
5. the interface will stay busy until released by the ``stop`` method.

Stopping
--------

Stopping the interface ensures that:

1. if the pump is working, it is stopped (see the pump control section),
2. air and (for casting) water and motor is turned off, if hardware supports this,
3. the "interface in use" LED is turned off, if hardware supports this,
4. the interface is released for the future clients to claim.

Stopping is done by a request to ``/machine``, or a ``MachineStopped`` exception occurring in the program.
This happens if the machine has been waiting for the signal from the cycle sensor for too long, or was stopped
with an emergency stop button (for those control computers which support it).

Pump control
------------

The ``/pump`` endpoint gets or sets the status of the Monotype caster's pump.

During the pump switch-off procedure, an "alarm" LED (red) is lit to prompt the operator to turh the
machine's main shaft a few times. The interface will then send a ``NJS+0005+X`` signals combination, where X is the
current 0005 justification wedge's position. This way, stopping the pump does not reset the wedge position.

The pump switch-on procedure sends the ``NKS+0075+Y`` combination, where Y is the current 0075 justification
wedge's position.

Motor control
-------------

The ``/motor`` endpoint gets the status of the caster's motor.
``PUT``, ``POST`` or ``DELETE`` requests change its state.
If the interface's configuration does not support motor relay, the server replies with ``501 Not Implemented``.


Sending signals
---------------

The most important part of the controller.

Based on the caster's current testing/operation mode and row 16 addressing mode, the signals are changed
in order to activate the mechanisms such as ribbon advance and row 16 addressing attachment.

The signals can be sent for repeated casting/punching.

Sending signals can take place only when the interface has been previously started and claimed as busy;
otherwise, ``InterfaceNotStarted`` is raised in the casting mode, and the startup is done automatically
in the punching and testing modes.

If sending no signals, the valves are closed.


Operation modes
_______________

The interface / driver can operate in different modes, denoted by the ``mode`` parameter
in the ``modes`` POST request's JSON payload. Depending on the mode, the behavior and signals sent vary.

casting
~~~~~~~

Signals O and 15 are stripped, as they are the signals the caster defaults to
if no signal in the ribbon is found.
When the machine is working, the interface driver:

1. waits for a machine cycle sensor (photodiode) going ON,
2. activates specified valves,
3. waits until the cycle sensor goes OFF,
4. checks the current pump status,
5. turns all valves off,
6. returns a reply to the request, allowing the client to cast the next combination.

However, a machine sometimes stops during casting (e.g. when the operator sees a lead squirt
and has to stop immediately to prevent damage). In this case, the driver will check whether
the pump is working, and if this is the case, run a full signals send cycle with a pump stop
combination (NJS 0005 0075), which works both with unit-adding system on and off.
After the pump stop procedure is completed, the interface replies with an error message.

punching
~~~~~~~~

This mode modifies the signals, so that at least two of them are always present in a combination.
This way the pneumatic perforator from the Monotype keyboard can advance the ribbon.
When less than two signals are present, the driver adds an extra O+15 signal driven by the 32nd valve
(not used when casting). The compressed air from this valve is routed to O and 15 blind punches,
which make no perforation in the ribbon, but trigger the ribbon advance mechanism.

This mode is fully automatic and driven by a configureble timer.
The control sequence is as follows:

1. turn the valves on,
2. wait time_on for punches to go up,
3. turn the valves off,
4. wait time_off for punches to come back down,
5. return a success reply to the request.


Testing mode
------------

All signals are sent as specified. ``O`` and ``15`` are converted to the combined ``O15`` signal
and present on the output. Row 16 addressing modifications are also done, based on mode. 

The driver closes any open valves, then turns on the valves corresponding to the signals
found in request, and returns a success message.


Additional row 16 addressing modes
----------------------------------

A ``row16_mode`` parameter in the ``modes`` POST request JSON sets the interface's
row 16 addressing mode. Once the interface is working, this mode will not change.
If ``mode`` is ``casting``, the choice of ``row16_mode`` is limited by the
``supported_row16_modes`` configuration parameters. On the other hands, the ``testing``,
``punching`` and ``manual punching`` modes can operate with all four row 16 addressing modes.
Depending on the selection, the signals sent to valves will be changed to fit the control system in use.

Why all this?
~~~~~~~~~~~~~

The typical Monotype matrix case contained 15 rows and 15 or 17 columns.
In 1950s and 1960s a further extension by an additional row was introduced,
which allowed more flexibility in defining the matrix case layouts, and
made it possible to contain more characters in the diecase.
Some Monotype casters (especially from 1960s and later) are equipped with special
attachments (either from the very beginning, or retrofitted) for addressing
the additional row. There were three such systems.

off
~~~

value: ``False``

This means that a sort will be cast from row 15 instead of 16.
No modification to signals apart from replacing row 16 with 15.

HMN
~~~

The earliest system, devised by one of Monotype's customers.
It is based on combined signals (similar to N+I, N+L addressing of two additional columns).
For rows 1...15 no modifications are done.
For row 16, additional signals are introduced based on column:

1. NI, NL, M - add H - HNI, HNL, HM
2. H - add N - HN
3. N - add M - MN
4. O (no signal) - add HMN
5. {ABCDEFGIJKL} - add HM - HM{ABCDEFGIJKL}

KMN
~~~

Devised by Monotype and similar to HMN.
The extra signals are a little bit different.

1. NI, NL, M - add K - KNI, KNL, KM
2. K - add N - KN
3. N - add M - KM
4. O (no signal) - add KMN
5. {ABCDEFGHIJL} - add KM - KM{ABCDEFGHIJL}

unit shift
~~~~~~~~~~

Introduced by Monotype in 1963 and standard on all machines soon after.
When the attachment is activated, a signal D is re-routed to an additional pin on
the front pin block, which boosts the left-right (rows) matrix case draw rod,
so that its end goes into an upper socket in the special matrix jaw. This socket is offset
by 0.2" to the left, allowing the matrix case to go a full row farther.

Column D addressing is done with a combined E+F signals instead.
So:

1. replace D with EF in the original combination,
2. add D if addressing the row 16.


Advanced features
-----------------

The Raspberry Pi based controller can be coupled with more devices than the basic functionality requires.

The program supports configuring multiple control interfaces (i.e. sensor + valve sets).

Apart from getting the machine cycle sensor state and sending signals to solenoid valves,
the program can start and stop the machine's motor, control additional water and air cutoff valves,
use an emergency stop button to stop the machine when something bad happens, and light a LED
when the controller is trying to stop the caster's pump.


API documentation
=================

to be added later...
