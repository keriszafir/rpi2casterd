rpi2casterd
=================

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
and send specified signals to the caster or perforator.
GET requests are used for obtaining the interface's state and configuration,
water/air/motor/pump state and current justification wedge positions.

Operation modes
---------------

The interface / driver can operate in different modes, denoted by the ``mode`` parameter
in the ``start`` request's JSON payload. Depending on the mode, the behavior and signals sent vary.

testing
~~~~~~~

All signals are sent as specified.
No additional modifications are made (except for row 16 addressing, if needed).
The driver closes any opened valves, then turns on the valves corresponding to the signals
found in request, and returns a success message.

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

manual punching
~~~~~~~~~~~~~~~

As above, but the control behavior relies on the operator to advance the process:

1. turn the valves on,
2. wait time_on for the punches to go up,
3. turn the valves off,
4. return a success reply.

The client side pauses the execution until the operator confirms the advance
to the next combination.

Additional row 16 addressing modes
----------------------------------

A ``row16_mode`` parameter in the ``start`` request JSON sets the interface's
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
---------------

The Raspberry Pi based controller can be coupled with more devices than the basic functionality requires.

The program supports configuring multiple control interfaces (i.e. sensor + valve sets).

Apart from getting the machine cycle sensor state and sending signals to solenoid valves,
the program can start and stop the machine's motor, control additional water and air cutoff valves,
use an emergency stop button to stop the machine when something bad happens, and light a LED
when the controller is trying to stop the caster's pump.

API documentation
=================

to be added later...
