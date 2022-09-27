`strain_control` is a package for controlling a Razorbill CSX1x0 strain cell with the Razorbill RP100 power supply (PS) and a Keysight E4980 LCR meter. The package is composed of two main submodules, the `strain_server`, which coordinates between the PS and LCR meter to control for strain, and the `strain_client`, which allows for control of the `strain_server` from a Jupyter notebook or other scripting environment independent of the `strain_server`. This design has been chosen to decouple the day-to-day use of the instrument from software that makes it run continuously and reliably, and log performance, over the course of a multi-day experiment.


# Setup

First, this package must be cloned from GitHub, as well as my (Alex Liebman-Pelaez) particular fork of `PyMeasure`, since I have modified a few files. On the command line, run the following

```
git clone git@github.com:alexliebmanp/strain_control.git
git clone git@github.com:alexliebmanp/pymeasure.git
```

In addition, the following packages must be installed with `conda` or `pip` if they aren't on your system already:

```
simple-pid
pyvisa
numpy
matplotlib
OrensteinLab_git
threading
```

Lastly, `strain_control` and `PyMeasure` must be added to PYTHONPATH  or to the site-packages folder.

# Usage

To initiate the strain control server, edit the configurations as desired (described below), especially the initial sample length `L0_SAMP`, which is needed to get a correct measure of the strain, and simply run `strain_server.py`. On the command line,

```
python strain_server.py
```

This will popup a Qt display. The server captures whatever the current state of the system is, and no closed-loop control enabled.

To apply a voltage on the PS, enable closed-loop control, change the strain setpoint, modify the PS slew rate, or read off the current value for strain, etc., the user may import `strain_client.py` into any python environment. For example,

```
'''
An example notebook to demonstrate usage within a Jupyter Notebook.
'''

from strain_control.strain_client import StrainClient

sc = StrainClient()
sc.set_pid(1000,100,0.1)
sc.start_strain_control('PID')
sc.set_setpoint(0.05)
sc.get_voltage(1)
sc.get_strain()
sc.set_slew_rate(10)
sc.stop_strain_control()
sc.set_output(1,1)
sc.set_voltage(1,10)
sc.shutdown_server()
```

The first line after the import generates a `StrainClient` object, whose methods transmit messages to the `strain_server` and receive a response. See below for a description of the client methods.

NOTE: in the current version of this package, the socket communication host and port are hard-coded as `localhost` and `8888` respectively. In principle, the server can live on a remote computer and communication can be carried over a network.

# Strain Server

Most laboratory instruments come equipped with a dedicated control unit. For example, a Lakeshore temperature controller coordinates an applied *voltage* (to a heater) and a measured *resistance* (from a thermometer) to control for *temperature*, a third variable. The control unit runs it's own software to handle changes in temperature and maintain a set temperature. A user will often interact with the control unit (which also acts as a server) via a communications python package (client) to query the controller, change it's state, and read off measured values from buffers.

A Razorbill strain cell operates on much the same principle, except that it does not ship with a control unit that controls for *strain*. Instead, it comes with a power supply and a suggestion for purchasing an LCR meter. The power supply can output a *voltage* across two different channels to each piezo stack in the strain cell, and hence expand or contract the gap between the sample plates. The LCR meter can read a *capacitance* from a sensor. Based on a calibration curve and an initial sample length `L0_SMAP`, the capacitance can be related to the strain as follows:

```math
\sqrt(3)
```

In principle, the voltage can be changed to induce a change in strain, as measured with the capacitor.

Clearly, there is a need to coordinate these two instruments and control for *strain*. This is the job of the `strain_server`. The server runs over a few different threads and two processes:

1. main thread - initializes instruments and starts the display process, which launches a graphical display. When the display is closed, it also initiates the shutdown procedure
2. monitor thread - continuously queries PS and LCR meter to get values for voltage and capacitance, and converts the capacitance to a strain
3. control thread - when activated, initiates closed-loop control for strain in one of three modes, (1) PID, (2) Set Voltage, and (3) Combined
4. communications thread - listens for connections from the client and reacts appropriately to incoming messages
5. logging thread - continuously writes system state values to a log file

The server also includes features for properly and safely applying a voltage to the piezos. In particular, the `MAX_VOLTAGE` and `MIN_VOLTAGE` variables are used to limit the output voltage of the PS to user specified values (as described in the Razorbill manuals).

Here is a brief description of the three control modes:

1. PID: while in this mode, the server continuously updates the voltage output of the PS to achieve a desired `setpoint` utilizing a PID algorithm.
2. Set Voltage: in this mode, the server linearly ramps the voltage until the `setpoint` is achieved within some tolerance. It then maintains this voltage until the strain falls outside of the tolerance, at which point the loop repeats.
3. Combined: in this mode, the strain is roughly achieved using the Set Voltage algorithm, and then maintained with a PID loop.

(Eventually move these to a configuration file) The server may be configured in the source code by modifying the `strain_server.py` header,

```
SIM=True                    #  True to simulate PS and LCR meter for testing
STARTING_SETPOINT=0         # Initial setpoint
SLEW_RATE=0.5               # Initial slew rate
P=1000                      # Starting PID params
I=100
D=0.1
L0 = 68.68 # initial capacitor spacing    # gap between sensor plates at 0 strain
L0_SAMP = 68.68                           # sample length at 0 strain
C_MEASURED_0 = 0.812                      # pF, measured capacitance at 300K and 0V after a zeroing procedure.
C_0 = 0.808                               # pF, true capacitance at 300K and 0 V.

### LIMIT OUTPUT VOLTAGE HERE ###

MAX_VOLTAGE = 119 # V                     # limits to output voltage
MIN_VOLTAGE = -19 # V

### COMMUNICATION SETTINGS ###
LCR_ADDRESS = None                        # communications addresses.
PS_ADDRESS = None
MONTANA_ADDRESS = '10.1.1.15'
LAKESHORE_ADDRESS = ''
HOST = 'localhost'
PORT = 15200

### LOGGING
LOG_FILENAMEHEAD = r'C:\Users\orens\Google Drive\Shared drives\Orenstein Lab\Data\Strain cell log files'
```

Note: it is important that the server be shutdown properly using `shutdown_server()` command as described below. Otherwise, the strain cell system may be left in an unknown or unstable state, potentially leading to a dangerous situation.

# Strain Client

The `strain_client` takes the role of the communications package. It holds the lightweight `StrainClient` class, whose methods can pass commands to the server and receive responses. The user methods are as follows:

```
StrainClient.start_strain_control(self, mode='PID'):
    '''
    initiate control loop on strain server.

    returns:
        - response:

    kwargs:
        - mode(string):     'PID', 'Set Voltage', or 'Combined'
    '''

StrainClient.stop_strain_control(self):
    '''
    stop control loop on strain server.

    returns:
        - response: '1' if successful
    '''

StrainClient.get_strain(self):
    '''
    Get current strain from cell.

    args: None

    returns:
        - strain(float):        strain
    '''

StrainClient.get_dl(self):
    '''
    Get current gap from cell.

    args: None

    returns:
        - dl(float):        strain
    '''

StrainClient.get_cap(self):
    '''
    Get current capacitance from cell.

    args: None

    returns:
        - cap(float):        strain
    '''

StrainClient.get_voltage(self, channel):
    '''
    read voltage on given channel.

    args:
        - channel(int):     channel 1 or 2 on power supply

    returns:
        - voltage(float):
    '''

StrainClient.set_setpoint(self, new_setpoint):
    '''
    change target strain setpoint of control loop.

    args:
        - new_setpoint:     setpoint to set

    returns:
        - response:         '1' if successful
    '''

StrainClient.set_voltage(self, channel, voltage):
    '''
    sets voltage explicitly on channel 1 or 2

    args:
        - channel(int):     channel 1 or 2 on power supply
        - voltage(float):   voltage to set

    returns:
        - response:         '1' if successful
    '''

StrainClient.set_pid(self, p, i, d):
    '''
    Set PID parameters.

    args:
        - p(float):
        - i(float):
        - d(float):

    returns:
        - response(str):      '1' if successful
    '''


StrainClient.set_slew_rate(self, slew_rate):
    '''
    change voltage ramp rate on power supply:

    args:
        - slew_rate(float):    slew rate in V/s

    returns:
        - response:         '1' if successful
    '''

StrainClient.shutdown_server(self):
    '''
    Terminates strain server, correctly shutting down the system and leaving it in a stable, safe state (all voltages ramped to 0 and communications ports closed properly).

    args: None

    returns:
        - response(str):      '1' if successful
    '''
```
