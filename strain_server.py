'''
Starts strain control server application, which coordiantes Keysight LCR meter and Razrbill RP100 power supply to control for strain on the Razorbill CSX130 strain cell.

The server operates over various threads: (1) a main GUI thread for displaying and interacting with instruments. (2) a control loop which include options for setting power supply output voltage to achieve a desired strain and PID control. (3) a monitoring loop for reading and logging instrument values. (4) a communications loop for recieving and responding to commands from the strain client via python's implementation of socket programming.

SOME IMPORTANT SAFETY NOTES:
- include proper voltage limits for the power supply (ideally as a function of temperature with some backups safety to ensure it defaults to lowest limits)
- wire both the power supply and the capacitor correctly by rereading appropriate sections in the manual

To Do:

(2) read temperature on lakeshore
(3) expand GUI to be able to control, display PID params, etc.
(4) add documentation
(6) fix PID and how it interacts with rough ramp
(7) add some feedback control to rough ramp, or an option to do so
(10) safer socket communication, if possible, though so far it seems to be working okay.
(14) add PID tuning
(15) add logging

immediate: fix algorithm of set_strain()

ponder making multiprocessed rather than multithreaded, at least for the control loop

'''

from threading_classes import LockedVar, StoppableThread, LockedVal, StoppableProcess
from simulation import SimulatedLCR, SimulatedPS
from pymeasure.instruments.agilent import AgilentE4980
from pymeasure.instruments.razorbill import razorbillRP100
import pyvisa
from simple_pid import PID
from threading import Thread, Lock, Event
import threading
from multiprocessing import Process, Value
import multiprocessing as mp
import matplotlib.pyplot as plt
import numpy as np
import time
import sys
import socket
import re


##########################
### USER SETTINGS HERE ###
##########################
global SIM, STARTING_SETPOINT, P, I, D, L0, MAX_VOLTAGE, MIN_VOLTAGE, HOST, PORT, LCR_ADDRESS, PS_ADDRESS

SIM=True
STARTING_SETPOINT=0
P=1000
I=100
D=0.1
L0 = 68.68 # initial capacitor spacing
L0_SAMP = 68.68

### LIMIT OUTPUT VOLTAGE HERE ###

MAX_VOLTAGE = 119 # V
MIN_VOLTAGE = -19 # V

### COMMUNICATION SETTINGS ###
LCR_ADDRESS = None
PS_ADDRESS = None
HOST = 'localhost'
PORT = 8888

###########################
###########################
###########################

class StrainServer:

    def __init__(self, lcr, ps, serversocket, setpoint, P, I, D, l0_samp, l0=68.68, sim=False):
        '''
        class constructor.

        args:
            - lcr                   pymeasure LCR handle or simulation object
            - ps:                   pymeasure power supply handle or simulation object
            - s:                    a bound socket for communicating with strain client.
            - setpoint(float):       initial setpoint for PID
            - P(float):              proportional PID parameter
            - I(float):              integral PID parameter
            - D(float):              derivative PID parameter
            - l0_samp(float):        initial length of sample
            - l0(float):             initial capacitor gap

        returns: class instance object
        '''

        self.lcr = lcr
        self.ps = ps
        self.serversocket = s
        self.l0 = l0
        self.l0_samp = l0_samp
        self.sim = sim
        strain, cap, imaginary_impedance, dl = self.get_strain()
        self.strain = LockedVar(strain)
        self.setpoint = LockedVar(setpoint)
        self.cap = LockedVar(cap)
        self.imaginary_impedance = LockedVar(imaginary_impedance)
        self.dl = LockedVar(dl)
        v1, v2 = self.get_voltage(1), self.get_voltage(2)
        self.voltage_1 = LockedVar(v1)
        self.voltage_2 = LockedVar(v2)
        self.pid = PID(P, I, D, setpoint=self.setpoint.locked_read())
        self.ctrl_mode = LockedVar(1)
        self.run = LockedVar(True)
        self.host = HOST
        self.port = PORT

    def initialize_instruments(self):
        '''
        Sets initial setting and paramters for both LCR meter and power supply.
        '''

        return 1

    def start_strain_control(self, mode):
        '''
        High level handling of strain control. For now, sets new strain value by first slowly ramping voltage to approximate voltage and then maintaining strain with a restricted PID loop.

        args:
            - mode(int):     1:'PID, 2:'Set Voltage', or 3:'Combined'

        returns: None
        '''

        current_setpoint = self.setpoint.locked_read()

        if mode==1:

            pid_loop = StoppableThread(target=self.start_pid, args=(current_setpoint,), kwargs={'limit':False})
            print('Starting PID control.')
            pid_loop.start()

            current_thread = threading.current_thread()
            while current_thread.stopped()==False:
                new_setpoint = self.setpoint.locked_read()
                current_setpoint = new_setpoint
                self.pid.setpoint = current_setpoint

            print('Stopping PID control')
            pid_loop.stop()
            pid_loop.join()

        elif mode==2:

            print('Starting constant voltage control')
            self.set_strain(current_setpoint)

            current_thread = threading.current_thread()
            while current_thread.stopped()==False:
                new_setpoint = self.setpoint.locked_read()
                if new_setpoint != current_setpoint:
                    current_setpoint = new_setpoint
                    self.set_strain(current_setpoint)
            print('Stopping constant voltage control')

        elif mode==3:

            pid_loop = StoppableThread(target=self.start_pid, args=(current_setpoint,), kwargs={'limit':False})
            print('Setting strain with fixed voltage')
            self.set_strain(current_setpoint)
            print('Strain achieved, starting PID control')
            pid_loop.start()

            current_thread = threading.current_thread()
            while current_thread.stopped()==False:
                new_setpoint = self.setpoint.locked_read()
                if new_setpoint != current_setpoint:
                    current_setpoint = new_setpoint
                    if pid_loop.is_alive():
                        pid_loop.stop()
                        pid_loop.join()
                        print('Stopping PID control')
                    pid_loop = StoppableThread(target=self.start_pid, args=(current_setpoint,), kwargs={'limit':False})
                    print('Setting strain with fixed voltage')
                    self.set_strain(current_setpoint)
                    print('Strain achieved, starting PID control')
                    pid_loop.start()

            print('Stopping PID control')
            pid_loop.stop()
            pid_loop.join()

    def start_strain_monitor(self):
        '''
        Continuously reads lcr meter and ps and updates all state variables to class instance variables. In addition, in the future this should handle logging of instrument data.
        '''

        print('Starting strain monitor')

        current_thread = threading.current_thread()
        while current_thread.stopped() == False:
            strain, cap, imaginary_impedance, dl = self.get_strain()
            v1 = self.get_voltage(1)
            v2 = self.get_voltage(2)
            self.strain.locked_update(strain)
            self.cap.locked_update(cap)
            self.imaginary_impedance.locked_update(imaginary_impedance)
            self.dl.locked_update(dl)
            self.voltage_1.locked_update(v1)
            self.voltage_2.locked_update(v2)

    def start_display(self):
        '''
        initiate plotting.
        '''

        print('Starting GUI display')

        # setup plots
        fig, [[ax11, ax12], [ax21, ax22]] = plt.subplots(2,2)
        fig.set_size_inches(10, 8)
        window = 1000
        ax11.set_ylabel('strain (a.u.)')
        ax11.set_xlabel('time (s)')
        ax12.set_ylabel(r'dl ($\mu$m)')
        ax12.set_xlabel('time (s)')
        ax21.set_ylabel('voltage 1 (V)')
        ax21.set_xlabel('time (s)')
        ax22.set_ylabel('voltage 2 (V)')
        ax22.set_xlabel('time (s)')

        time_vect = np.zeros(window) #np.linspace(0,1,window)
        strain_vect = np.zeros(window)
        dl_vect = np.zeros(window)
        v1_vect = np.zeros(window)
        v2_vect = np.zeros(window)
        cap_vect = np.zeros(window)
        line11, = ax11.plot(time_vect, strain_vect, 'o', ms=3, color='orange')
        line12, = ax12.plot(time_vect, dl_vect, 'o', ms=3, color='blue')
        line21, = ax21.plot(time_vect, v1_vect, 'o', ms=3, color='red')
        line22, = ax22.plot(time_vect, v2_vect, 'o', ms=3, color='green')
        plt.tight_layout()
        fig.canvas.draw()

        j = 0
        t0 = time.time()
        t_old = t0
        i = 0
        update_dt = 0.1

        while self.run.locked_read() == True:

            t_new = time.time()
            dt = t_new - t_old

            if dt>=update_dt:

                t_old = t_new
                t = t_new - t0
                new_strain = self.strain.locked_read()
                new_dl = self.dl.locked_read()
                new_v1 = self.voltage_1.locked_read()
                new_v2 = self.voltage_2.locked_read()
                new_cap = self.cap.locked_read()
                new_sp = self.setpoint.locked_read()

                # update plot
                time_vect[j] = t
                strain_vect[j] = new_strain
                dl_vect[j] = new_dl
                v1_vect[j] = new_v1
                v2_vect[j] = new_v2
                cap_vect[j] = new_cap
                j = (j + 1) % window

                line11.set_xdata(time_vect)
                line11.set_ydata(strain_vect)
                line12.set_xdata(time_vect)
                line12.set_ydata(dl_vect)
                line21.set_xdata(time_vect)
                line21.set_ydata(v1_vect)
                line22.set_xdata(time_vect)
                line22.set_ydata(v2_vect)
                ax11.set_xlim(np.min(time_vect), np.max(time_vect))
                ax12.set_xlim(np.min(time_vect), np.max(time_vect))
                ax21.set_xlim(np.min(time_vect), np.max(time_vect))
                ax22.set_xlim(np.min(time_vect), np.max(time_vect))
                #ax11.autoscale(axis='y')
                #ax12.autoscale(axis='y')
                #ax21.autoscale(axis='y')
                #ax22.autoscale(axis='y')
                ax11.set_ylim(np.min(strain_vect)*0.8,np.max(strain_vect)*1.2)
                ax12.set_ylim(np.min(dl_vect)*0.8,np.max(dl_vect)*1.2)
                ax21.set_ylim(np.min(v1_vect)*0.8,np.max(v1_vect)*1.2)
                ax22.set_ylim(np.min(v2_vect)*0.8,np.max(v2_vect)*1.2)
                fig.suptitle(f'Setpoint: {new_sp}')
                plt.pause(0.05)
                fig.canvas.draw()
                fig.canvas.flush_events()

    def start_comms(self):
        '''
        start listening to serversocket and respond to connect requests with typical socket communications.
        '''

        print(f'Opening communication socket on {self.host} at port {self.port}')

        current_thread = threading.current_thread()
        while True:
            if current_thread.stopped()==False:
                self.serversocket.listen(1)
                print('Listening for strain client')
                conn, addr = self.serversocket.accept()
                with conn:
                    #print(f'Connected to strain client at address {addr}')
                    while True: # run main loop
                        message = conn.recv(1024)
                        #print('Receive from client initiated.')
                        if not message:
                            #print('Message received and strain client terminated connection.')
                            break
                        decoded_message = message.decode('utf8')
                        try:
                            response = self.parse_message(decoded_message)
                        except:
                            error_msg = 'Error: unable to parse message: '+str(message)
                            print(error_msg)
                            conn.sendall(error_msg.encode('utf8'))
                            break
                        try:
                            #print('Transmitting response to client')
                            conn.sendall(response.encode('utf8'))
                        except:
                            error_msg = 'Error: unable to transmit response to client.'
                            print(error_msg)
                            conn.sendall(error_msg.encode('utf8'))
                            break
            else:
                break

    def set_strain(self, setpoint):
        '''
        Ramp voltage on power supply to an approximately correct strain, returning once that strain has been achieved within tolerance. This can be proceeded by PID control.

        args:
            - setpoint(float):  strain setpoint. We take this as an explicit parameter to avoid possible conflicts and make this function cleaner.

        returns: None

        '''

        setpoint_val = self.setpoint.locked_read()
        strain_val = self.strain.locked_read()
        start_voltage = self.strain_to_voltage(setpoint_val)
        voltage_increment = 0.5
        strain_tol=0.005
        direction = (setpoint_val - strain_val)/abs(setpoint_val - strain_val)

        n=0
        while abs(strain_val) <= abs(setpoint_val):
            approx_voltage = start_voltage + n*voltage_increment
            self.ps_write(approx_voltage)

            loop_cond = True
            while loop_cond:
                v1, v2 = self.get_voltage(1), self.get_voltage(2)
                if v1 > (approx_voltage - self.ps.tol) or v1 < (approx_voltage + self.ps.tol):
                    loop_cond = False
                strain_val = self.strain.locked_read()
                if abs(strain_val) >= abs(setpoint_val):
                    loop_cond = False
            n=n+1

    def start_pid(self, setpoint, limit=False):
        '''
        Start PID loop to control strain.

        args:
            - setpoint(float):      PID setpoint. We take this as an explicit parameter to avoid possible conflicts and make this function cleaner.

        returns: None

        kwargs:
            - limit(bool):
        '''
        if limit==True:
            v1, v2 = self.ps_read()
            v0 = v1

        self.pid.setpoint = setpoint

        current_thread = threading.current_thread()
        while current_thread.stopped()==False:

            # update setpoint
            #self.pid.setpoint = self.setpoint.locked_read()

            # compute new output given current strain
            new_voltage = self.pid(self.strain.locked_read())
            if limit==True:
                dv = new_voltage - v0
                if abs(dv) > 5:
                    new_voltage = v0 + 5*(dv/abs(dv))
            #print(new_voltage)
            # set the new output and get current value
            self.ps_write(new_voltage)

    def ps_write(self, voltage):
        '''
        update both channels of power supply to new voltage. change in future to coordinate the voltages in the best way (ie, not just same voltage on each channel, maybe we just pick both?) - really, I think this function may eventually use some other data such as direction of applied voltage or setpoint-strain to determine which channel should be energized corresponding to compression or tension.
        '''
        self.set_voltage(1, voltage)
        #self.set_voltage(2, voltage)

    def set_voltage(self, channel, voltage):
        '''
        update power supply voltage within proper limits.

        args:
            - channel(int):         channel on ps to set, must be 1 or 2
            - voltage(float):       voltage to set on power supply.

        returns: None

        '''
        try:
            if not (channel==1 or channel==2):
                raise ValueError('channel must be int 1 or 2.')

            # limit max/min voltage
            if voltage > MAX_VOLTAGE:
                voltage = MAX_VOLTAGE
            elif voltage < MIN_VOLTAGE:
                voltage = MIN_VOLTAGE

            # set voltages
            if self.sim==True:
                self.ps.set_voltage(channel, voltage)
            else:
                if channel==1:
                    self.ps.voltage_1 = voltage
                elif channel==2:
                    self.ps.voltage_2 = voltage
            #print(f'Ramping voltage on channel {channel} to {voltage} V')
        except:
            print('Error: invaid voltage channel, please choose 1 or 2.')

    def get_voltage(self, channel):
        '''
        returns voltage 1 or voltage 2 from power supply.

        args: None

        returns:
            - v1(float)
            - v2(float)s

        '''
        try:
            if not (channel==1 or channel==2):
                raise ValueError('channel must be int 1 or 2.')

            if self.sim==True:
                if channel==1:
                    v = self.ps.voltage_1.locked_read()
                elif channel==2:
                    v = self.ps.voltage_2.locked_read()
            else:
                if channel==1:
                    v = self.ps.voltage_1
                elif channel==2:
                    v = self.ps.voltage_2
            return v
        except:
            print('Error: invaid voltage channel, please choose 1 or 2.')

    def set_slew_rate(self, slew_rate):
        '''
        utility to set slew rate on power supply
        '''

        print(f'Setting slew rate on power supply to {slew_rate} V/s')
        self.ps.slew_rate.locked_update(slew_rate)

    def get_strain(self):
        '''
        Querys LCR meter for current impedance measurement and uses calibration curve to return strain. Assumes that LCR measurement mode is set to one of the parallel modes, which is appropriate for measuring small capacitance (such as CPD)

        args: None

        returs:
            - strain(float):        calculated strain
            - l(float):             gap between sample plates in um
            - dl(float):            l - l0
        '''

        impedance = self.lcr.impedance # or read impedance as posted by another process
        cap = impedance[0]
        imaginary_impedance = impedance[1]
        dl = self.capacitance_to_dl(cap)
        strain = dl/self.l0_samp
        return strain, cap, imaginary_impedance, dl

    def capacitance_to_dl(self, capacitance):
        '''
        helper function that returns change gap between sample plates from initial gap (dl = l - l0) given a capacitance reading based on the CS130 capacitor calibration.

        In the future will account for the tempreature offset. That is,  we must first do cap = cap - cap_offset and then can get length with the calibration curve.

        args:
            - capacitance(float):         capacitance in pF

        returns:
            - dl(float):                  l - l0, the change in gap between sample plates from initial value in um
        '''

        # capacitor specifications
        area = 5.95e6 # um^2
        l0 = self.l0 # um
        cap_initial_value = 0.8 # pF
        response = 12e-3 # pF/um
        eps0 = 8.854e-6 # pF/um - vacuum permitivity
        cap_offset = 0.04 # pf - eventually should obtain this from a temperature calibration of "0" strain.
        # temperature calibration curve: d = eps*A/(C - offset) - x0
        dl = eps0*area/(capacitance - cap_offset) - l0 # um
        return dl

    def strain_to_voltage(self, strain):
        '''
        Helper function that returns power supply voltage estimated for a desired strain.

        args:
            - strain(float):        desired strain

        returns:
            - voltage(float):       estimated required voltage to achieve strain
        '''

        l0 = self.l0_samps
        response = 0.05 # um/V
        dl = strain*l0
        voltage = dl/response
        return voltage

    def parse_message(self, message):
        '''
        Utility that implements communications protocol with strain client. If message is not parable, error handling is handled by communication loop.

        args:
            - message(string):

        returns:
            - response(string):
        '''

        if re.match(r'SCTRL:[1-3]', message):
            mode = int(re.search(r'[1-3]', message)[0])
            if self.strain_control_loop.is_alive():
                current_mode = self.ctrl_mode.locked_read()
                if mode!=current_mode:
                    print(f'Stopping control thread in mode {current_mode} and restarting in mode {mode}')
                    self.ctrl_mode.locked_update(mode)
                    self.strain_control_loop.stop()
                    self.strain_control_loop.join()
                    self.strain_control_loop = StoppableThread(target=self.start_strain_control, args=(mode,))
                    self.strain_control_loop.start()
                else:
                    print(f'Control thread in mode {mode} already in progress, no action taken')
            else:
                print(f'Starting control thread in mode {mode}')
                self.ctrl_mode.locked_update(mode)
                self.strain_control_loop = StoppableThread(target=self.start_strain_control, args=(mode,))
                self.strain_control_loop.start()
            response = '1'
        elif message == 'ECTRL:':
            if self.strain_control_loop.is_alive():
                v1, v2 = self.get_voltage(1), self.get_voltage(2)
                self.strain_control_loop.stop()
                self.strain_control_loop.join()
                self.set_voltage(1,v1)
                self.set_voltage(1,v2)
            response = '1'
        elif message == 'STR:?':
            response = str(self.strain.locked_read())
        elif message == 'DL:?':
            response = str(self.dl.locked_read())
        elif message == 'CAP:?':
            response = str(self.cap.locked_read())
        elif re.match(r'STR:-?[0-9]+[\.]?[0-9]*', message):
            setpoint = float(re.search(r'-?[0-9]+[\.]?[0-9]*', message)[0])
            self.setpoint.locked_update(setpoint)
            response = '1'
        elif re.match(r'VOL[1-2]:\?', message):
            channel = int(re.search(r'[1-2]', message)[0])
            v = self.get_voltage(channel)
            response = str(v)
        elif re.match(r'VOL[1-2]:-?[0-9]+[\.]?[0-9]*', message):
            channel = int(re.search(r'[1-2]', message)[0])
            voltage = float(re.findall(r'-?[0-9]+[\.]?[0-9]*', message)[1])
            self.set_voltage(channel, voltage) # change ps_write to specify channel as well.
            response = '1'
        elif re.match(r'VSLW:[0-9]+[\.]?[0-9]*', message):
            slew_rate = float(re.search(r'[0-9]+[\.]?[0-9]*', message)[0])
            self.set_slew_rate(slew_rate)
            response = '1'
        elif message=='SHTDWN':
            self.run.locked_update(False)
            self.comms_loop.stop()
            response = '1'

        return response

    def do_test_loop(self):
        '''
        This test is meant to illustrate the usage of this package.
        '''
        self.initialize_instruments()

        self.setpoint.locked_update(0.02)

        # start monitoring lcr meter
        strain_read_loop = StoppableThread(target=self.start_strain_monitor)
        strain_read_loop.start()

        # start control loop
        strain_control_loop = StoppableThread(target=self.start_strain_control)
        #control_loop = StoppableThread(target=self.start_pid, args=(self.setpoint.locked_read(),))
        strain_control_loop.start()

        self.start_display()

        strain_control_loop.stop()
        strain_read_loop.stop()
        strain_control_loop.join()
        strain_read_loop.join()

    def do_main_loop(self):
        '''
        Main loop. Starts listening to client server for various commands, starting and closing threads as necessary.
        '''

        # this should not start any control explicitly, such that if anything fails and the server must be restarted, it starts off in a stable state.

        self.initialize_instruments()

        # start monitoring lcr meter
        self.strain_monitor_loop = StoppableThread(target=self.start_strain_monitor)
        self.strain_monitor_loop.start()

        # create conntrol thread
        self.strain_control_loop = StoppableThread(target=self.start_strain_control, args=(self.ctrl_mode.locked_read(),))

        # start comms
        self.comms_loop = StoppableThread(target=self.start_comms)
        self.comms_loop.start()

        # infinite loop displaying strain
        self.start_display()
        #while self.run.locked_read()==True:
        #    continue

        # close everything
        print('Shutting down strain server:')
        if self.strain_control_loop.is_alive():
            self.strain_control_loop.stop()
            self.strain_control_loop.join()
            print('\t Shut down control thread')
        print('\t Ramping voltage on all channels to 0')
        self.set_voltage(1, 0) # change to set strain to 0 once that function is refined.
        self.set_voltage(2, 0)
        if self.strain_monitor_loop.is_alive():
            self.strain_monitor_loop.stop()
            self.strain_monitor_loop.join()
            print('\t Shut down monitor thread')
        if self.comms_loop.is_alive():
            self.comms_loop.join()
        print('\t Shut down communications thread')
        print('Strain server shutdown complete')

if __name__=='__main__':
    '''
    Set SIM to True to simulate power supply and lcr meter for testing of code. Opens up communication channels and starts main server loop.
    '''

    if SIM==True:

        lcr = SimulatedLCR(0.808)
        ps = SimulatedPS(lcr)

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))

            strainserver = StrainServer(lcr, ps, s, STARTING_SETPOINT, P, I, D, L0_SAMP, l0=L0, sim=SIM)

            # start test with command line argument '-test'
            args = sys.argv
            if len(args)>=2:
                if args[1]=='-test':
                    strainserver.do_test_loop()
            else:
                strainserver.do_main_loop()

    else:

        # visa
        rm = pyvisa.ResourceManager()
        resources = rm.list_resources()
        print(resources)

        with AgilentE4980(LCR_ADDRESS) as lcr:
            with razorbillRP100(PS_ADDRESS) as ps:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind((HOST, PORT))

                    strainserver = StrainServer(lcr, ps, s, STARTING_SETPOINT, P, I, D, l0=L0, sim=SIM)

                    # start test with command line argument '-test'
                    args = sys.argv
                    if args[1]=='-test':
                        strainserver.do_test_loop()
                    else:
                        strainserver.do_main_loop()
