'''
Strain client indended to be run from jupyter notebook.
'''
import socket
import time

##########################
### USER SETTINGS HERE ###
##########################
global HOST, PORT

### COMMUNICATION SETTINGS ###
HOST = 'localhost'
PORT = 15200

###########################
###########################
###########################


class StrainClient:

    def __init__(self):
         self.host = HOST
         self.port = PORT

    def transmit(self, message):
        '''
        Implement basic query of strain server.

        args:
            - message(string):      message to send to strain server

        returns:
            - response:             response from server.

        '''

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.connect((self.host, self.port))
            except:
                raise RuntimeError('unable to connect to strain server.')
            try:
                message = str(message).encode('utf-8')
                s.sendall(message)
            except:
                raise RuntimeError('unable to send query to strain server.')
            try:
                response = s.recv(1024)
            except:
                raise RuntimeError('unable to recieve response from strain server.')

        time.sleep(0.1)

        #print(f'Received: {response}')
        return response.decode('utf8')

    def start_strain_control(self, channel=1):
        '''
        initiate control loop on strain server.

        returns:
            - response:     '1' if successful

        kwargs:
            - channel(int):     1: channel 1, 2: channel2, 3: both
        '''
        message = 'SCTRL:'+str(channel)
        response = self.transmit(message)
        return response

    def start_cap_control(self, channel=1):
        '''
        initiate control loop on strain server.

        returns:
            - response:     '1' if successful

        kwargs:
            - channel(int):     1: channel 1, 2: channel2, 3: both
        '''
        message = 'SCAPCTRL:'+str(channel)
        response = self.transmit(message)
        return response

    def stop_strain_control(self):
        '''
        stop control loop on strain server.

        returns:
            - response:     '1' if successful
        '''

        message = 'ECTRL:'
        response = self.transmit(message)
        return response

    def stop_cap_control(self):
        '''
        stop control loop on strain server.

        returns:
            - response:     '1' if successful
        '''

        message = 'ECAPCTRL:'
        response = self.transmit(message)
        return response

    def get_strain(self):
        '''
        Get current strain from cell.

        args: None

        returns:
            - strain(float):        strain
        '''

        message = 'STR:?'
        strain = float(self.transmit(message))
        return strain

    def get_dl(self):
        '''
        Get current gap from cell.

        args: None

        returns:
            - dl(float):        strain
        '''

        message = 'DL:?'
        dl = float(self.transmit(message))
        return dl

    def get_cap(self):
        '''
        Get current capacitance from cell.

        args: None

        returns:
            - cap(float):        strain
        '''

        message = 'CAP:?'
        cap = float(self.transmit(message))
        return cap

    def get_voltage(self, channel):
        '''
        read voltage on given channel.

        args:
            - channel(int):     channel 1 or 2 on power supply

        returns:
            - voltage(float):
        '''

        if not(channel==1 or channel==2):
            raise ValueError('Invalid power supply voltage channel, please choose either 1 or 2.')
        message = 'VOL'+str(channel)+':?'
        voltage = float(self.transmit(message))
        return voltage

    def get_ps(self):
        '''
        reads total voltage on power supply.

        args: None

        returns:
            - voltage(float):
        '''
        message = 'PS:?'
        voltage = float(self.transmit(message))
        return voltage

    def set_setpoint(self, new_setpoint):
        '''
        change target strain setpoint of control loop.

        args:
            - new_setpoint:     setpoint to set

        returns:
            - response:         '1' if successful
        '''

        message = f'SETPT:{new_setpoint:f}'
        response = self.transmit(message)
        return response

    def set_output(self, channel, state):
        '''
        enable or disable output voltage on given channel.

        args:
            - channel(int):     channel 1 or 2 on power supply
            - state(int):       0 or 1

        returns:
            - response:         '1' if successful
        '''
        if not(channel==1 or channel==2):
            raise ValueError('Invalid power supply voltage channel, please choose either 1 or 2.')
        if not(int(state)==0 or int(state)==1):
            raise ValueError('Invalid power supply state, please choose either 0 or 1.')
        message = 'OUT'+str(channel)+':'+str(state)
        response = self.transmit(message)
        return response

    def get_output(self, channel):
        '''
        sets voltage explicitly on channel 1 or 2

        args: None

        returns:
            - state:         0 or 1
        '''
        if not(channel==1 or channel==2):
            raise ValueError('Invalid power supply voltage channel, please choose either 1 or 2.')
        message = 'OUT'+str(channel)+':?'
        response = self.transmit(message)
        return response

    def set_ps(self, voltage):
        '''
        sets ps voltage as given by set_ps()

        args:
            - voltage(float):   voltage to set

        returns:
            - response:         '1' if successful
        '''

        message = f'PS:{voltage:f}'
        response = self.transmit(message)
        return response

    def set_voltage(self, channel, voltage):
        '''
        sets voltage explicitly on channel 1 or 2

        args:
            - channel(int):     channel 1 or 2 on power supply
            - voltage(float):   voltage to set

        returns:
            - response:         '1' if successful
        '''

        if not(channel==1 or channel==2):
            raise ValueError('Invalid power supply voltage channel, please choose either 1 or 2.')
        message = 'VOL'+str(channel)+':'+f'{voltage:f}'
        response = self.transmit(message)
        return response

    def set_voltage_limits(self, channel, min, max):
        '''
        sets voltage limits on channel 1 or 2

        args:
            - channel(int):     channel 1 or 2 on power supply
            - min(float):       min voltage
            - max(float):       max voltage

        returns:
            - response:         '1' if successful
        '''

        if not(channel==1 or channel==2):
            raise ValueError('Invalid power supply voltage channel, please choose either 1 or 2.')
        message = f'VLIMS{channel}:{min},{max}'
        response = self.transmit(message)
        return response

    def set_sample_l0(self, samp_l0):
        '''
        sets 0 strain sample length for correctly calculating strain.

        args:
            - samp_l0(float):   0 strain length of sample in microns

        returns:
            - response:         '1' if successful
        '''
        message = f'SAMPL0:{samp_l0:f}'
        response = self.transmit(message)
        return response

    def set_cap0(self, cap0):
        '''
        sets 0 strain capacitance.

        args:
            - cap0(float):   0 strain capactiance in pF

        returns:
            - response:         '1' if successful
        '''
        message = f'CAP0:{cap0:f}'
        response = self.transmit(message)
        return response

    def set_pid(self, p, i, d):
        '''
        Set PID parameters.

        args:
            - p(float):
            - i(float):
            - d(float):

        returns:
            - response(str):            '1' if successful
        '''
        message = f'PID:{p:f},{i:f},{d:f}'
        response = self.transmit(message)
        return response

    def set_slew_rate(self, slew_rate):
        '''
        change voltage ramp rate on power supply:

        args:
            - slew_rate(float):    slew rate in V/s

        returns:
            - response:             '1' if successful
        '''

        message = f'VSLW:{slew_rate:f}'
        response = self.transmit(message)
        return response

    def shutdown_server(self, mode=0):
        '''
        Terminates strain server, correctly shutting down the system and leaving it in a stable, safe state (by default, all voltages ramped to 0 and communications ports closed properly).

        returns:
            - response(str):            '1' if successful

        kwargs:
            -mode(int):                1 to ramp voltages down to 0 and 0 to leave state of system as is.
        '''

        if not(mode==0 or mode==1):
            raise ValueError('Invalid shutdown mode. Please input either 0 to leave system in current state or 1 to ramp voltages to 0.')
        message = f'SHTDWN:{mode}'
        response = self.transmit(message)
        return response
