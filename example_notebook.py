'''
An example notebook to demonstrate usage within a Jupyter Notebook.
'''

from strain_control.strain_client import StrainClient

sc = StrainClient()
#sc.transmit('0.075')
sc.start_strain_control('PID')
sc.start_cap_control('Set Cap')
sc.set_setpoint(0.78)
sc.get_voltage(1)
sc.get_strain()
sc.get_cap()
sc.set_slew_rate(.1)
sc.set_output(1,1)
sc.get_output(1)
sc.set_voltage(1,0)
sc.set_voltage_limits(2, -20,120)
sc.set_pid(100000,1000,1)
sc.set_sample_l0(68)
sc.stop_cap_control()
sc.set_ps(0)
sc.get_ps()
sc.set_cap(0.78)
sc.shutdown_server(1)

import re
message = 'VLIMS1:-12,120'
[float(i) for i in re.findall(r'-?[0-9]+[\.]?[0-9]*', message)[1:]]
