'''
An example notebook to demonstrate usage within a Jupyter Notebook.
'''

from strain_control.strain_client import StrainClient

sc = StrainClient()
#sc.transmit('0.075')
sc.start_strain_control('PID')
sc.set_setpoint(0.08)
sc.get_voltage(1)
sc.get_strain()
sc.get_cap()
sc.set_slew_rate(0.1)
sc.set_output(1,1)
sc.get_output(1)
sc.set_voltage(1,0)
sc.set_voltage_limits(1, -5,5)
sc.set_pid(100,100,0.1)
sc.set_sample_l0(68)
sc.stop_strain_control()
sc.shutdown_server(1)
