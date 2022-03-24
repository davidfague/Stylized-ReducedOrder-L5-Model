from neuron import h
import math
from stylized_module.point_current import Point_current

class Current_injection(Point_current):
    """A module for current injection"""
    def __init__(self,cell,sec_index,loc=0.5,pulse=True,current=[0],Dt=None,record=False,**pulse_param):
        """
        cell: target cell object
        sec_index: index of the target section in the section list
        loc: location on a section, between [0,1]
        pulse: If True, use pulse injection with keyword arguments in 'pulse_param'
               If False, use waveform data in vector 'current' as injection
        Dt: current vector time step size
        record: If True, enable recording current injection history
        """
        super().__init__(cell,sec_index,loc)
        self.pp_obj = h.IClamp(self.get_section()(loc))
        self.inj_vec = None
        self.setup(pulse,current,Dt,record,**pulse_param)
    
    def setup(self,pulse,current,Dt,record,**pulse_param):
        if pulse:
            self.setup_pulse(**pulse_param)
        else:
            self.setup_current(current,Dt)
        if record:
            self.setup_recorder()
    
    def setup_pulse(self,**pulse_param):
        """Set IClamp attributes. Argument keyword: attribute name, arugment value: attribute value"""
        for param,value in pulse_param.items():
            setattr(self.pp_obj,param,value)
    
    def setup_current(self,current,Dt):
        """Set current injection with the waveform in vector 'current'"""
        ccl = self.pp_obj
        ccl.dur = 0
        ccl.dur = h.tstop if hasattr(h,'tstop') else 1e30
        if Dt is None:
            Dt = h.dt
        self.inj_vec = h.Vector()
        self.inj_vec.from_python(current)
        self.inj_vec.append(0)
        self.inj_vec.play(ccl._ref_amp,Dt)
