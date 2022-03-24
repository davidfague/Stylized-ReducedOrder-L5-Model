from neuron import h
import numpy as np

class Recorder(object):
    """A module for recording variables"""
    def __init__(self,obj_list,var_name='v'):
        """
        obj_list: list of (or a single) target objects
        var_name: string of variable to be recorded
        """
        if not isinstance(obj_list, (list,tuple,np.ndarray)):
            obj_list = [obj_list]
            self.single = True
        else:
            self.single = False
        self.obj_list = obj_list
        self.var_name = var_name
        self.vectors = []
        self.setup_recorder()
        
    def setup_recorder(self):
        size = [round(h.tstop/h.dt)+1] if hasattr(h,'tstop') else []
        attr_name = '_ref_'+self.var_name
        for obj in self.obj_list:
            self.vectors.append( h.Vector(*size).record( getattr(obj,attr_name) ) )
    
    def as_numpy(self):
        """
        Return a numpy 2d-array of recording, n objects-by-time
        Return a 1d-array if a single object is being recorded
        """
        if self.single:
            x = self.vectors[0].as_numpy()
        else:
            x = np.array([v.as_numpy() for v in self.vectors])
        return x
