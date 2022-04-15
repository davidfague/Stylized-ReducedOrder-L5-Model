from neuron import h
import math
import numpy as np

class Point_current(object):
    """A module for current injection"""
    def __init__(self,cell,sec_index,loc=0.5):
        """
        cell: target cell object
        sec_index: index of the target section in the section list
        loc: location on a section, between [0,1]
        """
        self.cell = cell
        self.sec_index = sec_index
        self.pp_obj = None  # point process object
        self.rec_vec = None  # vector for recording
    
    def setup(self,record=None):
        pass
    
    
    def setup_recorder(self):
        size = [round(h.tstop/h.dt)+1] if hasattr(h,'tstop') else []
        try:
            self.rec_vec = h.Vector(*size).record(self.pp_obj._ref_igaba)
        except:
            self.rec_vec = MultiSynCurrent()
            vec_inmda = h.Vector(*size).record(self.pp_obj._ref_inmda)
            vec_iampa = h.Vector(*size).record(self.pp_obj._ref_iampa)
            self.rec_vec.add_vec(vec_inmda)
            self.rec_vec.add_vec(vec_iampa)
    
    def get_section(self):
        return self.cell.all[self.sec_index]
    
    def get_segment(self):
        return self.pp_obj.get_segment()
    
    def get_segment_id(self):
        """Get the index of the injection target segment in the segment list"""
        iseg = math.floor(self.get_segment().x*self.get_section().nseg)
        return self.cell.sec_id_in_seg[self.sec_index]+iseg

class MultiSynCurrent(object):
    def __init__(self):
        self.vec_list = []
    
    def add_vec(self,vec):
        self.vec_list.append(vec)
    
    def as_numpy(self):
        return np.sum(np.array([vec.as_numpy() for vec in self.vec_list]), axis=0)