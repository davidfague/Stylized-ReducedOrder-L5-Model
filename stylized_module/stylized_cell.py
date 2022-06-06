from abc import ABC, abstractmethod
from neuron import h
import math
import numpy as np
import pandas as pd
from typing import List, Optional, Sequence, Dict, Union, TypeVar

from stylized_module.current_injection import Current_injection

h.load_file('stdrun.hoc')

class Stylized_Cell(object):
    def __init__(self,geometry=None,dL=30,vrest=-70.0,nbranch=4):
        """
        Initialize cell model
        geometry: pandas dataframe of cell morphology properties
        dL: maximum segment length
        vrest: reversal potential of leak channel for all segments
        nbranch: number of branches of each non axial section
        """
        self._h = h
        self._dL = dL
        self._vrest = vrest
        self._nbranch = max(nbranch,2)
        self._nsec = 0
        self._nseg = 0
        self.all = []  # list of all sections
        self.segments = []  # list of all segments
        self.sec_id_lookup = {} # dictionary from section type id to section index
        self.sec_id_in_seg = []
        self.seg_coords = {}
        self.injection = []
        self.set_geometry(geometry)
        self.setup_all()

    def setup_all(self):
        if self.geometry is not None:
            self.set_morphology()
            self.set_channels()

    def set_geometry(self,geometry):
        if geometry is None:
            self.geometry = None
        else:
            if not isinstance(geometry,pd.DataFrame):
                raise TypeError("geometry must be a pandas dataframe")
            if geometry.iloc[0]['type']!=1:
                raise ValueError("first row of geometry must be soma")
            self.geometry = geometry.copy()

    def set_morphology(self):
        """Create cell morphology"""
        if self.geometry is None:
            print("Warning: geometry is not loaded.")
            return None
        self._nsec = 0
        self.all = []
        rot = 2*math.pi/self._nbranch
        for id,sec in self.geometry.iterrows():
            start_idx = self._nsec
            if id==0:
                R0 = sec['R']
                pt0 = [0.,-2*R0,0.]
                pt1 = [0.,0.,0.]
                self.soma = self.add_section(name=sec['name'],diam=2*R0)
                self.set_location(self.soma,pt0,pt1,1)
            else:
                L = sec['L']
                R = sec['R']
                ang = sec['ang']
                nseg = math.ceil(L/self._dL)
                pid = self.sec_id_lookup[sec['pid']][0]
                psec = self.all[pid]
                pt0 = [psec.x3d(1),psec.y3d(1),psec.z3d(1)]
                if sec['axial']:
                    nbranch = 1
                    X = 0
                    pt1[1] = pt0[1]+L
                else:
                    nbranch = self._nbranch
                    X = L*math.cos(ang)
                    pt1[1] = pt0[1]+L*math.sin(ang)
                for i in range(nbranch):
                    pt1[0] = pt0[0]+X*math.cos(i*rot)
                    pt1[2] = pt0[2]+X*math.sin(i*rot)
                    section = self.add_section(name=sec['name'],diam=2*R)
                    section.connect(psec(1),0)
                    self.set_location(section,pt0,pt1,nseg)
            self.sec_id_lookup[id] = list(range(start_idx,self._nsec))
        self.set_location(self.soma,[0.,-R0,0.],[0.,R0,0.],1)
        self.store_segments()

    def add_section(self,name='null_sec',diam=500.0):
        sec = h.Section(name=name)
        sec.diam = diam
        self.all.append(sec)
        self._nsec += 1
        return sec

    def set_location(self,sec,pt0,pt1,nseg):
        sec.pt3dclear()
        sec.pt3dadd(*pt0,sec.diam)
        sec.pt3dadd(*pt1,sec.diam)
        sec.nseg = nseg

    def store_segments(self):
        self.segments = []
        self.sec_id_in_seg = []
        nseg = 0
        for sec in self.all:
            self.sec_id_in_seg.append(nseg)
            nseg += sec.nseg
            for seg in sec:
                self.segments.append(seg)
        self._nseg = nseg

    def calc_seg_coords(self):
        """Calculate segment coordinates for ECP calculation"""
        p0 = np.empty((self._nseg,3))
        p1 = np.empty((self._nseg,3))
        p05 = np.empty((self._nseg,3))
        r = np.empty(self._nseg)
        for isec,sec in enumerate(self.all):
            iseg = self.sec_id_in_seg[isec]
            nseg = sec.nseg
            pt0 = np.array([sec.x3d(0),sec.y3d(0),sec.z3d(0)])
            pt1 = np.array([sec.x3d(1),sec.y3d(1),sec.z3d(1)])
            pts = np.linspace(pt0,pt1,2*nseg+1)
            p0[iseg:iseg+nseg,:] = pts[:-2:2,:]
            p1[iseg:iseg+nseg,:] = pts[2::2,:]
            p05[iseg:iseg+nseg,:] = pts[1:-1:2,:]
            r[iseg:iseg+nseg] = sec.diam/2
        self.seg_coords = {}
        self.seg_coords['dl'] = p1-p0  # length direction vector
        self.seg_coords['pc'] = p05  # center coordinates
        self.seg_coords['r'] = r  # radius

    def set_spike_recorder(self, threshold: Optional[float] = 0) -> None:
        if threshold is None:
            self.spikes = None
            self._record_spike = False
        else:
            vec = h.Vector()
            nc = h.NetCon(self.soma(0.5)._ref_v, None, sec=self.soma)
            nc.threshold = threshold
            nc.record(vec)
            self.spikes = vec
            self._record_spike = True

    def get_sec_by_id(self,index=None):
        """Get section(s) objects by index(indices) in the section list"""
        if not hasattr(index,'__len__'):
            sec = self.all[index]
        else:
            sec = [self.all[i] for i in index]
        return sec

    def get_seg_by_id(self,index=None):
        """Get segment(s) objects by index(indices) in the segment list"""
        if not hasattr(index,'__len__'):
            seg = self.segments[index]
        else:
            seg = [self.segments[i] for i in index]
        return seg

    def set_channels(self):
        """Abstract method for setting biophysical properties, inserting channels"""
        pass

    def set_all_passive(self,gl=0.0003):
        """A use case of 'set_channels', set all sections passive membrane"""
        for sec in self.all:
            sec.cm = 1.0
            sec.insert('pas')
            sec.g_pas = gl
            sec.e_pas = self._vrest

    def add_injection(self,sec_index,**kwargs):
        """Add current injection to a section by its index"""
        self.injection.append(Current_injection(self,sec_index,**kwargs))
