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
        self.allsections = [] # list of sections
        self.all = h.SectionList() # SectionList hoc object
        self.somatic = h.SectionList()
        self.apical = h.SectionList()
        self.basal = h.SectionList()
        self.axonal = h.SectionList() # list of axonal sections
        self.apic = []
        self.dend = []
        self.axon = []
        self.soma = []
        self.segments = []  # list of all segments
        self.sec_id_lookup = {} # dictionary from section type id to section index
        self.sec_id_in_seg = []
        self.seg_coords = {}
        self.injection = []
        self.set_geometry(geometry)
        self.setup_all()

    def setup_all(self):
        if self.geometry is not None:
            self.__create_morphology()
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

    def __create_morphology(self) -> None:
        """Create cell morphology"""
        if self.geometry is None:
            raise ValueError("Warning: geometry is not loaded.")
        self._nsec = 0
        for sec_id, sec in self.geometry.iterrows():
            start_idx = self._nsec
            nbranch = sec['nbranch']
            rot = 2 * math.pi / nbranch
            if sec_id == 0:
                r0 = sec['R']
                pt0 = [0., -2 * r0, 0.]
                pt1 = [0., 0., 0.]
                soma = self.__create_section(name=sec['name'], diam=2 * r0,sectype=sec['type'])
                self.soma.append(soma)
                self.__set_location(self.soma[0], pt0, pt1, 1)
            else:
                print('now attempting sec_id:',sec_id)
                length = sec['L']
                radius = sec['R']
                ang = sec['ang']
                nseg = math.ceil(length / self._dL)
                pid = self.sec_id_lookup[sec['pid']]
                if sec['axial']:
                    nbranch = 1
                    x = 0
                    y = length*((ang>=0)*2-1)
                else:
                    nbranch = sec['nbranch']#nbranch = self._nbranch
                    x = length * math.cos(ang)
                    y = length * math.sin(ang)
                    if len(pid) == 1:
                        pid = pid*nbranch
                for i in range(nbranch):
                    print('allsections:',self.allsections,'| pid[i]:',pid[i],'| psec:',self.allsections[pid[i]])
                    psec = self.allsections[pid[i]]
                    pt0 = [psec.x3d(1), psec.y3d(1), psec.z3d(1)]
                    pt1[1] = pt0[1] + y
                    pt1[0] = pt0[0] + x * math.cos(i * rot)
                    pt1[2] = pt0[2] + x * math.sin(i * rot)
                    section = self.__create_section(name=sec['name'], diam=2 * radius,sectype=sec['type'],nbranch_index=i)
                    section.connect(psec(1), 0)
                    self.__set_location(section, pt0, pt1, nseg)
            self.sec_id_lookup[sec_id] = list(range(start_idx, self._nsec))
        self.__set_location(self.soma[0], [0., -r0, 0.], [0., r0, 0.], 1)
        self.__store_segments()

    def __create_section(self,name='null_sec',diam=500.0,sectype=int,nbranch_index=int):
        if sectype==4:
            name=name+'['+str(nbranch_index)+'].'+'apic['+str(len(list(self.apical)))+']'
            sec = h.Section(name=name)
            sec.diam = diam
            self.apical.append(sec)
            self.apic.append(sec)
        elif sectype==3:
            name=name+'['+str(nbranch_index)+'].'+'dend['+str(len(list(self.basal)))+']'
            sec = h.Section(name=name)
            sec.diam = diam
            self.basal.append(sec)
            self.dend.append(sec)
        elif sectype==2:
            name=name+'['+str(nbranch_index)+'].'+'axon['+str(len(list(self.axonal)))+']'
            sec = h.Section(name=name)
            sec.diam = diam
            self.axonal.append(sec)
            self.axon.append(sec)
        elif sectype==1:
            name=name+'['+str(len(list(self.somatic)))+']' #soma[0]
#             name=name+str(len(list(self.somatic))) #soma0
            sec=h.Section(name=name)
            sec.diam=diam
            self.somatic.append(sec)
        self.allsections.append(sec)
        self.all.append(sec)
        self._nsec += 1
        return sec

    def __set_location(self,sec,pt0,pt1,nseg):
        sec.pt3dclear()
        sec.pt3dadd(*pt0,sec.diam)
        sec.pt3dadd(*pt1,sec.diam)
        sec.nseg = nseg

    def __store_segments(self):
        self.segments = []
        self.sec_id_in_seg = []
        nseg = 0
        for sec in self.allsections:
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
        for isec,sec in enumerate(self.allsections):
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
            nc = h.NetCon(self.soma[0](0.5)._ref_v, None, sec=self.soma[0])
            nc.threshold = threshold
            nc.record(vec)
            self.spikes = vec
            self._record_spike = True

    def get_sec_by_id(self,index=None):
        """Get section(s) objects by index(indices) in the section list"""
        if not hasattr(index,'__len__'):
            sec = self.allsections[index]
        else:
            sec = [self.allsections[i] for i in index]
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
        for sec in self.allsections:
            sec.cm = 1.0
            sec.insert('pas')
            sec.g_pas = gl
            sec.e_pas = self._vrest

    def add_injection(self,sec_index,**kwargs):
        """Add current injection to a section by its index"""
        self.injection.append(Current_injection(self,sec_index,**kwargs))
