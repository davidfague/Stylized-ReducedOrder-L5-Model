import h5py
import numpy as np
import scipy.signal as ss
import neuron as nrn
from stylized_module.recorder import Recorder
from neuron import h
import pandas as pd

#Additional functions
def lognormal(m, s):
    """get underlying normal distribution parameters from lognormal distribution parameters"""
    mean = np.log(m) - 0.5 * np.log((s/m)**2+1)
    std = np.sqrt(np.log((s/m)**2 + 1))
    #return max(np.random.lognormal(mean, std, 1), 0.00000001)
    return mean, std


def normalize(x):
    """normalizes the given array"""
    return (x - np.min(x))/(np.max(x)-np.min(x))

def make_noise(num_traces=100,num_samples=4999):
    """Creates a noise trace used in generating spike rasters.
    Parameters
    ----------
    num_traces : int, optionalj
        number of noise traces to create (first dimension), by default 100
    num_samples : int, optional
        length of the trace (second dimension), by default 4999
    Returns
    -------
    np.array
        noise trace
    """    
    B = [0.049922035, -0.095993537, 0.050612699, -0.004408786]
    A = [1, -2.494956002,   2.017265875,  -0.522189400]
    invfn = np.zeros((num_traces,num_samples))
    for i in np.arange(0,num_traces):
        wn = np.random.normal(loc=1,
        scale=0.5,size=num_samples+2000)
        invfn[i,:] = normalize(ss.lfilter(B, A, wn)[2000:])+0.5                             # Create '1/f' Noise
    return invfn
def shift_exc_noise(ts, nid, milliseconds, time_shift=4):
    """Creates a shifted, min-max normalized average traces of the given spike raster.
    Parameters
    ----------
    ts : list
        times (float) where spikes occur
    nid : int
        node id associated with each spike
    seconds : float
        length of the raster in seconds
    time_shift : int, optional
        how many ms to shift the average trace by, by default 4
    Returns
    -------
    [type]
        [description]
    """    
    h = np.histogram(ts,bins=np.arange(0,milliseconds,1))

    fr_prof = h[0]/(0.001*(np.max(nid)+1))
    wrap = fr_prof[-4:]
    fr_prof[4:] = fr_prof[0:-4]
    fr_prof[0:4] = wrap

    fr_prof = normalize(fr_prof)+0.5
    return fr_prof
  
class SonataWriter:
    """Class used to dynamically writing spike rasters to an h5 file.
    Attributes
    ----------
    file : h5py.File
        file object being worked on
    group : h5py.Group
        gropu where the datasets reside
    datasets : dict
        datasets that are saved to the file
    Methods
    -------
    append_ds(vals, ds)
        appends the given values to the end of the given dataset
    append_repeat(ds, val, N)
        appends the given value N times to the end of the given dataset
    close()
        close the h5py file
    """
    def __init__(self, f_name, groups, datasets, types):
        """
        Parameters
        ----------
        f_name : str
            name of file locationmake
        groups : list
            list of group names (str) that are layered into the h5py file
            in the order given.
        datasets : list
            list of dataset names (str)
        types : list
            list of data types that corresponds to the datasets list
        """        
        self.file = h5py.File(f_name, 'w')

        self.group = self.file
        for group in groups:
            self.group = self.group.create_group(group)

        self.datasets = {}
        for i, ds in enumerate(datasets):
            self.datasets[ds] = self.group.create_dataset(ds, data=[], dtype=types[i], chunks=True, maxshape=(None,))

    def append_ds(self, vals, ds):
        """appends the given values to the end of the given dataset
        Parameters
        ----------
        vals : list
            list of values to be appended to the dataset
        ds : str
            key of the dataset to append to
        """        
        length = len(self.datasets[ds])
        self.datasets[ds].resize((length + len(vals), ))
        self.datasets[ds][length:] = vals

    def append_repeat(self, ds, val, N):
        """appends the given value N times to the end of the given dataset
        Parameters
        ----------
        ds : str
            key of the dataset to append to
        val : [type]
            value to be appended N times
        N : int
            number of vals to append to the dataset
        """        
        self.append_ds([val for i in range(N)], ds)

    def close(self):
        """Closes the h5py File
        """        
        self.file.close()
        
class Record_Axial_Current(object):
    """A module for recording axial currents from section object to segments attached to section object"""
    def __init__(self, section, dend_type: str = None, record_t: bool = False, single_seg: bool = False) -> None:
        """
        section: section object
        dend_type: list of section names of the dendrite types that need to be recorded
        record_t: whether or not to record time points
        single_seg: whether or not to record only one segment for each dendrite type
        """
        self.section = section
        if dend_type is None:
            # import pdb; pdb.set_trace()
            sec_names = [sec.name().split('.')[-1].split('[')[0] for sec in section.children()]
            if section.parentseg() is not None: # check if section has a parent section
              sec_names.append(section.parentseg().sec.name().split('.')[-1].split('[')[0]) #get name of parent section
            self.dend_type = list(set(sec_names))
        else:
            self.dend_type = dend_type
        self.dend = {};
        # import pdb; pdb.set_trace()
        for d in self.dend_type:
            self.dend[d] = Adjacent_Section(self.section,d)
        self.single_seg = single_seg
        self.setup_recorder(record_t)
        
    
    def setup_recorder(self, record_t: bool = False):
        if record_t:
            self.t_vec = h.Vector(round(h.tstop / h.dt) + 1).record(h._ref_t)
        else:
            self.t_vec = None
        for dend in self.dend.values():
            dend.setup_recorder(self.single_seg)
    
    def t(self):
        if self.t_vec is None:
            t = None
        else:
            t = self.t_vec.as_numpy().copy()
        return t
    
    def get_current(self, dend_type: str = None) -> np.ndarray:
        if dend_type is None:
            axial_current = {}
            for name,dend in self.dend.items():
                axial_current[name] = dend.get_current()
        else:
            axial_current = self.dend[dend_type].get_current()
        return axial_current

class Adjacent_Section(object):
    """A module for recording and calculating axial current from the soma to its adjacent sections of a dendrite type"""
    def __init__(self, section, name: str = 'dend') -> None:
        """
        section: section section object
        name: section names of the dendrite type
        """
        self.name = name
        self.section=section
        #calculate section children seg to section seg
        self.init_sec = [s for s in section.children() if name in s.name()] #get array of section children and check if they are in the sec_names list

        if (section.parentseg() is not None):
          if (name == section.parentseg().sec.name().split('.')[-1].split('[')[0]): # In this case we have passed no children sections and the name of the parent section is passed
            self.init_sec = [section.parentseg().sec] #set parent section as initial section
            self.init_seg= section.parentseg() # set inital segment as parent segment
        # import pdb; pdb.set_trace()
        self.nseg = [s.nseg for s in self.init_sec]
        self.init_seg = [s(0.5/n) for s,n in zip(self.init_sec,self.nseg)] #calculate middle of the first segment in the section for every child section
        # #calculate section to parent section
        # self.parent_seg=section.parentseg()
        # self.first_seg=section(0.5/section.nseg)
    
    def setup_recorder(self, single_seg: bool = False):
        self.pre_seg = [s.parentseg() for s in self.init_sec]
        if (self.section.parentseg() is not None):
          if self.init_sec == [self.section.parentseg().sec]: #if parent section
            self.pre_seg = [self.section((1+((1-self.section.nseg)/self.section.nseg))/2)] #set pre_seg to last segment of section
        if len(set(self.pre_seg)) == 1 and len(self.pre_seg)>1:
            self.pre_seg = [self.pre_seg[0]]
        if single_seg:
            self.init_seg = [self.init_seg[0]]
        self.pre_v = Recorder(self.pre_seg)
        self.post_v = Recorder(self.init_seg)
    
    def get_current(self) -> np.ndarray:
        v_pre = self.pre_v.as_numpy()
        v_post = self.post_v.as_numpy()
        axial_r = np.array([[seg.ri()] for seg in self.init_seg])
        if (self.section.parentseg() is not None):
          if self.init_sec == [self.section.parentseg().sec]: #in the case of section connected to parent section:
            axial_r= np.array([[seg.ri()] for seg in self.pre_seg]) #ri() calculates from seg to parentseg() so adjust to go from pre_seg to init_seg rather than init_seg to pre_seg
        axial_current = (v_pre-v_post)/axial_r
        return axial_current
def save_degrees(cell):
    degrees = {}
    calculate_degree(h.SectionRef(sec=cell.soma), degrees, 0)

    df_dict = {}
    df_dict["SectionName"] = list(degrees.keys())
    df_dict["Degrees"] = list(degrees.values())
    
    df = pd.DataFrame(df_dict)
    try:
      os.remove("ReducedSegmentDegrees.csv")
    except:
      print('No ReducedSegmentDegrees.csv to remove.')
    df.to_csv("ReducedSegmentDegrees.csv", index=False)

def calculate_degree(sref, degrees, deg):
    degrees[sref.sec.name()] = deg

    for c in sref.child:
        calculate_degree(h.SectionRef(sec=c), degrees, deg+1)

def make_seg_df(cell, filename=None):
    if filename==None:
        filename='ReducedSegmentDegrees.csv'
    try: secdegrees=pd.read_csv(filename)
    except: print(filename, 'not in directory. Try another filename kwarg or add the file to the directory')
    df = pd.DataFrame()
    i = 0
    j = 0
    sec_types_bysec=[]
    lens = []
    seg_lens = []
    diams = []
    segdiams = []
    bmtk_ids = []
    seg_ids = []
    sec_ids = []
    full_names = []
    xs = []
    parts = []
    distances = []
    elec_distances = []
    elec_distances_nexus = []
    h.distance(sec=cell.all[0])
    nsegs=[]
    RAs=[]
    Parentx=[]
    zz = h.Impedance()
    zz.loc(cell.all[0](0.5))
    zz.compute(25)
    ww = h.Impedance()
    ww.loc(cell.all[19](0.5)) #disttrunk
    ww.compute(25)
    psegids=[]
    segments=[]
    sections=[]
    fullsecnames=[]
    seg_degrees=[]
    apic = ['proxtrunk','midtrunk','disttrunk','proxtuft','midtuft','disttuft','oblique']
    dend = ['proxbasal','midbasal','distbasal']

    for sec in cell.all:
        print(sec.name)
        for seg in sec:

            lens.append(seg.sec.L)
            seg_lens.append(seg.sec.L/seg.sec.nseg)
            diams.append(seg.sec.diam)
            segdiams.append(seg.diam)
            distances.append(h.distance(seg))
            bmtk_ids.append(i)
            seg_ids.append(j)
            xs.append(seg.x)
            fullsecname = sec.name()
            fullsecnames.append(fullsecname)
            degree=secdegrees.loc[secdegrees.SectionName==fullsecname]['Degrees']
            seg_degrees.append(int(degree))
            if str(fullsecname)=='soma':
              sec_type = 'soma'
            elif str(fullsecname) in apic:
              sec_type = 'apic'
            elif str(fullsecname) in dend:
              sec_type = 'dend'
            else:
              sec_type = str(fullsecname)
            sec_id=sec_types_bysec.count(sec_type)
            sec_ids.append(sec_id)
            nsegs.append(seg.sec.nseg)
            RAs.append(seg.sec.Ra)
            parts.append(sec_type)
            full_names.append(str(seg))
            elec_distances.append(zz.ratio(seg))
            elec_distances_nexus.append(ww.ratio(seg))
            #if seg.sec.parentseg() is not None:
              #get parentseg coordinates or something to identify parentseg by
            j += 1
            segments.append(seg)
        sec_types_bysec.append(sec_type)
        i += 1
        sections.append(sec)
    #print(segments)
    for i in range(len(segments)): #calculate parentseg id using seg index on section
      idx = int(np.floor(segments[i].x * segments[i].sec.nseg)) #get seg index on section
      #case where segment is not first segment on section:
      if idx != 0: #if the segment is not the first on the section then the parent segment is the previous segment index
        psegid=i-1 #set parent segment id to the previous segment index
        psegids.append(psegid)
      #case where segment is first segment on section:
      else:
        pseg = segments[i].sec.parentseg()
        if pseg == None:
          psegids.append(None)
        else:
          psec=pseg.sec
          nseg = psec.nseg
          pidx = int(np.floor(pseg.x * nseg)) #get parent seg index on section
          if pseg.x == 1.:
            pidx -= 1
          psegid=segments.index(psec((pidx + .5) / nseg)) #find the segment id of the parent seg by comparing with segment list after calculating the parent seg's section(x)
          psegids.append(psegid)


    numSyn = len(cell.injection)
    nseg = len(sim.cells[0].segments)
    excSynPerSeg = [0]*nseg
    inhSynPerSeg = [0]*nseg
    excSynPerSegL = [0]*nseg
    inhSynPerSegL = [0]*nseg
    SynParentSeg = []
    SourcePop = []
    SynType = []
    SynDist = []

    i_NMDA_bySeg= [[0] * (numTstep+1) ] * nseg

    #print(len(sim.cells[0].injection))

    for j in range(numSyn):
      seg = cell.injection[j].get_segment_id() 
      SynParentSeg.append(seg)
      SynType.append(AllSegType[seg])
      SynDist.append(AllSegDist[seg])

      if(cell.injection[j].syntype == 'exc'):
        excSynPerSeg[seg] += 1
        SourcePop.append('exc_stim')
      else:
        inhSynPerSeg[seg] += 1
        SourcePop.append('dist_inh_stim')
    df["segmentID"] = seg_ids
    df["BMTK ID"] = bmtk_ids
    df["Seg_L"] = seg_lens
    df["Seg_diam"] = segdiams
    df["X"] = xs
    df["Type"] = parts
    df["Sec ID"] = sec_ids
    df["Distance"] = distances
    df["Section_L"] = lens
    df["Section_diam"] = diams
    df["Section_nseg"] = nsegs
    df["Section_Ra"] = RAs
    df["Coord X"] = AllSegXCoord
    df["Coord Y"] = AllSegYCoord
    df["Coord Z"] = AllSegZCoord
    df["ParentSegID"] = psegids
    df["Elec_distance"] = elec_distances
    df["Elec_distance_nexus"] = elec_distances
    df["Sec Name"] = fullsecnames
    df['Degrees']=seg_degrees
    df['num_syns_exc']=excSynPerSeg
    df['num_syns_inh']=inhSynPerSeg


    df.to_csv("ReducedSegments.csv", index=False)
