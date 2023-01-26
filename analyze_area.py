import pandas as pd
import numpy as np
from neuron import h

def save_degrees(cell):
    degrees = {}
    try:
        calculate_degree(h.SectionRef(sec=cell.hobj.soma[0]), degrees, 0)
    except:
        calculate_degree(h.SectionRef(sec=cell.soma[0]), degrees, 0)

    df_dict = {}
    df_dict["SectionName"] = list(degrees.keys())
    df_dict["Degrees"] = list(degrees.values())
    
    df = pd.DataFrame(df_dict)
    df.to_csv("SectionDegrees.csv", index=False)

def calculate_degree(sref, degrees, deg):
    degrees[sref.sec.name()] = deg

    for c in sref.child:
        calculate_degree(h.SectionRef(sec=c), degrees, deg+1)

def make_seg_df(cell):
    seg_locs = cell.morphology.seg_coords['p05']
    px = seg_locs[0]
    py = seg_locs[1]
    pz = seg_locs[2]
    df = pd.DataFrame()
    i = 0
    j = 0
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
    h.distance(sec=cell.hobj.soma[0])
    nsegs=[]
    RAs=[]
    Parentx=[]
    zz = h.Impedance()
    zz.loc(cell.hobj.soma[0](0.5))
    zz.compute(25)
#############################################################
    
    psegids=[]
    segments=[]
    sections=[]
#    segments=cell.hobj.segments
#    for i in range(len(cell.hobj.segments)):
#      pseg = segments[i].sec.parentseg()
#      psec=pseg.sec
#      nseg = psec.nseg
#      idx = int(np.floor(pseg.x * nseg))
#      if pseg.x == 1.:
#        idx -= 1
#      psegid=segments.index(psec((i + .5) / nseg))
#      psegids.append(psegid)
################################################################
    
    for sec in cell.hobj.all:
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
            sec_ids.append(int(fullsecname.split("[")[2].split("]")[0]))
            sec_type = fullsecname.split(".")[1][:4]
            nsegs.append(seg.sec.nseg)
            RAs.append(seg.sec.Ra)
            parts.append(sec_type)
            full_names.append(str(seg))
            elec_distances.append(zz.ratio(seg))
            #if seg.sec.parentseg() is not None:
              #get parentseg coordinates or something to identify parentseg by
            j += 1
            segments.append(seg)
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
    df["Coord X"] = px
    df["Coord Y"] = py
    df["Coord Z"] = pz
    df["ParentSegID"] = psegids
    df["Elec_distance"] = elec_distances


    df.to_csv("Segments.csv", index=False)
    #import pdb; pdb.set_trace()

def analyze_area(prop):
    #import pdb; pdb.set_trace()

    types = prop['type']
    # areas = prop['area']
    lens = prop['length']
    dists = prop['dist']

    soma_ids = np.where(types == 1)[0]
    close_dend_ids = np.where((types == 3) & (dists < 50))[0]
    far_dend_ids = np.where((types == 3) & (dists >= 50))[0]
    apic_ids = np.where(types == 4)[0]

    print("LENGTHS:")
    print("Close Dend:", np.trunc(sum(lens[close_dend_ids])))
    print("Further Dend:", np.trunc(sum(lens[far_dend_ids])))
    print("Apic:", np.trunc(sum(lens[apic_ids])))
    print()
    # soma_areas = areas[soma_ids]
    # dend_areas = areas[dend_ids]
    # apic_areas = areas[apic_ids]

    # soma_lens = lens[soma_ids]
    # dend_lens = lens[dend_ids]
    # apic_lens = lens[apic_ids]
    
    # print("Dend Excitatory:", np.trunc(sum(dend_lens) * 1.4))
    # print("Dend Inhibitory:", np.trunc(sum(dend_lens) * 0.14))
    # print("Apic Excitatory:", np.trunc(sum(apic_lens) * 1.4))
    # print("Apic Inhibitory:", np.trunc(sum(apic_lens) * 0.14))
    # print("Soma Inhibitory:", 148)

    print("Dend Excitatory:", np.trunc(sum(lens[far_dend_ids]) * 1.4))
    print("Beta Dend Inhibitory:", np.trunc(sum(lens[far_dend_ids]) * 0.14))
    print("Gamma Dend Inhibitory:", np.trunc(sum(lens[close_dend_ids]) * 0.14))
    print("Apic Excitatory:", np.trunc(sum(lens[apic_ids]) * 1.4))
    print("Apic Inhibitory:", np.trunc(sum(lens[apic_ids]) * 0.14))
    print("Soma Inhibitory:", 148)
