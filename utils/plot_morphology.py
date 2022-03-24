from stylized_module.ecp import newposition
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np

def plot_morphology(sim,cellid=0,electrodes=None,axes=[2,0,1],clr=['g','b','c','pink','purple'],elev=20,azim=10,figsize=None):
    """
    Plot morphology in 3D.
    sim: simulation object
    cellid: cell id. Default: 0
    electrodes: electrode positions.  Default: None, not shown.
    axes: sequence of axes to display in 3d plot axes. Default: [2,0,1] show z,x,y in 3d plot x,y,z axes, so y is upward.
    clr: list of colors for each type of section
    """
    cell = sim.cells[cellid]
    move_cell = sim.loc_param[cellid]
    dl = newposition([0.,0.,0.],move_cell[1],cell.seg_coords['dl'])
    pc = newposition(move_cell[0],move_cell[1],cell.seg_coords['pc'])
    xyz = 'xyz'
    box = np.vstack([np.full(3,np.inf),np.full(3,np.NINF)])
    if electrodes is not None:
        box[0,axes[0:2]] = np.amin(electrodes[:,axes[0:2]],axis=0)
        box[1,axes[0:2]] = np.amax(electrodes[:,axes[0:2]],axis=0)
    fig = plt.figure(figsize=figsize)
    ax = plt.axes(projection='3d')
    ax.scatter(*[pc[0,j] for j in axes],c=clr[0],s=30,label='soma')
    pretype = 'soma'
    itype = 0
    for i,sec in enumerate(cell.all[1:],start=1):
        i0 = cell.sec_id_in_seg[i]
        i1 = i0+sec.nseg-1
        p0 = pc[i0]-dl[i0]/2
        p1 = pc[i1]+dl[i1]/2
        if pretype!=sec.name():
            pretype = sec.name()
            label = pretype
            itype += 1
        else:
            label = None
        ax.plot3D(*[[p0[j],p1[j]] for j in axes],color=clr[itype],label=label)
        box[0,:] = np.minimum(box[0,:],np.minimum(p0,p1))
        box[1,:] = np.maximum(box[1,:],np.maximum(p0,p1))
    ctr = np.mean(box,axis=0)
    r = np.amax(box[1,:]-box[0,:])/2
    box = np.vstack([ctr-r,ctr+r])
    if electrodes is not None:
        idx = np.logical_and(np.all(electrodes>=box[0,:],axis=1),np.all(electrodes<=box[1,:],axis=1))
        ax.scatter(*[[electrodes[idx,j],electrodes[idx,j]] for j in axes],color='orange',s=10,label='electrodes')
    box = box[:,axes]
    ax.auto_scale_xyz(*box.T)
    ax.view_init(elev,azim)
    ax.legend(loc=1)
    ax.set_xlabel(xyz[axes[0]])
    ax.set_ylabel(xyz[axes[1]])
    ax.set_zlabel(xyz[axes[2]])
    plt.show()
    return fig,ax