import matplotlib.pyplot as plt
import numpy as np

def plot_LFP_traces(t,lfp,savefig=None,fontsize=40,labelpad=-30,tick_length=15,nbins=3):
    """
    Plot LFP traces.
    t: time points (ms). 1D array
    lfp: LFP traces (uV). If is 2D array, each column is a channel.
    savefig: if specified as string, save figure with the string as file name.
    """
    t = np.asarray(t)
    lfp = np.asarray(lfp)
    fig = plt.figure()
    ax = plt.plot(t,lfp)
    plt.xlabel('ms',fontsize=fontsize)
    plt.ylabel('LFP (\u03bcV)',fontsize=fontsize,labelpad=labelpad)
    plt.locator_params(axis='both',nbins=nbins)
    plt.tick_params(length=tick_length,labelsize=fontsize)
    plt.show()
    if savefig is not None:
        if type(savefig) is not str:
            savefig = 'LFP_trace.pdf'
        fig.savefig(savefig,bbox_inches='tight',transparent=True)
    return fig,ax
    
def plot_LFP_heatmap(t,elec_d,lfp,savefig=None,vlim='auto',fontsize=40,ticksize=30,labelpad=-12,nbins=3,cbbox=[.91,0.118,.03,0.76],cmap='viridis'):
    """
    Plot LFP heatmap.
    t: time points (ms). 1D array
    elec_d: electrode distance (um). 1D array
    lfp: LFP traces (uV). If is 2D array, each column is a channel.
    savefig: if specified as string, save figure with the string as file name.
    vlim: value limit for color map, using +/- 3-sigma of lfp for bounds as default. Use 'max' for maximum bound range.
    """
    lfp = np.asarray(lfp).T
    elec_d = np.asarray(elec_d)/1000
    if vlim == 'auto':
        vlim = 3*np.std(lfp)*np.array([-1,1])
    elif vlim == 'max':
        vlim = [np.min(lfp),np.max(lfp)]
    fig,ax = plt.subplots()
    pcm = plt.pcolormesh(t,elec_d,lfp,cmap=cmap,vmin=vlim[0],vmax=vlim[1])
    cbaxes = fig.add_axes(cbbox)
    cbar = fig.colorbar(pcm,ax=ax,ticks=np.linspace(vlim[0],vlim[1],nbins),cax=cbaxes)
    cbar.ax.tick_params(labelsize=ticksize)
    cbar.set_label('LFP (\u03bcV)',fontsize=fontsize,labelpad=labelpad)
    ax.set_xticks(np.linspace(t[0],t[-1],nbins))
    ax.set_yticks(np.linspace(elec_d[0],elec_d[-1],nbins))
    ax.tick_params(labelsize=ticksize)
    ax.set_xlabel('time (ms)',fontsize=fontsize)
    ax.set_ylabel('dist_y (mm)',fontsize=fontsize)
    plt.show()
    if savefig is not None:
        if type(savefig) is not str:
            savefig = 'LFP_heatmap.pdf'
        fig.savefig(savefig,bbox_inches='tight',transparent=True)
    return fig,ax
