import numpy as np

def first_pk_tr(lfp):
    """Find the time index of first peak/trough in "lfp" (2D array, each column is a channel)."""
    m = np.argmin(np.amin(lfp,axis=1))
    M = np.argmax(np.amax(lfp,axis=1))
    return min(m,M)
    
def get_spike_window(lfp,win_size,align_at=0):
    """
    Get the window of the spike waveform, where the first peak/trough is aligned at a fixed point in the window.
    lfp: input lfp with spike waveform (2D array, each column is a channel)
    win_size: window size (time samples)
    align_at: time index in the window to align with the first peak/trough in "lfp"
    return (start,end), the time index of the window in "lfp"
    """
    L = lfp.shape[0]
    if win_size>L:
        raise ValueError("win_size cannot be greater than length of lfp")
    align_pt = first_pk_tr(lfp)
    start = max(align_pt-align_at,0)
    end = start+win_size
    if end>L:
        start -= end-L
        end=L
    return (start,end)