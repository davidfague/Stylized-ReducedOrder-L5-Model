import numpy as np

def corrcoef(x,y):
    """
    function for calculating correlation coefficient
    """
    return (np.mean(x*y)-np.mean(x)*np.mean(y))/(np.std(x)*np.std(y))

def max_corrcoef(x,y,window_size=None):
    """
    Calculate correlation coefficient between input x and y inside sliding time window
    (time should be the first axis, i.e. a column is a channel)
    Find the maximum correlation coefficient and the corresponding window location.
    Return first the maximum value, then return the index of the start point of corresponding window.
    If window_size is not specified, use the length of x as window size. Return the index of window in y.
    If window_size is specified, slide the window separately in x and y. Return the indices of windows in x and y respectively.
    """
    if window_size is None:
        win = x.shape[0]
    else:
        win = window_size
    nx = x.shape[0]-win+1
    ny = y.shape[0]-win+1
    corr = np.empty((nx,ny))
    for i in range(nx):
        for j in range(ny):
            corr[i,j] = corrcoef(x[i:i+win,:].ravel(),y[j:j+win,:].ravel())
    maxind = np.argmax(corr)
    maxind = np.unravel_index(maxind,(nx,ny))
    max_corr = corr[maxind]
    if window_size is None:
        output = (max_corr,maxind[1])
    else:
        output = (max_corr,maxind[0],maxind[1])
    return output