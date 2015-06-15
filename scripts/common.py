import numpy;
import math;
from math import factorial


def euclideanDistance(a, b):
    return numpy.linalg.norm(a - b);

def getLBL1():
    lbl1 = numpy.array([1., 1., 10.]);
    return lbl1

def getLBL2():
    lbl2 = numpy.array([1000., 5., 20.]);
    return lbl2
    
def getSeaDepth():
    seaDepth = 30;
    return seaDepth
    
def getCosBetweenTwoVectors(a, b):
    if (a[0] == b[0] and a[1] == b[1]):
        return numpy.cos(0)
    dist = euclideanDistance(a, b);
    depthDifference = a[2] - b[2];
    return (b[1] - a[1]) / math.sqrt(dist**2 - depthDifference**2)
    
def getSinBetweenTwoVectors(a, b):
    if (a[0] == b[0] and a[1] == b[1]):
        return numpy.sin(0)
    dist = euclideanDistance(a, b);
    depthDifference = a[2] - b[2];
    return (b[0] - a[0]) / math.sqrt(dist**2 - depthDifference**2)

def computeXDisplacement(bearing, xyzDistance, zDifference):
    return numpy.sin(bearing)*math.sqrt(xyzDistance**2 - zDifference**2)
    
def computeYDisplacement(bearing, xyzDistance, zDifference):
    return numpy.cos(bearing)*math.sqrt(xyzDistance**2 - zDifference**2)

def savitzky_golay(y, window_size, order, deriv=0, rate=1):
    r"""Smooth (and optionally differentiate) data with a Savitzky-Golay filter.
    The Savitzky-Golay filter removes high frequency noise from data.
    It has the advantage of preserving the original shape and
    features of the signal better than other types of filtering
    approaches, such as moving averages techniques.
    Parameters
    ----------
    y : array_like, shape (N,)
        the values of the time history of the signal.
    window_size : int
        the length of the window. Must be an odd integer number.
    order : int
        the order of the polynomial used in the filtering.
        Must be less then `window_size` - 1.
    deriv: int
        the order of the derivative to compute (default = 0 means only smoothing)
    Returns
    -------
    ys : ndarray, shape (N)
        the smoothed signal (or it's n-th derivative).
    Notes
    -----
    The Savitzky-Golay is a type of low-pass filter, particularly
    suited for smoothing noisy data. The main idea behind this
    approach is to make for each point a least-square fit with a
    polynomial of high order over a odd-sized window centered at
    the point.
    Examples
    --------
    t = numpy.linspace(-4, 4, 500)
    y = numpy.exp( -t**2 ) + numpy.random.normal(0, 0.05, t.shape)
    ysg = savitzky_golay(y, window_size=31, order=4)
    import matplotlib.pyplot as plt
    plt.plot(t, y, label='Noisy signal')
    plt.plot(t, numpy.exp(-t**2), 'k', lw=1.5, label='Original signal')
    plt.plot(t, ysg, 'r', label='Filtered signal')
    plt.legend()
    plt.show()
    References
    ----------
    .. [1] A. Savitzky, M. J. E. Golay, Smoothing and Differentiation of
       Data by Simplified Least Squares Procedures. Analytical
       Chemistry, 1964, 36 (8), pp 1627-1639.
    .. [2] Numerical Recipes 3rd Edition: The Art of Scientific Computing
       W.H. Press, S.A. Teukolsky, W.T. Vetterling, B.P. Flannery
       Cambridge University Press ISBN-13: 9780521880688
    """

    try:
        window_size = numpy.abs(numpy.int(window_size))
        order = numpy.abs(numpy.int(order))
    except ValueError:
        raise ValueError("window_size and order have to be of type int")
    if window_size % 2 != 1 or window_size < 1:
        raise TypeError("window_size size must be a positive odd number")
    if window_size < order + 2:
        raise TypeError("window_size is too small for the polynomials order")
    order_range = range(order+1)
    half_window = (window_size -1) // 2
    # precompute coefficients
    b = numpy.mat([[k**i for i in order_range] for k in range(-half_window, half_window+1)])
    m = numpy.linalg.pinv(b).A[deriv] * rate**deriv * factorial(deriv)
    # pad the signal at the extremes with
    # values taken from the signal itself
    firstvals = y[0] - numpy.abs( y[1:half_window+1][::-1] - y[0] )
    lastvals = y[-1] + numpy.abs(y[-half_window-1:-1][::-1] - y[-1])
    y = numpy.concatenate((firstvals, y, lastvals))   
    return numpy.convolve( m[::-1], y, mode='valid')