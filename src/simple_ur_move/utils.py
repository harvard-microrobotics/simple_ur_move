import rospy
import yaml
import numpy as np
from scipy.interpolate import CubicSpline

def call_service(service_name, service_type, **fargs):
    '''
    Call a service and handle the result

    Parameters
    ----------
    service_name : str
        Name of the service to call
    service_type : ROS srv
        ROS service class (must be correct for the service name you are calling)
    **args : Any
        Arguments to pass to the service

    Returns
    -------
    response : ROS srv response
        The response from the service. Returns ``None`` if the service
        call was unsuccessful.
    '''
    rospy.wait_for_service(service_name)
    try:
        fun = rospy.ServiceProxy(service_name, service_type)
        resp1 = fun(**fargs)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return None


def load_yaml(filename):
    '''
    Load a yaml file safely

    Parameters
    ----------
    filename : str
        Filename to load

    Returns
    -------
    data : Any()
        The python object loaded from the yaml file
    '''
    out = None
    try:
        with open(filename, 'r') as f:
            out = yaml.safe_load(f)
    except:
        raise  
    return out


def save_yaml(data, filename):
    '''
    Save a yaml file

    Parameters
    ----------
    data : Any()
        Data to save
    filename : str
        Filename to save

    Returns
    -------
    success : bool
        If the save was successful
    '''
    out=False
    try:
        with open(filename, 'w') as f:
            yaml.dump(data, f, default_flow_style=None)
        out=True
    except:
        pass
    
    return out


def quaternion_avg_markley(Q, weights=None):
    '''
    Save a yaml file

    Parameters
    ----------
    Q : numpy.ndarray
        An Mx4 ndarray of quaternions
    weights : list
        an M elements list, a weight for each quaternion

    Returns
    -------
    mean : List (quaternion)
        The weighted average quaternion
    '''

    if weights is None:
        weights = np.ones((Q.shape[0]))
    else:
        weights = np.array(weights)

    # Form the symmetric accumulator matrix
    A = np.zeros((4, 4))
    M = Q.shape[0]
    wSum = 0

    for i in range(M):
        q = Q[i, :]
        w_i = weights[i]
        A += w_i * (np.outer(q, q)) # rank 1 update
        wSum += w_i

    # scale
    A /= wSum

    # Get the eigenvector corresponding to largest eigen value
    return np.linalg.eigh(A)[1][:, -1]


def interp_spline(x, y, bc_start=None, bc_end=None):
    """
    Interpolate using a spline

    Parameters
    ----------
    x : list
        Input values (Nx1)
    y : list
        Output values (NxM)
    bc_start : list
        Boundary conditions for the start point (Nx1). If ``None``, zero is used
    bc_end : list
        Boundary conditions for the end point (Nx1). If ``None``, zero is used

    Returns
    -------
    interp_fun : function
        Interpolation function
    """
    x=np.array(x)
    y=np.array(y)
    
    cs=[]

    if bc_start is None:
        bc_start = np.zeros((len(x)))
    if bc_end is None:
        bc_end = np.zeros((len(x)))

    for y0, bc_0, bc_1  in zip(y, bc_start, bc_end):
        cs.append(CubicSpline(x,y0, bc_type=((1, bc_0), (1, bc_1))))
        
    def fun(x0):
        vals=np.zeros((len(x0),len(cs)))
        for idx, cs_curr in enumerate(cs):
            vals[:,idx] = cs_curr(x0)
        return vals
            
    return fun


def interp_quaternion(self,x, y):
    """
    Interpolate quaterions

    Parameters
    ----------
    x : list
        A list of M time points
    y : list of lists
        An array of Mx4 quaternions
    bc_start : list
        Boundary conditions for the start point (Mx4). If ``None``, zero is used
    bc_end : list
        Boundary conditions for the end point (Mx4). If ``None``, zero is used

    Returns
    -------
    interp_fun : function
        Interpolation function 
    """
    diff = x[-1]-x[0]
    x_init = x[0]

    def fun(x0):
        p = (x0-x_init)/diff
        return quaternion_avg_markley(y, [1-p, p])

    return fun