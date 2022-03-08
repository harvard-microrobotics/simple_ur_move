import rospy
import yaml

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
        pass  
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