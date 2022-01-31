import rospy
import rosservice


def make_service_proxy(name, cls, wait=True, timeout=None):
    """TODO: docstring for make_service_proxy"""
    service_list = rosservice.get_service_list()
    is_available = name in service_list
    
    if wait and not is_available:
        s = 'Waiting for {} service...'.format(name)
        if timeout is not None:
            s += ' (timeout in {}s)'.format(timeout)
        rospy.loginfo(s)
        rospy.wait_for_service(name, timeout=timeout)

    try:
        proxy = rospy.ServiceProxy(name, cls)
        rospy.loginfo('Created a proxy for {} service.'.format(name))
    except Exception as e:
        proxy = None
        s = '' if is_available else 'is not '
        rospy.logerr('Service {} {}available.'.format(name, s))
        rospy.logerr(e)

    return proxy
