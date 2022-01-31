import rospy


def log_publish(publisher, message):
    rospy.logdebug('{} published to {}:\n {}'.format(
        rospy.get_caller_id(), publisher.name, message))


def log_heard(subscriber, message):
    rospy.logdebug('{} heard from {}:\n {}'.format(
        rospy.get_caller_id(), subscriber.name, message))


def log_service_response(service, data=None, response=None,
                         logger_name='human_window'):
    s = 'Responding service {}'.format(service.resolved_name)
    if data is not None:
        s += '\n Data: \n {}'.format(data)
    if response is not None:
        s += '\n Response: \n {}'.format(response)
    rospy.logdebug(s, logger_name=logger_name)


def log_service_call(service, data=None, response=None,
                     logger_name='robot'):
    s = 'Calling service {}'.format(service.resolved_name)
    if data is not None:
        s += '\n Data: \n {}'.format(data)
    if response is not None:
        s += '\n Response: \n {}'.format(response)
    rospy.logdebug(s, logger_name=logger_name)
