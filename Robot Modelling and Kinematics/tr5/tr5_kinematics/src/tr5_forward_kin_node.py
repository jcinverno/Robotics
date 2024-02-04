#!/usr/bin/env python  
import rospy
import math

from geometry_msgs.msg import Pose
from tr5_kinematics.srv import DoForwardKinematics, DoForwardKinematicsResponse

"""
TODO add any missing imports (e.g. custom messages/services)
"""

# load private parameters
svc_fk = rospy.get_param("~for_kin_service", "/do_fk") 

# DH Parameters for ROB3/TR5
d1 = 0.275
a2 = 0.200
a3 = 0.130
d5 = 0.130

"""
TODO Implement the callback to handle the forward kinematics calculations 
"""
def doForwardKinematics(srv):

    end_forward = Pose()
    
    teta_1 = srv.start_forward.position[0]
    teta_2 = srv.start_forward.position[1]
    teta_3 = srv.start_forward.position[2]
    teta_4 = srv.start_forward.position[3]

    end_forward.position.x = math.cos(teta_1)*(a2*math.cos(teta_2) + a3*math.cos(teta_2+teta_3) + d5*math.cos(teta_2+teta_3+teta_4))
    end_forward.position.y = math.sin(teta_1)*(a2*math.cos(teta_2) + a3*math.cos(teta_2+teta_3) + d5*math.cos(teta_2+teta_3+teta_4))
    end_forward.position.z = d1 + a2*math.sin(teta_2) + a3*math.sin(teta_2+teta_3) + d5*math.sin(teta_2+teta_3+teta_4)

    return DoForwardKinematicsResponse(end_forward)

if __name__ == "__main__":
    rospy.init_node('tr5_forward_kinematics')
    """
    TODO Create and advertise the service for forward kinematics
    """
    fk_srv_server_handler = rospy.Service(svc_fk, DoForwardKinematics, doForwardKinematics)

    while not rospy.is_shutdown():
        rospy.spin()
    