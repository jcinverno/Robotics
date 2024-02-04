#!/usr/bin/env python  
import rospy
import math
import tf

from sensor_msgs.msg import JointState
from tr5_kinematics.srv import DoInverseKinematics, DoInverseKinematicsResponse
"""
TODO add any missing imports (e.g. custom messages/services)
"""

# load private parameters
svc_ik = rospy.get_param("~inv_kin_service", "/do_ik")     

# DH Parameters for ROB3/TR5
d1 = 0.275
a2 = 0.200
a3 = 0.130
d5 = 0.130

"""
TODO Implement the callback to handle the inverse kinematics calculations 
"""
def doInverseKinematics(srv):
    
    posx = srv.start_inverse.position.x
    posy = srv.start_inverse.position.y
    posz = srv.start_inverse.position.z

    n = srv.start_inverse.orientation.x
    s = srv.start_inverse.orientation.y
    a = srv.start_inverse.orientation.z
    p = srv.start_inverse.orientation.w

    transformer = tf.TransformerROS().fromTranslationRotation((posx, posy, posz), (n, s, a, p))

    nx = transformer[0][0]
    ny = transformer[1][0]
    nz = transformer[2][0]
    sx = transformer[0][1]
    sy = transformer[1][1]
    sz = transformer[2][1]
    ax = transformer[0][2]
    ay = transformer[1][2]
    az = transformer[2][2]
    px = transformer[0][3]
    py = transformer[1][3]
    pz = transformer[2][3]

    if(px == 0):
        teta_1 = 0
    else:
        teta_1 = math.atan2(py, px)
    
    if(teta_1 < -1.39626):
        teta_1 = -1.39626
    if(teta_1 > 1.39626):
        teta_1 = 1.39626
    
    if((ax*math.cos(teta_1) + ay*math.sin(teta_1)) == 0):
        teta_234 = 0
    else:
        teta_234 = math.atan2(az, (ax*math.cos(teta_1) + ay*math.sin(teta_1)))
    
    cos_teta_3 = (((px*math.cos(teta_1) + py*math.sin(teta_1) - d5*math.cos(teta_234))**2) + ((pz - d1 - d5*math.sin(teta_234))**2) - (a2**2) - (a3**2))/(2*a2*a3)
    
    if((1/(cos_teta_3**2) - 1) < 0):
        teta_3 = 0
    else:
        teta_3 = -(math.atan(math.sqrt(1/(cos_teta_3**2) - 1)))

    if(teta_3 < -1.74533):
        teta_3 = -1.74533
    if(teta_3 > 0):
        teta_3 = 0

    if(((a2 + a3*math.cos(teta_3)) * (-d5*math.cos(teta_234) + math.cos(teta_1)*px + math.sin(teta_1)*py) + a3*math.sin(teta_3) * (-d1 + pz - d5*math.sin(teta_234))) == 0):
        teta_2 = 0
    else:
        teta_2 = math.atan2(((a2 + a3*math.cos(teta_3)) * (-d1 + pz - d5*math.sin(teta_234))), ((a2 + a3*math.cos(teta_3)) * (-d5*math.cos(teta_234) + math.cos(teta_1)*px + math.sin(teta_1)*py) - a3*math.sin(teta_3) * (-d1 + pz - d5*math.sin(teta_234))))
    
    if(teta_2 < -0.52360):
        teta_2 = -0.52360
    if(teta_2 > 1.22173):
        teta_2 = 1.22173

    teta_4 = teta_234 - teta_2 - teta_3

    if(teta_4 < -1.74533):
        teta_4 = -1.74533
    if(teta_4 > 1.74533):
        teta_4 = 1.74533

    teta_5 = math.atan2((ny*math.cos(teta_1) - nx*math.sin(teta_1)), (sy*math.cos(teta_3) - sx*math.sin(teta_1)))
    
    if(teta_5 < -1.74533):
        teta_5 = -1.74533
    if(teta_5 > 1.74533):
        teta_5 = 1.74533

    print(teta_1, teta_2, teta_3, teta_4, teta_5)
    end_inverse = JointState()

    end_inverse.header.stamp = rospy.Time.now()
    end_inverse.position = [teta_1, teta_2, teta_3, teta_4, teta_5, 0.0, 0.0]
    end_inverse.name = ["tr5shoulder_pan_joint", 
                        "tr5shoulder_lift_joint", 
                        "tr5elbow_joint",
                        "tr5wrist_1_joint",
                        "tr5wrist_2_joint", 
                        "tr5gripper_1_joint",
                        "tr5gripper_2_joint"]

    end_inverse.velocity = []
    end_inverse.effort = []

    print(teta_1, teta_2, teta_3, teta_4, teta_5)

    return DoInverseKinematicsResponse(end_inverse)

if __name__ == "__main__":
    rospy.init_node('tr5_inverse_kinematics')
    """
    TODO Create and advertise the service for inverse kinematics
    """

    ik_srv_server_handler = rospy.Service(svc_ik, DoInverseKinematics, doInverseKinematics)


    while not rospy.is_shutdown():
        rospy.spin()
    