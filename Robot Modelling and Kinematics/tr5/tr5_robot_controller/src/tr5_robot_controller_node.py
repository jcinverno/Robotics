#!/usr/bin/env python  
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarker

from tr5_robot_controller.srv import KinematicMode, KinematicModeResponse
from tr5_kinematics.srv import DoForwardKinematics, DoInverseKinematics
"""
TODO add any missing imports (e.g. custom messagess/services)
Done
"""

class Controller:

    def __init__(self):
        # load private parameters from the Parameter Server
        # services
        self.svc_kin_mode = rospy.get_param("~kin_mode_service", "/kin_mode")
        self.svc_fk = rospy.get_param("~for_kin_service", "/do_fk")
        self.svc_ik = rospy.get_param("~inv_kin_service", "/do_ik")
        # topics
        self.tp_gui_jt = rospy.get_param("~gui_joints_topic", "/gui_joints")
        self.tp_goal_jt = rospy.get_param("~goal_joints_topic", "/goal_joints")
        self.tp_goal_ps = rospy.get_param("~goal_pose_topic", "/visualization_marker")
        
        # kinematics mode flag
        self.ik_mode = False

        # non-interactive visual marker object for rviz
        self.m = Marker()
        self.m.header.frame_id = "/world"
        self.m.header.stamp = rospy.Time.now()
        self.m.ns = "fk marker"
        self.m.id = 0
        self.m.type = self.m.SPHERE
        self.m.action = self.m.ADD
        self.m.scale.x = self.m.scale.y = self.m.scale.z = 0.05
        self.m.color.r = 1.0
        self.m.color.a = 0.5
        self.m.color.g = self.m.color.b = 0.0
        self.m.lifetime = rospy.Duration(1.0)

        # interactive marker
        self.i_marker = InteractiveMarker()
        self.dot_marker = Marker()
        
        # marker server
        self.m_marker_server = InteractiveMarkerServer("input_pose")
        self.create_interactive_marker()
    
        """
        TODO initialize ROS services and message publishers/subscribers
        Done
        """
        self.gui_tp_sub = rospy.Subscriber(self.tp_gui_jt, JointState, self.gui_joints_callback)
        self.goal_jt_pub = rospy.Publisher(self.tp_goal_jt, JointState, queue_size=10)
        self.goal_ps_pub = rospy.Publisher(self.tp_goal_ps, Marker, queue_size=10)

        self.forward_srv = rospy.ServiceProxy(self.svc_fk, DoForwardKinematics)
        self.inverse_srv = rospy.ServiceProxy(self.svc_ik, DoInverseKinematics)

        self.kin_mode_srv = rospy.Service(self.svc_kin_mode, KinematicMode, self.selectKinMode)


    def create_interactive_marker(self):
        self.i_marker.header.frame_id = "/world"
        self.i_marker.header.stamp = rospy.Time.now()
        self.i_marker.name = "user_input_marker"
        self.i_marker.description = "Input Pose"
        
        # create a grey box marker
        self.dot_marker.type = self.dot_marker.SPHERE
        self.dot_marker.scale.x = self.dot_marker.scale.y = self.dot_marker.scale.z = 0.05
        self.dot_marker.color.r = self.dot_marker.color.g = 0.0
        self.dot_marker.color.b = 1.0
        self.dot_marker.color.a = 0.5
        self.dot_marker.pose.position.x = 0.55
        self.dot_marker.pose.position.y = 0.0
        self.dot_marker.pose.position.z = 0.27
        
        #create a non-interactive control which contains the dot marker
        dot_control = InteractiveMarkerControl()
        dot_control.always_visible = True
        dot_control.markers.append(self.dot_marker)
        dot_control.orientation_mode = InteractiveMarkerControl.FIXED
        dot_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D

        #add the control to the interactive marker
        self.i_marker.controls.append(dot_control)
        self.m_marker_server.insert(self.i_marker, self.input_pose_callback)
        self.m_marker_server.applyChanges()
    
    def input_pose_callback(self, msg):
        # ignore feedback from the interactive marker while in forward kinematics mode
        
        if not self.ik_mode:
            return
        


        srv = Pose()
        srv.position.x = msg.pose.position.x + 0.55
        srv.position.y = msg.pose.position.y
        srv.position.z = msg.pose.position.z + 0.27

        srv.orientation.x = msg.pose.orientation.x
        srv.orientation.y = msg.pose.orientation.y
        srv.orientation.z = msg.pose.orientation.z
        srv.orientation.w = msg.pose.orientation.w

        """
        TODO call the required ROS service
        done
        """

        try:
            rospy.wait_for_service(self.svc_ik)
            res = self.inverse_srv.call(srv)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)


        if res:
            self.goal_jt_pub.publish(res.end_inverse)
        else:
            rospy.logerr("Failed to call service inverse kinematics...")


    def gui_joints_callback(self, msg):

        # ignore values from JointState GUI when in ik mode
        if(self.ik_mode):
            return

        self.goal_jt_pub.publish(msg) #nota de lado

        """
        TODO create and fill the ROS service proxy
        """
        
        srv = JointState()
        srv.position = msg.position
        srv.effort = msg.effort
        srv.velocity = msg.velocity

        """
        TODO call the required ROS service
        """

        try:
            rospy.wait_for_service(self.svc_fk)
            res = self.forward_srv.call(srv)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)


        if res:             
            """
            TODO update rviz marker's pose from forward kinematics and publish i
            """
            
            self.m.pose.position.x = res.end_forward.position.x
            self.m.pose.position.y = res.end_forward.position.y
            self.m.pose.position.z = res.end_forward.position.z

            self.goal_ps_pub.publish(self.m)

        else:
            rospy.logerr("Failed to call service for forward kinematics...")

    
    def selectKinMode(self, srv):
        
        if srv.kin_mode == 0:
            self.ik_mode = False
            self.m_marker_server.erase("user_input_marker")
            self.m_marker_server.applyChanges()
            return KinematicModeResponse(False)

        elif srv.kin_mode == 1:

            self.ik_mode = True
            self.create_interactive_marker()
            return KinematicModeResponse(True)
        

    def run(self):
        rospy.spin()   

if __name__ == "__main__":
    rospy.init_node("tr5_robot_controller_node")
    controller = Controller()
    controller.run()