import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

class NavigationNode(Node):
    def __init__(self):
        super().__init__('nav_goal_node')
        self.navigation_goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # service for starting navigation
        self.start_navigation_service = self.create_service(Trigger, '/start_navigation', self.start_navigation_callback)
        self.get_logger().info('Navigation node is ready')

    def start_navigation_callback(self, request, response):
        # Handle the service request
        self.get_logger().info('Received start_navigation request')

        #Tamanho Mapa
        #width = 288   X
        #height = 118  Y
         
        #map_position_x = [-9.49]
        #map_position_y = [-0.972]
        
        # Create a PoseStamped message with the desired goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'  
        goal_pose.pose.position.x = -8.80  
        goal_pose.pose.position.y = -0.4  
        goal_pose.pose.orientation.w = 1.0  

        # Publish the goal pose
        self.navigation_goal_publisher.publish(goal_pose)
        self.get_logger().info('Sent navigation goal')

        # Populate the response for the service call
        response.success = True
        response.message = 'Navigation goal sent successfully'

        return response

def main(args=None):
    rclpy.init(args=args)
    nav_goal_node = NavigationNode()
    rclpy.spin(nav_goal_node)
    nav_goal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
