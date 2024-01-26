# ROS2 specifications
import rclpy # Import the ROS client library for Python
import message_filters
from rclpy.node import Node # Enables the use of rclpy's Node class
#from std_msgs.msg import String
#from std_msgs.msg import Float64MultiArray # Enable use of the std_msgs/Float64MultiArray message type        
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import PointStamped # /gps/gps

from std_msgs.msg import Float32           # /gps/speed
from nav_msgs.msg import Odometry           # /gps/speed
#from rclpy.qos import qos_profile_sensor_data


class TurtleBotControl(Node): 
    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('TurtleBotControl')
        self.subscription_1 = message_filters.Subscriber(self, PointStamped, '/TurtleBot3Burger/gps')
        self.subscription_2 = message_filters.Subscriber(self, Odometry, '/odom')
        self.subscription_3 = message_filters.Subscriber(self, Vector3, '/TurtleBot3Burger/gps/speed_vector')
        self.publisher_cmd = self.create_publisher(Twist, '/TurtleBot3Burger/cmd_vel', 10)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.subscription_1, self.subscription_3], 30, 0.01, allow_headerless=True)
        self.ts.registerCallback(self.callbackGPS)


        #self.create_subscription(PointStamped, '/TurtleBot3Burger/gps', self.callbackGPS, 1)

        self.next_pose=PointStamped()

    def callbackGPS(self, msg, msg2):
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        # Calcul trajectoire
        print(f"POS ROBOT : {x} , {y} , {z}")
        # send vel
        
        kinematic = [0.3, 0.2]
        # Kinetic : position, angular
        self.publish_results([0.3,0.0])

    def publish_results(self, vel):
        msg = Twist() # Create a message of this type   
        msg.linear.x = vel[0]
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = vel[1]
        self.publisher_cmd.publish(msg) # Publish the position to the topic
        
    



def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()