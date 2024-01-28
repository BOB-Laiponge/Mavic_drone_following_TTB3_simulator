# ROS2 specifications
import rclpy # Import the ROS client library for Python
import message_filters
from rclpy.node import Node # Enables the use of rclpy's Node class
#from std_msgs.msg import String
#from std_msgs.msg import Float64MultiArray # Enable use of the std_msgs/Float64MultiArray message type        
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped # /gps/gps
from sensor_msgs.msg import Imu            # /
from std_msgs.msg import Bool
from std_msgs.msg import Float32           # /gps/speed
from nav_msgs.msg import Odometry           # /gps/speed
#from rclpy.qos import qos_profile_sensor_data

# Python libraries
import numpy as np
import math
import random # Python library to generate random numbers
from .euler import euler_from_quaternion
from .polynomial import quintic_polynomials_planner
from .dynamicwindow import dwa_local_planning


class MyPlannerNode(Node):
    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('MavicPlannerNode')
        self.subscription_1 = message_filters.Subscriber(self, PointStamped, '/Mavic_2_PRO/gps')
        self.subscription_2 = message_filters.Subscriber(self, Imu, '/imu')
        self.subscription_3 = message_filters.Subscriber(self, Float32, '/Mavic_2_PRO/gps/speed')

        #self.subscription_4 = message_filters.Subscriber(self, PointStamped, 'next_pose')
        self.next_pose=PointStamped()
        self.create_subscription(PointStamped, '/TurtleBot3Burger/gps', self.next_pose_callback, 1)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.subscription_1, self.subscription_2, self.subscription_3], 30, 0.01, allow_headerless=True)
        self.ts.registerCallback(self.callback)
        #self.ts = message_filters.TimeSynchronizer([self.subscription_1, self.subscription_2], 10)

        self.publisher_cmd = self.create_publisher(Twist, '/Mavic_2_PRO/cmd_vel', 10)
        #timer_period = 1.0 
        #self.timer = self.create_timer(timer_period, self.coordinate_generation)
        #print('working?')
          
        self.landing = False
        self.subscription_landing = self.create_subscription(Bool, '/pioneer/ready_to_land', self.callback_landing, 10)

        print("NODE INITIALIZED")

    def callback_landing(self, msg):
        self.landing = msg.data

    def next_pose_callback(self,msg):  
        self.next_pose.point.x = msg.point.x
        self.next_pose.point.y = msg.point.y
        self.next_pose.point.z = msg.point.z
        #print(f"TTB3 POSE : {self.next_pose.point.x} ; {self.next_pose.point.y} ; {self.next_pose.point.z}")
        
    def callback(self,msg1,msg2,msg3):
        """
        Callback function.
        """
        #print("DRONE CALLBACK")
        x0 = msg1.point.x
        y0 = msg1.point.y
        z0 = msg1.point.z

        qx = msg2.orientation.x
        qy = msg2.orientation.y
        qz = msg2.orientation.z
        qw = msg2.orientation.w

        v0 = msg3.data # /gps/speed
        w0 = msg2.angular_velocity.z

        #print(f"DRONE : {x0} ; {y0} ; {z0}")

        """
        Quaternion to Euler angles
        """
        oris = euler_from_quaternion(qx, qy, qz, qw)

        #print(f"Yaw : {oris[2]}")

        yaw0 = oris[2]
        #self.get_logger().info('mavic yaw: %s' % yaw0)
        yaw0rad = np.deg2rad(yaw0)
        ####################################    
        # Target position and orientation  
        #x1 = msg4.pose.pose.position.x
        #y1 = msg4.pose.pose.position.y
        x1 = self.next_pose.point.x
        y1 = self.next_pose.point.y
        #self.get_logger().info('Publishing "%s" - "%s" - "%s"'% (msg.point.x, msg.point.y, msg.point.z))
        #self.get_logger().info('next pos "%s" - "%s"'% (x1,y1))
    
        #target_pos = [-10.0, 10.0]
        #x1 = target_pos[0]
        #y1 = target_pos[1]
        # x1 = msg4.pose.pose.position.x
        # y1 = msg4.pose.pose.position.y

        x = np.array([x0, y0, yaw0rad, v0, w0])
        goal = np.array([x1, y1])

        #trajectory = np.array(x)
        ob = np.array([ [-14.0, -14.5],
                        [-19.6, -24.4],
                        [-22.0,   6.2],
                        [-9.37,  14.0],
                        [-10.7, -25.1],
                        [-14.3,  14.6],
                        [-26.6, -7.17],
                        [-41.5,  4.34],
                        [-43.9, -19.8],
                        [-44.3, -27.3],
                        [-46.2,  30.6],
                        [ 38.8,  23.4]
                        ])

        kinematic = [1.0, 1.0, 50.0 * math.pi/180.0, 1.0, 100.0 * math.pi/180.0]
        # robot_radius+obstacle_radius, vmax, wmax,vtmax,wtmax 
        # mavic para : v_max = 20m/s? w_max = 200deg/s
        
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        # s = v²/(2a)
        if dist_to_goal <= 1.5:
            #u = [v0 * dist_to_goal*2, w0 * dist_to_goal*2] 
            u = [0.0, 0.0]
        else:    
            u, predicted_trajectory = dwa_local_planning(x, kinematic, goal, ob)


        print(f"DRONE VEL : {u[0]} ; {u[1]}")
        # Publish the coordinates to the topic
        #vx_omega = [vx, omega]
        #print('Speeds: ', vx_omega)
        self.publish_results(u)
     
    def publish_results(self, vel):

        msg = Twist() # Create a message of this type
        if self.landing == False:   
          msg.linear.x = vel[0]
          msg.linear.y = 0.0
          msg.linear.z = 0.0
          msg.angular.x = 0.0
          msg.angular.y = 0.0
          msg.angular.z = vel[1]
          self.publisher_cmd.publish(msg) # Publish the position to the topic
    
def main(args=None):
    rclpy.init(args=args)
    node = MyPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()