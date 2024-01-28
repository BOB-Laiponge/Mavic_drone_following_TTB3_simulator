# ROS2 specifications
import random
import numpy as np

from .utils import quintic_polynomials_planner
from .euler import euler_from_quaternion
import rclpy # Import the ROS client library for Python
import message_filters
from rclpy.node import Node # Enables the use of rclpy's Node class
#from std_msgs.msg import String
#from std_msgs.msg import Float64MultiArray # Enable use of the std_msgs/Float64MultiArray message type        
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import PointStamped # /gps/gps

from std_msgs.msg import Float32           # /gps/speed
from nav_msgs.msg import Odometry           # /gps/speed
from sensor_msgs.msg import Imu            # /

#from rclpy.qos import qos_profile_sensor_data


class TurtleBotControl(Node): 
    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('TurtleBotControl')
        self.subscription_1 = message_filters.Subscriber(self, PointStamped, '/TurtleBot3Burger/gps')
        #self.subscription_2 = message_filters.Subscriber(self, Imu, '/TurtleBot3Burger/imu')
        #self.subscription_3 = message_filters.Subscriber(self, Vector3, '/TurtleBot3Burger/gps/speed_vector')
        #self.publisher_cmd = self.create_publisher(Twist, '/TurtleBot3Burger/imu', 10)

        #self.ts = message_filters.ApproximateTimeSynchronizer([self.subscription_1, self.subscription_2], 30, 0.01, allow_headerless=True)
        #self.ts.registerCallback(self.callbackGPS)

        self.publisher_cmd = self.create_publisher(Twist, '/TurtleBot3Burger/cmd_vel', 10)


        #self.create_subscription(Imu, '/TurtleBot3Burger/imu', self.callbackGPS, 1)
        self.create_subscription(Float32, '/TurtleBot3Burger/gps/speed', self.callbackSpeed, 1)
        self.create_subscription(Imu,'/TurtleBot3Burger/imu', self.callbackImu, 1)
        self.create_subscription(PointStamped,'/TurtleBot3Burger/gps', self.callbackGPS, 1)


        self.last_speed = None # Float32()
        self.last_imu = None # Imu()
        self.last_pose = None
        self.rx, self.ry, self.ryaw, self.rv = [], [], [], []
        self.e = 0.1
        self.vel = [0,0]
        
        print("Node initialized")

    def callbackImu(self, imu):
        #print(f"imu")
        self.last_imu = imu
        oris = euler_from_quaternion(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        self.yaw = oris[2]


    def callbackSpeed(self, speed):
        #print("speed")
        self.last_speed = speed

    def callbackGPS(self, msg):
        self.posx = msg.point.x
        self.posy = msg.point.y
        self.posz = msg.point.z
        #print("gps")

        if self.last_imu == None or self.last_speed == None:
            return 

        if len(self.rx) == 0:
            self.create_trajectory(30,-10)
            print(self.rx)
            print(self.ry)
        
        if abs(self.rx[0]-self.posx) < self.e and abs(self.ry[0]-self.posy) < self.e : 
            # arrivée au prochain waypoint
            
            self.rx.pop(0)
            self.ry.pop(0)

            if len(self.rx) == 0 : 
                self.create_trajectory(random.randint(-30,30), random.randint(-30,30))
                self.publish_results([0.0,0.0])
                print("changement de trajectoire")
                return

            self.vel[1] = self.ryaw[1] - self.ryaw[0]
            self.ryaw.pop(0)
            self.rv.pop(0)
            self.vel[0] = self.rv[0]
            print("changement de direction")

        self.publish_results(self.vel)



        



    def create_trajectory(self, gx, gy):
        print(f"New Traj : {gx}, {gy}") 
        sx = self.posx # start x position [m]
        sy = self.posy # start y position [m]
        syaw = np.deg2rad(self.yaw)  # start yaw angle [rad]
        sv = 0  # start speed [m/s]
        sa = 0  # start accel [m/ss]
        gyaw = np.deg2rad(0)  # goal yaw angle [rad]
        gv = 0.3  # goal speed [m/s]
        ga = 0.1  # goal accel [m/ss]
        max_accel = 0.3  # max accel [m/ss]
        max_jerk = 0.2  # max jerk [m/sss]
        dt = 1  # time tick [s]

        time, self.rx, self.ry, self.ryaw, self.rv, ra, rj = quintic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt)


    def generate_trajectory(self):
        pass

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