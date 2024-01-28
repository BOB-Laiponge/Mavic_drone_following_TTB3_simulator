
# Python libraries
import time
import rclpy # Import the ROS client library for Python
import message_filters
from rclpy.node import Node # Enables the use of rclpy's Node class

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped # /gps/gps
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Imu            # /
from std_msgs.msg import Bool
from std_msgs.msg import Float32           # /gps/speed
from nav_msgs.msg import Odometry           # /gps/speed


import numpy as np
import math
import random # Python library to generate random numbers
from .euler import euler_from_quaternion
from .dynamicwindow import dwa_local_planning
from .quintic_polynomials_planner import *

class TurtleBotControl(Node): 
    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('TurtleBotControl')
        self.subscription_1 = message_filters.Subscriber(self, PointStamped, '/TurtleBot3Burger/gps')
        self.subscription_2 = message_filters.Subscriber(self, Imu, '/TurtleBot3Burger/imu')

        self.ts = message_filters.ApproximateTimeSynchronizer([self.subscription_1, self.subscription_2], 30, 0.01, allow_headerless=True)
        self.ts.registerCallback(self.callback)

        self.freq_pose = 10
        self.tab_deplacement = list()
        self.v0 = 0.0
        self.dist_prec = 99999.0

        self.vel_pub = self.create_publisher(Twist, '/TurtleBot3Burger/cmd_vel', 10)

        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()

        self.run()

    def run(self):

        sx = 6.33  # start x position [m]
        sy = 0.0  # start y position [m]
        syaw = np.deg2rad(0.0)  # start yaw angle [rad]
        sv = 0.0  # start speed [m/s]
        sa = 0.0  # start accel [m/ss]
        
        iv = 0.22  # intermediat speed [m/s]
        ia = 0.00  # intermediat accel [m/ss]

        gv = 0.0  # goal speed [m/s]
        ga = 0.0  # goal accel [m/ss]

        max_accel = 0.02  # max accel [m/ss]
        max_jerk = 0.02  # max jerk [m/sss]
        dt = 2.0  # time tick [s]

        w1 = [8, 0, np.deg2rad(90)]
        w2 = [8, 2, np.deg2rad(180)]
        w3 = [6.33, 2, np.deg2rad(270)]
        w4 = [6.33, 0, np.deg2rad(0)]

        mvmt = [[w4[0], w4[1], w4[2], iv, ia, w1[0], w1[1], w1[2], iv, ia, max_accel, max_jerk, dt], 
                [w1[0], w1[1], w1[2], iv, ia, w2[0], w2[1], w2[2], iv, ia, max_accel, max_jerk, dt], 
                [w2[0], w2[1], w2[2], iv, ia, w3[0], w3[1], w3[2], iv, ia, max_accel, max_jerk, dt], 
                [w3[0], w3[1], w3[2], iv, ia, w4[0], w4[1], w4[2], iv, ia, max_accel, max_jerk, dt]]
                

        for i in range(len(mvmt)):
            self.generate_trajectory(*mvmt[i])

        self.back_generate_trajectory = self.tab_deplacement.copy()

    def generate_trajectory(self, sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt):
        time, x, y, yaw, v, a, j = quintic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt)
        
        for i in range(len(x)):
            self.tab_deplacement.append([time[i], x[i], y[i], yaw[i], v[i], a[i], j[i]])
            print(f"info : x = {x[i]:.2f}, y = {y[i]:.2f}, theta = {yaw[i]:.2f}, speed = {v[i]:.2f}")

    def calculer_commande(self, x, y, yaw, vitesse_desiree, x_final, y_final, dist_prec):
        # Paramètres du contrôleur
        atteint = False
        distance_tolerance = 0.24  # Tolérance de distance pour considérer l'objectif atteint
        angle_tolerance = np.deg2rad(5)  # Tolérance d'angle pour considérer l'objectif atteint

        # Calcul des erreurs de position
        distance = math.sqrt((x_final - x)**2 + (y_final - y)**2)
        angle_to_goal = math.atan2(y_final - y, x_final - x)
        angle_error = angle_to_goal - np.deg2rad(yaw)

        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        # Calcul de la commande linéaire
        #print(f"dist = {distance}, dist_prec = {dist_prec}")
        #if(distance >= dist_prec):
        print(f"dist = {distance} < 0.24 , angle_error = {angle_error} < {angle_tolerance}")
        #if((distance <= 0.2) and (abs(angle_error) < angle_tolerance)):
        if(distance <= distance_tolerance):
            #print("true ?")
            atteint = True
        #else:
            #print("false?")

        # Calcul de la commande angulaire
        vitesse_angulaire = angle_error #if abs(angle_error) > angle_tolerance else 0.0

        return vitesse_desiree, vitesse_angulaire, distance, atteint

    def speed_callback(self, msg):
        #print("speedcallback")
        self.v0 = msg.data # /gps/speed

    def publish_velocity(self, velocity):
        # Publier les consignes de vitesse au robot
        msg = Twist()
        msg.linear.x = velocity[0]
        msg.linear.y = 0.0
        msg.angular.z = velocity[1]
        print(f"ttb vel : {velocity[0]} ; {velocity[1]}")
        self.vel_pub.publish(msg)


    #def callback(self,msg1,msg2,msg3):
    def callback(self,msg1,msg2):
        """
        Callback function.
        """
        oris = euler_from_quaternion(msg2.orientation.x, msg2.orientation.y, msg2.orientation.z, msg2.orientation.w)
        roll = oris[0]
        pitch = oris[1]
        yaw = oris[2]
        
        self.pose.theta = yaw
        self.pose.x = msg1.point.x
        self.pose.y = msg1.point.y

        self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
        

        print(f"info ttb : x = {self.pose.x}, y = {self.pose.y}, theta = {yaw}, speed = {self.v0}")
        
        
        if len(self.tab_deplacement) != 0:
            #print("debug")
            temp = self.tab_deplacement[0]
            vitesse_desiree = temp[4]
            x_desiree = temp[1]
            y_desiree = temp[2]
            vitesse_desiree, vitesse_angulaire, dist_precf, atteint = self.calculer_commande(self.pose.x, self.pose.y, self.pose.theta, vitesse_desiree, x_desiree, y_desiree, self.dist_prec)
            print(f"self.dist = {self.dist_prec}, dist_prec = {dist_precf}")
            

            if(atteint):
                print("checkpoint atteint")
                self.tab_deplacement.pop(0)
                self.dist_prec = 99999.0
            else:
                self.dist_prec = dist_precf

            self.publish_velocity([vitesse_desiree, vitesse_angulaire])

        else:
            self.tab_deplacement = self.back_generate_trajectory.copy()
        
        
        



def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()