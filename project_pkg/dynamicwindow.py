import math
import matplotlib.pyplot as plt
import numpy as np

show_animation = True

def dwa_local_planning(x, kinematic, goal, ob):

    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    ########### velocity space setting ##################
    # Dynamic window from robot specification
    robot_radius  = kinematic[0]  # [m] for collision check
    max_speed_x   = kinematic[1]  # [m/s]
    max_speed_yaw = kinematic[2]  # [rad/s]
    max_accel_x   = kinematic[3]  # [m/ss]
    max_accel_yaw = kinematic[4]  # [rad/ss]

    Vs = [0,    
          max_speed_x,
         -max_speed_yaw, 
          max_speed_yaw]
    dt = 0.1
    # Dynamic window from motion model
    Vd = [x[3] - max_accel_x * dt,
          x[3] + max_accel_x * dt,
          x[4] - max_accel_yaw * dt,
          x[4] + max_accel_yaw * dt]

    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]: limits
    dw = [max(Vs[0], Vd[0]), 
          min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), 
          min(Vs[3], Vd[3])]
    #####################################################
    
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])
    # velocity space discretization
    # dwa settings
    v_resolution = 0.01  # [m/s]
    w_resolution = 0.1 * math.pi / 180.0  # [rad/s]

    predict_time = 2.0  # [s]
    alpha = 0.15
    beta  = 1.0
    gamma = 1.0
    robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], v_resolution):
        for w in np.arange(dw[2], dw[3], w_resolution):

            trajectory = predict_trajectory(x_init, v, w, dt, predict_time)
            # u = [v, y]
            # calc cost
            to_goal_cost = alpha * calc_to_goal_cost(trajectory, goal)
            speed_cost = beta * (max_speed_x - trajectory[-1, 3])
            ob_cost = gamma * calc_obstacle_cost(trajectory, ob, robot_radius)

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, w]
                best_trajectory = trajectory
                if abs(best_u[0]) < robot_stuck_flag_cons \
                        and abs(x[3]) < robot_stuck_flag_cons:
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_u[1] = -max_accel_yaw

    return best_u, best_trajectory

def predict_trajectory(x_init, v, w, dt, predict_time):
    """
    predict trajectory with an input
    """

    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= predict_time:
        x = motion(x, [v, w], dt)
        trajectory = np.vstack((trajectory, x))
        time += dt

    return trajectory

def motion(x, u, dt):
    """
    motion model
    """

    x[2] += u[1] * dt  # yaw = yaw + w*dt
    x[0] += u[0] * math.cos(x[2]) * dt # x = x + v*cos(theta)*dt 
    x[1] += u[0] * math.sin(x[2]) * dt # y = y + v*sin(theta)*dt
    x[3] = u[0] # v = v
    x[4] = u[1] # w = w

    return x

def calc_obstacle_cost(trajectory, ob, robot_radius):
    """
    calc obstacle cost inf: collision
    """
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)


    if np.array(r <= robot_radius).any():
        return float("Inf")

    min_r = np.min(r)
    return 1.0 / min_r  # OK


def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost