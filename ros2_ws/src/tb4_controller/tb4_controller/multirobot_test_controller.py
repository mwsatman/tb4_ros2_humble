import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from functools import partial
import signal
import numpy as np

# NOTE: The code works for the following 4 robots.
# However there are occasional hiccup, where one or 2 robots refuse to move
# During those time set DEBUG SPIN to True to make the robot spin
# After spinning they usually work again 

# Additional NOTE: the look ahead controller seems jittery for now. 
# need to look further on what causing the issue
# Paola also mentioned that the turtlebot4 is not designed for moving backwards.
# Need to check this as well.

# Future NOTE: 
# maybe it is better to work with the robots existing automation 
# to make it dock by itself.


robot_list = ['01', '02', '03', '05']
# robot_list = ['02']
robot_num = len(robot_list)

# goal_for_robots = np.array([
#     [1.0, 1.0, 0.0], 
#     [1.0, 2.0, 0.0], 
#     [2.0, 2.0, 0.0], 
#     [2.0, 1.0, 0.0], 
# ]) # + np.array([3.0, 1.0, 0.])

DEBUG_SPIN = False
PRINT_CMD = False

# Charging dock position
goal_for_robots = np.array([
    [1.8, -0.5, 0.0], 
    [2.5, -0.5, 0.0], 
    [3.2, -0.5, 0.0], 
    [4.0, -0.5, 0.0], 
])


SI_UNI_ell = 0.1

class RobotControl():

    def __init__(self):
        self.goal_pos = None
        self.goal_orientation = None

        self.robot_pos = None
        self.robot_orientation = None

    def update_state(self, pos, orientation):
        self.robot_pos = pos
        self.robot_orientation = orientation

    def compute_control_input(self):
        si_cmd = np.zeros(3)

        if self.goal_pos is not None:
            # Proportional control
            k_gain = 1.
            si_cmd = k_gain * (self.goal_pos -self.robot_pos)
        
            # Put velocity limit
            speed_limit = 0.2
            norm = np.hypot(si_cmd[0], si_cmd[1])
            si_cmd = speed_limit * si_cmd / norm  # max

        return si_cmd


class TB4Controller(Node):

    def __init__(self):
        super().__init__('tb4_controller')

        self.ros_pubs = []
        self.global_poses = {}

        self.controller = []

        for i in range(robot_num):
            robot_id = robot_list[i]
            tb_name = f'tb4_' + robot_id

            # Create pose subscribers
            self.get_logger().info(f'Creating pose subscriber /tb_{robot_id}/pos')
            self.pose_sub = self.create_subscription(PoseStamped,
                                    f'/vrpn_mocap/tb_{robot_id}/pose',
                                    partial(self.pose_callback, index=robot_id),
                                    qos_profile=qos_profile_sensor_data)

            # create handler for the publisher of the control input
            self.ros_pubs += [self.create_publisher(Twist, '/{}/cmd_vel'.format(tb_name), 10)]

            # Initialize Controller 
            self.controller += [RobotControl()]
            self.controller[i].goal_pos = goal_for_robots[i]


        # TIMER for calculating control input and publish it
        self.ROS_RATE = 50.
        self.it = 0
        self.check_t = self.time()
        # Set timer for controller loop in each iteration
        self.controller_timer = self.create_timer(1./self.ROS_RATE, self.control_loop)

        # Add handler if CTRL+C is pressed --> then save data to pickle if needed
        signal.signal(signal.SIGINT, self.signal_handler)


    def time(self):
        """Returns the current time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def pose_callback(self, msg, index):
        self.global_poses[index] = {}
        pos, ori_rpy = self.poseStamped_to_np(msg)
        pos_lahead = self.lookahead_uni(pos, ori_rpy[2], SI_UNI_ell)

        self.global_poses[index]['msg'] = msg
        self.global_poses[index]['pos'] = pos
        self.global_poses[index]['ori_rpy'] = ori_rpy
        self.global_poses[index]['pos_lahead'] = pos_lahead

    @staticmethod
    def si2D_to_uni(u_x, u_y, yaw, ell):
        vel_lin = u_x*np.cos(yaw) + u_y*np.sin(yaw)
        vel_ang = (- u_x*np.sin(yaw) + u_y*np.cos(yaw))/ell
        # print(vel_lin, vel_ang) 
        return vel_lin, vel_ang

    @staticmethod
    def lookahead_uni(pos, yaw, ell):
        return np.array([
            pos[0] + ell*np.cos(yaw),
            pos[1] + ell*np.sin(yaw),
            pos[2]
        ])

    @staticmethod
    def euler_from_quat(x, y, z, w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def poseStamped_to_np(self, msg_poseStamp):
        pos = np.array([
            msg_poseStamp.pose.position.x,
            msg_poseStamp.pose.position.y,
            msg_poseStamp.pose.position.z
        ])

        roll, pitch, yaw = self.euler_from_quat(
            msg_poseStamp.pose.orientation.x,
            msg_poseStamp.pose.orientation.y,
            msg_poseStamp.pose.orientation.z,
            msg_poseStamp.pose.orientation.w
        )
        ori_rpy = np.array([roll, pitch, yaw])
        return pos, ori_rpy

    @staticmethod
    def create_tb4_twist(lin_vel, angular_vel):
        TBvel = Twist()
        TBvel.linear.x = lin_vel
        TBvel.linear.y = 0.0
        TBvel.linear.z = 0.0
        TBvel.angular.x = 0.0
        TBvel.angular.y = 0.0
        TBvel.angular.z = angular_vel

        return TBvel

    def control_loop(self):

        # --------------------------------------------------------------------------
        ## CHECKER FOR THE CONTROL LOOP TIMING
        # --------------------------------------------------------------------------
        now = self.time()
        diff = (now - self.check_t)
        if diff > (1.1/self.ROS_RATE): # Add 10% extra margin
            self.get_logger().info('loop period: {:0.2f}'.format((now - self.check_t)*1000 ))     
        self.check_t = now

        # Showing Time Stamp
        if (self.it > 0) and (self.it % self.ROS_RATE == 0):
            t = self.it/self.ROS_RATE
            self.get_logger().info('Experiment t = {}s.'.format(t))
            # self.ctrlProfiling.printStatus()
        self.it += 1

        # --------------------------------------------------------------------------
        ## MAIN CONTROLLER COMPUTATION
        # --------------------------------------------------------------------------

        # set different spin for each robot
        for i in range(robot_num):
            robot_id = robot_list[i]

            if robot_id in self.global_poses:
                # Control the robot using the look-ahead point SI_UNI_ell away from the robot
                pos_i = self.global_poses[robot_id]['pos_lahead']
                ori_i = self.global_poses[robot_id]['ori_rpy']
                self.controller[i].update_state(pos_i, ori_i)
    
                # TODO: update other robot state for cooperative control

                cmd_si = self.controller[i].compute_control_input()            
                vel_lin, vel_ang = self.si2D_to_uni(cmd_si[0], cmd_si[1], ori_i[2], SI_UNI_ell)
                
                if DEBUG_SPIN:
                    # Debug with spinning
                    self.ros_pubs[i].publish(self.create_tb4_twist(0.0, 1.))
                else: 
                    if PRINT_CMD: print(cmd_si, vel_lin, vel_ang)
                    self.ros_pubs[i].publish(self.create_tb4_twist(vel_lin, vel_ang))


            else:
                self.get_logger().info('Warning, tb4_{} has no pose data.'.format(robot_id))


    # Allow CTRL+C to stop the controller and dump the log into pickle
    def signal_handler(self, sig, frame):
        print('You pressed Ctrl+C. Turning off the controller.')

        exit()


def main(args=None):
    rclpy.init(args=args)

    node = TB4Controller()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()