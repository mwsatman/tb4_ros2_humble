import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from functools import partial
import signal
import numpy as np

robot_list = ['02']
# robot_list = ['01', '02', '03', '05']
robot_num = len(robot_list)


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


class TB4Controller(Node):

    def __init__(self):
        super().__init__('tb4_controller')

        self.ros_pubs = []
        self.global_poses = {}

        for robot_id in robot_list:
            tb_name = f'tb4_' + robot_id

            # Create pose subscribers
            self.get_logger().info(f'Creating pos ahead subscriber /{tb_name}/pos')
            self.pose_sub = self.create_subscription(PoseStamped,
                                    f'/vrpn_mocap/tb_{robot_id}/pose',
                                    partial(self.pose_callback, index=robot_id),
                                    qos_profile=qos_profile_sensor_data)

            # create handler for the publisher of the control input
            self.ros_pubs += [self.create_publisher(Twist, '/{}/cmd_vel'.format(tb_name), 10)]


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
        self.global_poses[index] = msg

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
        ang_vel = [0.5, -0.5, 1.0, -1.0]
        

        for i in range(robot_num):
            robot_id = robot_list[i]

            if robot_id in self.global_poses:
            
                # goal_x, goal_y = 6, 6
                goal_x, goal_y = 3, -0.5

                u_x = goal_x - self.global_poses[robot_id].pose.position.x
                u_y = goal_y - self.global_poses[robot_id].pose.position.y

                # change from quaternion to xyz
                quat = self.global_poses[robot_id].pose.orientation
                roll, pitch, yaw = euler_from_quat(quat.x, quat.y, quat.z, quat.w)

                print(u_x, u_y)
                vel_lin = u_x*np.cos(yaw) + u_y*np.sin(yaw)
                vel_ang = (- u_x*np.sin(yaw) + u_y*np.cos(yaw))/0.1
                # print(vel_lin, vel_ang) 
            
                self.ros_pubs[i].publish(self.create_tb4_twist(vel_lin, vel_ang))
                # self.ros_pubs[i].publish(self.create_tb4_twist(0.0, ang_vel[i]))


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