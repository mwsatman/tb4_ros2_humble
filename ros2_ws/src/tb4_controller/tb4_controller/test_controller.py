import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import signal

robot_list = ['01', '02', '03', '05']
robot_num = len(robot_list)

class TB4Controller(Node):

    def __init__(self):
        super().__init__('tb4_controller')

        self.ros_pubs = []

        for robot_id in robot_list:
            tb_name = f'tb4_' + robot_id

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
            self.ros_pubs[i].publish(self.create_tb4_twist(0.0, ang_vel[i]))


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