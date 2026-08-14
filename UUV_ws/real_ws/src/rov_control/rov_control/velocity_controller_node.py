import math



import rclpy

from rclpy.node import Node



from geometry_msgs.msg import Twist, TwistStamped

from std_msgs.msg import String

from pymavlink import mavutil





class ROVVelocityController(Node):

    def __init__(self):

        super().__init__('rov_velocity_controller')



        self.cmd_sub = self.create_subscription(

            Twist,

            '/rov/cmd_vel',

            self.cmd_callback,

            10

        )



        self.dvl_sub = self.create_subscription(

            TwistStamped,

            '/dvl/twist',

            self.dvl_callback,

            10

        )



        self.state_pub = self.create_publisher(

            TwistStamped,

            '/rov/state_twist',

            10

        )



        self.debug_pub = self.create_publisher(

            String,

            '/rov/manual_control_debug',

            10

        )



        self.cmd = Twist()

        self.dvl = TwistStamped()



        self.yaw = 0.0

        self.yaw_rate = 0.0

        self.yaw_target = 0.0

        self.yaw_initialized = False



        self.depth = 0.0

        self.depth_target = 0.0

        self.depth_initialized = False

        

        

        self.was_vertical_cmd = False    

        

        self.idepth = 0.0

        self.ki_depth = 40.0            

        

        self.ix = 0.0

        self.iy = 0.0

        self.ki_x = 80.0

        self.ki_y = 120.0

        



        self.kp_x = 500.0

        self.kp_y = 850.0

        self.kp_depth = 120.0

        self.kd_depth = 100.0

        self.kp_yaw_rate = 400.0

        self.kp_heading = 350.0



        self.max_xy = 700

        self.max_z = 300

        self.max_yaw = 600



        self.timeout = 0.02

        self.get_logger().info('Conectando controlador por MAVLink real en udpin:0.0.0.0:14550...')

        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

        self.master.wait_heartbeat()

        self.get_logger().info('Controlador conectado al vehículo')



        self.master.arducopter_arm()

        self.master.motors_armed_wait()

        self.get_logger().info('ROV armado')



        self.timer = self.create_timer(0.05, self.control_loop)



    def cmd_callback(self, msg):

        self.cmd = msg



    def dvl_callback(self, msg):

        self.dvl = msg



    def clamp(self, value, limit):

        return max(min(value, limit), -limit)



    def deadband(self, value, threshold):

        if abs(value) < threshold:

            return 0.0

        return value



    def wrap_angle(self, angle):

        return math.atan2(math.sin(angle), math.cos(angle))



    def update_mavlink_state(self):

        while True:

            msg = self.master.recv_match(blocking=False)

            if msg is None:

                break



            if msg.get_type() == 'ATTITUDE':

                self.yaw = float(msg.yaw)

                self.yaw_rate = float(msg.yawspeed)



                if not self.yaw_initialized:

                    self.yaw_target = self.yaw

                    self.yaw_initialized = True



            elif msg.get_type() == 'LOCAL_POSITION_NED':

                self.depth = float(msg.z)



                if not self.depth_initialized:

                    self.depth_target = self.depth

                    self.depth_initialized = True



    def get_body_velocity(self):

        vx_ned = self.dvl.twist.linear.x

        vy_ned = self.dvl.twist.linear.y

        vz = self.dvl.twist.linear.z



        c = math.cos(self.yaw)

        s = math.sin(self.yaw)



        vx = c * vx_ned + s * vy_ned

        vy = -s * vx_ned + c * vy_ned



        return vx, vy, vz



    def publish_state(self, vx, vy, vz):

        state = TwistStamped()

        state.header.stamp = self.get_clock().now().to_msg()

        state.header.frame_id = 'rov_body'



        state.twist.linear.x = vx

        state.twist.linear.y = vy

        state.twist.linear.z = vz

        state.twist.angular.z = self.yaw_rate



        self.state_pub.publish(state)



    def control_loop(self):



        self.get_logger().info(

            f"cmd: x={self.cmd.linear.x:.2f}, "

            f"y={self.cmd.linear.y:.2f}, "

            f"z={self.cmd.linear.z:.2f}, "

            f"yaw={self.cmd.angular.z:.2f}"

        )



        x_out = int(self.clamp(self.cmd.linear.x * 1000.0, 1000))

        y_out = int(self.clamp(self.cmd.linear.y * 1000.0, 1000))



        # MANUAL_CONTROL:

        # z = 0 ... 1000

        # 500 = neutro

        z_out = int(500 - self.clamp(self.cmd.linear.z * 500.0, 500))



        r_out = int(self.clamp(self.cmd.angular.z * 1000.0, 1000))

        

        print(

            f"x={x_out} y={y_out} z={z_out} r={r_out}",

            flush=True

        )



        self.master.mav.manual_control_send(

            self.master.target_system,

            x_out,

            y_out,

            z_out,

            r_out,

            0

        )





def main(args=None):

    rclpy.init(args=args)

    node = ROVVelocityController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()





if __name__ == '__main__':

    main()

