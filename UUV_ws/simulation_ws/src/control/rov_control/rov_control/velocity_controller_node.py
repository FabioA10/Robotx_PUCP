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

        self.get_logger().info('Conectando controlador por MAVLink en puerto 14552...')
        self.master = mavutil.mavlink_connection('udpin:127.0.0.1:14552')
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
        self.update_mavlink_state()

        vx_ref = self.clamp(self.cmd.linear.x, 0.5)
        vy_ref = self.clamp(self.cmd.linear.y, 0.5)
        z_cmd = self.clamp(self.cmd.linear.z, 0.2)
        yaw_rate_ref = self.clamp(self.cmd.angular.z, 0.8)

        vx, vy, vz = self.get_body_velocity()
        
    

        # Prioridad: no mandar avance y lateral fuerte al mismo tiempo
        if abs(vx_ref) > abs(vy_ref):
            vy_ref = 0.0
        elif abs(vy_ref) > abs(vx_ref):
            vx_ref = 0.0

        # ---------------------------------
        # CONTROL HORIZONTAL DESACOPLADO
        # ---------------------------------

        ex = vx_ref - vx
        ey = vy_ref - vy

        dt = 0.05

        if abs(vx_ref) > 0.05:
            self.iy += ey * dt
            self.iy = self.clamp(self.iy, 1.0)
        else:
            self.iy *= 0.95

        if abs(vy_ref) > 0.05:
            self.ix += ex * dt
            self.ix = self.clamp(self.ix, 1.0)
        else:
            self.ix *= 0.95
            
            
        # Si avanzo, cancelar drift lateral más agresivamente
        if abs(vx_ref) > 0.05:
            ey *= 2.0

        # Si voy lateral, cancelar drift frontal
        if abs(vy_ref) > 0.05:
            ex *= 2.0

        ex = self.deadband(ex, 0.04)
        ey = self.deadband(ey, 0.04)

        x_control = self.kp_x * ex + self.ki_x * self.ix
        y_control = self.kp_y * ey + self.ki_y * self.iy

        x_out = int(
            self.clamp(x_control, self.max_xy)
        )

        y_out = int(
            self.clamp(y_control, self.max_xy)
        )

        # Profundidad:
        # z_cmd < 0: subir manualmente
        # z_cmd > 0: bajar manualmente
        # z_cmd = 0: mantener profundidad objetivo
        # SUBIR / BAJAR MANUAL
        # SUBIR / BAJAR MANUAL
        # Profundidad:
        # z_cmd < 0: subir
        # z_cmd > 0: bajar
        # z_cmd = 0: mantener profundidad objetivo

        # -----------------------------
        # CONTROL VERTICAL / DEPTH HOLD
        # -----------------------------
        # z_cmd > 0: bajar
        # z_cmd < 0: subir
        # z_cmd = 0: mantener profundidad actual

        # -----------------------------
        # CONTROL VERTICAL / DEPTH HOLD
        # -----------------------------
        # z_cmd > 0: bajar
        # z_cmd < 0: subir
        # z_cmd = 0: mantener profundidad actual

        vertical_cmd = abs(z_cmd) > 0.01

        if vertical_cmd:
            manual_z = 900.0 * z_cmd
            self.was_vertical_cmd = True

        else:
            manual_z = 0.0

            # justo al soltar f/g, fijar nueva profundidad objetivo
            if self.was_vertical_cmd:
                self.depth_target = self.depth
                self.was_vertical_cmd = False

        depth_error = self.depth_target - self.depth

        # Integral solo cuando NO estás presionando subir/bajar
        if not vertical_cmd:
            self.idepth += depth_error * 0.05
            self.idepth = self.clamp(self.idepth, 1.0)
        else:
            self.idepth *= 0.9

        depth_control = (
            self.kp_depth * depth_error
            - self.kd_depth * vz
            + self.ki_depth * self.idepth
            + manual_z
        )

        z_out = int(
            500 - self.clamp(depth_control, self.max_z)
        )
        
        
        # Heading:
        # Si mando giro, controlo yaw_rate.
        # Si no mando giro, mantengo heading.
        if abs(yaw_rate_ref) > 0.01:
            self.yaw_target = self.yaw
            eyaw = yaw_rate_ref - self.yaw_rate
            r_out = int(self.clamp(self.kp_yaw_rate * eyaw, self.max_yaw))
        else:
            heading_error = self.wrap_angle(self.yaw_target - self.yaw)
            r_out = int(self.clamp(self.kp_heading * heading_error - 120.0 * self.yaw_rate, self.max_yaw))
            eyaw = heading_error

        self.master.mav.manual_control_send(
            self.master.target_system,
            x_out,
            y_out,
            z_out,
            r_out,
            0
        )

        self.publish_state(vx, vy, vz)

        debug = String()
        debug.data = (
            f'vx_ref={vx_ref:.2f}, vx={vx:.2f}, ex={ex:.2f}, x_out={x_out}, '
            f'vy_ref={vy_ref:.2f}, vy={vy:.2f}, ey={ey:.2f}, y_out={y_out}, '
            f'z_cmd={z_cmd:.2f}, depth={self.depth:.2f}, '
            f'depth_target={self.depth_target:.2f}, vz={vz:.2f}, z_out={z_out}, '
            f'yaw={math.degrees(self.yaw):.1f}, yaw_target={math.degrees(self.yaw_target):.1f}, '
            f'yaw_rate_ref={yaw_rate_ref:.2f}, yaw_rate={self.yaw_rate:.2f}, '
            f'eyaw={eyaw:.2f}, r_out={r_out}'
        )
        self.debug_pub.publish(debug)


def main(args=None):
    rclpy.init(args=args)
    node = ROVVelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
