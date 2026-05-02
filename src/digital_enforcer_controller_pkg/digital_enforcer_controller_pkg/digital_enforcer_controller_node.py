import math
import rclpy
from rclpy.node import Node
from sfr_coursework1_interface_package.msg import WheelAngularVelocities, TaskSpacePose
from sfr_coursework1_interface_package.srv import TurnRobotOn, TurnRobotOff

class DigitalEnforcerControllerNode(Node):
    
    def __init__(self):
        super().__init__('digital_enforcer_controller_node')

        # Robot parameters
        self.r = 0.09
        self.l = 0.14
        self.v_max = 0.1
        self.v_min = -0.1

        # Task parameters
        self.desired_angle_deg = -145.0
        self.desired_angle = math.radians(self.desired_angle_deg)
        self.forward_distance = 1.0
        self.tolerance_angle = 0.01
        self.tolerance_distance = 0.001
        self.rotation_speed = 0.05
        self.forward_speed = 0.05

        # State variables
        self.phase = 'rotate'
        self.x_init = None
        self.y_init = None
        self.x_current = 0.0
        self.y_current = 0.0
        self.phi_z = 0.0

        # Publisher & Subscriber
        self.wheel_pub = self.create_publisher(WheelAngularVelocities,'digital_enforcer/wheel_angular_velocities', 10)
        self.pose_sub = self.create_subscription(TaskSpacePose,'digital_enforcer/task_space_pose',self.pose_callback, 10)

        # Service clients
        self.turn_on_client = self.create_client(TurnRobotOn, 'digital_enforcer/turn_robot_on')
        self.turn_off_client = self.create_client(TurnRobotOff, 'digital_enforcer/turn_robot_off')

        # Wait for services
        self.get_logger().info("Controller: Waiting for robot services...")
        while not self.turn_on_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("turn_robot_on service not available, waiting...")
        while not self.turn_off_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("turn_robot_off service not available, waiting...")
        self.get_logger().info("Controller: Robot services are available.")

        # Async future objects
        self.future_turn_on = None
        self.future_turn_off = None

        # Turn robot ON asynchronously
        self.turn_robot_on()

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    # Pose callback
    def pose_callback(self, msg: TaskSpacePose):
        self.x_current = msg.x
        self.y_current = msg.y
        self.phi_z = msg.phi_z

        # Capturing initial pose at first callback
        if self.x_init is None or self.y_init is None:
            self.x_init = self.x_current
            self.y_init = self.y_current
            self.get_logger().info(f"Controller: Initial pose : x={self.x_init}, y={self.y_init}")

    # Turn robot ON asynchronously
    def turn_robot_on(self):
        req = TurnRobotOn.Request()
        self.future_turn_on = self.turn_on_client.call_async(req)
        self.future_turn_on.add_done_callback(self.turn_on_response)

    def turn_on_response(self, future):
        result = future.result()
        if result.success:
            self.get_logger().info("Controller: Robot turned ON successfully.")
        else:
            self.get_logger().info("Controller: Robot already ON.")

    # Turn robot OFF asynchronously
    def turn_robot_off(self):
        req = TurnRobotOff.Request()
        self.future_turn_off = self.turn_off_client.call_async(req)
        self.future_turn_off.add_done_callback(self.turn_off_response)

    def turn_off_response(self, future):
        result = future.result()
        if result.success:
            self.get_logger().info("Controller: Robot turned OFF successfully.")
        else:
            self.get_logger().info("Controller: Robot already OFF.")

    # Publish wheel velocities
    def publish_wheel_velocities(self, omega_r, omega_l):
        msg = WheelAngularVelocities()
        msg.right_wheel_angular_velocity = omega_r
        msg.left_wheel_angular_velocity = omega_l
        self.wheel_pub.publish(msg)

    # Control loop
    def control_loop(self):
        if self.x_init is None or self.y_init is None:
            # Wait until initial pose is known
            return

        if self.phase == 'rotate':
            # Rotate to desired angle
            angle_error = self.desired_angle - self.phi_z
            if abs(angle_error) < self.tolerance_angle:
                self.publish_wheel_velocities(0.0, 0.0)
                self.get_logger().info("Rotation complete. Switching to forward movement.")
                self.get_logger().info(f"Pose after rotation: x={self.x_current} m, y={self.y_current} m, phi={self.phi_z}(in rads)")
                self.phase = 'forward'
            else:
                omega_robot = self.rotation_speed if angle_error > 0 else -self.rotation_speed
                omega_r = (self.l / (2 * self.r)) * omega_robot
                omega_l = -omega_r
                omega_r = max(min(omega_r, self.v_max / self.r), self.v_min / self.r)
                omega_l = max(min(omega_l, self.v_max / self.r), self.v_min / self.r)
                self.publish_wheel_velocities(omega_r, omega_l)

        elif self.phase == 'forward':
            dx = self.x_current - self.x_init
            dy = self.y_current - self.y_init
            distance = math.sqrt(dx**2 + dy**2)
            if distance >= self.forward_distance - self.tolerance_distance:
                self.publish_wheel_velocities(0.0, 0.0)
                self.get_logger().info("Forward movement complete. Switching to stop robot.")
                self.get_logger().info(f"Pose after forward movement: x={self.x_current} m, y={self.y_current} m,  phi={self.phi_z}(in rads)")
                self.phase = 'stop'
            else:
                omega_r = self.forward_speed / self.r
                omega_l = self.forward_speed / self.r
                omega_r = max(min(omega_r, self.v_max / self.r), self.v_min / self.r)
                omega_l = max(min(omega_l, self.v_max / self.r), self.v_min / self.r)
                self.publish_wheel_velocities(omega_r, omega_l)

        elif self.phase == 'stop':
            self.publish_wheel_velocities(0.0, 0.0)
            self.turn_robot_off()
            self.get_logger().info("Robot stopped. Task complete.")
            self.phase = 'done'   # prevent further calls
        
        elif self.phase == 'done':
            # Node stays alive but performs no further actions
            pass

# Main function
def main(args=None):
    try:
        rclpy.init(args=args)
        digital_enforcer_controller_node = DigitalEnforcerControllerNode()
        rclpy.spin(digital_enforcer_controller_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()
