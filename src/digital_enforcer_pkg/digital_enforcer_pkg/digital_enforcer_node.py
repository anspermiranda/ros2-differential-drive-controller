import rclpy
from rclpy.node import Node
from sfr_coursework1_interface_package.msg import TaskSpacePose, WheelAngularVelocities
from sfr_coursework1_interface_package.srv import TurnRobotOn, TurnRobotOff
import math

class DigitalEnforcerNode(Node):
    def __init__(self):
        super().__init__('digital_enforcer_node')

        self.r = 0.09 #wheel radius
        self.l = 0.14 #distance between wheels
        self.T = 0.1 #sampling time
        
        #pose state
        self.x = 0.0
        self.y = 0.0
        self.phi_z = 0.0
        
        #wheel angular velocities
        self.wl = 0.0
        self.wr = 0.0
        
        #velocity limits
        self.v_min = -0.1
        self.v_max = 0.1
        
        #robot state (i.e it'll be OFF by default)
        self.robot_on = False
        
        #Service servers
        self.turn_on_service_server = self.create_service(srv_type=TurnRobotOn, srv_name='digital_enforcer/turn_robot_on', callback=self.turn_on_service_callback)
        self.turn_off_service_server = self.create_service(srv_type= TurnRobotOff, srv_name='digital_enforcer/turn_robot_off', callback=self.turn_off_service_callback)
        
        # Publishers & Subscribers
        self.task_space_pose_publisher = self.create_publisher(msg_type=TaskSpacePose, topic='digital_enforcer/task_space_pose', qos_profile=10)
        self.wheel_angular_velocites_subscriber = self.create_subscription(msg_type=WheelAngularVelocities, topic='digital_enforcer/wheel_angular_velocities', callback=self.wheel_angular_velocities_subscriber_callback, qos_profile=10)
        
        #Timer for 10 Hz updates
        self.timer = self.create_timer(self.T, self.timer_callback)
        self.get_logger().info("Digital Enforcer node initialzed (state = OFF).")
    
    #Service Callbacks
    def turn_on_service_callback(self, request, response):
        if not self.robot_on:
            self.robot_on = True
            response.success = True
            self.get_logger().info("Robot turned ON.")
        else:
            response.success = False
            self.get_logger().info("Robot is already ON.")
        return response
    
    def turn_off_service_callback(self, request, response):
        if self.robot_on:
            self.robot_on = False
            self.wl = 0.0
            self.wr = 0.0
            response.success = True
            self.get_logger().info("Robot turned OFF.")
        else:
            response.success = False
            self.get_logger().info("Robot is already OFF.")
        return response

    
    # Timer Callback
    def timer_callback(self):
        if not self.robot_on:
            return
        # Compute linear and angular velocities
        v = (self.r / 2.0) * (self.wr + self.wl)
        w = (self.r / self.l) * (self.wr - self.wl)
        
        # Update pose using discrete integration
        self.x += v * math.cos(self.phi_z) * self.T
        self.y += v * math.sin(self.phi_z) * self.T
        self.phi_z += w * self.T
        
        # Publish the pose
        msg = TaskSpacePose()
        msg.x = self.x
        msg.y = self.y
        msg.phi_z = self.phi_z
        self.task_space_pose_publisher.publish(msg)
        
    # Subscriber Callback
    def wheel_angular_velocities_subscriber_callback(self, msg: WheelAngularVelocities):
        if not self.robot_on:
            return
        
        # Extract angular velocities (rad/s)
        omega_r = msg.right_wheel_angular_velocity
        omega_l = msg.left_wheel_angular_velocity
        
        # Convert to linear velocity and clamp within limits
        v_r = self.r * omega_r
        v_l = self.r * omega_l
        
        v_r = max(self.v_min, min(v_r, self.v_max))
        v_l = max(self.v_min, min(v_l, self.v_max))
        
        # Convert clamped linear velocity back to angular for internal use
        self.wr = v_r / self.r
        self.wl = v_l / self.r

def main(args=None):
    try:
        rclpy.init(args=args)
        digital_enforcer_node = DigitalEnforcerNode()
        rclpy.spin(digital_enforcer_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()