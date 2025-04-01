import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math


class MultiRobotVelocityPublisher(Node):
    def __init__(self):
        super().__init__('multi_robot_cmd_vel_publisher')

        self.robot_namespaces = ["robot1", "robot2", "robot3", "robot4", "robot5"]

        self.r1_vel_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.r2_vel_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        self.r3_vel_pub = self.create_publisher(Twist, '/robot3/cmd_vel', 10)
        self.r4_vel_pub = self.create_publisher(Twist, '/robot4/cmd_vel', 10)
        self.r5_vel_pub = self.create_publisher(Twist, '/robot5/cmd_vel', 10)

        self.r1_odom_sub = self.create_subscription(Odometry, '/robot1/odom', self.r1_odom_callback, 10)
        self.r2_odom_sub = self.create_subscription(Odometry, '/robot2/odom', self.r2_odom_callback, 10)
        self.r3_odom_sub = self.create_subscription(Odometry, '/robot3/odom', self.r3_odom_callback, 10)
        self.r4_odom_sub = self.create_subscription(Odometry, '/robot4/odom', self.r4_odom_callback, 10)
        self.r5_odom_sub = self.create_subscription(Odometry, '/robot5/odom', self.r5_odom_callback, 10)

        self.r1_yaw = 0.0
        self.r2_yaw = 0.0
        self.r3_yaw = 0.0
        self.r4_yaw = 0.0
        self.r5_yaw = 0.0

        self.r1_target_yaw = 0.0
        self.r2_target_yaw = 0.0
        self.r3_target_yaw = 0.0
        self.r4_target_yaw = 0.0
        self.r5_target_yaw = 0.0

        self.r1_error = 0.0
        self.r2_error = 0.0
        self.r3_error = 0.0
        self.r4_error = 0.0
        self.r5_error = 0.0

        self.r1_prev = 0.0
        self.r2_prev = 0.0
        self.r3_prev = 0.0
        self.r4_prev = 0.0
        self.r5_prev = 0.0

        self.Kp = 12.0
        self.Ki = 0.5
        self.Kd = 1.0

        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

    def yaw(self, msg):
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y 
        z = msg.pose.pose.orientation.z 
        w = msg.pose.pose.orientation.w 
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def publish_cmd_vel(self):
        # Create Twist messages for each robot
        r1_twist = Twist()
        r2_twist = Twist()
        r3_twist = Twist()
        r4_twist = Twist()
        r5_twist = Twist()
        
        # Set linear velocity (forward movement)
        linear_speed = 0.4  # Use a low, stable speed
        r1_twist.linear.x = linear_speed
        r2_twist.linear.x = linear_speed
        r3_twist.linear.x = linear_speed
        r4_twist.linear.x = linear_speed
        r5_twist.linear.x = linear_speed
        
        # PID control for robot1
        # P term is proportional to error
        r1_p_term = self.Kp * self.r1_error
        
        # I term accumulates error over time (needs to be stored as class variable)
        if not hasattr(self, 'r1_error_sum'):
            self.r1_error_sum = 0.0
        self.r1_error_sum += self.r1_error * 0.1  # 0.1 is the timer period
        r1_i_term = self.Ki * self.r1_error_sum
        
        # D term is the rate of change of error
        r1_error_change = self.r1_error - self.r1_prev
        r1_d_term = self.Kd * r1_error_change / 0.1  # 0.1 is the timer period
        
        # Calculate angular velocity for correction
        r1_angular_correction = r1_p_term + r1_i_term + r1_d_term
        r1_twist.angular.z = r1_angular_correction
        
        # PID control for robot2
        # P term is proportional to error
        r2_p_term = self.Kp * self.r2_error
        
        # I term accumulates error over time
        if not hasattr(self, 'r2_error_sum'):
            self.r2_error_sum = 0.0
        self.r2_error_sum += self.r2_error * 0.1
        r2_i_term = self.Ki * self.r2_error_sum
        
        # D term is the rate of change of error
        r2_error_change = self.r2_error - self.r2_prev
        r2_d_term = self.Kd * r2_error_change / 0.1
        
        # Calculate angular velocity for correction
        r2_angular_correction = r2_p_term + r2_i_term + r2_d_term
        r2_twist.angular.z = r2_angular_correction
        
        # PID control for robot3
        # P term is proportional to error
        r3_p_term = self.Kp * self.r3_error
        
        # I term accumulates error over time
        if not hasattr(self, 'r3_error_sum'):
            self.r3_error_sum = 0.0
        self.r3_error_sum += self.r3_error * 0.1
        r3_i_term = self.Ki * self.r3_error_sum
        
        # D term is the rate of change of error
        r3_error_change = self.r3_error - self.r3_prev
        r3_d_term = self.Kd * r3_error_change / 0.1
        
        # Calculate angular velocity for correction
        r3_angular_correction = r3_p_term + r3_i_term + r3_d_term
        r3_twist.angular.z = r3_angular_correction
        
        # PID control for robot4
        # P term is proportional to error
        r4_p_term = self.Kp * self.r4_error
        
        # I term accumulates error over time
        if not hasattr(self, 'r4_error_sum'):
            self.r4_error_sum = 0.0
        self.r4_error_sum += self.r4_error * 0.1
        r4_i_term = self.Ki * self.r4_error_sum
        
        # D term is the rate of change of error
        r4_error_change = self.r4_error - self.r4_prev
        r4_d_term = self.Kd * r4_error_change / 0.1
        
        # Calculate angular velocity for correction
        r4_angular_correction = r4_p_term + r4_i_term + r4_d_term
        r4_twist.angular.z = r4_angular_correction
        
        # PID control for robot5
        # P term is proportional to error
        r5_p_term = self.Kp * self.r5_error
        
        # I term accumulates error over time
        if not hasattr(self, 'r5_error_sum'):
            self.r5_error_sum = 0.0
        self.r5_error_sum += self.r5_error * 0.1
        r5_i_term = self.Ki * self.r5_error_sum
        
        # D term is the rate of change of error
        r5_error_change = self.r5_error - self.r5_prev
        r5_d_term = self.Kd * r5_error_change / 0.1
        
        # Calculate angular velocity for correction
        r5_angular_correction = r5_p_term + r5_i_term + r5_d_term
        r5_twist.angular.z = r5_angular_correction
        
        # Log corrections (for debugging)
        self.get_logger().info(f"R1 correction: {r1_angular_correction}, error: {self.r1_error}")
        self.get_logger().info(f"R2 correction: {r2_angular_correction}, error: {self.r2_error}")
        self.get_logger().info(f"R3 correction: {r2_angular_correction}, error: {self.r3_error}")
        self.get_logger().info(f"R4 correction: {r2_angular_correction}, error: {self.r4_error}")
        self.get_logger().info(f"R5 correction: {r2_angular_correction}, error: {self.r5_error}")
        
        # Publish the twist messages
        self.r1_vel_pub.publish(r1_twist)
        self.r2_vel_pub.publish(r2_twist)
        self.r3_vel_pub.publish(r3_twist)
        self.r4_vel_pub.publish(r4_twist)
        self.r5_vel_pub.publish(r5_twist)

    #P term is error
    #I term is sum of error
    #D term is diff of error


    def r1_odom_callback(self, msg):
        self.r1_yaw = self.yaw(msg)
        # self.get_logger().info(f"r1 yaw = {self.r1_yaw}")
        self.r1_prev = self.r1_error
        self.r1_error = self.r1_target_yaw - self.r1_yaw


    def r2_odom_callback(self, msg):
        self.r2_yaw = self.yaw(msg)
        # self.get_logger().info(f"r2 yaw = {self.r2_yaw}")
        self.r2_prev = self.r2_error
        self.r2_error = self.r2_target_yaw - self.r2_yaw

    def r3_odom_callback(self, msg):
        self.r3_yaw = self.yaw(msg)
        # self.get_logger().info(f"r3 yaw = {self.r3_yaw}")
        self.r3_prev = self.r3_error
        self.r3_error = self.r1_target_yaw - self.r3_yaw

    def r4_odom_callback(self, msg):
        self.r4_yaw = self.yaw(msg)
        # self.get_logger().info(f"r4 yaw = {self.r4_yaw}")
        self.r4_prev = self.r4_error
        self.r4_error = self.r4_target_yaw - self.r4_yaw

    def r5_odom_callback(self, msg):
        self.r5_yaw = self.yaw(msg)
        # self.get_logger().info(f"r5 yaw = {self.r5_yaw}")
        self.r5_prev = self.r5_error
        self.r5_error = self.r5_target_yaw - self.r5_yaw


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotVelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()