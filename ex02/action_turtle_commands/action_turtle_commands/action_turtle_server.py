import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer
import rclpy.time
from turtlesim.msg import Pose
import math

from message_turtle_commands.action import MessageTurtleCommands


class TurtleActionServer(Node):
    def __init__(self):
        super().__init__('turtle_action_server')
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'turtle_action',
            self.execute_callback)
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.current_pose = Pose()
        self.initial_pose = Pose()

    def pose_callback(self, msg):
        self.current_pose = msg

    def execute_callback(self, goal_handle):
        feedback_msg = MessageTurtleCommands.Feedback()
        result = MessageTurtleCommands.Result()

        self.initial_pose = self.current_pose

        command = goal_handle.request.command
        distance = goal_handle.request.s
        angle = goal_handle.request.angle

        if command == "forward":
            self.move_forward(distance, feedback_msg, goal_handle)
        elif command == "turn_left":
            self.rotate(angle, feedback_msg, goal_handle)
        elif command == "turn_right":
            self.rotate(-angle, feedback_msg, goal_handle)

        goal_handle.succeed()
        result.result = True
        return result

    def move_forward(self, distance, feedback_msg, goal_handle):
        twist = Twist()
        twist.linear.x = 1.0

        while self.calculate_distance() < distance:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
            
        feedback_msg.odom += int(self.calculate_distance())
        
        print(feedback_msg.odom)
        goal_handle.publish_feedback(feedback_msg)
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

    def rotate(self, angle, feedback_msg, goal_handle):
        twist = Twist()
        twist.angular.z = math.radians(angle)

        while self.calculate_angle() < abs(angle):
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
            
        feedback_msg.odom = 0

        goal_handle.publish_feedback(feedback_msg)
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def calculate_distance(self):
        return math.sqrt((self.current_pose.x - self.initial_pose.x)**2 + (self.current_pose.y - self.initial_pose.y)**2)

    def calculate_angle(self):
        return abs(math.degrees(self.current_pose.theta - self.initial_pose.theta))

def main(args=None):
    rclpy.init(args=args)
    turtle_action_server = TurtleActionServer()
    rclpy.spin(turtle_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()