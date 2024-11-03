import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import sys

class TurtleController(Node):
    def __init__(self, x_goal, y_goal, theta_goal):
        super().__init__('turtle_controller')
        
        self.x_goal = x_goal
        self.y_goal = y_goal
        self.theta_goal = theta_goal
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.first = True

        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.move_to_goal)

    def pose_callback(self, data):
        self.x = data.x
        self.y = data.y
        self.theta = data.theta

    def get_angle_to_goal(self):
        angle_to_goal = math.atan2(self.y_goal - self.y, self.x_goal - self.x)
        angle_diff = angle_to_goal - self.theta

        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        return angle_diff

    def move_to_goal(self):
        vel_msg = Twist()
        
        distance = math.sqrt((self.x_goal - self.x)**2 + (self.y_goal - self.y)**2)
        
        angle_to_goal = self.get_angle_to_goal()

        if abs(angle_to_goal) > 0.01 and self.first:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.5 * angle_to_goal
        elif distance >= 0.1:
            vel_msg.linear.x = min(1.0, 0.5 * distance)
            vel_msg.angular.z = 0.0
            self.first = False
        else:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.5 * abs(self.theta - self.theta_goal)
        

        self.velocity_publisher.publish(vel_msg)

        if distance < 0.1 and abs(self.theta - self.theta_goal) < 0.1:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(vel_msg)
            self.get_logger().info('Goal reached')
            self.destroy_timer(self.timer)

def main(args=None):

    coms = sys.argv
    x_goal = float(coms[1])
    y_goal = float(coms[2])
    theta_goal = float(coms[3])

    if not (0.0 <= x_goal <= 11.0 and 0.0 <= y_goal <= 11.0 and -math.pi <= theta_goal <= math.pi):
        print("Error: wrong argument")
        sys.exit()

    rclpy.init(args=args)
    turtle_controller = TurtleController(x_goal, y_goal, theta_goal)
    
    try:
        rclpy.spin(turtle_controller)
    except KeyboardInterrupt:
        pass

    turtle_controller.destroy_node()
    rclpy.shutdown()
    sys.exit()

if __name__ == '__main__':
    main()