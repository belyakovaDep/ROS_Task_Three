import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import rclpy.time
from message_turtle_commands.action import MessageTurtleCommands

class TurtleActionClient(Node):
    def __init__(self):
        super().__init__('turtle_action_client')
        self._action_client = ActionClient(self, MessageTurtleCommands, 'turtle_action')
        self._futures = []
        self.res = 0
        self.count_goals = 0

    def send_goal(self, command, distance, angle):
        goal_msg = MessageTurtleCommands.Goal()
        goal_msg.command = command
        goal_msg.s = distance
        goal_msg.angle = angle

        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self._futures.append(send_goal_future)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
        self._futures.append(get_result_future)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))

        if len(self._futures) == 0:
            rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.res+=feedback_msg.feedback.odom
        self.count_goals+=1
        self.get_logger().info(f'Received feedback: {self.res} meters')

def main(args=None):
    rclpy.init(args=args)
    action_client = TurtleActionClient()

    goals = [
        ("forward", 2, 0),
        ("turn_right", 0, 90),
        ("forward", 1, 0)
    ]

    for command, distance, angle in goals:
        action_client.send_goal(command, distance, angle)

    while rclpy.ok():
        rclpy.spin_once(action_client)
        if action_client.count_goals == len(goals):
            break

if __name__ == '__main__':
    main()