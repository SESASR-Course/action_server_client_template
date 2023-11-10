# This template is derived from ROS 2 action cliet example. You can find the complete example here:
# https://github.com/ros2/examples/blob/master/rclpy/actions/minimal_action_client/examples_rclpy_minimal_action_client/client_asyncio.py


import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from lab02_interfaces.srv import ComputeTrajectory
from turtlesim.action import RotateAbsolute

import asyncio


class GoalGeneratorActionClient(Node):
    def __init__(self):
        super().__init__("goal_generator_action_client")

        # Create a Service client instance
        self.compute_traj_cli = self.create_client(ComputeTrajectory, "compute_trajectory")
        # Create an ActionClient instance
        self.rotate_absolute_client = ActionClient(self, RotateAbsolute, "rotate_absolute")

        # Example to set the log level from a parameter
        log_level = self.declare_parameter("log_level", "info")
        if log_level.get_parameter_value().string_value == "debug":
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        else:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

    def rotate_absolute_get_feedback(self, feedback_msg):
        '''
        Callback function to get action feedback callback
        '''
        feedback = feedback_msg.feedback
        # the throttle duration is used to limit the rate at which the message is printed
        self.get_logger().info(f"Remaining: {feedback.remaining}", throttle_duration_sec=1.0)

    # Async call of send goal function (call anction server)
    async def send_goal(self, client: ActionClient, goal, feedback_callback):
        '''
        Async function to send goal to an action server

        Arguments:
            client {ActionClient} -- Action client instance
            goal {object} -- Goal object
            feedback_callback {function} -- Callback function to get feedback

        Returns:
            result {object or None} -- Result object
            status {int or None} -- Status code
        '''
        self.get_logger().info("Waiting for action server...")
        client.wait_for_server()

        self.get_logger().info("Sending goal request...")

        goal_handle = await client.send_goal_async(goal, feedback_callback=feedback_callback)

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            return None, None
        
        action_result = await goal_handle.get_result_async()

        result = action_result.result
        status = action_result.status

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn("Goal failed with status: {0}".format(status))

        return result, status


async def spinning(node):
    '''
    Async function to spin the node. 
    This function is used to spin the node inside the run function
    '''
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.001)


async def run(args, loop):
    '''
    The function where you can write your code
    '''
    rclpy.init(args=args)

    # Instantiate the node and spin it
    goal_generator = GoalGeneratorActionClient()
    spin_task = loop.create_task(spinning(goal_generator))

    ################################################################################################
    # YOUR CODE SECTION STARTS HERE
    ################################################################################################
    goal = [8.0, 5.5, 1.8769]

    goal_generator.get_logger().debug("Sending goal: " + str(goal))

    # Example of calling a SERVICE using async/await
    goal_generator.compute_traj_cli.wait_for_service()
    request = ComputeTrajectory.Request()
    request.x = goal[0]
    request.y = goal[1]
    future = goal_generator.compute_traj_cli.call_async(request)
    response = await future

    # Example of calling the rotate absolute ACTION SERVER using async/await
    goal_angle = RotateAbsolute.Goal()
    goal_angle.theta = goal[2]
    result, status = await loop.create_task(
        goal_generator.send_goal(
            goal_generator.rotate_absolute_client,
            goal_angle,
            goal_generator.rotate_absolute_get_feedback,
        )
    )

    goal_generator.get_logger().info("All goals completed! Shutting down...")
    spin_task.cancel()

    ################################################################################################
    # YOUR CODE SECTION END HERE
    ################################################################################################

    # Wait for the spinning task to finish and destroy the node
    try:
        await spin_task
    except asyncio.exceptions.CancelledError:
        pass
    finally:
        goal_generator.destroy_node()
        rclpy.try_shutdown()


def main(args=None):
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(args, loop))
    loop.close()


if __name__ == "__main__":
    main()
