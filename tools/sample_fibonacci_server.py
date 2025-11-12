#!/usr/bin/env python3

"""Standalone Fibonacci action server for integration testing."""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from example_interfaces.action import Fibonacci

ACTION_NAME = "/unity_fibonacci"


class FibonacciServer(Node):
    def __init__(self):
        super().__init__("fibonacci_test_server")
        self._action_server = ActionServer(
            self,
            Fibonacci,
            ACTION_NAME,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def goal_callback(self, goal_request):
        if goal_request.order <= 0:
            self.get_logger().warn(
                "Rejecting goal with non-positive order %s" % goal_request.order
            )
            return GoalResponse.REJECT
        self.get_logger().info("Accepting Fibonacci goal order=%d" % goal_request.order)
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        sequence = [0, 1]
        feedback = Fibonacci.Feedback()
        for i in range(2, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                feedback.sequence = sequence
                self.get_logger().info("Goal canceled")
                result = Fibonacci.Result()
                result.sequence = sequence
                return result
            next_value = sequence[i - 1] + sequence[i - 2]
            sequence.append(next_value)
            feedback.sequence = sequence.copy()
            goal_handle.publish_feedback(feedback)
            self.get_logger().info("Publishing feedback: %s" % sequence)
            time.sleep(0.5)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
        self.get_logger().info("Goal succeeded: %s" % sequence)
        return result


def main():
    rclpy.init()
    node = FibonacciServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
