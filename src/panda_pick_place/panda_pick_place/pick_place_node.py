"""Left/right pick-and-place using joint waypoints, a trajectory controller and the gripper."""

import math
import sys

from control_msgs.action import FollowJointTrajectory
from franka_msgs.action import Grasp, Move
import rclpy
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [f'panda_joint{i}' for i in range(1, 8)]
WAYPOINTS = ['home', 'pick_above', 'pick', 'place_above', 'place']


class PickPlaceNode(Node):

    def __init__(self):
        super().__init__('pick_place_node')
        for name in WAYPOINTS:
            self.declare_parameter(name, [0.0] * 7)
        self.declare_parameter('max_joint_speed', 0.2)
        self.declare_parameter('min_segment_time', 2.0)
        self.declare_parameter('gripper_open_width', 0.08)
        self.declare_parameter('gripper_speed', 0.05)
        self.declare_parameter('grasp_width', 0.03)
        self.declare_parameter('grasp_force', 20.0)
        self.declare_parameter('grasp_epsilon', 0.05)
        self.declare_parameter('repeat', 1)
        self.declare_parameter('use_gripper', True)
        self.declare_parameter('start_delay', 3.0)

        self.waypoints = {
            name: list(self.get_parameter(name).value) for name in WAYPOINTS}
        for name, q in self.waypoints.items():
            if len(q) != 7:
                raise ValueError(f'waypoint {name} must have 7 joint values')

        self.arm = ActionClient(
            self, FollowJointTrajectory, '/panda_arm_controller/follow_joint_trajectory')
        self.gripper_move = ActionClient(self, Move, '/panda_gripper/move')
        self.gripper_grasp = ActionClient(self, Grasp, '/panda_gripper/grasp')

        self.current = None
        self.active_arm_goal = None
        self.create_subscription(JointState, '/franka/joint_states', self._on_joints, 10)

    def _on_joints(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            self.current = [pos[j] for j in JOINTS]

    def _wait(self, future, timeout=None):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def _sleep(self, seconds):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def connect(self):
        clients = [(self.arm, 'arm trajectory controller')]
        if self.get_parameter('use_gripper').value:
            clients += [(self.gripper_move, 'gripper move'),
                        (self.gripper_grasp, 'gripper grasp')]
        for client, label in clients:
            self.get_logger().info(f'Waiting for {label}...')
            while rclpy.ok() and not client.wait_for_server(timeout_sec=1.0):
                pass
        self.get_logger().info('Waiting for joint states...')
        while rclpy.ok() and self.current is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return rclpy.ok()

    def move_to(self, name):
        target = self.waypoints[name]
        distance = max(abs(t - c) for t, c in zip(target, self.current))
        duration = max(distance / self.get_parameter('max_joint_speed').value,
                       self.get_parameter('min_segment_time').value)

        point = JointTrajectoryPoint()
        point.positions = target
        point.velocities = [0.0] * 7
        sec = math.floor(duration)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = int((duration - sec) * 1e9)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        goal.trajectory.points = [point]

        self.get_logger().info(f'-> {name} ({duration:.1f} s)')
        handle = self._wait(self.arm.send_goal_async(goal))
        if handle is None or not handle.accepted:
            self.get_logger().error(f'Trajectory to {name} rejected')
            return False
        self.active_arm_goal = handle
        result = self._wait(handle.get_result_async())
        self.active_arm_goal = None
        if result is None or result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f'Trajectory to {name} failed')
            return False
        self.current = list(target)
        return True

    def stop_arm(self):
        """Cancel the trajectory in flight so the arm halts instead of finishing the move."""
        if self.active_arm_goal is not None:
            self.get_logger().info('Stopping arm')
            self._wait(self.active_arm_goal.cancel_goal_async(), timeout=2.0)
            self.active_arm_goal = None

    def open_gripper(self):
        if not self.get_parameter('use_gripper').value:
            return True
        goal = Move.Goal()
        goal.width = float(self.get_parameter('gripper_open_width').value)
        goal.speed = float(self.get_parameter('gripper_speed').value)
        return self._run_gripper(self.gripper_move, goal, 'open')

    def close_gripper(self):
        if not self.get_parameter('use_gripper').value:
            return True
        goal = Grasp.Goal()
        goal.width = float(self.get_parameter('grasp_width').value)
        goal.speed = float(self.get_parameter('gripper_speed').value)
        goal.force = float(self.get_parameter('grasp_force').value)
        eps = float(self.get_parameter('grasp_epsilon').value)
        goal.epsilon.inner = eps
        goal.epsilon.outer = eps
        return self._run_gripper(self.gripper_grasp, goal, 'grasp')

    def _run_gripper(self, client, goal, label):
        self.get_logger().info(f'gripper: {label}')
        handle = self._wait(client.send_goal_async(goal))
        if handle is None or not handle.accepted:
            self.get_logger().error(f'Gripper {label} rejected')
            return False
        result = self._wait(handle.get_result_async())
        if result is None or not result.result.success:
            error = result.result.error if result else 'no result'
            self.get_logger().error(f'Gripper {label} failed: {error}')
            return False
        return True

    def run(self):
        if not self.connect():
            return False
        self._sleep(self.get_parameter('start_delay').value)
        if not self.move_to('home'):
            return False

        if not self.open_gripper():
            return False

        # home -> A(left) -> close -> B(right) -> open -> A -> close -> B -> open -> ...
        # The arm only returns home when the loop ends (Ctrl+C or `repeat` reached).
        steps = [
            lambda: self.move_to('pick_above'),
            lambda: self.move_to('pick'),
            lambda: self.close_gripper(),
            lambda: self.move_to('pick_above'),
            lambda: self.move_to('place_above'),
            lambda: self.move_to('place'),
            lambda: self.open_gripper(),
            lambda: self.move_to('place_above'),
        ]
        cycle = 0
        repeat = self.get_parameter('repeat').value
        while rclpy.ok() and (repeat == 0 or cycle < repeat):
            cycle += 1
            self.get_logger().info(f'Cycle {cycle}')
            for step in steps:
                if not step():
                    return False
        return self.move_to('home')

    def stop_and_go_home(self):
        """Halt wherever the arm is, then return to the home pose."""
        self.stop_arm()
        self._sleep(0.5)  # let the joint state catch up with the halted arm
        self.get_logger().info('Returning to home')
        try:
            return self.move_to('home')
        except KeyboardInterrupt:
            self.get_logger().warn('Interrupted again, stopping')
            self.stop_arm()
            return True


def main():
    # Keep the context alive on Ctrl+C so the running trajectory can be cancelled.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PickPlaceNode()
    ok = False
    try:
        try:
            ok = node.run()
        except KeyboardInterrupt:
            node.get_logger().info('Interrupted')
            ok = node.stop_and_go_home()
    finally:
        node.stop_arm()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
