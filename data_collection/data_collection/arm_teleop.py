import select
import termios
import tty
import sys

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmTeleop(Node):
    def __init__(self):
        super().__init__("arm_teleop")
        self.arm_step = 0.2
        self.gripper_step = 0.2
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_joint",
        ]
        self.positions = {name: None for name in self.joint_names}
        self.gripper_pos = None
        self.publisher = self.create_publisher(
            JointTrajectory, "/mirte_master_arm_controller/joint_trajectory", 1
        )
        self.gripper_client = ActionClient(
            self, GripperCommand, "/mirte_master_gripper_controller/gripper_cmd"
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._log("Arm teleop via q/a w/s e/d r/f t/g; Printing the joints via space bar")

    def _on_joint_state(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in self.positions:
                self.positions[name] = position
            if name == "gripper_joint":
                self.gripper_pos = position

    def handle_key(self, key):
        if key == " ":
            self._print_joint_states()
            return
        if key == "t":
            self._step_gripper(self.gripper_step)
            return
        if key == "g":
            self._step_gripper(-self.gripper_step)
            return
        mapping = {
            "q": ("shoulder_pan_joint", self.arm_step),
            "a": ("shoulder_pan_joint", -self.arm_step),
            "w": ("shoulder_lift_joint", self.arm_step),
            "s": ("shoulder_lift_joint", -self.arm_step),
            "e": ("elbow_joint", self.arm_step),
            "d": ("elbow_joint", -self.arm_step),
            "r": ("wrist_joint", self.arm_step),
            "f": ("wrist_joint", -self.arm_step),
        }
        if key in mapping:
            joint, delta = mapping[key]
            if self.positions[joint] is None:
                self.get_logger().warning(f"No position yet for {joint}")
                return
            self.positions[joint] += delta
            self._publish_trajectory()

    def _publish_trajectory(self):
        missing = [name for name in self.joint_names if self.positions[name] is None]
        if missing:
            self.get_logger().warning("No position yet for: " + ", ".join(missing))
            return
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [self.positions[name] for name in self.joint_names]
        point.time_from_start.sec = 1
        traj.points = [point]
        self.publisher.publish(traj)

    def _print_joint_states(self):
        states = [f"{name}={self.positions[name]}" for name in self.joint_names]
        states.append(f"gripper_joint={self.gripper_pos}")
        self._log("Joint states: " + ", ".join(states))

    def _step_gripper(self, delta):
        if self.gripper_pos is None:
            self._log("No gripper position yet")
            return
        self._send_gripper(self.gripper_pos + delta)

    def _send_gripper(self, position):
        if not self.gripper_client.wait_for_server(timeout_sec=0.5):
            self._log("Gripper action server not available")
            return
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 2.0

        def on_result(future):
            response = future.result()
            status = {
                GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                GoalStatus.STATUS_CANCELED: "CANCELED",
                GoalStatus.STATUS_ABORTED: "ABORTED",
            }.get(response.status, str(response.status))
            self._log(
                "Gripper result: "
                f"status={status}, "
                f"position={response.result.position:.4f}, "
                f"effort={response.result.effort:.4f}, "
                f"stalled={response.result.stalled}, "
                f"reached_goal={response.result.reached_goal}"
            )

        def on_goal(future):
            goal_handle = future.result()
            if goal_handle.accepted:
                goal_handle.get_result_async().add_done_callback(on_result)
            else:
                self._log("Gripper goal rejected")

        self._log(f"Sending gripper goal: goal position={position:.4f}, current position={self.gripper_pos:.4f}")
        self.gripper_client.send_goal_async(goal).add_done_callback(on_goal)

    def _log(self, message):
        sys.stdout.write("\r\n")
        sys.stdout.flush()
        self.get_logger().info(message)

def _keyboard_loop(node):
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == "\x03":
                    break
                node.handle_key(key)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleop()
    try:
        _keyboard_loop(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
