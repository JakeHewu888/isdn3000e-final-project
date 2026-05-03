from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState as MoveItRobotState
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import GetPositionIK
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from ttt_interfaces.msg import GameSnapshot, TurnPlan
from ttt_interfaces.srv import PlanTurn, RegisterPlayer


PANDA_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
HOME = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]

_LINK8_QUAT = (0.9238795325112867, -0.3826834323650898, 0.0, 0.0)
_LINK8_TCP_Z_OFFSET = 0.1034

WIN_LINES = (
    (0, 1, 2),
    (3, 4, 5),
    (6, 7, 8),
    (0, 3, 6),
    (1, 4, 7),
    (2, 5, 8),
    (0, 4, 8),
    (2, 4, 6),
)
CELL_PRIORITY = (4, 0, 2, 6, 8, 1, 3, 5, 7)


def _make_point(positions: list[float], seconds: float) -> JointTrajectoryPoint:
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    whole_seconds = int(seconds)
    point.time_from_start.sec = whole_seconds
    point.time_from_start.nanosec = int((seconds - whole_seconds) * 1e9)
    return point


def _make_trajectory(
    start: list[float], goal: list[float], total_time: float = 1.5
) -> RobotTrajectory:
    midpoint = [(a + b) * 0.5 for a, b in zip(start, goal)]
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list(PANDA_JOINTS)
    trajectory.joint_trajectory.points = [
        _make_point(start, 0.0),
        _make_point(midpoint, total_time * 0.5),
        _make_point(goal, total_time),
    ]
    return trajectory


class RuleBasedPlayerNode(Node):
    def __init__(self) -> None:
        super().__init__("rule_based_player")
        self.declare_parameter("player_name", self.get_name())
        self.declare_parameter("plan_turn_service", "/rule_based_player/plan_turn")

        self.player_name = str(self.get_parameter("player_name").value)
        self.plan_turn_service = str(self.get_parameter("plan_turn_service").value)
        self.registered = False
        self.registration_in_flight = False
        self.player_id = 255

        self.cb_group = ReentrantCallbackGroup()

        self.register_client = self.create_client(
            RegisterPlayer, "/ttt/register_player"
        )
        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=self.cb_group
        )
        self.plan_service = self.create_service(
            PlanTurn,
            self.plan_turn_service,
            self._handle_plan_turn,
            callback_group=self.cb_group,
        )
        self.register_timer = self.create_timer(0.5, self._try_register)

    def _try_register(self) -> None:
        if self.registered or self.registration_in_flight:
            return
        if not self.register_client.wait_for_service(timeout_sec=0.1):
            return

        request = RegisterPlayer.Request()
        request.player_name = self.player_name
        request.service_name = self.plan_turn_service
        self.registration_in_flight = True
        future = self.register_client.call_async(request)
        future.add_done_callback(self._on_registered)

    def _on_registered(self, future) -> None:
        self.registration_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Registration failed: {exc}")
            return
        if not response.success:
            self.get_logger().warning(f"Registration rejected: {response.message}")
            return
        self.registered = True
        self.player_id = int(response.assigned_player_id)
        self.get_logger().info(f"Registered as player_{self.player_id}.")
        self.register_timer.cancel()

    def _compute_ik(
        self, target_pose: Pose, seed_positions: list[float]
    ) -> list[float] | None:
        seed_state = JointState()
        seed_state.name = list(PANDA_JOINTS)
        seed_state.position = list(seed_positions)

        request = GetPositionIK.Request()
        request.ik_request.group_name = "panda_arm"
        request.ik_request.robot_state = MoveItRobotState(joint_state=seed_state)
        request.ik_request.pose_stamped.header.frame_id = "panda_link0"
        request.ik_request.pose_stamped.pose = target_pose
        request.ik_request.timeout.sec = 5
        request.ik_request.avoid_collisions = False

        future = self.ik_client.call_async(request)
        while rclpy.ok() and not future.done():
            time.sleep(0.01)

        result = future.result()
        if result is None or result.error_code.val != 1:
            self.get_logger().error(
                f"IK failed (code={getattr(result, 'error_code', 'N/A')})"
            )
            return None

        joint_positions = dict(
            zip(result.solution.joint_state.name, result.solution.joint_state.position)
        )
        return [joint_positions[name] for name in PANDA_JOINTS]

    @staticmethod
    def _link8_pose_from_tcp_target(x: float, y: float, z: float) -> Pose:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z + _LINK8_TCP_Z_OFFSET
        pose.orientation.x = _LINK8_QUAT[0]
        pose.orientation.y = _LINK8_QUAT[1]
        pose.orientation.z = _LINK8_QUAT[2]
        pose.orientation.w = _LINK8_QUAT[3]
        return pose

    def _handle_plan_turn(
        self, request: PlanTurn.Request, response: PlanTurn.Response
    ) -> PlanTurn.Response:
        if request.player_id != self.player_id:
            response.accepted = False
            response.message = "Plan request does not match registered player id."
            return response

        move = self._select_rule_based_move(request)
        if move is None:
            response.accepted = False
            response.message = "No rule-based move available for current board state."
            return response

        piece_id, cell_id = move

        try:
            piece_pose = self._find_piece_pose(request, piece_id)
        except ValueError as exc:
            response.accepted = False
            response.message = str(exc)
            return response

        cell_pose = request.layout.cell_poses[cell_id]
        pick_target = self._link8_pose_from_tcp_target(
            piece_pose.position.x, piece_pose.position.y, piece_pose.position.z
        )
        place_target = self._link8_pose_from_tcp_target(
            cell_pose.position.x, cell_pose.position.y, cell_pose.position.z
        )

        pick_goal = self._compute_ik(pick_target, HOME)
        if pick_goal is None:
            response.accepted = False
            response.message = "IK failed for pick target."
            return response

        place_goal = self._compute_ik(place_target, HOME)
        if place_goal is None:
            response.accepted = False
            response.message = "IK failed for place target."
            return response

        self.get_logger().info(f"IK solved: piece {piece_id} -> cell {cell_id}")

        plan = TurnPlan()
        plan.match_id = request.match_id
        plan.turn_index = request.turn_index
        plan.player_id = request.player_id
        plan.piece_id = piece_id
        plan.cell_id = cell_id
        plan.home_to_pick = _make_trajectory(HOME, pick_goal)
        plan.pick_to_home = _make_trajectory(pick_goal, HOME)
        plan.home_to_place = _make_trajectory(HOME, place_goal)
        plan.place_to_home = _make_trajectory(place_goal, HOME)

        response.accepted = True
        response.message = "IK-based rule plan generated."
        response.plan = plan
        return response

    @staticmethod
    def _find_piece_pose(request: PlanTurn.Request, piece_id: int) -> Pose:
        for piece in request.snapshot.pieces:
            if piece.piece_id == piece_id:
                return piece.pose
        for piece in request.layout.initial_pieces:
            if piece.piece_id == piece_id:
                return piece.pose
        raise ValueError(f"Piece {piece_id} not found in request data.")

    @staticmethod
    def _select_rule_based_move(request: PlanTurn.Request) -> tuple[int, int] | None:
        available = sorted(
            piece.piece_id
            for piece in request.snapshot.pieces
            if piece.available and piece.owner == request.player_id
        )
        legal = [
            index
            for index, enabled in enumerate(request.snapshot.legal_actions)
            if enabled == 1
        ]
        if not available or not legal:
            return None

        cell_id = RuleBasedPlayerNode._choose_cell(
            board=list(request.snapshot.board),
            legal=legal,
            player_id=int(request.player_id),
        )
        return available[0], cell_id

    @staticmethod
    def _choose_cell(board: list[int], legal: list[int], player_id: int) -> int:
        own_mark = player_id + 1

        winning_cell = RuleBasedPlayerNode._find_completion(
            board, legal, own_mark
        )
        if winning_cell is not None:
            return winning_cell

        for cell_id in CELL_PRIORITY:
            if cell_id in legal:
                return cell_id

        return legal[0]

    @staticmethod
    def _find_completion(
        board: list[int], legal: list[int], mark: int
    ) -> int | None:
        legal_set = set(legal)
        for line in WIN_LINES:
            values = [board[index] for index in line]
            if values.count(mark) != 2 or values.count(GameSnapshot.EMPTY) != 1:
                continue
            empty_cell = line[values.index(GameSnapshot.EMPTY)]
            if empty_cell in legal_set:
                return empty_cell
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RuleBasedPlayerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
