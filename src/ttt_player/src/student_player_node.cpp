#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "ttt_interfaces/msg/game_snapshot.hpp"
#include "ttt_interfaces/msg/turn_plan.hpp"
#include "ttt_interfaces/msg/workspace_layout.hpp"
#include "ttt_interfaces/srv/plan_turn.hpp"
#include "ttt_interfaces/srv/register_player.hpp"

using namespace std::chrono_literals;

namespace {

std::vector<std::string> panda_joint_names() {
  return {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
          "panda_joint5", "panda_joint6", "panda_joint7"};
}

const std::vector<double> kHomePositions = {0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398};

const std::vector<std::pair<uint8_t, uint8_t>> kScriptedMoves = {
    {6, 2},
    {7, 4},
    {8, 6},
};

// link8 orientation quaternion (x, y, z, w) for gripper pointing down
constexpr double kLink8QuatX = 0.9238795325112867;
constexpr double kLink8QuatY = -0.3826834323650898;
constexpr double kLink8QuatZ = 0.0;
constexpr double kLink8QuatW = 0.0;
// Fixed Z offset from panda_link8 to panda_hand_tcp
constexpr double kLink8TcpZOffset = 0.1034;

geometry_msgs::msg::Pose link8_pose_from_tcp_target(double x, double y, double z) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z + kLink8TcpZOffset;
  pose.orientation.x = kLink8QuatX;
  pose.orientation.y = kLink8QuatY;
  pose.orientation.z = kLink8QuatZ;
  pose.orientation.w = kLink8QuatW;
  return pose;
}

trajectory_msgs::msg::JointTrajectoryPoint make_point(
    const std::vector<double> &positions,
    double time_sec) {
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  const auto whole_seconds = static_cast<int32_t>(std::floor(time_sec));
  point.time_from_start.sec = whole_seconds;
  point.time_from_start.nanosec =
      static_cast<uint32_t>((time_sec - static_cast<double>(whole_seconds)) * 1e9);
  return point;
}

moveit_msgs::msg::RobotTrajectory make_three_point_trajectory(
    const std::vector<double> &start_positions,
    const std::vector<double> &end_positions,
    double end_time_sec) {
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = panda_joint_names();

  const std::vector<double> midpoint = [&]() {
    std::vector<double> result;
    result.reserve(start_positions.size());
    for (size_t index = 0; index < start_positions.size(); ++index) {
      result.push_back((start_positions[index] + end_positions[index]) * 0.5);
    }
    return result;
  }();

  trajectory.joint_trajectory.points.push_back(make_point(start_positions, 0.0));
  trajectory.joint_trajectory.points.push_back(make_point(midpoint, end_time_sec * 0.5));
  trajectory.joint_trajectory.points.push_back(make_point(end_positions, end_time_sec));
  return trajectory;
}

}  // namespace

class StudentPlayerNode : public rclcpp::Node {
 public:
  StudentPlayerNode() : Node("student_player") {
    this->declare_parameter<std::string>("player_name", this->get_name());
    this->declare_parameter<std::string>("plan_turn_service", "/student_player/plan_turn");

    player_name_ = this->get_parameter("player_name").as_string();
    plan_turn_service_ = this->get_parameter("plan_turn_service").as_string();

    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    register_client_ =
        this->create_client<ttt_interfaces::srv::RegisterPlayer>("/ttt/register_player");

    ik_client_ = this->create_client<moveit_msgs::srv::GetPositionIK>(
        "/compute_ik",
        rmw_qos_profile_services_default,
        cb_group_);

    plan_turn_service_server_ = this->create_service<ttt_interfaces::srv::PlanTurn>(
        plan_turn_service_,
        std::bind(&StudentPlayerNode::handle_plan_turn, this, std::placeholders::_1,
                  std::placeholders::_2),
        rmw_qos_profile_services_default,
        cb_group_);

    register_timer_ =
        this->create_wall_timer(500ms, std::bind(&StudentPlayerNode::try_register, this));
  }

 private:
  void try_register() {
    if (registered_ || registration_in_flight_) {
      return;
    }
    if (!register_client_->wait_for_service(100ms)) {
      return;
    }

    auto request = std::make_shared<ttt_interfaces::srv::RegisterPlayer::Request>();
    request->player_name = player_name_;
    request->service_name = plan_turn_service_;
    registration_in_flight_ = true;

    register_client_->async_send_request(
        request,
        [this](rclcpp::Client<ttt_interfaces::srv::RegisterPlayer>::SharedFuture future) {
          registration_in_flight_ = false;
          try {
            const auto response = future.get();
            if (!response->success) {
              RCLCPP_WARN(this->get_logger(), "Registration rejected: %s",
                          response->message.c_str());
              return;
            }
            registered_ = true;
            player_id_ = response->assigned_player_id;
            RCLCPP_INFO(this->get_logger(), "Registered as player_%u.", player_id_);
            register_timer_->cancel();
          } catch (const std::exception &exc) {
            RCLCPP_ERROR(this->get_logger(), "Registration failed: %s", exc.what());
          }
        });
  }

  // ----------------------------------------------------------------
  // IK helpers
  // ----------------------------------------------------------------
  std::optional<std::vector<double>> compute_ik(
      const geometry_msgs::msg::Pose &target_pose,
      const std::vector<double> &seed_positions) {

    // TODO(student): Call the MoveIt `/compute_ik` service here.
    // Suggested steps:
    // 1. Create a `moveit_msgs::srv::GetPositionIK::Request`.
    // 2. Set `group_name = "panda_arm"`.
    // 3. Fill the seed joint state with the provided `seed_positions`.
    // 4. Set the target pose in frame `panda_link0`.
    // 5. Send the request through `ik_client_` and wait for the response.
    // 6. Extract the 7 Panda arm joints from the solution and return them.
    // 7. Return `std::nullopt` if IK times out or fails.
    //
    // The dummy return below keeps the starter code buildable, but it does not
    // solve IK. Students should replace it with a real implementation.

    if (!ik_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "/compute_ik service is not available.");
      return std::nullopt;
    }

    sensor_msgs::msg::JointState seed_state;
    seed_state.name = panda_joint_names();
    seed_state.position = seed_positions;

    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name = "panda_arm"; 
    request->ik_request.robot_state.joint_state = seed_state;
    request->ik_request.pose_stamped.header.frame_id = "panda_link0";
    request->ik_request.pose_stamped.pose = target_pose;
    request->ik_request.timeout.sec = 5;
    request->ik_request.avoid_collisions = false;

    auto future = ik_client_->async_send_request(request);
    if (future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "IK service call timed out.");
      return std::nullopt;
    }

    const auto result = future.get();
    if (!result || result->error_code.val != 1) {
      RCLCPP_ERROR(this->get_logger(),
                   "IK failed (code=%d).",
                   result ? result->error_code.val : -1);
      return std::nullopt;
    }

    const auto joint_names = panda_joint_names();
    std::vector<double> joints;
    joints.reserve(joint_names.size());
    for (const auto &name : joint_names) {
      bool found = false;
      for (size_t i = 0; i < result->solution.joint_state.name.size(); ++i) {
        if (result->solution.joint_state.name[i] == name) {
          joints.push_back(result->solution.joint_state.position[i]);
          found = true;
          break;
        }
      }
      if (!found) {
        RCLCPP_ERROR(this->get_logger(),
                     "IK solution missing joint %s.", name.c_str());
        return std::nullopt;
      }
    }
    return joints;
  } 

  // ----------------------------------------------------------------
  // Turn planning
  // ----------------------------------------------------------------
    void handle_plan_turn(
      const std::shared_ptr<ttt_interfaces::srv::PlanTurn::Request> request,
      std::shared_ptr<ttt_interfaces::srv::PlanTurn::Response> response) {
    if (request->player_id != player_id_) {
      response->accepted = false;
      response->message = "Plan request does not match registered player id.";
      return;
    }

    // TODO(student): Implement your turn-planning logic here.
    // Suggested structure:
    // 1. Choose a legal `(piece_id, cell_id)` pair from `request->snapshot`.
    // 2. Look up the current pose of the chosen stock piece.
    // 3. Look up the target board cell pose from `request->layout.cell_poses`.
    // 4. Convert those TCP targets into `panda_link8` poses using
    //    `link8_pose_from_tcp_target(...)`.
    // 5. Call `compute_ik(...)` for the pick target and place target.
    // 6. Build the four required trajectories:
    //      - home_to_pick
    //      - pick_to_home
    //      - home_to_place
    //      - place_to_home
    // 7. Fill `response->plan` and set `response->accepted = true` on success.

    // ---- (a) Gather "my pieces" and "legal cells" ------------------------
    std::unordered_set<uint8_t> available_pieces;
    for (const auto &piece : request->snapshot.pieces) {
      if (piece.available && piece.owner == request->player_id) {
        available_pieces.insert(piece.piece_id);
      }
    }
    std::vector<uint8_t> legal_cells;
    for (size_t i = 0; i < request->snapshot.legal_actions.size(); ++i) {
      if (request->snapshot.legal_actions[i] == 1) {
        legal_cells.push_back(static_cast<uint8_t>(i));
      }
    }
    if (available_pieces.empty() || legal_cells.empty()) {
      response->accepted = false;
      response->message = "No legal move available for current board state.";
      return;
    }

    // ---- (b) Tic-tac-toe strategy (Newell-Simon ordering) ----------------
    // Works regardless of turn order, because `my_mark` is derived from
    // `request->player_id`, not hard-coded.
    //   player_id == 0  -> my_mark = PLAYER_0 = 1, opp_mark = 2
    //   player_id == 1  -> my_mark = PLAYER_1 = 2, opp_mark = 1
    const auto &board = request->snapshot.board;
    const uint8_t my_mark = static_cast<uint8_t>(request->player_id + 1);
    const uint8_t opp_mark = static_cast<uint8_t>(my_mark == 1 ? 2 : 1);

    static constexpr std::array<std::array<uint8_t, 3>, 8> kLines = {{
        {{0, 1, 2}}, {{3, 4, 5}}, {{6, 7, 8}},
        {{0, 3, 6}}, {{1, 4, 7}}, {{2, 5, 8}},
        {{0, 4, 8}}, {{2, 4, 6}},
    }};

    auto is_legal = [&](uint8_t cell) {
      return std::find(legal_cells.begin(), legal_cells.end(), cell) !=
             legal_cells.end();
    };

    // If `mark` already has two on a line and the third is empty + legal,
    // return that third cell (so we can complete the line).
    auto find_completing_cell =
        [&](uint8_t mark) -> std::optional<uint8_t> {
      for (const auto &line : kLines) {
        int mine = 0;
        int empty_cell = -1;
        bool blocked = false;
        for (uint8_t c : line) {
          if (board[c] == mark) {
            ++mine;
          } else if (board[c] == 0) {
            empty_cell = c;
          } else {
            blocked = true;
            break;
          }
        }
        if (!blocked && mine == 2 && empty_cell >= 0 &&
            is_legal(static_cast<uint8_t>(empty_cell))) {
          return static_cast<uint8_t>(empty_cell);
        }
      }
      return std::nullopt;
    };

    // After hypothetically placing `mark` on `cell`, count how many lines
    // become a "two-of-mark + one-empty" threat.  A cell with >= 2 threats
    // is a fork (the opponent can only block one of them next turn).
    auto count_threats_after = [&](uint8_t cell, uint8_t mark) -> int {
      int threats = 0;
      for (const auto &line : kLines) {
        if (std::find(line.begin(), line.end(), cell) == line.end()) continue;
        int mine = 0;
        int empties = 0;
        bool blocked = false;
        for (uint8_t c : line) {
          uint8_t v = (c == cell) ? mark : board[c];
          if (v == mark) {
            ++mine;
          } else if (v == 0) {
            ++empties;
          } else {
            blocked = true;
            break;
          }
        }
        if (!blocked && mine == 2 && empties == 1) ++threats;
      }
      return threats;
    };

    auto find_fork = [&](uint8_t mark) -> std::optional<uint8_t> {
      for (uint8_t cell : legal_cells) {
        if (count_threats_after(cell, mark) >= 2) return cell;
      }
      return std::nullopt;
    };

    std::optional<uint8_t> chosen_cell;

    // 1) Win immediately if possible.
    chosen_cell = find_completing_cell(my_mark);
    // 2) Otherwise block opponent's immediate win.
    if (!chosen_cell) chosen_cell = find_completing_cell(opp_mark);
    // 3) Create a fork (two simultaneous threats).
    if (!chosen_cell) chosen_cell = find_fork(my_mark);
    // 4) Block opponent's fork.
    if (!chosen_cell) chosen_cell = find_fork(opp_mark);
    // 5) Take the center.
    if (!chosen_cell && is_legal(4)) chosen_cell = static_cast<uint8_t>(4);
    // 6) Take the corner opposite to any opponent-occupied corner.
    if (!chosen_cell) {
      static constexpr std::array<std::pair<uint8_t, uint8_t>, 4> kOppCorner = {{
          {0, 8}, {2, 6}, {6, 2}, {8, 0},
      }};
      for (const auto &pair : kOppCorner) {
        if (board[pair.first] == opp_mark && is_legal(pair.second)) {
          chosen_cell = pair.second;
          break;
        }
      }
    }
    // 7) Take any empty corner.
    if (!chosen_cell) {
      static constexpr std::array<uint8_t, 4> kCorners = {0, 2, 6, 8};
      for (uint8_t c : kCorners) {
        if (is_legal(c)) { chosen_cell = c; break; }
      }
    }
    // 8) Take any empty edge (last resort).
    if (!chosen_cell) {
      static constexpr std::array<uint8_t, 4> kEdges = {1, 3, 5, 7};
      for (uint8_t c : kEdges) {
        if (is_legal(c)) { chosen_cell = c; break; }
      }
    }
    // Absolute fallback (should never hit because legal_cells is non-empty).
    if (!chosen_cell) chosen_cell = legal_cells.front();

    const uint8_t cell_id = *chosen_cell;
    // Any of our pieces is interchangeable; pick whichever is in stock first.
    const uint8_t piece_id = *available_pieces.begin();

    // ---- (c) Resolve poses ----------------------------------------------
    geometry_msgs::msg::Pose piece_pose;
    try {
      piece_pose = find_piece_pose(request->snapshot, piece_id);
    } catch (const std::exception &exc) {
      response->accepted = false;
      response->message = std::string("Failed to locate piece: ") + exc.what();
      return;
    }
    const auto &cell_pose = request->layout.cell_poses[cell_id];

    const auto pick_target = link8_pose_from_tcp_target(
        piece_pose.position.x, piece_pose.position.y, piece_pose.position.z);
    const auto place_target = link8_pose_from_tcp_target(
        cell_pose.position.x, cell_pose.position.y, cell_pose.position.z);

    // ---- (d) Solve IK ---------------------------------------------------
    const auto pick_goal = compute_ik(pick_target, kHomePositions);
    if (!pick_goal) {
      response->accepted = false;
      response->message = "IK failed for pick target.";
      return;
    }
    const auto place_goal = compute_ik(place_target, kHomePositions);
    if (!place_goal) {
      response->accepted = false;
      response->message = "IK failed for place target.";
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Strategy chose: piece %u -> cell %u (mark=%u)",
                piece_id, cell_id, my_mark);

    // ---- (e) Build trajectories and the response plan -------------------
    constexpr double kTrajTime = 1.5;
    ttt_interfaces::msg::TurnPlan plan;
    plan.match_id = request->match_id;
    plan.turn_index = request->turn_index;
    plan.player_id = request->player_id;
    plan.piece_id = piece_id;
    plan.cell_id = cell_id;
    plan.home_to_pick =
        make_three_point_trajectory(kHomePositions, *pick_goal, kTrajTime);
    plan.pick_to_home =
        make_three_point_trajectory(*pick_goal, kHomePositions, kTrajTime);
    plan.home_to_place =
        make_three_point_trajectory(kHomePositions, *place_goal, kTrajTime);
    plan.place_to_home =
        make_three_point_trajectory(*place_goal, kHomePositions, kTrajTime);

    response->accepted = true;
    response->message = "Strategic plan generated.";
    response->plan = plan;
  }

  static geometry_msgs::msg::Pose find_piece_pose(
      const ttt_interfaces::msg::GameSnapshot &snapshot,
      uint8_t piece_id) {
    // TODO(student): Search `snapshot.pieces` for the requested `piece_id` and
    // return its pose. You may choose to throw an exception or return a
    // fallback pose if the piece is missing.
    for (const auto &piece : snapshot.pieces) {
      if (piece.piece_id == piece_id) {
        return piece.pose;
      }
    }
    throw std::runtime_error("Piece " + std::to_string(piece_id) + " not found in snapshot.");
  }

  std::string player_name_;
  std::string plan_turn_service_;
  bool registered_{false};
  bool registration_in_flight_{false};
  uint8_t player_id_{255};

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Client<ttt_interfaces::srv::RegisterPlayer>::SharedPtr register_client_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Service<ttt_interfaces::srv::PlanTurn>::SharedPtr plan_turn_service_server_;
  rclcpp::TimerBase::SharedPtr register_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StudentPlayerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
