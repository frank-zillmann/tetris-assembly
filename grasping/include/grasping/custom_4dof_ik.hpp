// Simplified analytic inverse kinematics for the 4-DOF Mirte Master arm.
//
// shoulder_pan -> azimuth in the horizontal plane
// wrist -> kept so the last link (wrist -> gripper_center) points vertically downward
// shoulder_lift + elbow -> a planar 2-link problem solved analytically, always taking the elbow-up branch
//
// Sign convention: all of shoulder_lift / elbow / wrist are 0 when the arm points straight up, and become negative as the arm leans forward / downward.

#pragma once

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

namespace grasping
{
static constexpr double kPi = 3.14159265358979323846;
// Thrown by CustomIK::solve (besides tf2::TransformException for TF failures).
struct IKError : std::runtime_error { using std::runtime_error::runtime_error; };
struct IKUnreachable : IKError { using IKError::IKError; };       // target outside the arm's workspace
struct IKJointLimit : IKError { using IKError::IKError; };        // a joint would exceed +/- pi/2

class CustomIK
{
public:
  struct JointConfig
  {
    double lower_limit     = -kPi / 2.0;  // lower bound [rad]
    double upper_limit     =  kPi / 2.0;  // upper bound [rad]
    double lower_tolerance = 0.0;         // clip up to this violation instead of throwing [rad]
    double upper_tolerance = 0.0;         // clip up to this violation instead of throwing [rad]
  };

  using JointConfigMap = std::map<std::string, JointConfig>;

  explicit CustomIK(const tf2_ros::Buffer & tf,
                    JointConfigMap joint_configs = {})
  : tf_(tf), joint_configs_(std::move(joint_configs))
  {}

  // Joint targets that place gripper_center at `target` (given in any TF frame) with the gripper link pointing vertically downward.
  // Throws tf2::TransformException, IKUnreachable or IKJointLimit on failure.
  std::map<std::string, double> solve(const geometry_msgs::msg::PointStamped & target) const
  {
    geometry_msgs::msg::PointStamped t_base;
    tf_.transform(target, t_base, "base_link");
    const auto v_base_pan = tf_.lookupTransform("base_link", "shoulder_pan", tf2::TimePointZero).transform.translation;

    // Target coordinates from the position of shoulder_pan with the orientation of base_link.
    const auto t_x = t_base.point.x - v_base_pan.x;
    const auto t_y = t_base.point.y - v_base_pan.y;
    const auto t_z = t_base.point.z - v_base_pan.z;

    const auto radius = std::hypot(t_x, t_y);
    const auto pan = std::atan2(t_y, t_x);

    // shoulder_lift origin in shoulder_pan frame: offsets in the direction of the arm (negative z-axis) and in height (positive y-axis).
    const auto v_pan_lift = tf_.lookupTransform("shoulder_pan", "shoulder_lift", tf2::TimePointZero).transform.translation;
    const auto d_forward_pan_lift = -v_pan_lift.z;
    const auto d_upward_pan_lift = v_pan_lift.y;

    // Link lengths read from TF (constant regardless of joint state).
    const auto l1 = segmentLength("shoulder_lift", "elbow");
    const auto l2 = segmentLength("elbow", "wrist");
    const auto l3 = segmentLength("wrist", "gripper_center");

    // Wrist position in the 2-link plane (origin = shoulder_lift pivot):
    //   r_w: radial distance (gripper_center hangs straight down from the wrist, so the wrist shares the target's radius)
    //   z_w: height above the shoulder_lift pivot (the wrist sits l3 above gripper_center)
    const auto r_w = radius - d_forward_pan_lift;
    const auto z_w = t_z - d_upward_pan_lift + l3;

    // --- 2-link planar IK (law of cosines) --------------------------------
    const auto dd = r_w * r_w + z_w * z_w;
    const auto d = std::sqrt(dd);
    if (d > l1 + l2 || d < std::fabs(l1 - l2)) {
      throw IKUnreachable("target out of reach (wrist distance " + std::to_string(d) + " m)");
    }

    // alpha: angle between the pivot->wrist-line and the vertical upright position
    const auto alpha = std::atan2(r_w, z_w);
    // beta: angle at the shoulder in the triangle (shoulder_lift, elbow, wrist)
    const auto beta = std::acos((dd + l1 * l1 - l2 * l2) / (2.0 * d * l1));
    // gamma: angle at the elbow in the triangle (shoulder_lift, elbow, wrist)
    const auto gamma = std::acos((l1 * l1 + l2 * l2 - dd) / (2.0 * l1 * l2));

    // Angles are defined negative to the front -> extra minus sign.
    const auto theta2 = -(alpha - beta);
    const auto theta3 = -(kPi - gamma);
    // The last link must be bent a full pi from the vertical-up position to point
    // straight down: (-theta2) + (-theta3) + (-theta4) = pi -> theta4 = -pi - theta2 - theta3
    const auto theta4 = -kPi - theta2 - theta3;

    std::map<std::string, double> joints = {
      {"shoulder_pan_joint",  pan},
      {"shoulder_lift_joint", theta2},
      {"elbow_joint",         theta3},
      {"wrist_joint",         theta4},
    };

    for (auto & [name, value] : joints) {
      const auto it = joint_configs_.find(name);
      const JointConfig & cfg = (it != joint_configs_.end()) ? it->second : JointConfig{};
      if (value < cfg.lower_limit - cfg.lower_tolerance ||
          value > cfg.upper_limit + cfg.upper_tolerance) {
        throw IKJointLimit(name + " out of limit (" + std::to_string(value) + " rad)");
      }
      value = std::clamp(value, cfg.lower_limit, cfg.upper_limit);
    }
    return joints;
  }

private:
  // Constant length of the link between two consecutive joint frames.
  double segmentLength(const std::string & from, const std::string & to) const
  {
    const auto v = tf_.lookupTransform(from, to, tf2::TimePointZero).transform.translation;
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  }

  const tf2_ros::Buffer & tf_;
  JointConfigMap joint_configs_;
};

}  // namespace flower_grasping
