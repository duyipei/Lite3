#!/usr/bin/env python3
"""Prepare a pinned private VOA overlay; NEVER edits the factory workspace.

Vendor sources have no usable distribution license in package.xml. They are
copied on the owning Jetson, not bundled in this repository. Build separately
with colcon after this deterministic, fail-closed source transformation.
"""
import argparse
import hashlib
from pathlib import Path
import shutil

PINNED = {
    'SafetyController.hpp': '52c2268286a596e705920ce6f4c6fdba755283efae1d4ef0e7230bb7be3f1ae0',
    'PointCloudProcessor.hpp': '5a2785276efaa2a7536bc0701924e1c7ed6600d603cd8437657c08155134a4d9',
    'GridMapper.hpp': 'c56de8d382900b78c68efa45a23c9d343ddf52a476f56db19290d647a322f3ff',
    'MyGridMap.hpp': 'ea104d655e3795d38c4d3125a7aad4ed73606063d2dc51a2786e7e1e50e4a2b2',
}


def replace(text, old, new, count=1):
    if text.count(old) != count:
        raise ValueError('factory anchor mismatch: ' + old[:90])
    return text.replace(old, new)


def transform(name, text):
    # Factory sensor headers use the steady time domain on this robot. Keep
    # that contract; fail closed on wrong-domain headers, never restamp them.
    if name == 'MyGridMap.hpp':
        text = replace(text, 'rclcpp::Time time((double)stamp_ref_pc.sec, (double)stamp_ref_pc.nanosec/1e9, RCL_STEADY_TIME);',
                       'rclcpp::Time time(stamp_ref_pc.sec, stamp_ref_pc.nanosec, RCL_STEADY_TIME);')
    if name == 'SafetyController.hpp':
        text = replace(text, '#include <geometry_msgs/msg/twist.hpp>',
                       '#include <geometry_msgs/msg/twist.hpp>\n#include <geometry_msgs/msg/twist_stamped.hpp>\n#include <cmath>')
        text = replace(text, 'return pointPolygonTest(rect, point2check, false);',
                       'return pointPolygonTest(rect, point2check, false) >= 0;')
        start = text.index('        handle_state_sub_ = ')
        end = text.index('        RCLCPP_INFO(this->get_logger(), "SafetyController initialized!");', start)
        text = text[:start] + '''        // Never subscribe to joystick or public motor command topics.
        stamped_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/patrol/voa/input", best_effort_qos,
            [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                if (!Finite(msg->twist)) return;
                request_header_ = msg->header;
                CmdVelCallback(std::make_shared<geometry_msgs::msg::Twist>(msg->twist));
                command_received_ = std::chrono::steady_clock::now();
            });
        stamped_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/patrol/voa/corrected", best_effort_qos);
        // Default mutually exclusive callback group serializes sensor updates
        // and processing; no unsafe worker access to safety-controller state.
        safety_timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                         [this]() { ProcessVelocity(); });

''' + text[end:]
        text = replace(text, '        new_odom_ = true;',
                       '        odom_received_ = std::chrono::steady_clock::now();\n        new_odom_ = true;')
        text = replace(text, '        if(!first_grid_map_)',
                       '        grid_received_ = std::chrono::steady_clock::now();\n        if(!first_grid_map_)')
        text = replace(text, '        if(!first_ultrasound_)',
                       '        if (!std::isfinite(msg->data) || msg->data <= 0) { first_ultrasound_ = false; return; }\n        ultrasound_received_ = std::chrono::steady_clock::now();\n        if(!first_ultrasound_)')
        start = text.index('    void CorrectVelocityThreadFunc(){')
        end = text.index('    void CopyData(', start)
        text = text[:start] + '''    static bool Finite(const geometry_msgs::msg::Twist &v) {
        return std::isfinite(v.linear.x) && std::isfinite(v.linear.y) &&
               std::isfinite(v.linear.z) && std::isfinite(v.angular.x) &&
               std::isfinite(v.angular.y) && std::isfinite(v.angular.z);
    }

    void ProcessVelocity() {
        const auto now = std::chrono::steady_clock::now();
        auto fresh = [now](std::chrono::steady_clock::time_point t) {
            return std::chrono::duration<double>(now - t).count() <= 0.5;
        };
        // No healthy response on a missing/stale sensor. The downstream gate
        // independently times out, latches DISARMED, and publishes zero.
        if (!first_odom_ || !first_ultrasound_ || !first_grid_map_ || !first_vel_in_ ||
            !fresh(odom_received_) || !fresh(ultrasound_received_) ||
            !fresh(grid_received_) || !fresh(command_received_) || !new_vel_in_) return;
        const auto processing_start = std::chrono::steady_clock::now();
        grid_map_msgs::msg::GridMap::SharedPtr grid_map;
        std_msgs::msg::Float64::SharedPtr ultrasound;
        RobotVel vel_predict;
        geometry_msgs::msg::Twist::SharedPtr vel_in;
        geometry_msgs::msg::Twist vel_out;
        CopyData(grid_map, ultrasound, vel_predict, vel_in);
        if (!Finite(*vel_in)) return;
        // Pure rotation uses forward geometry, not the factory reverse 0.7 gain.
        // Reverse and sideways motion stay disabled in this initial integration.
        if (vel_in->linear.x >= 0 && vel_in->linear.x <= 0.15 &&
            vel_in->linear.y == 0 && std::abs(vel_in->angular.z) <= 0.4 &&
            (vel_in->linear.x != 0 || vel_in->angular.z != 0)) {
            CorrectForwardVelocity(grid_map, vel_predict, vel_in, vel_out);
        }
        if (!Finite(vel_out) || vel_out.linear.x < 0 || vel_out.linear.x > 0.15 ||
            vel_out.linear.y != 0 || std::abs(vel_out.angular.z) > 0.4) {
            vel_out = geometry_msgs::msg::Twist();
        }
        // Correlate each correction to its exact gate request (no Twist replay).
        geometry_msgs::msg::TwistStamped response;
        response.header = request_header_;
        response.twist = vel_out;
        stamped_pub_->publish(response);
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - processing_start).count();
        voa_compute_max_ms_ = std::max(voa_compute_max_ms_, elapsed_ms);
        ++voa_requests_;
        if (vel_in->linear.x != 0 || vel_in->angular.z != 0) ++voa_motion_requests_;
        if (voa_requests_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "[VOA_PROFILE] requests=%llu motion=%llu compute_ms=%.3f max_ms=%.3f",
                static_cast<unsigned long long>(voa_requests_),
                static_cast<unsigned long long>(voa_motion_requests_), elapsed_ms, voa_compute_max_ms_);
        }
    }

''' + text[end:]
        text = replace(text, 'for(int i=0; i<5; i++)', 'for(int i=0; i<6; i++)')
        text = replace(text, 'for(int j=-5; j<6; j++)', 'for(int j=-4; j<5; j++)')
        text = replace(text, 'double vx = 0.20*i;', 'double vx = 0.03*i;')
        # Extra angular changes after collision checking invalidate the check.
        for velocity in ('test_vel.yaw -= 0.1;', 'test_vel.yaw += 0.1;'):
            text = replace(text, velocity, 'test_vel.yaw = test_vel.yaw;', count=2)
        text = replace(text, 'vel_out.linear.x = collision_best_vel.x;', 'vel_out.linear.x = 0;')
        text = replace(text, 'vel_out.angular.z = collision_best_vel.yaw;', 'vel_out.angular.z = 0;')
        # Unknown elevation is not evidence of free space.
        text = replace(text, 'if(height > safety_control_params_.kObstatleHeight){',
                       'if(!std::isfinite(height) || height > safety_control_params_.kObstatleHeight){', count=2)
        # The downstream gate rejects alternate steering. Search only scalar
        # reductions of the commanded twist, retaining the same collision test.
        start = text.index('    void CorrectForwardVelocity(')
        end = text.index('    void CorrectBackwardVelocity(', start)
        text = text[:start] + '''    void CorrectForwardVelocity(grid_map_msgs::msg::GridMap::SharedPtr &grid_map,
            RobotVel &vel_predict, geometry_msgs::msg::Twist::SharedPtr &vel_in,
            geometry_msgs::msg::Twist &vel_out) {
        vel_out = geometry_msgs::msg::Twist();
        // Keep the measured-motion hazard check before testing commands.
        if (robot_state_ == RobotState::Fast) {
            std::vector<RobotCollisionRect> measured_path;
            PredictCollisionPath(grid_map, vel_predict, measured_path);
            AdjustRouteParams measured_params;
            if (DetectCollision(grid_map, measured_path, vel_in, measured_params)) return;
        }
        for (const double scale : {1.0, 0.75, 0.5, 0.25}) {
            RobotVel candidate(vel_in->linear.x * scale, 0, vel_in->angular.z * scale);
            std::vector<RobotCollisionRect> path;
            PredictCollisionPath(grid_map, candidate, path);
            AdjustRouteParams params;
            if (!DetectCollision(grid_map, path, vel_in, params)) {
                vel_out.linear.x = candidate.x;
                vel_out.angular.z = candidate.yaw;
                return;
            }
        }
        // No collision-free same-path candidate: retain the zero command.
    }

''' + text[end:]
        text = replace(text, '    std::thread correct_velocity_thread_;', '''    rclcpp::TimerBase::SharedPtr safety_timer_;
    unsigned long long voa_requests_{0}, voa_motion_requests_{0};
    double voa_compute_max_ms_{0};
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_pub_;
    std_msgs::msg::Header request_header_;
    std::chrono::steady_clock::time_point odom_received_{}, grid_received_{},
        ultrasound_received_{}, command_received_{};''')
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path('/home/ysc/lite_cog_ros2/voa/src/voa'))
    parser.add_argument('--workspace', type=Path, required=True)
    args = parser.parse_args()
    source = args.source.resolve()
    workspace = args.workspace.resolve()
    if workspace == source or source in workspace.parents or workspace in source.parents:
        raise SystemExit('source and overlay must be disjoint')
    output = workspace / 'src/voa'
    if output.exists():
        raise SystemExit('overlay source already exists; use a new workspace (never overwrite)')
    patched = {}
    for name, expected in PINNED.items():
        path = source / 'src' / name
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise SystemExit('Unsupported factory source version: ' + name)
        patched[name] = transform(name, path.read_text())
    shutil.copytree(str(source), str(output))
    for name, content in patched.items():
        (output / 'src' / name).write_text(content)
    print('Prepared isolated VOA sources:', output)
    print('NOT deployed to motion. Build with colcon; validate in shadow mode first.')


if __name__ == '__main__':
    main()
