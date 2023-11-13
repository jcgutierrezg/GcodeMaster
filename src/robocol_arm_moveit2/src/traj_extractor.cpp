#include <iostream>
// ROS libraries
#include <rclcpp/rclcpp.hpp>
// Messages
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
// MoveIt libraries
#include <moveit_msgs/msg/display_trajectory.hpp>
// Placeholders
using std::placeholders::_1;

class Plan2ArmNode : public rclcpp::Node {
	public :
		Plan2ArmNode() : Node("plan_2_arm_node") {
			// Create a ROS Subscription to robocol/arm_desired_pose with geometry_msgs::msg::Pose type message
			auto display_planned_path = "/display_planned_path";
			RCLCPP_INFO(this->get_logger(), "Subscribing to %s topic...", display_planned_path);
			display_subscription_ = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
				display_planned_path,
				1,
				std::bind(&Plan2ArmNode::get_trajectory, this, _1)
			);
			RCLCPP_INFO(this->get_logger(), "Subscribed to %s.", display_planned_path);
			auto next_position = "/robocol/arm/next_position";
			RCLCPP_INFO(this->get_logger(), "Subscribing to %s topic...", next_position);
			next_position_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
				next_position,
				1,
				std::bind(&Plan2ArmNode::send_next_trajectory, this, _1)
			);
			RCLCPP_INFO(this->get_logger(), "Subscribed to %s.", next_position);
			traj_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/robocol/arm/Trajectory_info", 10);
			endflag_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/robocol/arm/Trajectory_end", 10);
		}
	
	private :
		rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_subscription_;
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr next_position_subscription_;
		trajectory_msgs::msg::JointTrajectory joint_trajectory;
		rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr traj_publisher_;
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr endflag_publisher_;
		int actual_trajectory;

		void get_trajectory(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg) {
			RCLCPP_INFO(this->get_logger(), "get_trajectory");
			joint_trajectory = msg->trajectory[0].joint_trajectory;
			actual_trajectory = 0;
			RCLCPP_INFO(this->get_logger(), "Number of trajectory points: '%d'",  (int)(std::size(joint_trajectory.points)));
		}

		void send_next_trajectory(const std_msgs::msg::Bool::SharedPtr msg) {
			RCLCPP_INFO(this->get_logger(), "send_next_trajectory");
			RCLCPP_INFO(this->get_logger(), "msg data: %d", msg->data);
			if (actual_trajectory < (int)std::size(joint_trajectory.points)) {
				RCLCPP_INFO(this->get_logger(), "Actual trajectory: '%d'", actual_trajectory);
				auto pos = joint_trajectory.points[actual_trajectory].positions;
				auto vel = joint_trajectory.points[actual_trajectory].velocities;
				RCLCPP_INFO(this->get_logger(), "t%d: Angle_J1: %f, Angle_J2: %f, Angle_J3: %f, Angle_J4: %f, Angle_J5: %f, Angle_J6: %f", actual_trajectory, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
				RCLCPP_INFO(this->get_logger(), "t%d: Veloc_J1: %f, Veloc_J2: %f, Veloc_J3: %f, Veloc_J4: %f, Veloc_J5: %f, Veloc_J6: %f", actual_trajectory, vel[0], vel[1], vel[2], vel[3], vel[4], vel[5]);
				auto traj = std_msgs::msg::Float32MultiArray();

				for (int i = 0; i < 6; ++i) {
        			traj.data.push_back(pos[i]);
        			traj.data.push_back(vel[i]);
    			}

				actual_trajectory++;	
				traj_publisher_->publish(traj);
			} else {
				RCLCPP_INFO(this->get_logger(), "Trajectory was finished, plan again to start a new one.");
				auto flag = std_msgs::msg::Bool();
				flag.data = true;
				endflag_publisher_->publish(flag);
			}
		}
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	auto plan_2_arm_node = std::make_shared<Plan2ArmNode>();
	rclcpp::spin(plan_2_arm_node);
	rclcpp::shutdown();
	return 0;
}
