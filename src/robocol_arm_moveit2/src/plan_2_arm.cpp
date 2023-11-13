#include <iostream>
// ROS libraries
#include <rclcpp/rclcpp.hpp>
// Messages
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
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
			pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/ATTinyinfo", 10);
		}
	
	private :
		rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_subscription_;
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr next_position_subscription_;
		trajectory_msgs::msg::JointTrajectory joint_trajectory;
		rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
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
				RCLCPP_INFO(this->get_logger(), "t%d: j0: %f, j1: %f, j2: %f, j3: %f, j4: %f, j5: %f", actual_trajectory, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
				auto pose = geometry_msgs::msg::Pose();
				if (actual_trajectory == 0) {
					//pose.position.x = (pos[0]*180)/(3.14159);
					//pose.position.y = -(pos[1]*180)/(3.14159);
					//pose.position.z = -(pos[2]*180)/(3.14159);
					//pose.orientation.x = (pos[3]*180)/(3.14159);
					//pose.orientation.y = (pos[4]*180)/(3.14159);
					//pose.orientation.z = (pos[5]*180)/(3.14159);
					//pose.orientation.w = 0.0;
					pose.position.x = 0.0;
					pose.position.y = 0.0;
					pose.position.z = 0.0;
					pose.orientation.x = 0.0;
					pose.orientation.y = 0.0;
					pose.orientation.z = 0.0;
					pose.orientation.w = 0.0;
				} else {
					auto last_pos = joint_trajectory.points[actual_trajectory-1].positions;
					RCLCPP_INFO(this->get_logger(), "pos: '%f'", pos[0]);
					RCLCPP_INFO(this->get_logger(), "pos: '%f'", last_pos[0]);
					RCLCPP_INFO(this->get_logger(), "minus: '%f'", pos[0] - last_pos[0]);
					pose.position.x = ((pos[0] - last_pos[0])*180)/(3.14159);
					pose.position.y = -((pos[1] - last_pos[1])*180)/(3.14159);
					pose.position.z = -((pos[2] - last_pos[2])*180)/(3.14159);
					pose.orientation.x = ((pos[3] - last_pos[3])*180)/(3.14159);
					pose.orientation.y = -((pos[4] - last_pos[4])*180)/(3.14159);
					pose.orientation.z = ((pos[5] - last_pos[5])*180)/(3.14159);
					pose.orientation.w = 0.0;
				}
				actual_trajectory++;	
				pose_publisher_->publish(pose);
			} else {
				RCLCPP_INFO(this->get_logger(), "Trajectory was finished, plan again to start a new one.");
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
