#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

class MainClass : public rclcpp::Node {
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Logger logger = rclcpp::get_logger("rclcpp logger");

	public : MainClass() : Node("main_node") {
		RCLCPP_INFO(this->get_logger(), "Running main_node...");
		timer_ = this->create_wall_timer(500ms, std::bind(&MainClass::main_function, this));
		// Create the MoveIt MoveGroup Interface
		RCLCPP_INFO(logger, "Creating node...");
		auto const node = std::make_shared<rclcpp::Node>("hello_moveit");
		RCLCPP_INFO(logger, "Node created.");
		RCLCPP_INFO(logger, "Creating move group interface...");
		auto move_group_interface = MoveGroupInterface(node, "robocol_arm_group");
		// auto move_group_interface = MoveGroupInterface();
		// RCLCPP_INFO(logger, typeid(&this).name());
		RCLCPP_INFO(logger, "Move group interface created.");
	}
	
	private : void main_function() {
		RCLCPP_INFO(this->get_logger(), "Executing main_function...");
	}
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MainClass>());
	rclcpp::shutdown();
	return 0;
}
