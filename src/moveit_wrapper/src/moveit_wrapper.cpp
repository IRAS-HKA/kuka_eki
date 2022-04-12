#include <moveit_wrapper/moveit_wrapper.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;


namespace moveit_wrapper
{
    MoveitWrapper::MoveitWrapper(const rclcpp::NodeOptions &options) : Node("moveit_wrapper", options)
    {
        _i_move_group_initialized = false;
//        this->declare_parameter("planning_group", "manipulator");
        this->get_parameter("planning_group", _planning_group);
        _move_to_pose = this->create_service<iras_srvs::srv::Pose>("move_to_pose", std::bind(&MoveitWrapper::move_to_pose, this, _1, _2));
        rclcpp::Rate loop_rate(1);
        loop_rate.sleep();

        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "Initialized.");
    }

    void MoveitWrapper::init_move_group()
    {
        static const std::string PLANNING_GROUP = "manipulator";
        _move_group.reset(new moveit::planning_interface::MoveGroupInterface(shared_from_this(), PLANNING_GROUP));

        _i_move_group_initialized = true;
        rclcpp::Rate loop_rate(1000);
        loop_rate.sleep();
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "Ready to receive commands.");
    }

    void MoveitWrapper::move_to_pose(const std::shared_ptr<iras_srvs::srv::Pose::Request> request,
                std::shared_ptr<iras_srvs::srv::Pose::Response> response)
    {
        std::cout << "message received" << '\n';
        bool success = false;
        if(_i_move_group_initialized)
        {
            if(!request->cart) {
                _move_group->setPoseTarget(request->pose);
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                success = (_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                if(success) {
                    _move_group->move();
                }
            } else {
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(request->pose);

                moveit_msgs::msg::RobotTrajectory trajectory;
                const double jump_threshold = 0.0;
                const double eef_step = 0.01;
                double fraction = _move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

                if(fraction > 0.0) {
                    success = true;
                    _move_group->execute(trajectory);
                }
                RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "******************************* %lf", fraction);
            }
        }
        response->success = success;
    }
}