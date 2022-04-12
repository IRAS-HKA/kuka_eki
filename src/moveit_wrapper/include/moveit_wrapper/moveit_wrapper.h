#ifndef MOVEIT_WRAPPER
#define MOVEIT_WRAPPER

#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <iras_srvs/srv/pose.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <stdlib.h>

namespace moveit_wrapper
{
    class MoveitWrapper : public rclcpp::Node
    {
        public:
            MoveitWrapper(const rclcpp::NodeOptions &options);
            ~MoveitWrapper() {};
            void init_move_group();
        private:
            std::string _planning_group;
            bool _i_move_group_initialized;
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> _move_group;
            rclcpp::Service<iras_srvs::srv::Pose>::SharedPtr _move_to_pose;
            void move_to_pose(const std::shared_ptr<iras_srvs::srv::Pose::Request> request,
                std::shared_ptr<iras_srvs::srv::Pose::Response> response);
    };
}


#endif