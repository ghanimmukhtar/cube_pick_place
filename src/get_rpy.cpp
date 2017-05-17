#include <crustcrawler_mover_utils/crustcrawler_mover_utils/crustcrawler_mover.hpp>
#include <crustcrawler_mover_utils/move_crustcrawler_arm.h>
#include <crustcrawler_mover_utils/crustcrawler_mover_utils/parameters.hpp>
#include <crustcrawler_core_msgs/EndpointState.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


void publish_rpy_cb(const crustcrawler_core_msgs::EndpointState::ConstPtr& end_point){
    tf::Quaternion quat(end_point->pose.orientation.x, end_point->pose.orientation.y,
                        end_point->pose.orientation.z, end_point->pose.orientation.w);
    tf::Matrix3x3 rotation(quat);
    double roll, pitch, yaw;
    rotation.getRPY(roll, pitch, yaw);
    ROS_WARN_STREAM("the Roll is: " << roll);
    ROS_WARN_STREAM("the Pitch is: " << pitch);
    ROS_WARN_STREAM("the Yaw is: " << yaw);
    ROS_INFO("*********************************************");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_rpy_node");
    ros::NodeHandle nh;

    ros::Subscriber eef_state_sub = nh.subscribe<crustcrawler_core_msgs::EndpointState>("/crustcrawler/endpoint_state", 1, publish_rpy_cb);

    ros::spin();
    return 0;
}


