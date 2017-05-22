#include <crustcrawler_mover_utils/crustcrawler_mover_utils/crustcrawler_mover.hpp>
#include <crustcrawler_mover_utils/move_crustcrawler_arm.h>
#include <crustcrawler_mover_utils/crustcrawler_mover_utils/parameters.hpp>
#include <crustcrawler_core_msgs/EndEffectorCommand.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>

using namespace crustcrawler_mover;

class Cube_picker_placer{
public:
    Cube_picker_placer(){
        init();
    }

    //Convert object position from camera frame to robot frame
    void tf_base_conversion(Eigen::Vector3d& object_pose_in_robot_frame, Eigen::Vector3d& object_pose_in_camera_frame){
        tf::TransformListener listener;
        tf::StampedTransform stamped_transform;
        //std::string child_frame = "/camera_depth_optical_frame";
        std::string child_frame = "/camera_rgb_optical_frame";
        std::string parent_frame = "base";
        try{
            listener.lookupTransform(child_frame, parent_frame,
                                     ros::Time::now(), stamped_transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        geometry_msgs::PointStamped camera_point;
        geometry_msgs::PointStamped base_point;
        camera_point.header.frame_id = child_frame;

        //we'll just use the most recent transform available for our simple example
        camera_point.header.stamp = ros::Time();

        //just an arbitrary point in space
        camera_point.point.x = object_pose_in_camera_frame(0);
        camera_point.point.y = object_pose_in_camera_frame(1);
        camera_point.point.z = object_pose_in_camera_frame(2);

        try{

            listener.transformPoint(parent_frame, camera_point, base_point);

            ROS_INFO("camera_depth_optical_frame: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                     camera_point.point.x, camera_point.point.y, camera_point.point.z,
                     base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
        }
        catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
        }
        object_pose_in_robot_frame << base_point.point.x,
                base_point.point.y,
                base_point.point.z;
    }
    void cube_position_cb(const geometry_msgs::Point::ConstPtr& cube_position){
        _cube_position_camera_frame(0) = cube_position->x;
        _cube_position_camera_frame(1) = cube_position->y;
        _cube_position_camera_frame(2) = cube_position->z;
        tf_base_conversion(_cube_position_robot_frame, _cube_position_camera_frame);
    }

    Eigen::Vector3d get_cube_position(){
        return _cube_position_robot_frame;
    }

    void init(){
        _crustcrawler_mover.reset(new CRUSTCRAWLER_Mover(_node));
        _gripper_command_publisher.reset(new ros::Publisher(_node.advertise<crustcrawler_core_msgs::EndEffectorCommand>
                                                            ("/crustcrawler/end_effector/gripper/command", 1, this)));
        _cube_position_sub = _node.subscribe<geometry_msgs::Point>("/cube_pos_pub/cube_position", 1, &Cube_picker_placer::cube_position_cb, this);

        _crustcrawler_mover->group->setPlannerId("PRMstarkConfigDefault");
        _crustcrawler_mover->group->setPlanningTime(5);
        //first go to home
        _home_variable_values.insert ( std::pair<std::string, double>("joint_1", -1.3) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_2",  0.3) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_3", -1.1) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_4",  0.0) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_5", -0.5) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_6",  0.0) );

        _arranged_joints_positions = std::vector<double>(6,0);
        _home_joints_position = {-1.3, 0.3, -1.1, 0.0, -0.5, 0.0};

        if(!_node.getParam("/approach_distance", _approach_distance))
            _approach_distance = 0.12;
        if(!_node.getParam("/retract_distance", _retract_distance))
            _retract_distance = 0.2;
        if(!_node.getParam("/Roll", _roll))
            _roll = 0;
        if(!_node.getParam("/Pitch", _pitch))
            _pitch = 0.0;
        if(!_node.getParam("/Yaw", _yaw))
            _yaw = M_PI/2;
        if(!_node.getParam("/option", _option))
            _option = false;

        ros::AsyncSpinner my_spinner(4);
        my_spinner.start();
    }
    ~Cube_picker_placer(){

    }

    void open_gripper(){
        _gripper_command.args = "{position: 100.0}";
        _gripper_command.command = "go";
        _gripper_command_publisher->publish(_gripper_command);
    }

    void close_gripper(){
        _gripper_command.args = "{position: 0.0}";
        _gripper_command.command = "go";
        _gripper_command_publisher->publish(_gripper_command);
    }
    void fill_arranged_joint_position(){
        _arranged_joints_positions[0] = _crustcrawler_mover->global_parameters.get_joint_state().position[7];
        _arranged_joints_positions[1] = _crustcrawler_mover->global_parameters.get_joint_state().position[5];
        _arranged_joints_positions[2] = _crustcrawler_mover->global_parameters.get_joint_state().position[6];
        _arranged_joints_positions[3] = _crustcrawler_mover->global_parameters.get_joint_state().position[3];
        _arranged_joints_positions[4] = _crustcrawler_mover->global_parameters.get_joint_state().position[4];
        _arranged_joints_positions[5] = _crustcrawler_mover->global_parameters.get_joint_state().position[2];
    }

    //get largest difference between elements of two vectors
    double largest_difference(std::vector<double> &first, std::vector<double> &second){
        Eigen::VectorXd difference(first.size());
        double my_max = 0;
        for(size_t j = 0; j < first.size(); ++j)
            difference(j) = fabs(first[j] - second[j]);
        for(size_t j = 0; j < first.size(); ++j){
            if(difference(j) > my_max)
                my_max = difference(j);
        }
        return my_max;
    }

    bool go_home(){
        _crustcrawler_mover->group->setJointValueTarget(_home_variable_values);
        if(_crustcrawler_mover->group->plan(_group_plan))
            _crustcrawler_mover->group->execute(_group_plan);
        fill_arranged_joint_position();
        if(largest_difference(_arranged_joints_positions, _home_joints_position) < 0.1)
            return true;
        else
            return false;
    }

    bool approach(Eigen::Vector3d goal){
        tf::Quaternion quat_nagles;
        //double pitch = 0, step = 0.1;
        //ROS_WARN_STREAM("planning for angle : " << atan2(goal(1), goal(0)));
        if(_option)
            quat_nagles.setRPY(atan2(goal(1), goal(0)),  _pitch, _yaw);
        else
            quat_nagles.setRPY(_roll,  _pitch, atan2(goal(1), goal(0)));
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = "/world";
        goal_pose.pose.position.x = goal(0);
        goal_pose.pose.position.y = goal(1);
        goal_pose.pose.position.z = goal(2) + _approach_distance;
        goal_pose.pose.orientation.w = quat_nagles.getW();
        goal_pose.pose.orientation.x = quat_nagles.getX();
        goal_pose.pose.orientation.y = quat_nagles.getY();
        goal_pose.pose.orientation.z = quat_nagles.getZ();
        open_gripper();
        _crustcrawler_mover->group->setPoseTarget(goal_pose);
        double step = 0.1;
        int iteration = 0;
        while(!_crustcrawler_mover->group->plan(_group_plan) && iteration < 10){
            quat_nagles.setRPY(_roll, _pitch, atan2(goal(1), goal(0)));
            goal_pose.pose.orientation.w = quat_nagles.getW();
            goal_pose.pose.orientation.x = quat_nagles.getX();
            goal_pose.pose.orientation.y = quat_nagles.getY();
            goal_pose.pose.orientation.z = quat_nagles.getZ();
            _crustcrawler_mover->group->setPoseTarget(goal_pose);
            _pitch = _pitch + step;
            iteration+=1;
        }
        //ROS_WARN_STREAM("found solution with pitch = " << pitch);
        //_crustcrawler_mover->group->setPositionTarget(goal(0), goal(1), goal(2) + _approach_distance);
        if(_crustcrawler_mover->group->plan(_group_plan))
            if(_crustcrawler_mover->group->execute(_group_plan))
                return true;
            else
                return false;
        else
            return false;
    }

    bool pick_object(Eigen::Vector3d goal){
        open_gripper();

        tf::Quaternion quat_nagles;
        //double pitch = 0, step = 0.1;
        //ROS_WARN_STREAM("planning for angle : " << atan2(goal(1), goal(0)));
        if(_option)
            quat_nagles.setRPY(atan2(goal(1), goal(0)),  _pitch, _yaw);
        else
            quat_nagles.setRPY(_roll,  _pitch, atan2(goal(1), goal(0)));
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = "/world";
        goal_pose.pose.position.x = goal(0);
        goal_pose.pose.position.y = goal(1);
        goal_pose.pose.position.z = goal(2);
        goal_pose.pose.orientation.w = quat_nagles.getW();
        goal_pose.pose.orientation.x = quat_nagles.getX();
        goal_pose.pose.orientation.y = quat_nagles.getY();
        goal_pose.pose.orientation.z = quat_nagles.getZ();

        double step = 0.1;
        int iteration = 0;
        //_crustcrawler_mover->group->setPositionTarget(goal(0), goal(1), goal(2));
        _crustcrawler_mover->group->setPoseTarget(goal_pose);
        while(!_crustcrawler_mover->group->plan(_group_plan) && iteration < 10){
            quat_nagles.setRPY(_roll, _pitch, atan2(goal(1), goal(0)));
            goal_pose.pose.orientation.w = quat_nagles.getW();
            goal_pose.pose.orientation.x = quat_nagles.getX();
            goal_pose.pose.orientation.y = quat_nagles.getY();
            goal_pose.pose.orientation.z = quat_nagles.getZ();
            _crustcrawler_mover->group->setPoseTarget(goal_pose);
            _pitch = _pitch + step;
            iteration+=1;
        }
        if(_crustcrawler_mover->group->plan(_group_plan))
            if(_crustcrawler_mover->group->execute(_group_plan)){
                close_gripper();
                return true;
            }
            else
                return false;
        else
            return false;
        /*
        //construct two points vector to plan for straight line motion with open gripper
        _waypoints.clear();
        _waypoints.push_back(_crustcrawler_mover->global_parameters.get_eef_pose());
        _waypoints.push_back(_crustcrawler_mover->global_parameters.get_eef_pose());
        _waypoints[1].position.x = goal(0);
        _waypoints[1].position.y = goal(1);
        _waypoints[1].position.z = goal(2);

        double fraction = _crustcrawler_mover->group->computeCartesianPath(_waypoints, 0.025, 0.0, _robot_trajectory);
        if(fraction == 1){
            _group_plan.trajectory_ = _robot_trajectory;
            if(_crustcrawler_mover->group->execute(_group_plan)){
                close_gripper();
                return true;
            }
            else
                return false;
        }
        else
            return false;*/
    }

    bool retract(){
        _crustcrawler_mover->group->setPositionTarget(_crustcrawler_mover->global_parameters.get_eef_pose().position.x,
                                                      _crustcrawler_mover->global_parameters.get_eef_pose().position.y,
                                                      _crustcrawler_mover->global_parameters.get_eef_pose().position.z + _approach_distance);
        if(_crustcrawler_mover->group->plan(_group_plan))
            if(_crustcrawler_mover->group->execute(_group_plan))
                return true;
            else
                return false;
        else
            return false;
        //construct two points vector to plan for straight line motion with open gripper
        /*_waypoints.clear();
        _waypoints.push_back(_crustcrawler_mover->global_parameters.get_eef_pose());
        _waypoints.push_back(_crustcrawler_mover->global_parameters.get_eef_pose());
        _waypoints[1].position.z += _retract_distance;

        double fraction = _crustcrawler_mover->group->computeCartesianPath(_waypoints, 0.025, 0.0, _robot_trajectory);
        if(fraction == 1){
            _group_plan.trajectory_ = _robot_trajectory;
            if(_crustcrawler_mover->group->execute(_group_plan))
                return true;
            else
                return true;
        }
        else
            return false;*/
    }

    bool drop_object(Eigen::Vector3d goal){
        _crustcrawler_mover->group->setPositionTarget(goal(0), goal(1), goal(2));
        if(_crustcrawler_mover->group->plan(_group_plan))
            if(_crustcrawler_mover->group->execute(_group_plan)){
                open_gripper();
                return true;
            }
            else
                return false;
        else
            return false;
    }



private:
    ros::NodeHandle _node;
    ros::Subscriber _cube_position_sub;
    std::shared_ptr<ros::Publisher> _gripper_command_publisher;
    CRUSTCRAWLER_Mover::Ptr _crustcrawler_mover;
    std::map<std::string, double> _home_variable_values;
    std::vector<double> _arranged_joints_positions, _home_joints_position;
    moveit::planning_interface::MoveGroup::Plan _group_plan;
    crustcrawler_core_msgs::EndEffectorCommand _gripper_command;
    std::vector<geometry_msgs::Pose> _waypoints;
    moveit_msgs::RobotTrajectory _robot_trajectory;
    double _approach_distance, _retract_distance, _roll, _pitch, _yaw;
    Eigen::Vector3d _cube_position_camera_frame, _cube_position_robot_frame;
    bool _option;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cube_pick_place_node");
    ros::NodeHandle outer_nh;


    Cube_picker_placer pick_place_arm;
    pick_place_arm.go_home();
    //Eigen::Vector3d goal(0.27, -0.15, 0.05);
    Eigen::Vector3d goal;

    Eigen::Vector3d drop_point(0.2, 0.26, 0.2);
    while(ros::ok()){
        pick_place_arm.open_gripper();
        pick_place_arm.go_home();
        /*get cube position*/
        goal = pick_place_arm.get_cube_position();
        while(goal(0) != goal(0) || goal(1) != goal(1) || goal(2) != goal(2))
            goal = pick_place_arm.get_cube_position();
        while(goal(0) == 0.0 && goal(1) == 0.0 && goal(2) == 0.0)
            goal = pick_place_arm.get_cube_position();
        if(fabs(goal(0) - drop_point(0)) < 0.02 && fabs(goal(1) - drop_point(1)) < 0.02){
            ROS_WARN("the cube is confused with target box :) :) try other point");
            break;
        }

        if(pick_place_arm.approach(goal))
            if(pick_place_arm.pick_object(goal))
                if(pick_place_arm.retract())
                    if(pick_place_arm.drop_object(drop_point))
                        ROS_INFO("Hurray!! all process is a success");
                    else
                        ROS_WARN("The dropping didn't succeed :(:(:(:(");
                else
                    ROS_WARN("The retraction didn't succeed :(:(:(:(");
            else
                ROS_WARN("The picking didn't succeed :(:(:(:(");
        else
            ROS_WARN("The approaching didn't succeed :(:(:(:(");
        ROS_INFO("This iteration has finished please, enter position and press enter for the next ...");
        //std::cin >> x, y, z;
        //goal(0) = x;
        //goal(1) = y;
        //goal(2) = z;
        ROS_WARN_STREAM("I am planning for goal : " << goal);
        std::cin.ignore();
    }

    return 0;
}

