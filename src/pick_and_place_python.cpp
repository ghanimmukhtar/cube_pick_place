#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/tf.h>

#include <actionlib/client/simple_action_client.h>
#include <yaml-cpp/yaml.h>

#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/PoseArray.h>
#include <shape_tools/solid_primitive_dims.h>
#include <moveit_simple_grasps/GenerateGraspsAction.h>
#include <moveit_simple_grasps/GenerateGraspsGoal.h>
#include <moveit_simple_grasps/GenerateGraspsResult.h>
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <moveit_msgs/PlaceLocation.h>

#include <crustcrawler_mover_utils/crustcrawler_mover.hpp>

using namespace moveit_simple_grasps;
using namespace moveit_msgs;
using namespace geometry_msgs;
using namespace moveit::planning_interface;
using namespace ros;
using namespace std;
using namespace actionlib;
using namespace crustcrawler_mover;

class Pick_Place{
    public:
        Pick_Place(){
                init();
            }

        void init(){
                //Retrieve params:
                _node.getParam("/", _parameters);
                _table_object_name = static_cast<string>(_parameters["table_object_name"]);
                _grasp_object_name = static_cast<string>(_parameters["grasp_object_name"]);
                _grasp_object_width = stod(_parameters["grasp_object_width"]);
                _arm_group = static_cast<string>(_parameters["arm"]);
                _gripper_group = static_cast<string>(_parameters["gripper"]);
                _approach_retreat_desired_dist = stod(_parameters["approach_retreat_desired_dist"]);
                _approach_retreat_min_dist = stod(_parameters["approach_retreat_min_dist"]);

                AsyncSpinner my_spinner(4);
                my_spinner.start();

                WallDuration(1.0).sleep();

                //Create (debugging) publishers:
                _grasps_pub = _node.advertise<PoseArray>("grasps", true);
                _places_pub = _node.advertise<PoseArray>("places", true);

                _pub_co = _node.advertise<CollisionObject>("collision_object", 10);
                _pub_aco = _node.advertise<AttachedCollisionObject>("attached_collision_object", 10);

                _crustcrawler_mover.reset(new CRUSTCRAWLER_Mover(_node));
                //_robot.reset(new MoveGroup(_arm_group));

                //Clean the scene:
                remove_world_object(_table_object_name);
                remove_world_object(_grasp_object_name);

                //Add table and Coke can objects to the planning scene:
                _pose_table = add_table(_table_object_name);
                _pose_coke_can = add_grasp_block(_grasp_object_name);

                WallDuration(1.0).sleep();

                //Define target place pose:
                _pose_place.position.x = _pose_coke_can.position.x;
                _pose_place.position.y = _pose_coke_can.position.y - 0.06;
                _pose_place.position.z = _pose_coke_can.position.z;

                _q.setEuler(0.0, 0.0, 0.0);
                _pose_place.orientation.w = _q.getW();
                _pose_place.orientation.x = _q.getX();
                _pose_place.orientation.y = _q.getY();
                _pose_place.orientation.z = _q.getZ();

                //Retrieve groups (arm and gripper):
                _arm = _crustcrawler_mover->group->getName();
                _gripper = _crustcrawler_mover->group->getEndEffector();

                //Create grasp generator 'generate' action client:
                _grasp_ac.reset(new SimpleActionClient<GenerateGraspsAction>("/moveit_simple_grasps_server/generate", true));
                if(!_grasp_ac->waitForServer(Duration(5.0))){
                        ROS_ERROR("Grasp generator action client not available!");
                        shutdown();
                        return;
                    }

                //Create move group 'pickup' action client:
                _pickup_ac.reset(new SimpleActionClient<PickupAction>("/pickup", true));
                if(!_pickup_ac->waitForServer(Duration(5.0))){
                        ROS_ERROR("Pick up action client not available!");
                        shutdown();
                        return;
                    }

                //Create move group 'place' action client:
                _place_ac.reset(new SimpleActionClient<PlaceAction>("/place", true));
                if(!_place_ac->waitForServer(Duration(5.0))){
                        ROS_ERROR("Place action client not available!");
                        shutdown();
                        return;
                    }

                //Pick Coke can object:
                while(!pickup(_arm_group, _grasp_object_name)){
                        ROS_WARN("Pick up failed! Retrying ...");
                        WallDuration(1.0).sleep();
                    }

                ROS_INFO("Pick up successfully");

                //Place Coke can object on another place on the support surface (table):
                while(!place(_arm_group, _grasp_object_name, _pose_place)){
                        ROS_WARN("Place failed! Retrying ...");
                        WallDuration(1.0).sleep();
                    }

                ROS_INFO("Place successfully");
            }
        ~Pick_Place(){

            }

        GenerateGraspsResultConstPtr generate_grasps(Pose pose, double width){
                /* *
                 * Generate grasps by using the grasp generator generate action; based on
                 server_test.py example on moveit_simple_grasps pkg
                 * */

                //Create goal:
                GenerateGraspsGoal goal;
                goal.pose = pose;
                goal.width = width;

                GraspGeneratorOptions options;
                //simple_graps.cpp doesn't implement GRASP_AXIS_Z!
                //options.grasp_axis      = GraspGeneratorOptions.GRASP_AXIS_Z
                options.grasp_direction = GraspGeneratorOptions::GRASP_DIRECTION_UP;
                options.grasp_rotation = GraspGeneratorOptions::GRASP_ROTATION_FULL;

                //@todo disabled because it works better with the default options
                //goal.options.append(options)

                //Send goal and wait for result:
                if(_grasp_ac->sendGoalAndWait(goal) != SimpleClientGoalState::SUCCEEDED){
                        ROS_ERROR_STREAM("Grasp goal failed!:" << _grasp_ac->getState().getText());
                        return NULL;
                    }
                GenerateGraspsResultConstPtr grasps = _grasp_ac->getResult();
                publish_grasps(grasps);

                return grasps;
            }

        vector<PlaceLocation> generate_places(Pose target){
                vector<PlaceLocation> places;
                for (double angle = 0.0; angle < 2*M_PI; angle = angle + (1*M_PI/180)){
                        PlaceLocation place;
                        place.place_pose.header.stamp = Time::now();
                        place.place_pose.header.frame_id = _crustcrawler_mover->group->getPlanningFrame();

                        //Set target position:
                        place.place_pose.pose = target;

                        //Generate orientation (wrt Z axis):
                        _q.setEuler(0.0, 0.0, angle);
                        place.place_pose.pose.orientation.w = _q.getW();
                        place.place_pose.pose.orientation.x = _q.getX();
                        place.place_pose.pose.orientation.y = _q.getY();
                        place.place_pose.pose.orientation.z = _q.getZ();

                        //Generate pre place approach:
                        place.pre_place_approach.desired_distance = _approach_retreat_desired_dist;
                        place.pre_place_approach.min_distance = _approach_retreat_min_dist;

                        place.pre_place_approach.direction.header.stamp = Time::now();
                        place.pre_place_approach.direction.header.frame_id = _crustcrawler_mover->group->getPlanningFrame();

                        place.pre_place_approach.direction.vector.x = 0;
                        place.pre_place_approach.direction.vector.y = 0;
                        place.pre_place_approach.direction.vector.z = 0.2;

                        //Generate post place approach:
                        place.post_place_retreat.direction.header.stamp = Time::now();
                        place.post_place_retreat.direction.header.frame_id = _crustcrawler_mover->group->getPlanningFrame();

                        place.post_place_retreat.desired_distance = _approach_retreat_desired_dist;
                        place.post_place_retreat.min_distance = _approach_retreat_min_dist;

                        place.post_place_retreat.direction.vector.x = 0;
                        place.post_place_retreat.direction.vector.y = 0;
                        place.post_place_retreat.direction.vector.z = 0.2;

                        //Add place:
                        places.push_back(place);
                    }

                //Publish places (for debugging/visualization purposes):
                publish_places(places);

                return places;
            }

        void publish_grasps(GenerateGraspsResultConstPtr grasps){
                //Publish grasps as poses, using a PoseArray message
                if(_grasps_pub.getNumSubscribers() > 0){
                        PoseArray msg;
                        msg.header.frame_id = _crustcrawler_mover->group->getPlanningFrame();
                        msg.header.stamp = Time::now();

                        for (size_t i = 0; i < grasps->grasps.size(); i++){
                                Pose p = grasps->grasps[i].grasp_pose.pose;
                                msg.poses.push_back(p);
                            }
                        _grasps_pub.publish(msg);
                    }
            }

        void publish_places(vector<PlaceLocation> places){
                /*
                 * Publish places as poses, using a PoseArray message
                 * */

                if(_places_pub.getNumSubscribers() > 0){
                        PoseArray msg;
                        msg.header.frame_id = _crustcrawler_mover->group->getPlanningFrame();
                        msg.header.stamp = Time::now();

                        for (size_t i = 0; i < places.size(); i++){
                                Pose p = places[i].place_pose.pose;
                                msg.poses.push_back(p);
                            }
                        _places_pub.publish(msg);
                    }
            }

        void remove_world_object(std::string object_name){
                _co.operation = CollisionObject::REMOVE;
                _co.id = object_name;
                _pub_co.publish(_co);
            }

        Pose add_table(string table_name){
                PoseStamped p;
                p.header.frame_id = _crustcrawler_mover->group->getPlanningFrame();
                p.header.stamp = Time::now();

                p.pose.position.x = 0.45;
                p.pose.position.y = 0.0;
                p.pose.position.z = 0.22;
                _q.setEuler(0.0, 0.0, M_PI/2);
                p.pose.orientation.w = _q.getW();
                p.pose.orientation.x = _q.getX();
                p.pose.orientation.y = _q.getY();
                p.pose.orientation.z = _q.getZ();

                _co.header.stamp = Time::now();
                _co.header.frame_id = _crustcrawler_mover->group->getPlanningFrame();

                _co.primitives.resize(1);
                _co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
                _co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
                _co.primitive_poses.resize(1);

                _co.id = table_name;
                _co.operation = CollisionObject::ADD;
                _co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
                _co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.4;
                _co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.02;

                _co.primitive_poses[0].position = p.pose.position;
                _pub_co.publish(_co);

                return p.pose;
            }

        Pose add_grasp_block(string object_name){
                PoseStamped p;
                p.header.frame_id = _crustcrawler_mover->group->getPlanningFrame();
                p.header.stamp = Time::now();

                p.pose.position.x = 0.25;
                p.pose.position.y = 0.05;
                p.pose.position.z = 0.32;
                _q.setEuler(0.0, 0.0, 0.0);
                p.pose.orientation.w = _q.getW();
                p.pose.orientation.x = _q.getX();
                p.pose.orientation.y = _q.getY();
                p.pose.orientation.z = _q.getZ();

                _co.header.stamp = Time::now();
                _co.header.frame_id = _crustcrawler_mover->group->getPlanningFrame();

                _co.primitives.resize(1);
                _co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
                _co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
                _co.primitive_poses.resize(1);

                _co.id = object_name;
                _co.operation = CollisionObject::ADD;
                _co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.03;
                _co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.03;
                _co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.09;

                _co.primitive_poses[0].position = p.pose.position;
                _pub_co.publish(_co);

                return p.pose;
            }

        bool pickup(string group, string target){
                /* *
                 * Pick up a target using the planning group
                 * */

                //Obtain possible grasps from the grasp generator server:
                GenerateGraspsResultConstPtr grasps = generate_grasps(_pose_coke_can, _grasp_object_width);

                //Create and send Pickup goal:
                PickupGoal goal = create_pickup_goal(group, target, grasps);

                if(_pickup_ac->sendGoalAndWait(goal) != SimpleClientGoalState::SUCCEEDED){
                        ROS_ERROR_STREAM("Pick up goal failed!: " << _pickup_ac->getState().getText());
                        return NULL;
                    }

                PickupResultConstPtr result = _pickup_ac->getResult();

                //Check for error:
                if(result->error_code.val != MoveItErrorCodes::SUCCESS){
                        ROS_WARN_STREAM("Group" << group << " cannot pick up target " << target << "!:");
                        return false;
                    }
                return true;
            }

        bool place(string group, string target, Pose place){
                /*
                 * Place a target using the planning group
                 * */

                //Obtain possible places:
                vector<PlaceLocation> places = generate_places(place);

                //Create and send Place goal:
                PlaceGoal goal = create_place_goal(group, target, places);

                if(_place_ac->sendGoalAndWait(goal) != SimpleClientGoalState::SUCCEEDED){
                        ROS_ERROR_STREAM("Place goal failed!: " << _place_ac->getState().getText());
                        return NULL;
                    }

                PlaceResultConstPtr result = _place_ac->getResult();

                //Check for error:
                if(result->error_code.val != MoveItErrorCodes::SUCCESS){
                        ROS_WARN_STREAM("Group" << group << " cannot place target " << target << "!:");
                        return false;
                    }

                return true;
            }

        PickupGoal create_pickup_goal(string group, string target, GenerateGraspsResultConstPtr grasps){
                /*
                 * Create a MoveIt! PickupGoal
                 * */

                //Create goal:
                PickupGoal goal;

                goal.group_name = group;
                goal.target_name = target;

                goal.possible_grasps = grasps->grasps;
                goal.allowed_touch_objects.push_back(target);
                goal.support_surface_name = _table_object_name;

                //Configure goal planning options:
                goal.allowed_planning_time = 7.0;
                goal.planning_options.planning_scene_diff.is_diff = true;
                goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
                goal.planning_options.plan_only = false;
                goal.planning_options.replan = true;
                goal.planning_options.replan_attempts = 20;

                return goal;
            }

        PlaceGoal create_place_goal(string group, string target, vector<PlaceLocation> places){
                /*
                 * Create a MoveIt! PlaceGoal
                 * */

                //Create goal
                PlaceGoal goal;

                goal.group_name = group;
                goal.attached_object_name = target;

                goal.place_locations = places;

                //Configure goal planning options:
                goal.allowed_planning_time = 7.0;

                goal.planning_options.planning_scene_diff.is_diff = true;
                goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
                goal.planning_options.plan_only = false;
                goal.planning_options.replan = true;
                goal.planning_options.replan_attempts = 20;

                return goal;
            }

    private:
        NodeHandle _node;
        string _table_object_name = "Grasp_Table", _grasp_object_name = "Grasp_Object", _arm_group = "arm", _gripper_group = "gripper", _arm, _gripper;
        double _grasp_object_width = 0.01, _approach_retreat_desired_dist = 0.1, _approach_retreat_min_dist = 0.05;
        tf::Quaternion _q;

        Publisher _grasps_pub, _places_pub, _pub_co, _pub_aco;
        shared_ptr<SimpleActionClient<GenerateGraspsAction>> _grasp_ac;
        shared_ptr<SimpleActionClient<PickupAction>> _pickup_ac;
        shared_ptr<SimpleActionClient<PlaceAction>> _place_ac;
        shared_ptr<moveit::planning_interface::MoveGroup> _robot;
        CRUSTCRAWLER_Mover::Ptr _crustcrawler_mover;
        XmlRpc::XmlRpcValue _parameters;
        CollisionObject _co;
        AttachedCollisionObject _aco;
        Pose _pose_table, _pose_coke_can, _pose_place;
    };

int main(int argc, char **argv)
    {
        ros::init(argc, argv, "pick_and_place_python_node");
        ros::NodeHandle outer_nh;


        Pick_Place pick_place_arm;


        return 0;
    }

