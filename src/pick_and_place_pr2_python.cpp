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

#include <pr2_mover_utils/pr2_mover.hpp>
#include <cube_pick_place/pick_place_pose.h>

using namespace moveit_simple_grasps;
using namespace moveit_msgs;
using namespace geometry_msgs;
using namespace moveit::planning_interface;
using namespace ros;
using namespace std;
using namespace actionlib;
using namespace pr2_mover;
using namespace shape_msgs;

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

                //Create subscribers for the pick pose and place pose
                _pickup_place_pose_sub = _node.subscribe<cube_pick_place::pick_place_pose>("/pick_place_pose", 10, &Pick_Place::pick_and_place_cb, this);

                //Create (debugging) publishers:
                _grasps_pub = _node.advertise<PoseArray>("grasps", true);
                _places_pub = _node.advertise<PoseArray>("places", true);

                _pub_co = _node.advertise<CollisionObject>("collision_object", true);
                _pub_aco = _node.advertise<AttachedCollisionObject>("attached_collision_object", true);

                _pr2_mover.reset(new PR2_Mover(_node));
                _pr2_mover->group->setPlannerId("RRTConnectkConfigDefault");
                ROS_INFO("Trying to Print the Planning Frame");
                ROS_WARN_STREAM("Planning frame is: " << _pr2_mover->group->getPlanningFrame());

                //Clean the scene:
                remove_world_object(_table_object_name);
                remove_world_object(_grasp_object_name);
                remove_world_object(_collision_object.id);


                //Retrieve groups (arm and gripper):
                _arm = _pr2_mover->group->getName();
                _gripper = _pr2_mover->group->getEndEffector();

                //Create grasp generator 'generate' action client:
                _grasp_ac.reset(new SimpleActionClient<GenerateGraspsAction>("/moveit_simple_grasps_server/generate", true));
                if(!_grasp_ac->waitForServer(Duration(5.0))){
                        ROS_ERROR("Grasp generator action client not available!");
                        shutdown();
                        return;
                    }

                //Create move group 'pickup' action client:
//                _pickup_ac.reset(new SimpleActionClient<PickupAction>("/pickup", true));
                _pickup_ac.reset(new SimpleActionClient<PickupAction>("/dream_babbling/controller_node/pickup", true));
                if(!_pickup_ac->waitForServer(Duration(5.0))){
                        ROS_ERROR("Pick up action client not available!");
                        shutdown();
                        return;
                    }

                //Create move group 'place' action client:
//                _place_ac.reset(new SimpleActionClient<PlaceAction>("/place", true));
                _place_ac.reset(new SimpleActionClient<PlaceAction>("/dream_babbling/controller_node/place", true));
                if(!_place_ac->waitForServer(Duration(5.0))){
                        ROS_ERROR("Place action client not available!");
                        shutdown();
                        return;
                    }

                //Preparation for partial removal of the octomap later on
                _collision_object.header.frame_id = _pr2_mover->group->getPlanningFrame();
                _collision_object.id = "box1";
                _primitive.type = _primitive.BOX;
                _primitive.dimensions.resize(3);
                _primitive.dimensions[0] = 0.1;
                _primitive.dimensions[1] = 0.1;
                _primitive.dimensions[2] = 0.1;
                _box_pose.position.x =  0.;
                _box_pose.position.y =  0.;
                _box_pose.position.z =  0.;

                _collision_object.primitives.push_back(_primitive);
                _collision_object.primitive_poses.push_back(_box_pose);
                _collision_object.operation = _collision_object.ADD;

                ROS_INFO("Add an object into the world");
                _my_scene.world.collision_objects.push_back(_collision_object);
                _my_scene.is_diff = true;
                _pr2_mover->publish_psm_msg(_my_scene);
            }

        ~Pick_Place(){

            }

        void pick_and_place_cb(const cube_pick_place::pick_place_poseConstPtr& pick_place_topic){
                _picked = false;
                _placed = false;
                remove_world_object(_table_object_name);
                remove_world_object(_grasp_object_name);
                WallDuration(1.0).sleep();
                if(pick_place_topic->pick_pose.empty() || pick_place_topic->place_pose.empty()){
                        ROS_ERROR("please fill the pick pose as well as the place pose, otherwise this wouldn't work");
                        return;
                    }
                int pick_number_trials = 0, place_number_trials = 0, max_trials = 1;
                //Add table and Coke can objects to the planning scene:
                _pose_table = add_table(_table_object_name);
                _pose_coke_can = add_grasp_block(_grasp_object_name, pick_place_topic->pick_pose);

                WallDuration(1.0).sleep();

                //Define target place pose:
                _pose_place.position.x = pick_place_topic->place_pose[0].x;
                _pose_place.position.y = pick_place_topic->place_pose[0].y;
                _pose_place.position.z = pick_place_topic->place_pose[0].z;

                _box_pose.position.x = pick_place_topic->pick_pose[0].x;
                _box_pose.position.y = pick_place_topic->pick_pose[0].y;
                _box_pose.position.z = pick_place_topic->pick_pose[0].z;

                _q.setEuler(0.0, 0.0, 0.0);
                _pose_place.orientation.w = _q.getW();
                _pose_place.orientation.x = _q.getX();
                _pose_place.orientation.y = _q.getY();
                _pose_place.orientation.z = _q.getZ();

                _collision_object.primitive_poses.clear();
                _collision_object.primitive_poses.push_back(_box_pose);

                _my_scene.world.collision_objects.clear();
                _my_scene.world.collision_objects.push_back(_collision_object);
                _my_scene.is_diff = true;
                _pr2_mover->publish_psm_msg(_my_scene);

                _pr2_mover->global_parameters.set_adding_octomap_to_acm(false);
                _pr2_mover->call_service_get_ps();
                _pr2_mover->global_parameters.get_ps_response().scene.allowed_collision_matrix.default_entry_names.push_back("box1");
                _pr2_mover->global_parameters.get_ps_response().scene.allowed_collision_matrix.default_entry_values.push_back(true);
                _pr2_mover->publish_psm_msg();

                //Pick Coke can object:
                while(!pickup(_arm_group, _grasp_object_name) && pick_number_trials < max_trials){
                        ROS_WARN("Pick up failed! Retrying ...");
                        WallDuration(1.0).sleep();
                        pick_number_trials++;
                    }

                if(!_picked){
                        ROS_ERROR("Pickup Total Failure !");
                        return;
                    }

                remove_world_object(_table_object_name);
                _box_pose.position.x = _pose_place.position.x;
                _box_pose.position.y = _pose_place.position.y;
                _box_pose.position.z = _pose_place.position.z;

                _collision_object.primitive_poses.clear();
                _collision_object.primitive_poses.push_back(_box_pose);

                _my_scene.world.collision_objects.clear();
                _my_scene.world.collision_objects.push_back(_collision_object);
                _my_scene.is_diff = true;
                _pr2_mover->publish_psm_msg(_my_scene);

                _pr2_mover->global_parameters.set_adding_octomap_to_acm(false);
                _pr2_mover->call_service_get_ps();
                _pr2_mover->global_parameters.get_ps_response().scene.allowed_collision_matrix.default_entry_names.push_back("box1");
                _pr2_mover->global_parameters.get_ps_response().scene.allowed_collision_matrix.default_entry_values.push_back(true);
                _pr2_mover->publish_psm_msg();
                WallDuration(1.0).sleep();

                //Place Coke can object on another place on the support surface (table):
                while(!place(_arm_group, _grasp_object_name, _pose_place) && place_number_trials < max_trials){
                        ROS_WARN("Place failed! Retrying ...");
                        WallDuration(1.0).sleep();
                        place_number_trials++;
                    }

                _pr2_mover->group->detachObject(_grasp_object_name);
                remove_world_object(_collision_object.id);
                remove_world_object(_grasp_object_name);
                WallDuration(1.0).sleep();
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
                        place.place_pose.header.frame_id = _pr2_mover->group->getPlanningFrame();

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
                        place.pre_place_approach.direction.header.frame_id = _pr2_mover->group->getPlanningFrame();

                        place.pre_place_approach.direction.vector.x = 0;
                        place.pre_place_approach.direction.vector.y = 0;
                        place.pre_place_approach.direction.vector.z = 0.2;

                        //Generate post place approach:
                        place.post_place_retreat.direction.header.stamp = Time::now();
                        place.post_place_retreat.direction.header.frame_id = _pr2_mover->group->getPlanningFrame();

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
                        msg.header.frame_id = _pr2_mover->group->getPlanningFrame();
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
                        msg.header.frame_id = _pr2_mover->group->getPlanningFrame();
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
                PoseStamped p;/*
                p.header.frame_id = _pr2_mover->group->getPlanningFrame();*/
                p.header.frame_id = "base";
                p.header.stamp = Time::now();

                p.pose.position.x = 0.5;
                p.pose.position.y = 0.0;
                p.pose.position.z = 0.0;
                _q.setEuler(0.0, 0.0, M_PI/2);
                p.pose.orientation.w = _q.getW();
                p.pose.orientation.x = _q.getX();
                p.pose.orientation.y = _q.getY();
                p.pose.orientation.z = _q.getZ();

                _co.header.stamp = Time::now();
                _co.header.frame_id = _pr2_mover->group->getPlanningFrame();

                _co.primitives.resize(1);
                _co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
                _co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
                _co.primitive_poses.resize(1);

                _co.id = table_name;
                _co.operation = CollisionObject::ADD;
                _co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
                _co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
                _co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.02;

                _co.primitive_poses[0].position = p.pose.position;
                _pub_co.publish(_co);

                return p.pose;
            }

        Pose add_grasp_block(string object_name, vector<Point> target_pose){
                PoseStamped p;
                //p.header.frame_id = _pr2_mover->group->getPlanningFrame();
                p.header.frame_id = "base";
                p.header.stamp = Time::now();

                p.pose.position.x = target_pose[0].x;
                p.pose.position.y = target_pose[0].y;
                p.pose.position.z = target_pose[0].z;
                _q.setEuler(0.0, 0.0, 0.0);
                p.pose.orientation.w = _q.getW();
                p.pose.orientation.x = _q.getX();
                p.pose.orientation.y = _q.getY();
                p.pose.orientation.z = _q.getZ();

                _co.header.stamp = Time::now();
                _co.header.frame_id = _pr2_mover->group->getPlanningFrame();

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
                ROS_INFO("Pick up successfully");
                _picked = true;
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
                        return false;
                    }

                PlaceResultConstPtr result = _place_ac->getResult();

                //Check for error:
                if(result->error_code.val != MoveItErrorCodes::SUCCESS){
                        ROS_WARN_STREAM("Group" << group << " cannot place target " << target << "!:");
                        return false;
                    }

                ROS_INFO("Place successfully");
                _placed = true;
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
        bool _picked = false, _placed = false;
        tf::Quaternion _q;

        Publisher _grasps_pub, _places_pub, _pub_co, _pub_aco;
        Subscriber _pickup_place_pose_sub;
        shared_ptr<SimpleActionClient<GenerateGraspsAction>> _grasp_ac;
        shared_ptr<SimpleActionClient<PickupAction>> _pickup_ac;
        shared_ptr<SimpleActionClient<PlaceAction>> _place_ac;
        shared_ptr<moveit::planning_interface::MoveGroup> _robot;
        PR2_Mover::Ptr _pr2_mover;

        XmlRpc::XmlRpcValue _parameters;
        CollisionObject _co, _collision_object;
        SolidPrimitive _primitive;
        Pose _box_pose;
        PlanningScene _my_scene;

        AttachedCollisionObject _aco;
        Pose _pose_table, _pose_coke_can, _pose_place;
    };

int main(int argc, char **argv)
    {
        ros::init(argc, argv, "pick_and_place_python_node");
        ros::NodeHandle outer_nh;


        Pick_Place pick_place_arm;

        ros::waitForShutdown();

        return 0;
    }

