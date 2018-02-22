/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>
#include <crustcrawler_mover_utils/crustcrawler_mover.hpp>
#include <eigen_conversions/eigen_msg.h>

static const std::string ROBOT_DESCRIPTION="robot_description";
using namespace crustcrawler_mover;
// Grasp axis orientation
enum grasp_axis_t {X_AXIS, Y_AXIS, Z_AXIS};
enum grasp_direction_t {UP, DOWN};
enum grasp_rotation_t {FULL, HALF};

class the_grapper{
    public:
        the_grapper(){
                init();
            }
        void init(){
                _nh.getParam("approach_retreat_desired_dist", approach_retreat_desired_dist_);
                _nh.getParam("approach_retreat_min_dist", approach_retreat_min_dist_);
                _nh.getParam("grasp_depth", grasp_depth_);
                _nh.getParam("object_size", object_size_);
                _nh.getParam("angle_resolution", angle_resolution_);
                _nh.getParam("ee_parent_link", ee_parent_link_);
                _nh.getParam("ee_group", ee_group_);
                _nh.getParam("base_link", base_link_);
                _nh.getParam("pregrasp_time_from_start", pregrasp_time_from_start_);
                _nh.getParam("grasp_time_from_start", grasp_time_from_start_);

                ROS_ERROR_STREAM("THE BASE IS : " << base_link_);

                std::vector<std::string> joint_names = {"right_finger_joint", "left_finger_joint"};

                pre_grasp_posture_.header.frame_id = base_link_;
                pre_grasp_posture_.header.stamp = ros::Time::now();
                // Name of joints:
                pre_grasp_posture_.joint_names = joint_names;
                // Position of joints
                pre_grasp_posture_.points.resize(1);
                pre_grasp_posture_.points[0].positions = {0.84, -0.84};
                pre_grasp_posture_.points[0].time_from_start = ros::Duration(pregrasp_time_from_start_);

                // Create grasp posture
                grasp_posture_.header.frame_id = base_link_;
                grasp_posture_.header.stamp = ros::Time::now();
                // Name of joints:
                grasp_posture_.joint_names = joint_names;
                // Position of joints
                grasp_posture_.points.resize(1);
                grasp_posture_.points[0].positions = {0.0, 0.0};
                grasp_posture_.points[0].time_from_start = ros::Duration(grasp_time_from_start_);
            }

        // Create all possible grasp positions for a object
        bool generateBlockGrasps(const geometry_msgs::Pose& object_pose,
                                 std::vector<moveit_msgs::Grasp>& possible_grasps)
            {
                // ---------------------------------------------------------------------------------------------
                // Calculate grasps in two axis in both directions
                //play with these .......................................................
                generateAxisGrasps( object_pose, X_AXIS, DOWN, HALF, 0, possible_grasps); // got no grasps with this alone
                generateAxisGrasps( object_pose, X_AXIS, UP,   HALF, 0, possible_grasps); // gives some grasps... looks ugly
                generateAxisGrasps( object_pose, Y_AXIS, DOWN, HALF, 0, possible_grasps); // GOOD ONES!
                generateAxisGrasps( object_pose, Y_AXIS, UP,   HALF, 0, possible_grasps); // gave a grasp from top... bad

                return true;
            }

        // Create grasp positions in one axis
        bool generateAxisGrasps(
                const geometry_msgs::Pose& object_pose,
                grasp_axis_t axis,
                grasp_direction_t direction,
                grasp_rotation_t rotation,
                double hand_roll,
                std::vector<moveit_msgs::Grasp>& possible_grasps)
            {
                // ---------------------------------------------------------------------------------------------
                // Create a transform from the object's frame (center of object) to /base_link
                tf::poseMsgToEigen(object_pose, object_global_transform_);

                // ---------------------------------------------------------------------------------------------
                // Grasp parameters

                // Create re-usable approach motion
                moveit_msgs::GripperTranslation pre_grasp_approach;
                pre_grasp_approach.direction.header.stamp = ros::Time::now();
                pre_grasp_approach.desired_distance = approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
                pre_grasp_approach.min_distance = approach_retreat_min_dist_; // half of the desired? Untested.

                // Create re-usable retreat motion
                moveit_msgs::GripperTranslation post_grasp_retreat;
                post_grasp_retreat.direction.header.stamp = ros::Time::now();
                post_grasp_retreat.desired_distance = approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
                post_grasp_retreat.min_distance = approach_retreat_min_dist_; // half of the desired? Untested.

                // Create re-usable blank pose
                geometry_msgs::PoseStamped grasp_pose_msg;
                grasp_pose_msg.header.stamp = ros::Time::now();
                grasp_pose_msg.header.frame_id = base_link_;

                // ---------------------------------------------------------------------------------------------
                // Angle calculations
                double radius = grasp_depth_; //0.12
                double xb;
                double yb = 0.0; // stay in the y plane of the object
                double zb;
                double theta1 = 0.0; // Where the point is located around the object
                double theta2 = 0.0; // UP 'direction'

                // Gripper direction (UP/DOWN) rotation. UP set by default
                if( direction == DOWN )
                    {
                        theta2 = M_PI;
                    }

                // ---------------------------------------------------------------------------------------------
                // ---------------------------------------------------------------------------------------------
                // Begin Grasp Generator Loop
                // ---------------------------------------------------------------------------------------------
                // ---------------------------------------------------------------------------------------------

                /* Developer Note:
   * Create angles 180 degrees around the chosen axis at given resolution
   * We create the grasps in the reference frame of the object, then later convert it to the base link
   */
                for(int i = 0; i <= angle_resolution_; ++i)
                    {
                        // Create a Grasp message
                        moveit_msgs::Grasp new_grasp;

                        // Calculate grasp pose
                        xb = radius*cos(theta1);
                        zb = radius*sin(theta1);

                        Eigen::Affine3d grasp_pose;

                        switch(axis)
                            {
                        case X_AXIS:
                            grasp_pose = Eigen::AngleAxisd(theta1, Eigen::Vector3d::UnitX())
                                    * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ())
                                    * Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX()); // Flip 'direction'

                            grasp_pose.translation() = Eigen::Vector3d( yb, xb ,zb);

                            break;
                        case Y_AXIS:
                            grasp_pose =
                                    Eigen::AngleAxisd(M_PI - theta1, Eigen::Vector3d::UnitY())
                                    *Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX()); // Flip 'direction'

                            grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);

                            break;
                        case Z_AXIS:
                            ROS_ERROR_STREAM_NAMED("grasp","Z Axis not implemented!");
                            return false;

                            break;
                            }

                        /* The estimated probability of success for this grasp, or some other measure of how "good" it is.
     * Here we base bias the score based on how far the wrist is from the surface, preferring a greater
     * distance to prevent wrist/end effector collision with the table
     */
                        double score = sin(theta1);
                        new_grasp.grasp_quality = std::max(score, 0.1); // don't allow score to drop below 0.1 b/c all grasps are ok

                        // Calculate the theta1 for next time
                        if (rotation == HALF)
                            theta1 += M_PI / angle_resolution_;
                        else
                            {
                                theta1 += 2*M_PI / angle_resolution_;
                            }

                        // A name for this grasp
                        static int grasp_id = 0;
                        new_grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);
                        ++grasp_id;

                        // PreGrasp and Grasp Postures --------------------------------------------------------------------------

                        // The internal posture of the hand for the pre-grasp only positions are used
                        new_grasp.pre_grasp_posture = pre_grasp_posture_;

                        // The internal posture of the hand for the grasp positions and efforts are used
                        new_grasp.grasp_posture = grasp_posture_;

                        // Grasp ------------------------------------------------------------------------------------------------
                        // ------------------------------------------------------------------------
                        // Optionally roll wrist with respect to object pose
                        Eigen::Affine3d roll_gripper;
                        roll_gripper = Eigen::AngleAxisd(hand_roll, Eigen::Vector3d::UnitX());
                        grasp_pose = grasp_pose * roll_gripper;

                        // ------------------------------------------------------------------------
                        // Change grasp to frame of reference of this custom end effector

                        // Convert to Eigen
                        Eigen::Affine3d eef_conversion_pose;
                        tf::poseMsgToEigen(grasp_pose_to_eef_pose_, eef_conversion_pose);

                        // Transform the grasp pose
                        grasp_pose = grasp_pose * eef_conversion_pose;

                        // ------------------------------------------------------------------------
                        // Convert pose to global frame (base_link)
                        tf::poseEigenToMsg(object_global_transform_ * grasp_pose, grasp_pose_msg.pose);

                        // The position of the end-effector for the grasp relative to a reference frame (that is always specified elsewhere, not in this message)
                        new_grasp.grasp_pose = grasp_pose_msg;

                        // Other ------------------------------------------------------------------------------------------------

                        // the maximum contact force to use while grasping (<=0 to disable)
                        new_grasp.max_contact_force = 0;

                        // -------------------------------------------------------------------------------------------------------
                        // -------------------------------------------------------------------------------------------------------
                        // Approach and retreat
                        // -------------------------------------------------------------------------------------------------------
                        // -------------------------------------------------------------------------------------------------------

                        // Straight down ---------------------------------------------------------------------------------------
                        // With respect to the base link/world frame

                        // Approach
                        pre_grasp_approach.direction.header.frame_id = base_link_;
                        pre_grasp_approach.direction.vector.x = 0;
                        pre_grasp_approach.direction.vector.y = 0;
                        pre_grasp_approach.direction.vector.z = -1; // Approach direction (negative z axis)  // TODO: document this assumption
                        new_grasp.pre_grasp_approach = pre_grasp_approach;

                        // Retreat
                        post_grasp_retreat.direction.header.frame_id = base_link_;
                        post_grasp_retreat.direction.vector.x = 0;
                        post_grasp_retreat.direction.vector.y = 0;
                        post_grasp_retreat.direction.vector.z = 1; // Retreat direction (pos z axis)
                        new_grasp.post_grasp_retreat = post_grasp_retreat;

                        // Add to vector
                        possible_grasps.push_back(new_grasp);

                        // Angled with pose -------------------------------------------------------------------------------------
                        // Approach with respect to end effector orientation

                        // Approach
                        pre_grasp_approach.direction.header.frame_id = ee_parent_link_;
                        pre_grasp_approach.direction.vector.x = 0;
                        pre_grasp_approach.direction.vector.y = 0;
                        pre_grasp_approach.direction.vector.z = 1;
                        new_grasp.pre_grasp_approach = pre_grasp_approach;

                        // Retreat
                        post_grasp_retreat.direction.header.frame_id = ee_parent_link_;
                        post_grasp_retreat.direction.vector.x = 0;
                        post_grasp_retreat.direction.vector.y = 0;
                        post_grasp_retreat.direction.vector.z = -1;
                        new_grasp.post_grasp_retreat = post_grasp_retreat;

                        // Add to vector
                        possible_grasps.push_back(new_grasp);

                    }

                ROS_INFO_STREAM_NAMED("grasp", "Generated " << possible_grasps.size() << " grasps." );

                return true;
            }

        void pick(moveit::planning_interface::MoveGroup &group,
                  geometry_msgs::Pose& object_pose)
            {
                std::vector<moveit_msgs::Grasp> grasps;
                /*
                generateAxisGrasps(object_pose,
                                   Y_AXIS,
                                   DOWN,
                                   HALF,
                                   0.0,
                                   grasps);
                                   */
                generateBlockGrasps(object_pose, grasps);
                group.setSupportSurfaceName("table");
                group.pick("part", grasps);
            }
    private:
        ros::NodeHandle _nh;
        trajectory_msgs::JointTrajectory pre_grasp_posture_; // when the end effector is in "open" position
        trajectory_msgs::JointTrajectory grasp_posture_; // when the end effector is in "close" position
        // Transform from frame of box to global frame
        Eigen::Affine3d object_global_transform_;
        geometry_msgs::Pose grasp_pose_to_eef_pose_;
        double approach_retreat_desired_dist_ = 0.6;
        double approach_retreat_min_dist_ = 0.4;
        double grasp_depth_ = 0.12;
        double object_size_ = 0.04;
        int angle_resolution_ = 16;
        double pregrasp_time_from_start_;
        double grasp_time_from_start_;
        std::string base_link_ = "/base_link";
        std::string ee_parent_link_, ee_group_;
        bool verbose_ = false;
    };



/*void place(moveit::planning_interface::MoveGroup &group)
{
  std::vector<moveit_msgs::PlaceLocation> loc;

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_footprint";
  p.pose.position.x = 0.7;
  p.pose.position.y = 0.0;
  p.pose.position.z = 0.5;
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1;
  moveit_msgs::PlaceLocation g;
  g.place_pose = p;

  g.pre_place_approach.direction.vector.z = -1.0;
  g.post_place_retreat.direction.vector.x = -1.0;
  g.post_place_retreat.direction.header.frame_id = "base_footprint";
  g.pre_place_approach.direction.header.frame_id = "r_gripper_tool_frame";
  g.pre_place_approach.min_distance = 0.1;
  g.pre_place_approach.desired_distance = 0.2;
  g.post_place_retreat.min_distance = 0.1;
  g.post_place_retreat.desired_distance = 0.25;

  g.post_place_posture.joint_names.resize(5);
  g.post_place_posture.joint_names[0] = "r_gripper_l_finger_joint";
  g.post_place_posture.joint_names[1] = "r_gripper_r_finger_joint";
  g.post_place_posture.joint_names[2] = "r_gripper_motor_screw_joint";
  g.post_place_posture.joint_names[3] = "r_gripper_motor_slider_joint";
  g.post_place_posture.joint_names[4] = "r_gripper_joint";
  g.post_place_posture.points.resize(1);
  g.post_place_posture.points[0].positions.resize(5);
  g.post_place_posture.points[0].positions[0] = 0;
  g.post_place_posture.points[0].positions[1] = 0;
  g.post_place_posture.points[0].positions[2] = 0;
  g.post_place_posture.points[0].positions[3] = 0;
  g.post_place_posture.points[0].positions[4] = 0;

  loc.push_back(g);
  group.setSupportSurfaceName("table");


  // add path constraints
  moveit_msgs::Constraints constr;
  constr.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = p.header.frame_id;
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.2;
  ocm.absolute_z_axis_tolerance = M_PI;
  ocm.weight = 1.0;
  //  group.setPathConstraints(constr);
  group.setPlannerId("RRTConnectkConfigDefault");

  group.place("part", loc);
}*/

int main(int argc, char **argv)
    {
        ros::init (argc, argv, "right_arm_pick_place");
        ros::AsyncSpinner spinner(1);
        spinner.start();

        ros::NodeHandle nh;
        ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
        ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

        ros::WallDuration(1.0).sleep();

        //moveit::planning_interface::MoveGroup group("right_arm");
        //group.setPlanningTime(45.0);
        //group.setPlannerId("RRTConnectkConfigDefault");

        CRUSTCRAWLER_Mover::Ptr crustcrawler_mover;
        crustcrawler_mover.reset(new CRUSTCRAWLER_Mover(nh));
        the_grapper my_grapper;


        moveit_msgs::CollisionObject co;
        co.header.stamp = ros::Time::now();
        co.header.frame_id = "base";

        // remove pole
        co.id = "pole";
        co.operation = moveit_msgs::CollisionObject::REMOVE;
        pub_co.publish(co);

        // add pole
        co.operation = moveit_msgs::CollisionObject::ADD;
        co.primitives.resize(1);
        co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
        co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
        co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
        co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
        co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
        co.primitive_poses.resize(1);
        co.primitive_poses[0].position.x = 0.4;
        co.primitive_poses[0].position.y = 0.0;
        co.primitive_poses[0].position.z = 0.0;
        co.primitive_poses[0].orientation.w = 1.0;
        pub_co.publish(co);



        // remove table
        co.id = "table";
        co.operation = moveit_msgs::CollisionObject::REMOVE;
        pub_co.publish(co);

        // add table
        co.operation = moveit_msgs::CollisionObject::ADD;
        co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
        co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
        co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.05;
        co.primitive_poses[0].position.x = 0.4;
        co.primitive_poses[0].position.y = 0.0;
        co.primitive_poses[0].position.z = 0.0;
        pub_co.publish(co);



        co.id = "part";
        co.operation = moveit_msgs::CollisionObject::REMOVE;
        pub_co.publish(co);

        geometry_msgs::Pose object_pose;
        object_pose.position.x = 0.3;
        object_pose.position.y = -0.3;
        object_pose.position.z = 0.04;

        moveit_msgs::AttachedCollisionObject aco;
        aco.object = co;
        pub_aco.publish(aco);

        co.operation = moveit_msgs::CollisionObject::ADD;
        co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.03;
        co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.03;
        co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.1;

        co.primitive_poses[0].position.x = object_pose.position.x;
        co.primitive_poses[0].position.y = object_pose.position.y;
        co.primitive_poses[0].position.z = object_pose.position.z;
        pub_co.publish(co);

        // wait a bit for ros things to initialize
        ros::WallDuration(1.0).sleep();

        my_grapper.pick(*crustcrawler_mover->group, object_pose);
        //pick(group);

        ros::WallDuration(1.0).sleep();

        //place(*crustcrawler_mover->group);
        //place(group);

        //ros::waitForShutdown();
        return 0;
    }
