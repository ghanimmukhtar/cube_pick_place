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

static const std::string ROBOT_DESCRIPTION="robot_description";

using namespace crustcrawler_mover;

bool pick(moveit::planning_interface::MoveGroup &group)
    {
        std::vector<moveit_msgs::Grasp> grasps;

        geometry_msgs::PoseStamped p;
        p.header.frame_id = "base";
        p.pose.position.x = 0.3;
        p.pose.position.y = -0.1;
        p.pose.position.z = 0.0;
        p.pose.orientation.x = 0;
        p.pose.orientation.y = 1;
        p.pose.orientation.z = 0;
        p.pose.orientation.w = 0;
        moveit_msgs::Grasp g;
        g.grasp_pose = p;

        g.pre_grasp_approach.direction.vector.z = 1.0;
        g.pre_grasp_approach.direction.header.frame_id = "link_gripper_base";
        g.pre_grasp_approach.min_distance = 0.05;
        g.pre_grasp_approach.desired_distance = 0.1;

        g.post_grasp_retreat.direction.header.frame_id = "base";
        g.post_grasp_retreat.direction.vector.x = 1.0;
        g.post_grasp_retreat.min_distance = 0.1;
        g.post_grasp_retreat.desired_distance = 0.05;

        g.pre_grasp_posture.joint_names.resize(2);
        g.pre_grasp_posture.joint_names[0] = "left_finger_joint";
        g.pre_grasp_posture.joint_names[1] = "right_finger_joint";
        g.pre_grasp_posture.points.resize(1);
        g.pre_grasp_posture.points[0].positions.resize(2);
        g.pre_grasp_posture.points[0].positions[0] = -0.84;
        g.pre_grasp_posture.points[0].positions[1] = 0.84;

        g.grasp_posture.joint_names.resize(2);
        g.grasp_posture.joint_names[0] = "left_finger_joint";
        g.grasp_posture.joint_names[1] = "right_finger_joint";
        g.grasp_posture.points.resize(1);
        g.grasp_posture.points[0].positions.resize(2);
        g.grasp_posture.points[0].positions[0] = -0.84;
        g.grasp_posture.points[0].positions[1] = 0.84;

        grasps.push_back(g);
        group.setSupportSurfaceName("table");
        return group.pick("part", grasps);
    }

bool place(moveit::planning_interface::MoveGroup &group)
    {
        std::vector<moveit_msgs::PlaceLocation> loc;

        geometry_msgs::PoseStamped p;
        p.header.frame_id = "base";
        p.pose.position.x = 0.4;
        p.pose.position.y = 0.0;
        p.pose.position.z = 0.07;
        p.pose.orientation.x = 0;
        p.pose.orientation.y = 1;
        p.pose.orientation.z = 0;
        p.pose.orientation.w = 0;
        moveit_msgs::PlaceLocation g;
        g.place_pose = p;

        g.pre_place_approach.direction.vector.x = -1.0;
        g.post_place_retreat.direction.vector.z = -1.0;
        g.post_place_retreat.direction.header.frame_id = "base";
        g.pre_place_approach.direction.header.frame_id = "link_gripper_base";
        g.pre_place_approach.min_distance = 0.05;
        g.pre_place_approach.desired_distance = 0.1;
        g.post_place_retreat.min_distance = 0.05;
        g.post_place_retreat.desired_distance = 0.1;

        g.post_place_posture.joint_names.resize(2);
        g.post_place_posture.joint_names[0] = "left_finger_joint";
        g.post_place_posture.joint_names[1] = "right_finger_joint";
        g.post_place_posture.points.resize(1);
        g.post_place_posture.points[0].positions.resize(2);
        g.post_place_posture.points[0].positions[0] = 0;
        g.post_place_posture.points[0].positions[1] = 0;

        loc.push_back(g);
        group.setSupportSurfaceName("table");


        // add path constraints
        moveit_msgs::Constraints constr;
        constr.orientation_constraints.resize(1);
        moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
        ocm.link_name = "link_gripper_base";
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

        return group.place("part", loc);
    }

int main(int argc, char **argv)
    {
        ros::init (argc, argv, "arm_pick_place");
        ros::AsyncSpinner spinner(1);
        spinner.start();

        ros::NodeHandle nh;
        ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
        ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

        ros::WallDuration(1.0).sleep();

//        moveit::planning_interface::MoveGroup group("arm");
//        group.setPlanningTime(45.0);
//        group.setPlannerId("RRTConnectkConfigDefault");

        CRUSTCRAWLER_Mover::Ptr crustcrawler_mover;
        crustcrawler_mover.reset(new CRUSTCRAWLER_Mover(nh));

        crustcrawler_mover->group->setPlanningTime(45.0);
        crustcrawler_mover->group->setPlannerId("RRTConnectkConfigDefault");

        ROS_ERROR_STREAM("PLANNING FRAME IS : " << crustcrawler_mover->group->getPlanningFrame());
        ROS_ERROR_STREAM("EEF IS : " << crustcrawler_mover->group->getEndEffectorLink());

        moveit_msgs::CollisionObject co;
        co.header.stamp = ros::Time::now();
        co.header.frame_id = "base";


        co.primitives.resize(1);
        co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
        co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
        co.primitive_poses.resize(1);



        // remove table
        co.id = "table";
        co.operation = moveit_msgs::CollisionObject::REMOVE;
        pub_co.publish(co);

        // add table
        //co.operation = moveit_msgs::CollisionObject::ADD;
        co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
        co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
        co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.2;
        co.primitive_poses[0].position.x = 0.4;
        co.primitive_poses[0].position.y = -0.2;
        co.primitive_poses[0].position.z = 0.0;
        //pub_co.publish(co);



        co.id = "part";
        co.operation = moveit_msgs::CollisionObject::REMOVE;
        pub_co.publish(co);

        moveit_msgs::AttachedCollisionObject aco;
        aco.object = co;
        pub_aco.publish(aco);

        co.operation = moveit_msgs::CollisionObject::ADD;
        co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.03;
        co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.03;
        co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.03;

        co.primitive_poses[0].position.x = 0.3;
        co.primitive_poses[0].position.y = -0.1;
        co.primitive_poses[0].position.z = 0.0;
        pub_co.publish(co);

        // wait a bit for ros things to initialize
        ros::WallDuration(1.0).sleep();

        //pick(group);
        int iteration = 0;
        while (!pick(*crustcrawler_mover->group) && iteration < 10){
                ros::WallDuration(1.0).sleep();
                iteration++;
            }

        //  ros::WallDuration(1.0).sleep();

        //  iteration = 0;
        //  while (!place(*crustcrawler_mover->group) && iteration < 10){
        //          ros::WallDuration(1.0).sleep();
        //          iteration++;
        //      }

        //  ros::waitForShutdown();
        return 0;
    }
