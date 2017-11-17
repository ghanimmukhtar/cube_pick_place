//
// Created by phlf on 06/07/16.
//

#ifndef DREAM_BABBLING_BABBLING_CONSTANTS_HPP
#define DREAM_BABBLING_BABBLING_CONSTANTS_HPP

#include <Eigen/Core>
#include <math.h>
#include <string>

#include <tf/transform_listener.h>

namespace global {
    std::string parse_arg(int& argc, char**& argv, const std::string& default_val)
    {
        std::string key;
        std::string value;
        std::string temp_str;
        std::string::size_type res;

        key = "__name:=";

        for (unsigned short i = 0; i < argc; ++i) {
            temp_str = argv[i];
            res = temp_str.find(key);

            if (res != std::string::npos) {
                value = temp_str.erase(res, key.length());
                break;
            }
            else if (i == argc - 1) {
                value = default_val;
            }
        }
        return value;
    }
}//global namespace

namespace babbling {

struct sv_param {
    static constexpr bool use_transform = false;
    static constexpr double voxel_resolution = 0.008f;
    static constexpr double color_importance = 0.2f;
    static constexpr double spatial_importance = 0.4f;
    static constexpr double normal_importance = 0.4f;
    static constexpr double seed_resolution = 0.05f;
    //CAMERA PARAMETERS
    static constexpr float depth_princ_pt_x = 479.75;
    static constexpr float depth_princ_pt_y = 269.75;
    static constexpr float rgb_princ_pt_x = 479.75;
    static constexpr float rgb_princ_pt_y = 269.75;
    static constexpr float focal_length_x = 540.68603515625;
    static constexpr float focal_length_y = 540.68603515625;
    static constexpr float height = 540;
    static constexpr float width = 960;
};

struct soi_param {
    static constexpr double interest_increment = .01f;
    static constexpr double non_interest_val = 0.f;
    static constexpr double color_normal_ratio = .5f;
    static constexpr double distance_threshold = .1f;
};

struct base_params {
    static constexpr double x_trans = 1.8;
    static constexpr double y_trans = 0.4;
    static constexpr double z_trans = 0.4;
    static constexpr double x_rot = -(M_PI / 2 + 0.5);
    static constexpr double y_rot = 0;
    static constexpr double z_rot = M_PI / 2;
};



/**
* @brief Conversion from the camera frame to the robot frame
* @param out_pose
* @param in_pose
* @param translation along x
* @param translation along y
* @param translation along z
* @param rotation around x (yaw)
* @param rotation around y (pitch)
* @param rotation around z (roll)
*/
    void base_conversion(Eigen::Vector4d& out_pose, const Eigen::Vector4d& in_pose)
    {
        Eigen::Matrix4d Trans_M;


        /*lectern camera
        Trans_M << 0.92568,  -0.18798,   0.21837,  -0.42638,
                -0.305504, -0.605985,  0.730977,  0.141395,
                0.0229272, -0.744332, -0.616878,   0.68228,
                0,         0,         0,         1;
        //*/

        //*front table camera
        Trans_M <<  -.62, .056, -.79, 1.21,
                -.0016, -1., -.07, .2,
                -.8, -.042, .61, .33,
                0, 0, 0, 1;
        //*/

        out_pose = Trans_M * in_pose;
    }


    //Convert object position from camera frame to robot frame
    void tf_base_conversion(Eigen::Vector3d& object_pose_in_robot_frame, Eigen::Vector3d& object_pose_in_camera_frame,
                            const std::string& child, const std::string& parent){
        tf::TransformListener listener;
        tf::StampedTransform stamped_transform;
        //std::string child_frame = "/camera_depth_optical_frame";
        std::string child_frame = child /*"/camera_rgb_optical_frame"*/;
        std::string parent_frame = parent;
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

}//babbling namespace

#endif // DREAM_BABBLING_BABBLING_CONSTANTS_HPP
