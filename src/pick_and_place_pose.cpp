#include <ros/ros.h>
#include <cube_pick_place/pick_place_pose.h>

int main(int argc, char **argv){
        ros::init(argc, argv, "pick_and_place_pose");
        ros::NodeHandle nh;

        ros::Publisher pick_place_pose_pub = nh.advertise<cube_pick_place::pick_place_pose>("/pick_place_pose", true);

        double pick_x, pick_y, pick_z, place_x, place_y, place_z;

        ros::Rate rate(1);

        while(ros::ok()){

                ROS_INFO("Please Enter pick point ...");

                std::cin >> pick_x >> pick_y >> pick_z;

                ROS_INFO("Please Enter place point ...");

                std::cin >> place_x >> place_y >> place_z;

                cube_pick_place::pick_place_pose msg;

                geometry_msgs::Point the_point;
                the_point.x = pick_x;
                the_point.y = pick_y;
                the_point.z = pick_z;

                msg.pick_pose.push_back(the_point);

                the_point.x = place_x;
                the_point.y = place_y;
                the_point.z = place_z;

                msg.place_pose.push_back(the_point);

                pick_place_pose_pub.publish(msg);

                ros::spinOnce();
                rate.sleep();
            }

        return 0;
    }
