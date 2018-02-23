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
                msg.pick_pose.push_back(pick_x);
                msg.pick_pose.push_back(pick_y);
                msg.pick_pose.push_back(pick_z);

                msg.place_pose.push_back(place_x);
                msg.place_pose.push_back(place_y);
                msg.place_pose.push_back(place_z);

                pick_place_pose_pub.publish(msg);

                ros::spinOnce();
                rate.sleep();
            }

        return 0;
    }
