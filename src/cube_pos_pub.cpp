#include <iostream>
#include <memory>
#include <string>

#include <cafer_core/cafer_core.hpp>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl_conversions/pcl_conversions.h>

#include <image_processing/SurfaceOfInterest.h>
#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>

#include <iagmm/nnmap.hpp>

#include <yaml-cpp/yaml.h>


using namespace cafer_core;
namespace ip = image_processing;

class CubePosition : public Component{
    using Component::Component;

public:
    void init(){
        _nnmap_class.emplace("color",iagmm::NNMap(3,2,.3,0.05));
        _nnmap_class.emplace("normal",iagmm::NNMap(3,2,.3,0.05));

        client_connect_to_ros();

        load_experiment();

    }

    void client_connect_to_ros(){
        XmlRpc::XmlRpcValue glob_params;
        XmlRpc::XmlRpcValue exp_params;

        cafer_core::ros_nh->getParam("/dream_babbling/params", glob_params);
        cafer_core::ros_nh->getParam("experiment", exp_params);

        _load_exp = static_cast<std::string>(exp_params["soi"]["load_exp"]);
        _threshold = std::stod(exp_params["soi"]["threshold"]);
        _modality = static_cast<std::string>(exp_params["soi"]["modality"]);

        _rgbd_sub.reset(new rgbd_utils::RGBD_Subscriber(glob_params["rgb_info_topic"],
                glob_params["rgb_topic"],
                glob_params["depth_info_topic"],
                glob_params["depth_topic"],
                *ros_nh));

        _position_pub.reset(
                    new ros::Publisher(
                        ros_nh->advertise<geometry_msgs::Point>("cube_position",5)));

        _update_workspace();


    }

    void client_disconnect_from_ros(){
        _rgbd_sub.reset();
        _position_pub.reset();
    }

    void update(){
        sensor_msgs::ImageConstPtr depth_msg(
                    new sensor_msgs::Image(_rgbd_sub->get_depth()));
        sensor_msgs::ImageConstPtr rgb_msg(
                    new sensor_msgs::Image(_rgbd_sub->get_rgb()));
        sensor_msgs::CameraInfoConstPtr info_msg(
                    new sensor_msgs::CameraInfo(_rgbd_sub->get_rgb_info()));

        ip::PointCloudT::Ptr input_cloud;

        if (!rgb_msg->data.empty() && !depth_msg->data.empty()) {
            rgbd_utils::RGBD_to_Pointcloud converter(depth_msg, rgb_msg, info_msg);

            sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();
            input_cloud = ip::PointCloudT::Ptr(new ip::PointCloudT);
            pcl::fromROSMsg(ptcl_msg, *input_cloud);

        }else {
            ROS_INFO_STREAM("Waiting for input point_cloud");
            return;
        }

        compute_saliency_map(input_cloud);
        find_center_object();
        _position_pub->publish(_position_msg);

    }

    bool load_experiment(){
        std::cout << "load : " << _load_exp << std::endl;
        if(_load_exp.empty())
            return false;
        boost::filesystem::directory_iterator dir_it(_load_exp);
        boost::filesystem::directory_iterator end_it;
        std::vector<std::string> split_str;
        std::string type;
        std::map<std::string,std::string> dataset_file;

        for(;dir_it != end_it; ++dir_it){
            boost::split(split_str,dir_it->path().string(),boost::is_any_of("/"));
            boost::split(split_str,split_str.back(),boost::is_any_of("_"));
            type = split_str[0];
            boost::split(split_str,split_str.back(),boost::is_any_of("."));

            if(type == "dataset")
                dataset_file.emplace(split_str[0],dir_it->path().string());
        }

        for(const auto& file : dataset_file){
            iagmm::TrainingData data = _load_dataset(file.second);
            _nnmap_class[file.first].set_samples(data);
        }
    }

    bool compute_saliency_map(const ip::PointCloudT::Ptr input_cloud){
        _soi.clear();
        _soi.setInputCloud(input_cloud);

        if(!_soi.computeSupervoxel(*_workspace))
            return false;
        for(auto& classifier: _nnmap_class){
           _soi.init_weights(classifier.first,.5);
           classifier.second.default_estimation = .5;
           _soi.compute_weights<iagmm::NNMap>(classifier.first,classifier.second);
        }

        return true;
    }

    void find_center_object(){
        for(auto it = _soi.get_weights()[_modality].begin();
            it != _soi.get_weights()[_modality].end(); it++){
            if(it->second < _threshold){
                _soi.getSupervoxels().erase(it->second);
                _soi.getAdjacencyMap().erase(it->second);
                _soi.consolidate();
            }
        }

        pcl::PointXYZ pos = _soi.globalPosition();
        _position_msg.x = pos.x;
        _position_msg.y = pos.y;
        _position_msg.z = pos.z;
    }



private:
    std::unique_ptr<rgbd_utils::RGBD_Subscriber> _rgbd_sub;
    std::unique_ptr<Publisher> _position_pub;
    ip::SurfaceOfInterest _soi;
    std::map<std::string,iagmm::NNMap> _nnmap_class;
    std::string _load_exp;
    std::string _modality;
    double _threshold;
    geometry_msgs::Point _position_msg;
    std::unique_ptr<ip::workspace_t> _workspace;

    iagmm::TrainingData _load_dataset(const std::string& filename)
    {
        iagmm::TrainingData dataset;

        YAML::Node fileNode = YAML::LoadFile(filename);
        if (fileNode.IsNull()) {
            ROS_ERROR("File not found.");
            return dataset;
        }

        YAML::Node features = fileNode["frame_0"]["features"];


        for (unsigned int i = 0; i < features.size(); ++i) {
            std::stringstream stream;
            stream << "feature_" << i;
            YAML::Node tmp_node = features[stream.str()];

            Eigen::VectorXd feature(tmp_node["value"].size());
            for(size_t i = 0; i < tmp_node["value"].size(); ++i)
                feature(i) = tmp_node["value"][i].as<double>();


            dataset.add(tmp_node["label"].as<int>(),feature);
        }
        return dataset;
    }

    void _update_workspace()
    {
        XmlRpc::XmlRpcValue wks;

        cafer_core::ros_nh->getParamCached("experiment/workspace", wks);

        _workspace.reset(
                new ip::workspace_t(true,
                                static_cast<double>(wks["sphere"]["x"]),
                               static_cast<double> (wks["sphere"]["y"]),
                                static_cast<double>(wks["sphere"]["z"]),
                               static_cast<double> (wks["sphere"]["radius"]),
                               static_cast<double> (wks["sphere"]["threshold"]),
                                {static_cast<double>(wks["csg_intersect_cuboid"]["x_min"]),
                static_cast<double>(wks["csg_intersect_cuboid"]["x_max"]),
                                static_cast<double> (wks["csg_intersect_cuboid"]["y_min"]),
                static_cast<double>(wks["csg_intersect_cuboid"]["y_max"]),
                                static_cast<double> (wks["csg_intersect_cuboid"]["z_min"]),
                static_cast<double>(wks["csg_intersect_cuboid"]["z_max"])}));

    }
};

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

int main(int argc, char** argv)
{
    std::string node_name;
    XmlRpc::XmlRpcValue cafer;

    node_name = parse_arg(argc, argv, "cube_pos_pub");

    cafer_core::init(argc, argv, node_name);
    cafer_core::ros_nh->getParam("cafer", cafer);

    CubePosition cube_pos(cafer["mgmt"], cafer["type"], cafer["freq"], cafer["uuid"]);

    cube_pos.wait_for_init();

    while (ros::ok() && !cube_pos.get_terminate()) {
        cube_pos.spin();
        cube_pos.update();
        cube_pos.sleep();
    }

    return 0;
}

