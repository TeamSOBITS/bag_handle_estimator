#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <bag_handle_estimator/execute_ctrl.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <string>
#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/surface/concave_hull.h>

class BagHandleEstimator {
 private:
  ros::NodeHandle    nh;
  ros::Subscriber    sub_cloud;
  ros::Subscriber    sub_ctrl;
  ros::Publisher     pub_plane;
  ros::Publisher     pub_marker;
  ros::ServiceServer service_execute_ctrl_;

  tf::TransformListener    listerner;
  tf::TransformBroadcaster broadcaster;

  // paramater
  bool   execute_flag;
  bool   estimated_flag;
  bool   Publish_Plane;
  bool   Publish_Object;
  double depth_z_min;
  double depth_z_max;
  double depth_x_min;
  double depth_x_max;

  std::string sub_point_topic_name;
  std::string sub_ctrl_topic_name;
  std::string camera_frame_name;
  std::string base_frame_name;
  std::string map_frame_name;

  geometry_msgs::PointStamped map_2_high_point;

 public:
  BagHandleEstimator() {
    this->estimated_flag = true;

    // load rosparam
    ros::param::get("pub_plane_cloud", this->Publish_Plane);
    ros::param::get("execute_default", this->execute_flag);

    ros::param::get("depth_range_min_x", this->depth_x_min);
    ros::param::get("depth_range_max_x", this->depth_x_max);
    ros::param::get("depth_range_min_z", this->depth_z_min);
    ros::param::get("depth_range_max_z", this->depth_z_max);

    ros::param::get("sub_point_topic_name", this->sub_point_topic_name);
    ros::param::get("sub_ctrl_topic_name", this->sub_ctrl_topic_name);

    ros::param::get("camera_frame_name", this->camera_frame_name);
    ros::param::get("base_frame_name", this->base_frame_name);
    ros::param::get("map_frame_name", this->map_frame_name);

    // Create a ROS subscriber and publisher
    this->sub_cloud = nh.subscribe(this->sub_point_topic_name, 1, &BagHandleEstimator::cloud_cb, this);
    this->service_execute_ctrl_ = nh.advertiseService("execute_ctrl", &BagHandleEstimator::execute_ctrl_server, this);
    this->pub_plane  = nh.advertise<sensor_msgs::PointCloud2>("cloud_plane", 1);
    this->pub_marker = nh.advertise<visualization_msgs::MarkerArray>("marker", 1);

    std::cout << "start bag_handle_estimator" << std::endl;

  }  // bag_handle_estimator

  bool execute_ctrl_server(bag_handle_estimator::execute_ctrl::Request&  req,
                           bag_handle_estimator::execute_ctrl::Response& res) {
    this->execute_flag = req.request;
    if (this->execute_flag == true) {
      ROS_INFO("Start bag_handle_estimator.");
    } else {
      ROS_INFO("Stop bag_handle_estimator.");
    }
    res.response = true;
    return true;
  }

  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (this->execute_flag == false) {
      return;
    }
    camera_frame_name = msg->header.frame_id;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_input);
    if (cloud_input->points.size() == 0) {
      return;
    }

    //ここで座標変換可能か確認
    bool key = listerner.canTransform(base_frame_name, camera_frame_name, ros::Time(0));
    if (key == false) {
      ROS_ERROR("bag_handle_estimator : pcl canTransform failue");
      return;
    }

    // base_flame基準のpointcloudに変換
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(base_frame_name, ros::Time(0), *cloud_input, camera_frame_name, *cloud, listerner);

    // filtering X limit
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cut_x(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ>     pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(depth_x_min, depth_x_max);
    pass_x.filter(*cloud_cut_x);
    if (cloud_cut_x->points.size() == 0) {
      return;
    }

    // filtering Z limit
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cuted(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ>     pass_z;
    pass_z.setInputCloud(cloud_cut_x);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(depth_z_min, depth_z_max * 1.1);
    pass_z.filter(*cloud_cuted);
    if (cloud_cuted->points.size() == 0) {
      return;
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ExtractIndices<pcl::PointXYZ>  extract;
    extract.setInputCloud(cloud_cuted);
    extract.setNegative(false);
    extract.filter(*cloud_plane);
    if (cloud_plane->points.size() == 0) {
      return;
    }
    if (this->Publish_Plane == true) {
      this->pub_plane_cloud(cloud_plane);
    }  // publish plane cloud

    //最も位置が高いポイントを探索
    int    most_high_point_index = 0;  
    double most_high_point      = DBL_MAX;
    for (int i = 0; i < cloud_cuted->points.size(); i++) {
      double temp_distance = cloud_cuted->points[i].x;
      if (temp_distance > most_high_point) {
        most_high_point      = temp_distance;
        most_high_point_index = i;
      }
    }

    bool key2 = listerner.canTransform(this->map_frame_name, this->base_frame_name, ros::Time(0));
    if (key2 == false) {
      ROS_ERROR("bag_handle_estimator : canTransform failue");
      return;
    }

    geometry_msgs::PointStamped base_2_high_point;
    base_2_high_point.header.frame_id = this->base_frame_name;
    base_2_high_point.header.stamp    = ros::Time(0);
    base_2_high_point.point.x         = cloud_cuted->points[most_high_point_index].x ;
    base_2_high_point.point.y         = cloud_cuted->points[most_high_point_index].y ;
    base_2_high_point.point.z         = cloud_cuted->points[most_high_point_index].z - 0.04;
    this->listerner.transformPoint(this->map_frame_name, base_2_high_point, this->map_2_high_point);
    this->estimated_flag = true;
    broadcast_handle_point();
  }  // cloud_cb

  // 点群の出力
  void pub_plane_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_color(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, *cloud_plane_color);
    for (int i = 0; i < cloud_plane_color->points.size(); i++) {
      cloud_plane_color->points[i].g = 255;
    }  // for
    sensor_msgs::PointCloud2 sensor_cloud_plane;
    pcl::toROSMsg(*cloud_plane_color, sensor_cloud_plane);
    sensor_cloud_plane.header.frame_id = base_frame_name;
    pub_plane.publish(sensor_cloud_plane);
  }  // pub_plane_cloud

  // tfの出力
  void broadcast_handle_point() {
    if (this->estimated_flag == false) {
      return;
    }

    broadcaster.sendTransform(tf::StampedTransform(
        tf::Transform(  // tfのブロードキャスト
            tf::Quaternion(0, 0, 0, 1),
            tf::Vector3(map_2_high_point.point.x, map_2_high_point.point.y, map_2_high_point.point.z)),
        ros::Time::now(),
        this->map_frame_name,
        "handle_point"));

  }  // broadcast_handle_point
};

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "bag_handle_estimator");
  ROS_INFO("Start bag_handle_estimator.");

  BagHandleEstimator ppe;
  while (ros::ok() == true) {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  return 0;
}  // main
