#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "bag_handle_estimator/execute_ctrl.h"

class BagHandleEstimator {
 private:
  ros::NodeHandle    nh;
  ros::Subscriber    sub_cloud;
  ros::Subscriber    sub_ctrl;
  ros::Publisher     pub_plane;
  ros::ServiceServer service_execute_ctrl_;

  tf2_ros::Buffer               tfBuffer_;
  tf2_ros::TransformBroadcaster broadcaster;

  // paramater
  bool   execute_flag;
  bool   estimated_flag;
  bool   publish_plane;
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
    ros::param::get("pub_plane_cloud", this->publish_plane);
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

    std::cout << "start bag_handle_estimator" << std::endl;

  }  // bag_handle_estimator

  // execute control
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

  // callback function
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

    bool key = tfBuffer_.canTransform(this->base_frame_name, this->camera_frame_name, ros::Time(0), ros::Duration(2.0));
    if (key == false) {
      ROS_ERROR("bag_handle_estimator : pcl canTransform failue");
      return;
    }
    
    // filtering X limit
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cut_x(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ>     pass_x;
    pass_x.setInputCloud(cloud_input);
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

    // publish plane cloud
    if (this->publish_plane == true) {
      this->pub_plane_cloud(cloud_plane);
    }  

    // search the most highest point
    int    most_high_point_index = 0;  
    double most_high_point      = DBL_MAX;
    for (int i = 0; i < cloud_cuted->points.size(); i++) {
      double temp_distance = cloud_cuted->points[i].x;
      if (temp_distance > most_high_point) {
        most_high_point      = temp_distance;
        most_high_point_index = i;
      }
    }

    // check if transformations are available
    bool key2 = tfBuffer_.canTransform(this->map_frame_name, this->base_frame_name, ros::Time(0));
    if (key2 == false) {
      ROS_ERROR("bag_handle_estimator : canTransform failue");
      return;
    }

    geometry_msgs::TransformStamped transformstamped;
    transformstamped = tfBuffer_.lookupTransform(this->base_frame_name, this->map_frame_name, ros::Time(0));


    // prepare broadcast tf 
    geometry_msgs::PointStamped base_2_high_point;
    base_2_high_point.header.frame_id = this->base_frame_name;
    base_2_high_point.header.stamp    = ros::Time(0);
    base_2_high_point.point.x         = cloud_cuted->points[most_high_point_index].x ;
    base_2_high_point.point.y         = cloud_cuted->points[most_high_point_index].y ;
    base_2_high_point.point.z         = cloud_cuted->points[most_high_point_index].z - 0.04;
    tf2::doTransform (base_2_high_point,this->map_2_high_point,transformstamped);
    this->estimated_flag = true;
    broadcast_handle_point();
  }  // cloud_cb

  // publish point cloud
  void pub_plane_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_color(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, *cloud_plane_color);
    for (int i = 0; i < cloud_plane_color->points.size(); i++) {
      cloud_plane_color->points[i].g = 255;
    }  
    sensor_msgs::PointCloud2 sensor_cloud_plane;
    pcl::toROSMsg(*cloud_plane_color, sensor_cloud_plane);
    sensor_cloud_plane.header.frame_id = base_frame_name;
    pub_plane.publish(sensor_cloud_plane);
  }  // pub_plane_cloud

  // broadcast tf
  void broadcast_handle_point() {
    if (this->estimated_flag == false) {
      return;
    }

    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = this->map_frame_name;
    transformStamped.child_frame_id = "handle_point";
    transformStamped.transform.translation.x = map_2_high_point.point.x;
    transformStamped.transform.translation.y = map_2_high_point.point.y;
    transformStamped.transform.translation.z = map_2_high_point.point.z;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    broadcaster.sendTransform(transformStamped);

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
