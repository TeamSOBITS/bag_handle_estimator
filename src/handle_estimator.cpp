#include <rclcpp/rclcpp.hpp>
// #include <ros/ros.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
// #include <pcl_ros/transforms.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "sobits_msgs/srv/run_ctrl.hpp"

class BagHandleEstimator {
 private:
  // ros::NodeHandle               nh;
  // ros::Subscriber               sub_cloud;
  // ros::Publisher                pub_plane;
  // ros::ServiceServer            service_execute_ctrl_;
  // tf2_ros::Buffer               tfBuffer_;
  // tf2_ros::TransformBroadcaster broadcaster;
  // tf2_ros::TransformListener    tfListener_;
  rclcpp::Node::SharedPtr nd_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_plane;
  rclcpp::Service<sobits_msgs::srv::RunCtrl>::SharedPtr service_execute_ctrl_;
  tf2_ros::Buffer               tfBuffer_;
  tf2_ros::TransformListener    tfListener_;
  tf2_ros::TransformBroadcaster broadcaster;

  // paramater
  bool   execute_flag;
  double depth_x_min;
  double depth_x_max;
  double depth_y_min;
  double depth_y_max;
  double depth_z_min;
  double depth_z_max;

  std::string sub_point_topic_name;
  std::string base_frame_name;

  geometry_msgs::msg::PointStamped base_2_high_point;

 public:
  BagHandleEstimator(std::shared_ptr<rclcpp::Node> nd) : nd_(nd), tfBuffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)), tfListener_(tfBuffer_), broadcaster(nd_) {
  
    // Initialize tfBuffer_ with a dedicated thread for populating data
    // tfBuffer_.setUsingDedicatedThread(true);/////?

    // load rosparam
    nd_->declare_parameter("execute_default", true);
    nd_->declare_parameter("depth_range_min_x", 0.10);
    nd_->declare_parameter("depth_range_max_x", 0.30);
    nd_->declare_parameter("depth_range_min_y",-0.20);
    nd_->declare_parameter("depth_range_max_y", 0.20);
    nd_->declare_parameter("depth_range_min_z", 0.00);
    nd_->declare_parameter("depth_range_max_z", 0.50);
    nd_->declare_parameter("sub_point_topic_name", 0.50);
    nd_->declare_parameter("base_frame_name", 0.50);

    execute_flag = nd_->get_parameter("execute_default").as_bool();
    depth_x_min = nd_->get_parameter("depth_range_min_x").as_double();
    depth_x_max = nd_->get_parameter("depth_range_max_x").as_double();
    depth_y_min = nd_->get_parameter("depth_range_min_y").as_double();
    depth_y_max = nd_->get_parameter("depth_range_max_y").as_double();
    depth_z_min = nd_->get_parameter("depth_range_min_z").as_double();
    depth_z_max = nd_->get_parameter("depth_range_max_z").as_double();
    sub_point_topic_name = nd_->get_parameter("sub_point_topic_name").as_string();
    base_frame_name = nd_->get_parameter("base_frame_name").as_string();
    // ros::param::get("execute_default", this->execute_flag);

    // ros::param::get("depth_range_min_x", this->depth_x_min);
    // ros::param::get("depth_range_max_x", this->depth_x_max);
    // ros::param::get("depth_range_min_y", this->depth_y_min);
    // ros::param::get("depth_range_max_y", this->depth_y_max);
    // ros::param::get("depth_range_min_z", this->depth_z_min);
    // ros::param::get("depth_range_max_z", this->depth_z_max);

    // ros::param::get("sub_point_topic_name", this->sub_point_topic_name);

    // ros::param::get("base_frame_name", this->base_frame_name);

    // Create a ROS subscriber and publisher
    sub_cloud = nd_->create_subscription<sensor_msgs::msg::PointCloud2>(sub_point_topic_name, 1, std::bind(&BagHandleEstimator::cloud_cb, this, std::placeholders::_1));
    service_execute_ctrl_ = nd_->create_service<sobits_msgs::srv::RunCtrl>("/bag_handle_estimator/run_ctr", std::bind(&BagHandleEstimator::execute_ctrl_server, this, std::placeholders::_1, std::placeholders::_2));
    pub_plane = nd_->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_plane", 1);
    // this->sub_cloud = nh.subscribe(this->sub_point_topic_name, 1, &BagHandleEstimator::cloud_cb, this);
    // this->service_execute_ctrl_ = nh.advertiseService("/bag_handle_estimator/run_ctr", &BagHandleEstimator::execute_ctrl_server, this);
    // this->pub_plane  = nh.advertise<sensor_msgs::PointCloud2>("cloud_plane", 1);

    std::cout << "start bag_handle_estimator" << std::endl;

  }  // bag_handle_estimator

  // execute control
  bool execute_ctrl_server(const std::shared_ptr<sobits_msgs::srv::RunCtrl::Request> req, 
                                 std::shared_ptr<sobits_msgs::srv::RunCtrl::Response> res) {
    execute_flag = req->request;
    if (execute_flag) {
      RCLCPP_INFO(nd_->get_logger(), "Start bag_handle_estimator.");
      // ROS_INFO("Start bag_handle_estimator.");
    } else {
      RCLCPP_INFO(nd_->get_logger(), "Stop bag_handle_estimator.");
      // ROS_INFO("Stop bag_handle_estimator.");
    }
    res->response = true;
    return true;
  }

  // callback function
  void cloud_cb(const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg) {
  // void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (!execute_flag) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_input);
    if (cloud_input->points.size() == 0) {
      return;
    }

    bool key = tfBuffer_.canTransform(base_frame_name, msg->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(2.0));
    // bool key = tfBuffer_.canTransform(this->base_frame_name, this->camera_frame_name, ros::Time(0), ros::Duration(2.0));
    if (!key) {
      RCLCPP_INFO(nd_->get_logger(), "bag_handle_estimator : pcl canTransform failue");
      // ROS_WARN("bag_handle_estimator : pcl canTransform failue");
      return;
    }
    // Transform point cloud to base frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    try {
        pcl_ros::transformPointCloud(base_frame_name, *cloud_input, *cloud_transformed, tfBuffer_);
    } catch (tf2::TransformException& ex) {
        RCLCPP_INFO(nd_->get_logger(), "[bag_handle_estimator] Failed to transform point cloud: %s", ex.what());
        // ROS_WARN("[bag_handle_estimator] Failed to transform point cloud: %s", ex.what());
        return;
    }

    // filtering X limit
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cut_x(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ>     pass_x;
    pass_x.setInputCloud(cloud_transformed);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(depth_x_min, depth_x_max);
    pass_x.filter(*cloud_cut_x);
    if (cloud_cut_x->points.size() == 0) {
      return;
    }

    // filtering Y limit
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cut_y(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ>     pass_y;
    pass_x.setInputCloud(cloud_cut_x);
    pass_x.setFilterFieldName("y");
    pass_x.setFilterLimits(depth_y_min, depth_y_max);
    pass_x.filter(*cloud_cut_y);
    if (cloud_cut_y->points.size() == 0) {
      return;
    }

    // filtering Z limit
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cuted(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ>     pass_z;
    pass_z.setInputCloud(cloud_cut_y);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(depth_z_min, depth_z_max);
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
    pub_plane_cloud(cloud_plane); 

    // search the most highest point
    int    most_high_point_index = 0;  
    double most_high_point      = DBL_MAX;
    for (size_t i = 0; i < cloud_cuted->points.size(); i++) {
      double temp_distance = cloud_cuted->points[i].x;
      if (temp_distance > most_high_point) {
        most_high_point      = temp_distance;
        most_high_point_index = i;
      }
    }

    // prepare broadcast tf 
    base_2_high_point.point.x = cloud_cuted->points[most_high_point_index].x ;
    base_2_high_point.point.y = cloud_cuted->points[most_high_point_index].y ;
    base_2_high_point.point.z = cloud_cuted->points[most_high_point_index].z ;
    broadcast_handle_point();
  }  // cloud_cb

  // publish point cloud
  void pub_plane_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_color(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, *cloud_plane_color);
    for (size_t i = 0; i < cloud_plane_color->points.size(); i++) {
      cloud_plane_color->points[i].g = 255;
    }  
    sensor_msgs::msg::PointCloud2 sensor_cloud_plane;
    pcl::toROSMsg(*cloud_plane_color, sensor_cloud_plane);
    sensor_cloud_plane.header.frame_id = base_frame_name;
    pub_plane->publish(sensor_cloud_plane);
  }  // pub_plane_cloud

  // broadcast tf
  void broadcast_handle_point() {

    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = nd_->now();
    // transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = base_frame_name;
    transformStamped.child_frame_id = "handle_point";
    transformStamped.transform.translation.x = base_2_high_point.point.x;
    transformStamped.transform.translation.y = base_2_high_point.point.y;
    transformStamped.transform.translation.z = base_2_high_point.point.z;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    broadcaster.sendTransform(transformStamped);

  }  // broadcast_handle_point
};

int main(int argc, char** argv) {
  // Initialize ROS
  // ros::init(argc, argv, "bag_handle_estimator");
  // ROS_INFO("Start bag_handle_estimator.");

  // BagHandleEstimator ppe;
  // while (ros::ok() == true) {
  //   ros::Duration(0.1).sleep();
  //   ros::spinOnce();
  // }
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nd = std::make_shared<rclcpp::Node>("bag_handle_estimator");
  RCLCPP_INFO(nd->get_logger(), "Start bag_handle_estimator.");
  auto ppe = std::make_shared<BagHandleEstimator>(nd);
  rclcpp::spin(nd);
  rclcpp::shutdown();
  return 0;
}  // main
