#include <chrono>
#include <linux/limits.h>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <cmath>
using namespace std::chrono_literals;
typedef pcl::PointXYZ PointT;

class TestingCloud: public rclcpp::Node
{
  public:
    float start_time;

    TestingCloud()
    : Node("testing_cloud")
    {

      test_pointcloud_pub = 
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/text_pointcloud_pub",10);

     test_pointcloud_timer = this->create_wall_timer(
        500ms, std::bind(&TestingCloud::test_pointcloud_callback, this));

    start_time = 0.0;

    RCLCPP_INFO(this->get_logger(), "Starting TestingCloud Node");

    }

  private:

    float angleX = 0;
    float angleZ = 0;

    void test_pointcloud_callback()
    {

      pcl::PointCloud<PointT> cloud;

      Eigen::MatrixXd cube(8,3);
      cube << 0.0, 0.0, 0.0, 
              0.0, 0.0, 1.0,
              0.0, 1.0, 0.0,
              1.0, 0.0, 0.0,
              0.0, 1.0, 1.0,
              1.0, 0.0, 1.0,
              1.0, 1.0, 0.0,
              1.0, 1.0, 1.0;

      Eigen::Matrix3d rotX(3,3);
      Eigen::Matrix3d rotZ(3,3);

      rotX << 1, 0, 0,
              0, cos(angleX), -sin(angleX),
              0, sin(angleX), cos(angleX);

      rotZ << cos(angleZ), -sin(angleZ), 0,
              sin(angleZ), cos(angleZ), 0,
              0, 0, 1;

      cube = cube * rotX;
      cube = cube * rotZ;

      float current_time = this->now().seconds()/1000;

      RCLCPP_INFO(this->get_logger(), "Current time: %f, Start time: %f, Angle: %f", current_time, start_time, angleX);
      RCLCPP_INFO(this->get_logger(), "%f, %f, %f \n%f, %f, %f ", cube(1,1), cube(1,1), cube(1,2), cube(2,1), cube(2,1), cube(2,2));


      if( (current_time - start_time) >= 1000){
        angleZ += M_PI/100;
        if (angleZ >= M_PI*2){
          angleZ = 0.0;
          start_time = this->now().seconds()/1000;
        }
      }

      for (int i=0; i< cube.rows(); i++){
        PointT point(cube(i,0), cube(i,1), cube(i,2));
        cloud.points.push_back(point);
      }

      sensor_msgs::msg::PointCloud2 ros_cloud;
      pcl::toROSMsg(cloud, ros_cloud);

      ros_cloud.header.frame_id = "map";
      ros_cloud.header.stamp = this->now();

      test_pointcloud_pub->publish(ros_cloud);

    }

    rclcpp::TimerBase::SharedPtr test_pointcloud_timer;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr test_pointcloud_pub;


  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestingCloud>());
  rclcpp::shutdown();
  return 0;
}
