/* Author: Yi-Chen Zhang
 * Email: chris7462@gmail.com
 * Homepage: https://chris7462.github.io
 */

// C++ lib
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

// ROS lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2::ConstPtr> pointCloudBuffer;

ros::Publisher pubLaserCloudCurb;

void velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  mutex_lock.lock();
  pointCloudBuffer.push(msg);
  mutex_lock.unlock();
}

void curb_detection() {
  while (ros::ok()) {
    if (!pointCloudBuffer.empty()) {
      // Read data
      mutex_lock.lock();
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*pointCloudBuffer.front(), *pointcloud_in);
      ros::Time pointcloud_time = (pointCloudBuffer.front())->header.stamp;
      pointCloudBuffer.pop();
      mutex_lock.unlock();

      std::chrono::time_point<std::chrono::steady_clock> start, end;
      start = std::chrono::steady_clock::now();
      //laserProcessing.featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);
      end = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed_seconds = end - start;
      ROS_INFO("Curb detection process time: %f ms", elapsed_seconds.count() * 1000);

      sensor_msgs::PointCloud2 pointCloudCurbMsg;
      //pcl::toROSMsg(*pointcloud_filtered, pointCloudCurbMsg);
      pointCloudCurbMsg.header.stamp = pointcloud_time;
      pointCloudCurbMsg.header.frame_id = "map";
      pubLaserCloudCurb.publish(pointCloudCurbMsg);
    }
    //sleep 2 ms every time
    //std::chrono::milliseconds dura(2);
    //std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "curb_detection_node");
  ros::NodeHandle nh;

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("points_raw", 100, velodyneHandler);

  pubLaserCloudCurb = nh.advertise<sensor_msgs::PointCloud2>("curb_detection", 100);

  std::thread curb_detection_prcess{curb_detection};

  ros::spin();

  return 0;
}