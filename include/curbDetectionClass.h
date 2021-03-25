/* Author: Yi-Chen Zhang
 * Email: chris7462@gmail.com
 * Homepage: https://chris7462.github.io
 */

#ifndef CURB_DETECTION_CLASS_H
#define CURB_DETECTION_CLASS_H

// C++ lib
#include <vector>

// ROS lib
#include <ros/ros.h>

// PCL lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class CurbDetectionClass
{
  public:
    CurbDetectionClass() = default;
    void init(int layer_num);
    void cleanPoints(const pcl::PointCloud<pcl::PointXYZI> &pc_in);
    pcl::PointCloud<pcl::PointXYZI> detector();
    void slideForGettingPoints(const pcl::PointCloud<pcl::PointXYZI> &points, pcl::PointCloud<pcl::PointXYZI> &curb_container,
                               int w_0 = 10, int w_d = 30, float xy_thresh = 0.1F, float z_thresh = 0.06F);
  private:
    int getRingID(const pcl::PointXYZI &point);

    pcl::PointCloud<pcl::PointXYZI> curb_left {};
    pcl::PointCloud<pcl::PointXYZI> curb_right {};
    //pcl::PointCloud<pcl::PointXYZI> curb_front {};
    std::vector<pcl::PointCloud<pcl::PointXYZI>> pointcloud_rings {};
    //std::vector<pcl::PointCloud<pcl::PointXYZI>> pointcloud_front_rings {};

    // parameters help to select the detection window
    float m_forward;
    float m_backward;
    float m_left;
    float m_right;
    float m_up;
    float m_down;

    // front curb detector bounding box
    //float m_curb_far;
    //float m_curb_close;
    //float m_curb_left;
    //float m_curb_right;
    //int m_curb_fine;

    // used to store ring index
    std::vector<int> m_ring_index;
};

#endif
