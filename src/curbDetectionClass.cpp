/* Author: Yi-Chen Zhang
 * Email: chris7462@gmail.com
 * Homepage: https://chris7462.github.io
 */

// C++ lib
#include <cmath>
#include <algorithm>
#include <numeric>

#include "curbDetectionClass.h"

/** \brief Help sort point cluds in ascending order. (For points where y is greater than 0)
 *
 * @param a The first point.
 * @param b The second point.
 * @return true if a.y < b.y
 */
inline bool compUp(const pcl::PointXYZI &a, const pcl::PointXYZI &b) {
  return a.y < b.y;
}

/** \brief Help sort point cluds in descending order. (For points where y is less than 0)
 *
 * @param a The first point.
 * @param b The second point.
 * @return true if a.y > b.y
 */
inline bool compDown(const pcl::PointXYZI &a, const pcl::PointXYZI &b) {
  return a.y > b.y;
}

/** \brief Help sort point cluds in descending order in z direction
 *
 * @param a The first point.
 * @param b The second point.
 * @return true if a.z < b.z
 */
inline bool compVert(const pcl::PointXYZI &a, const pcl::PointXYZI &b) {
  return a.z < b.z;
}

void CurbDetectionClass::init(int layer_num) {
  // Set the parameters of the area of interest. The four parameters set the rectangle of this region of interest.
  m_forward = 30.0F;
  m_backward = 0.0F;
  m_left = 10.0F;
  m_right = -10.0F;
  m_up = 0.0F;
  m_down = -1.5F;

  // Set the parameters of the front curb bounding box.
  //m_curb_far = -15.0F;
  //m_curb_close = -9.0F;
  //m_curb_left = 5.0F;
  //m_curb_right = -5.0F;
  //m_curb_fine = 2;

  // ring_index is the layer id. Total is 64. We select layer from 11 to 11+layer_num.
  m_ring_index = std::vector<int>(layer_num);
  std::iota(m_ring_index.begin(), m_ring_index.end(), 11);
}

void CurbDetectionClass::cleanPoints(const pcl::PointCloud<pcl::PointXYZI> &pc_in) {
  pcl::PointXYZI point;
  int scan_id;

  // laser_cloud_scans is used to story point clouds for each ring.
  pointcloud_rings.clear();
  pointcloud_rings.resize(m_ring_index.size());

  //int front_rings_size = static_cast<int>((m_curb_left-m_curb_right)*m_curb_fine);
  //pointcloud_front_rings.clear();
  //pointcloud_front_rings.resize(front_rings_size);

  for (size_t i = 0; i < pc_in.size(); ++i) {
    point.x = pc_in[i].x;
    point.y = pc_in[i].y;
    point.z = pc_in[i].z;
    point.intensity = pc_in[i].intensity;

    // discard NaN
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    // Discard vehicle point clouds
    //if ( sqrt(point.x * point.x + point.y * point.y + point.z * point.z <= 3.0F ) ) {
    //    continue;
    //}

    // Discard points outside of the box
    if ((point.y > m_left) || (point.x < m_right) ||
        (point.x < m_backward) || (point.x > m_forward) ||
        (point.z > m_up) || (point.z < m_down)) {
      continue;
    }

    scan_id = getRingID(point);

    for (size_t ring_num = 0; ring_num < m_ring_index.size(); ++ring_num) {
      if (scan_id == m_ring_index[ring_num]) {
        pointcloud_rings[ring_num].push_back(point);
      }
    }

    // Front rings
    //if ((point.y > m_curb_far) && (point.y < m_curb_close) && (point.x > m_curb_right) && (point.x < m_curb_left)) {
    //  pointcloud_front_rings[static_cast<int>((point.x-m_curb_right)*m_curb_fine)].push_back(point);
    //}
  }
}


pcl::PointCloud<pcl::PointXYZI> CurbDetectionClass::detector() {
  if( pointcloud_rings.empty() && pointcloud_front_rings.empty() ) {
    return pcl::PointCloud<pcl::PointXYZI>{};
  }

  // Left and right curb detection
  if ( !pointcloud_rings.empty() ) {
    for ( std::size_t i = 0; i < pointcloud_rings.size(); ++i ) {

      pcl::PointCloud<pcl::PointXYZI> pointsInTheRing = pointcloud_rings[i]; // Save the points on this line.
      pcl::PointCloud<pcl::PointXYZI> pc_left;    // Store the point where y is greater than 0 (the left point).
      pcl::PointCloud<pcl::PointXYZI> pc_right;   // Store the point where y is less than 0 (the right point).

      pcl::PointXYZI point;
      size_t numOfPointsInTheRing = pointsInTheRing.size();
      // Separate the left and right points and save them to the corresponding point cloud.
      for ( std::size_t idx = 0; idx < numOfPointsInTheRing; ++idx ) {
        point = pointsInTheRing[idx];
        if (point.x >= 0) {
          pc_left.push_back(point);
        } else {
          pc_right.push_back(point);
        }
      }

      // Sort. (In ascending order of absolute value)
      sort(pc_left.begin(), pc_left.end(), compUp);
      sort(pc_right.begin(), pc_right.end(), compDown);

      //if ( i < pointcloud_rings.size()-4 ) {
        slideForGettingPoints(pc_left, curb_left);
        slideForGettingPoints(pc_right, curb_right);
      //} else {
      //  for ( auto &point : pc_left ) {
      //    if (point.intensity > 0.6F ) {
      //      curb_left.push_back(point);
      //    }
      //  }
      //  for ( auto &point : pc_right ) {
      //    if ( point.intensity > 0.6F ) {
      //      curb_right.push_back(point);
      //    }
      //  }
      //}
    }
  }

  // Front curb detection
  //if ( !pointcloud_front_rings.empty() ) {
  //  for ( std::size_t i = 0; i < pointcloud_front_rings.size(); ++i ) {
  //    pcl::PointCloud<pcl::PointXYZI> pc_front = pointcloud_front_rings[i];
  //
  //    // Sort
  //    sort(pc_front.begin(), pc_front.end(), compVert);
  //
  //    slideForGettingPoints(pc_front, curb_front, 5, 15, 1.3F, 0.15F);
  //  }
  //}

  pcl::PointCloud<pcl::PointXYZI> curbs { curb_left + curb_right };
  //curbs += curb_front;
  curb_left.clear();
  curb_right.clear();
  //curb_front.clear();

  return curbs;
}

void CurbDetectionClass::slideForGettingPoints(const pcl::PointCloud<pcl::PointXYZI> &points, pcl::PointCloud<pcl::PointXYZI> &curb_container,
                                              int w_0, int w_d, float xy_thresh, float z_thresh) {

  int i = 0;
  int points_num = points.size();

  while ((i + w_d) < points_num) {
    float z_max = points[i].z;
    float z_min = points[i].z;

    int idx = 0;
    float z_dis = 0;

    for (int j = 0; j < w_d; ++j) {
      float dis = fabs(points[i + j].z - points[i + j + 1].z);
      if (dis > z_dis) {
        z_dis = dis;
        idx = i + j + 1;
      }
      if (points[i + j].z < z_min) {
        z_min = points[i + j].z;
      }
      if (points[i + j].z > z_max) {
        z_max = points[i + j].z;
      }
    }

    if (fabs(z_max - z_min) >= z_thresh) {
      for (int j = 0; j < (w_d - 1); ++j) {
        float p_dist = sqrt(((points[i + j].y - points[i + 1 + j].y) * (points[i + j].y - points[i + 1 + j].y))
                          + ((points[i + j].x - points[i + 1 + j].x) * (points[i + j].x - points[i + 1 + j].x)));
        if (p_dist >= xy_thresh) {
          curb_container.push_back(points[j + i + 1]);
          return;
        }
      }

      curb_container.push_back(points[idx]);
      return;
    }
    i += w_0;
  }
}

int CurbDetectionClass::getRingID(const pcl::PointXYZI &point) {
  // find which to which layer the point belongs
  double angle;
  int scanID;
  angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y));
  scanID = (int)(angle * 134.18714161056457 + 58.81598513011153);

  return scanID;
}