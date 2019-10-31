//
// Created by fei on 19-1-22.
// modified by ming on 19-3-23
//

#ifndef STEREO_MATCH_H
#define STEREO_MATCH_H

#include "elas.h"
#include <opencv2/core/mat.hpp>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class StereoMatch {
public:
    StereoMatch();
    StereoMatch(Elas::parameters &param);

    cv::Mat run(const cv::Mat &left, const cv::Mat &right, const Eigen::Isometry3d &T, const float baseline, const float fx);
    cv::Mat convertDisparityToDepth(const cv::Mat &disp, const float baseline, const float fx);

public:
    Elas::parameters mParam;
    double fx = 361.3;
    double fy = 381.8;
    double cx = 361.9;
    double cy = 246.0;
};

#endif //STEREO_MATCH_H
