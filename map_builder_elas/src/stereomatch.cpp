//
// Created by fei on 19-1-22.
// modified by ming on 19-3-23
//

#include "stereomatch.h"


StereoMatch::StereoMatch() {
    mParam.postprocess_only_left = true;
}

StereoMatch::StereoMatch(Elas::parameters &param) {
    mParam = param;
}

PointCloud::Ptr pointCloud(new PointCloud);

cv::Mat StereoMatch::run(const cv::Mat &left, const cv::Mat &right, const Eigen::Isometry3d &T, const float baseline, const float fx) {
    if (left.empty() || right.empty()) {
        std::cout << "Image is empty" << std::endl;
        exit(0);
    }
    if (left.channels() > 1 || right.channels() > 1) {
        std::cout << "Images must be gray image." << std::endl;
        exit(0);
    }

    Elas elas(mParam);

    int width = left.cols;
    int height = left.rows;
    const int32_t dims[3] = {width, height, width}; // bytes per line = width

    cv::Mat disparity1(height, width, CV_32F);
    cv::Mat disparity2(height, width, CV_32F);
    cv::Mat disparityMap, depthMap;

    bool valid_depth = elas.process(left.data, right.data, (float *) disparity1.data, (float *) disparity2.data, dims);

    // if (valid_depth) 
    //     depthMap = convertDisparityToDepth(disparity1, baseline, fx);

    for ( int v=0; v<height; v++)
        for ( int u=0; u<width; u++)
        {
            // std::cout << "The disparity is: "<< disparity1.at<unsigned short>(v, u) << std::endl;
            double disp = static_cast<double>(disparity1.at<float>(v, u));
            double d = 0;
            if (disp>5) d = (baseline * fx) / disp;
            else continue;
            // std::cout << "The depth is: "<< d << std::endl;
            Eigen::Vector3d point;
            point[2] = d;
            point[0] = (u-cx)*point[2]/fx;
            point[1] = (v-cy)*point[2]/fy;
            Eigen::Vector3d pointWorld = T*point;
            PointT p ;
            p.x = pointWorld[0];
            p.y = pointWorld[1];
            p.z = pointWorld[2];
            pointCloud->points.push_back( p );
        }
    pointCloud->is_dense = false;
    std::cout << "There are totally %d points" << pointCloud->size() << std::endl;
    pcl::io::savePCDFileBinary("map.pcd",*pointCloud);
    return depthMap; // return empty depth image if not valid
}

// cv::Mat StereoMatch::convertDisparityToDepth(const cv::Mat &disparityMap, const float baseline, const float fx) {
//     cv::Mat depthMap = cv::Mat(disparityMap.size(), CV_16U);
//     for (int i = 0; i < disparityMap.rows; i++) {
//         for (int j = 0; j < disparityMap.cols; j++) {
//             double d = static_cast<double>(disparityMap.at<float>(i, j));
//             depthMap.at<unsigned short>(i, j) = (baseline * fx) / d;
//             // depthMap.at<unsigned short>(i, j) = d;
//             // if (d < 10)
//             //     depthMap.at<unsigned short>(i, j) = -1;
//             if (d>0) std::cout << "The disparity is: "<< (baseline * fx) / d<< std::endl;
//             short disparity_ij = disparityMap.at<unsigned short>(i, j);
//             if(disparity_ij <= 1)
//                 depthMap.at<unsigned short>(i, j) = 0;

//             if (std::isnan(depthMap.at<unsigned short>(i, j)) || std::isinf(depthMap.at<unsigned short>(i, j)))
//                 depthMap.at<unsigned short>(i, j) = 0;
//         }
//     }

//     return depthMap;
// }