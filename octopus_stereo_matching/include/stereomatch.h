//
// Created by fei on 19-1-22.
// modified by ming on 19-3-23
//

#ifndef STEREO_MATCH_H
#define STEREO_MATCH_H

#include "elas.h"
#include <opencv2/core/mat.hpp>

class StereoMatch {
public:
    StereoMatch();
    StereoMatch(Elas::parameters &param);

    cv::Mat run(const cv::Mat &left, const cv::Mat &right, const float baseline, const float fx);
    cv::Mat convertDisparityToDepth(const cv::Mat &disp, const float baseline, const float fx);

public:
    Elas::parameters mParam;
};

#endif //STEREO_MATCH_H
