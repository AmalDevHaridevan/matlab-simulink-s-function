//#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>
#include <opencv2/video/tracking.hpp>
#include <cmath>
#include <vector>
#include <tuple>
#include "optical_flow_uav_velocity.hpp"

// Main class written in C++
// Your custom class can be defined here.


OpticalFlowTracking::OpticalFlowTracking(int method, float delta_t, float camera_focal_length, float cmos_width, float cmos_height)
    : _method(method), _delta_t(delta_t), _focal_length(camera_focal_length),
        _cmos_width(cmos_width), _cmos_height(cmos_height), _last_im(cv::Mat()), _features(std::vector<cv::Point2f>()),_debug_count(0) {
    _fov_h = 2 * std::atan(_cmos_width / (2 * _focal_length));
    _fov_v = 2 * std::atan(_cmos_height / (2 * _focal_length));
}

// Sets the delta_t variable which is used for camera velocity estimation
void OpticalFlowTracking::_set_delta_t_(double delta_t_){

    _delta_t = delta_t_;
}
// If we have features or not
bool OpticalFlowTracking::_has_features(){
    
    return _features.size()>0;
}
// Extracts features from the image
void OpticalFlowTracking::extractFeatures(const cv::Mat &img) {
    cv::Mat gray;
    if (img.channels() == 3) {
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = img;
    }
    _features.clear();
    cv::goodFeaturesToTrack(gray, _features, 1000, 0.1,8, cv::Mat(),2 );
    if (! _has_features()){
        return;
    }
    _last_im = gray.clone();
    _img_width = gray.cols;
    _img_height = gray.rows;
}
// calculates the camera velocity based on the tracked features, or optical flow
std::tuple<std::vector<float>, std::vector<float>, std::vector<cv::Point2f>, std::vector<cv::Point2f>, bool>
OpticalFlowTracking::calculateRealVel(const cv::Mat &img, float height) {
    if (_last_im.empty()) {
        extractFeatures(img);
        return {{}, {}, {}, {}, true};
    }

    _debug_count += 1;
    std::vector<uchar> status;
    std::vector<float> error;
    std::vector<cv::Point2f> new_features;
    /*method_parameters=dict( winSize  = (16, 16),
                maxLevel = 2,
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 8, 0.03))*/
    cv::calcOpticalFlowPyrLK(_last_im, img, _features, new_features, status, error, cv::Size(16,16),2, _criteria);

    std::vector<float> vels_x, vels_y;
    std::vector<cv::Point2f> old_features;

    for (size_t i = 0; i < new_features.size(); ++i) {
        if (status[i]) {
            float dx = new_features[i].x - _features[i].x;
            float dy = new_features[i].y - _features[i].y;
            vels_x.push_back(dy / _delta_t);  // Y-axis represents X of the drone
            vels_y.push_back(-dx / _delta_t); // X-axis represents -Y of the drone
            old_features.push_back(_features[i]);
        }
    }

    _features.clear();
    cv::goodFeaturesToTrack(img, _features, 1000, 0.1,8, cv::noArray(),2 );
    _last_im = img.clone();

    std::vector<float> v_est_x, v_est_y;
    for (size_t i = 0; i < vels_x.size(); ++i) {
        v_est_x.push_back(height * std::tan(vels_x[i] * _delta_t * _fov_v / _img_height) / _delta_t);
        v_est_y.push_back(height * std::tan(vels_y[i] * _delta_t * _fov_h / _img_width) / _delta_t);
    }

    return {v_est_x, v_est_y, new_features, old_features, true};
}
