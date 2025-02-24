//#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>
#include <opencv2/video/tracking.hpp>
#include <cmath>
#include <vector>
#include <tuple>

// Main class written in C++
// Your custom class can be forward declared here.
class OpticalFlowTracking {
public:
    static constexpr int FEATURE_EXTRACTION_PROCEDURE_DYNAMIC = 1;
    static constexpr int FEATURE_EXTRACTION_PROCEDURE_ONCE = 2;
    static constexpr int OPTICAL_FLOW_LUCAS_KANADE = 100;
    static constexpr int FEATURE_EXTRACTION_OPENCV_SIMPLE = 1000;

    OpticalFlowTracking(int method, float delta_t, float camera_focal_length, float cmos_width, float cmos_height);

    // Sets the delta_t variable which is used for camera velocity estimation
    void _set_delta_t_(double delta_t_);
    // If we have features or not
    bool _has_features();
    // Extracts features from the image
    void extractFeatures(const cv::Mat &img);
    // calculates the camera velocity based on the tracked features, or optical flow
    std::tuple<std::vector<float>, std::vector<float>, std::vector<cv::Point2f>, std::vector<cv::Point2f>, bool>
    calculateRealVel(const cv::Mat &img, float height) ;

private:
    int _method;
    float _delta_t;
    float _focal_length;
    float _cmos_width;
    float _cmos_height;
    float _fov_h;
    float _fov_v;
    int _img_width;
    int _img_height;
    int _debug_count;
    cv::Mat _last_im;
    std::vector<cv::Point2f> _features;
    cv::TermCriteria _criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 8, 0.03);
    
};