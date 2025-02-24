
// your name of the s-function, this will be the name of the mex file
#define S_FUNCTION_NAME matlab_simulink_s_function
#define S_FUNCTION_LEVEL 2
// Define this variable to use persistent memory, by default we use static objects to access our custom class instantiated in the s-function
// #define USE_PERSISTENT_MEMORY

#include "simstruc.h"
#include "optical_flow_uav_velocity.hpp"
#include <thread>
#include <chrono>
#include <map>
#include <iostream>
#include <stdexcept>

// Clamp function
template <typename T>
T clamp(T value, T minVal, T maxVal) {
    return (value < minVal) ? minVal : (value > maxVal) ? maxVal : value;
}

#ifndef USE_PERSISTENT_MEMORY
// We will use static objects to use our objects, antoher way is to use persistent memory
// If we want to use persistent memeory, then we can use ssGetPWorkValue(S, 0) to get the pointer to the object
// One caveat is to always perform void* conversion to the object type
static std::map<int, std::shared_ptr<OpticalFlowTracking>> obj_map;
static std::map<int, std::shared_ptr<cv::Mat>> img_map;
#endif

static void mdlInitializeSizes(SimStruct* S) {
    
    ssSetNumSFcnParams(S, 6); // 6 parameters, focal length of the camera, height of the camera sensor, width of the camera sensor (all in m), and finally a unique id for the
    // object. Since we store static objectm member , we dont want conflicts if multiple s-funcs are created
    // Finally the last 2 params are the height and width of the image
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; // Parameter mismatch
    }

    // Configure Input Port
    if (!ssSetNumInputPorts(S, 2)) return;
    int height_ = (int) mxGetScalar(ssGetSFcnParam(S, 4));
    int width_ = (int) mxGetScalar(ssGetSFcnParam(S, 5));
    int id_ = (int) mxGetScalar(ssGetSFcnParam(S,3));
    ssSetInputPortMatrixDimensions(S, 0, (int) mxGetScalar(ssGetSFcnParam(S, 4)) , (int) mxGetScalar(ssGetSFcnParam(S, 5))); // Input: image matrix
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    
    img_map[id_] = std::make_shared<cv::Mat>(height_, width_, CV_8UC1, cv::Scalar(0));

    ssSetInputPortWidth(S, 1,1); // Sampling time for the image , delta t between last image and current image,
    // We use it here because we also want to enable this is simulation, otherwise the delta t will be based on real time

    ssSetInputPortDirectFeedThrough(S, 1, 1);

    // Configure Output Port
    if (!ssSetNumOutputPorts(S,3)) return;
    ssSetOutputPortMatrixDimensions(S, 0, 2, 1000); // Output: 2x1000 matrix (max 1000 features)
    ssSetOutputPortWidth(S,1,1); // calculation time
    ssSetOutputPortWidth(S,2,1); // num samples
    // Configure Sample Times
    ssSetNumSampleTimes(S, 1);
    #ifdef USE_PERSISTENT_MEMORY
    // add persistent worker
    ssSetNumPWork(S, 1); 
    #endif

    /* specify the sim state compliance to be same as a built-in block */
    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);

    /* Set this S-function as runtime thread-safe for multicore execution */
    ssSetRuntimeThreadSafetyCompliance(S, RUNTIME_THREAD_SAFETY_COMPLIANCE_TRUE);
   
    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);


}


static void mdlInitializeSampleTimes(SimStruct* S) {

    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);

    real_T focal_length =  mxGetScalar(ssGetSFcnParam(S, 0));
    double focal_length_ = (double) focal_length;
    real_T cmos_width =  mxGetScalar(ssGetSFcnParam(S, 2));
    double cmos_width_ = (double) cmos_width;
    real_T cmos_height =  mxGetScalar(ssGetSFcnParam(S, 1));
    double cmos_height_ = (double) cmos_height;
    real_T id_ = mxGetScalar(ssGetSFcnParam(S,3));
    int id__ = (int) id_;
    #ifndef USE_PERSISTENT_MEMORY
    std::shared_ptr<OpticalFlowTracking> new_obj_ = std::make_shared<OpticalFlowTracking>(100, 1.0, focal_length_, cmos_width_, cmos_height_);
    obj_map[id__] = new_obj_;
    #else 
    OpticalFlowTracking* obj = new OpticalFlowTracking(100, 1.0, focal_length_, cmos_width_, cmos_height_);
    // error check the newly created obj
    if (&obj_ == nullptr) {
        ssSetErrorStatus(S, "Pointer to instatiated class is null during init.");
        return;
    }
    // cast to void* to store in the persistent memory
    ssSetPWorkValue(S, 0, static_cast<void*>(obj));
    #endif

}
static void mdlOutputs(SimStruct* S, int_T tid) {
  
    
    #ifdef USE_PERSISTENT_MEMORY
        OpticalFlowTracking* obj_ = static_cast<OpticalFlowTracking*>(ssGetPWorkValue(S,0));
        cv::Mat* image = new cv::Mat(480, 640, CV_8UC1, cv::Scalar(0));
    #else 
        std::shared_ptr<cv::Mat> image = img_map[(int) mxGetScalar(ssGetSFcnParam(S,3))];
        std::shared_ptr<OpticalFlowTracking> obj_ = obj_map[(int) mxGetScalar(ssGetSFcnParam(S,3))];
    #endif 
    auto start_time = std::chrono::high_resolution_clock::now(); 
    std::vector<cv::Point2f> _features;
    // Get input signals
    // We need to reshape the input to our matrix shape
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    InputRealPtrsType delta_t_ptr = ssGetInputPortRealSignalPtrs(S, 1);
    
    if (obj_ == nullptr){

        ssSetErrorStatus(S, "Couldn't retrive tracker....");
    return;
    }
    if (image == nullptr){

        ssSetErrorStatus(S, "Couldn't retrive image....");
    return;
    }
    // call objects functions here
    obj_->_set_delta_t_((double) delta_t_ptr[0][0]);
    // reshape input to matrix, we are expecting an image
    int height_ = (int) mxGetScalar(ssGetSFcnParam(S, 4));
    int width_ = (int) mxGetScalar(ssGetSFcnParam(S, 5));
    for (int r = 0; r < height_; r++) {
        for (int c = 0; c < width_; c++) {
            image->at<uchar>(r, c) = static_cast<uchar>(clamp((uPtrs[r + c * height_][0])*255, 0.0, 255.0));
        }
    }
    
    std::vector<float> velsx, velsy;
    try{
        std::tuple<std::vector<float>, std::vector<float>, std::vector<cv::Point2f>, std::vector<cv::Point2f>, bool> result = obj_->calculateRealVel(*image, 1.0);
        velsx = std::get<0>(result);
        velsy = std::get<1>(result);
    }
    catch (const cv::Exception& e){
        velsx = std::vector<float>();
        velsy = std::vector<float>();
        ssWarning(S, e.what());
    }
    // get the matrix dims we set during initialization
    int_T numRows = ssGetOutputPortDimensions(S, 0)[0]; 
    int_T numCols = ssGetOutputPortDimensions(S, 0)[1]; 
    // get the ptr to the output signal, to which we will assign any result obtained
    real_T* y = ssGetOutputPortRealSignal(S, 0);
    real_T* time_sig = ssGetOutputPortRealSignal(S, 1);
    real_T* n_sigs = ssGetOutputPortRealSignal(S, 2);
    int validCols = std::min(static_cast<int>(velsx.size()), numCols);

    for (int col = 0; col < validCols; col++) {
        y[0 + col * 2] =   velsx[col];
        y[1 + col * 2] = velsy[col];
    }

    auto end_time = std::chrono::high_resolution_clock::now(); // End timing
    auto elapsed_time = std::chrono::duration<double>(end_time - start_time).count(); 
    *time_sig = elapsed_time;
    *n_sigs = (double) validCols;
}

static void mdlTerminate(SimStruct* S) {
    #ifdef  USE_PERSISTENT_MEMORY
    OpticalFlowTracking* obj = static_cast<OpticalFlowTracking*>(ssGetPWorkValue(S, 0));
    if (obj != nullptr) {
        delete obj;
        ssSetPWorkValue(S, 0, nullptr);
    }
    #endif
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
