#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/tracking.hpp>

int main()
{
    // ****** Data Initialization ********* //
    // Kalman filter here, DP = 16, MP = 8, CP = 0
    int DP = 16, MP = 8, CP = 0;
    cv::KalmanFilter KF(DP, MP, CP);

    /////////////////////////////////////////////////////////
    // Predict Stage
    /////////////////////////////////////////////////////////    
    cv::Mat measurement = cv::Mat::ones(DP, 1, CV_32F);
    std::cout << "\nmeasurement:" << measurement << std::endl;
    cv::Mat predict = KF.predict();
    std::cout << "\npredict: \n" << predict << std::endl;
    
    /////////////////////////////////////////////////////////
    // Update Stage
    /////////////////////////////////////////////////////////    

    cv::Mat control = cv::Mat::ones(MP, 1, CV_32F);
    std::cout << "\n control: \n" << control << std::endl;
    cv::Mat update = KF.correct(control);  

    std::cout << "\npredict:" << predict << "control" << control << std::endl;

    return 0;
}